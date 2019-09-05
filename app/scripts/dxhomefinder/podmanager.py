"""
pod manager
"""
# pylint: disable=E1101

__version__ = "0.1.0"

__copyright__ = "Copyright 2017, Softbank Robotics"
__author__ = 'mcaniot'
__email__ = 'mcaniot@softbankrobotics.com'


import math
import almath
import numpy

# Import qi
import qi
import functools

# stk libs
import stk.runner
import stk.events
import stk.services
import stk.logging
import stk.coroutines

MOVE_CONFIG_LOW_SPEED = [["MaxVelXY", 0.15],["MaxAccXY", 0.15],["MaxVelTheta",0.5],["MaxAccTheta",0.325],["MaxJerkXY",0.5],["MaxJerkTheta",1.0]]

MOVE_CONFIG_HIGH_SPEED = [["MaxVelXY", 0.25],["MaxAccXY", 0.3]]

DISTANCE_FIRST_APPROACH = 1.0

DISTANCE_FROM_POD = 0.7

class PodManager(object):
    """
    Class for saving position
    """
    APP_ID = "com.softbankrobotics.PodManager"
    def __init__(self, session):
        self.session = session
        self.events = stk.events.EventHelper(self.session)
        self.services = stk.services.ServiceCache(self.session)
        self.logger = stk.logging.get_logger(self.session, self.APP_ID)
        self.pod_pos_in_world = None # We don't know it at start
        self.look_for_station_future = None
        self.pod_future = None
        self.search_future = None
        self.future_sleep = None
        self.search_future_track = None
        self.task = qi.PeriodicTask()
        self.task.setCallback(self.pod_detected)
        self.task.setUsPeriod(100000) # 100ms
        self.pod_position_from_robot = None

    ###########################################
    #            Public Method                #
    ###########################################

    @qi.nobind
    def on_start(self):
        "On start of the service"
        self.events.connect_decorators(self)
        self.logger.info(" on start")

    @qi.nobind
    def on_stop(self):
        "Cleanup"
        self.logger.info("Application finished.")
        self.events.clear()

    @qi.nobind
    def stop(self):
        "Standard way of stopping the application."
        self.qiapp.stop()

    def cancel(self):
        "Cancel return home"
        try:
            if self.pod_future:
                self.pod_future.cancel()
        except Exception as error_msg:
            self.logger.warning(error_msg)
        try:
            if self.future_sleep:
                self.future_sleep.cancel()
        except Exception as error_msg:
            self.logger.warning(error_msg)
        self.pod_future = None
        self.future_sleep = None
        self.services.ALRecharge.stopAll()
        self.services.ALMotion.killMove()
        self.cancel_search_home()

    @stk.coroutines.public_async_generator
    def init_home(self):
        value = True
        if not self.is_init():
            future_return = yield self.return_home()
            self.logger.info(future_return)
            value = future_return
        if value:
            self.services.PositionManager.init_position()
            self.pod_pos_in_world = [0.0, 0.0]
            self.pod_position_from_robot = None
        yield stk.coroutines.Return(value)

    def is_init(self):
        return self.pod_pos_in_world != None

    def uninit(self):
        self.pod_pos_in_world = None
        self.pod_position_from_robot = None

    def need_approach_home(self):
        if self.services.PositionManager:
            if self.pod_pos_in_world:
                dist_from_home = self.services.PositionManager.get_dist_from_robot(0,0)
                if dist_from_home > DISTANCE_FIRST_APPROACH + 0.2:
                    return True
        return False

    @stk.coroutines.async_generator
    def approach_home(self):
        x_coord, y_coord, theta = self.services.PositionManager.get_home_pos()
        coord_pod = [x_coord, y_coord, theta]

        p2D_world2robot = almath.Pose2D(self.services.ALMotion.getRobotPosition(True))

        p2D_world2target = almath.Pose2D(coord_pod)
        t_world2robot = almath.transformFromPose2D(p2D_world2robot)
        t_world2target = almath.transformFromPose2D(p2D_world2target)
        t_robot2target = t_world2robot.inverse() * t_world2target
        p6D_robot2target = almath.position6DFromTransform(t_robot2target)
        coord_pod = list(p6D_robot2target.toVector())

        coord_home = [coord_pod[0] + DISTANCE_FIRST_APPROACH * math.cos(coord_pod[-1]),
                      coord_pod[1] + DISTANCE_FIRST_APPROACH * math.sin(coord_pod[-1]),
                      coord_pod[-1] + math.pi]

        is_success = False
        if self.services.PositionManager.turn_toward_position([DISTANCE_FIRST_APPROACH, 0]):
            x, y, theta = coord_home
            r, r_theta = self.get_polar_coord([x, y, theta])
            if self.services.ALMotion.moveTo(r, 0, 0, MOVE_CONFIG_HIGH_SPEED):
                is_success = True
                yield self.services.PositionManager.turn_toward_position([0, 0],
                                                                         _async=True)
        yield stk.coroutines.Return(is_success)

    @stk.coroutines.async_generator
    def find_home(self, research_360_activated=False):
        "returns (coords) of any code it finds, or (None)."
        distance_pod = None
        mode_search = False
        if not self.services.PositionManager:
            yield stk.coroutines.Return(None)
        if self.pod_pos_in_world and not research_360_activated:
            pod_future = self.services.PositionManager.turn_toward_position([-1.0, 0],
                                                                     _async=True)
            yield pod_future
            distance_pod = self.services.PositionManager.get_dist_from_robot(-0.70,0)
            pod_future = self.services.ALTracker._lookAtWithEffector(
                    [distance_pod, 0, 0],
                    2,
                    2,
                    0.1,
                    0,
                    _async=True)
            yield pod_future
        else:
            mode_search = True
        pod_future = None
        if mode_search:
            pod_future = self.search_home()
            yield pod_future
            self.logger.info(repr(pod_future.value()))
        else:
            yield self.services.ALRecharge.setUseTrackerSearcher(False, _async=True)
            pod_future = qi.async(self.services.ALRecharge.lookForStation)
            self.future_sleep = stk.coroutines.sleep(5)
            yield self.future_sleep
            if pod_future.isRunning():
                self.services.ALRecharge.stopAll()
                yield stk.coroutines.Return(None)
        if pod_future:
            coord_pod = pod_future.value()
            if coord_pod and coord_pod[1] and len(coord_pod[1]) == 3:
                p2D_world2robot = almath.Pose2D(self.services.ALMotion.getRobotPosition(True))
                coord_pod = coord_pod[1]
                self.services.PositionManager.init_position_with_coord(coord_pod)
                self.pod_pos_in_world = [0.0, 0.0]
                p2D_world2target = almath.Pose2D(coord_pod)
                t_world2robot = almath.transformFromPose2D(p2D_world2robot)
                t_world2target = almath.transformFromPose2D(p2D_world2target)
                t_robot2target = t_world2robot.inverse() * t_world2target
                p6D_robot2target = almath.position6DFromTransform(t_robot2target)
                coord_pod = list(p6D_robot2target.toVector())

                coord_home = [coord_pod[0] + DISTANCE_FROM_POD * math.cos(coord_pod[-1]),
                              coord_pod[1] + DISTANCE_FROM_POD * math.sin(coord_pod[-1]),
                              coord_pod[-1]]
                yield stk.coroutines.Return(coord_home)
        yield stk.coroutines.Return(None)


    def get_polar_coord(self, coord_home):
        x , y , theta = coord_home
        r_theta = float(numpy.arctan(y/x))
        r = float(math.sqrt(math.pow(x,2) + math.pow(y,2)))
        return r, r_theta

    @stk.coroutines.async_generator
    @stk.logging.log_exceptions
    def last_move(self, x, y, theta):
        value = yield self.services.ALMotion.moveTo(
                                         x,
                                         y,
                                         0,
                                         MOVE_CONFIG_LOW_SPEED,
                                         _async=True)
        if value:
            value = yield self.services.ALMotion.moveTo(
                                         0,
                                         0,
                                         almath.modulo2PI(
                                                 theta),
                                         _async=True)
            if value:
                self.pod_pos_in_world = [0.0,0.0]
                self.pod_position_from_robot = None
                yield stk.coroutines.Return(True)
        yield stk.coroutines.Return(False)

    @stk.coroutines.async_generator
    def return_home(self, research_360_activated=False):
        on_home = False
        if self.need_approach_home() and not research_360_activated:
            self.pod_future = self.approach_home()
            yield self.pod_future
            if not self.pod_future.value():
                if self.services.DXHomeFinder:
                    self.services.DXHomeFinder.reason_cant_go_home.setValue("obstacle")
                yield stk.coroutines.Return(False)
        self.pod_future = self.find_home(research_360_activated)
        coord_home = yield self.pod_future
        if coord_home:
            r, r_theta = self.get_polar_coord(coord_home)
            if r > 0.5:
                x, y, theta = coord_home
                x += 0.3 * math.cos(theta)
                y += 0.3 * math.sin(theta)
                r, r_theta = self.get_polar_coord([x, y, theta])
                self.pod_future = self.services.ALMotion.moveTo(
                                                    0,
                                                    0,
                                                    almath.modulo2PI(
                                                            r_theta),
                                                    MOVE_CONFIG_LOW_SPEED,
                                                    _async=True)
                yield self.pod_future
                if self.pod_future.value():
                    self.pod_future = self.services.ALMotion.moveTo(
                                                    r,
                                                    0,
                                                    0,
                                                    MOVE_CONFIG_LOW_SPEED,
                                                    _async=True)
                    yield self.pod_future
                    if self.pod_future.value():
                        on_home = True
                        self.pod_future = self.services.ALMotion.moveTo(
                                                    0,
                                                    0,
                                                    almath.modulo2PI(
                                                    coord_home[-1] - r_theta + math.pi),
                                                    MOVE_CONFIG_LOW_SPEED,
                                                    _async=True)
                        yield self.pod_future
                        coord_home = None
            else:
                on_home = True
        else:
            # can't find pod
            if self.services.DXHomeFinder:
                self.services.DXHomeFinder.reason_cant_go_home.setValue("no_target")
        if on_home:
            if not coord_home:
                self.pod_future = self.find_home()
                coord_home = yield self.pod_future
            if coord_home:
                x, y, theta = coord_home
                self.pod_future = self.last_move(
                        x,
                        y,
                        theta)
                yield self.pod_future
                if not self.pod_future.value():
                    if self.services.DXHomeFinder:
                        self.services.DXHomeFinder.reason_cant_go_home.setValue("obstacle")
                yield stk.coroutines.Return(self.pod_future.value())
            else:
                # can't find pod
                if self.services.DXHomeFinder:
                    self.services.DXHomeFinder.reason_cant_go_home.setValue("no_target")
        if self.services.DXHomeFinder:
            if self.services.DXHomeFinder.reason_cant_go_home.value() == "unknown":
                self.services.DXHomeFinder.reason_cant_go_home.setValue("obstacle")
        yield stk.coroutines.Return(False)

    @stk.coroutines.async_generator
    def search_routine(self):
        angle_to_do = 50
        max_turn = 0
        if 360%angle_to_do > 0:
            max_turn += 1
        max_turn = 360/angle_to_do
        coord_look_at_list = [[3.0, 0, 0], [0.0, 0, 0], [1.0, 0, 0]]
        for i in range(0, max_turn):
            for coord_look_at in coord_look_at_list:
                self.search_future_track = self.services.ALTracker._lookAtWithEffector(
                        coord_look_at,
                        2,
                        2,
                        0.1,
                        1,
                        _async=True)
                yield self.search_future_track
            self.search_future_track = self.services.ALMotion.moveTo(0, 0, math.radians(angle_to_do), _async=True)
            yield self.search_future_track

    @stk.coroutines.public_async_generator
    @stk.logging.log_exceptions
    def search_home(self):
        yield self.services.ALRecharge.setUseTrackerSearcher(False, _async=True)
        self.look_for_station_future = None
        self.look_for_station_future = qi.async(self.services.ALRecharge.lookForStation)
        if self.task and self.task.isRunning():
            self.task.stop()
        self.task.start(True)
        try:
            self.search_future = self.search_routine()
            yield self.search_future
        except Exception as error_msg:
            self.logger.warning(error_msg)
        self.cancel_search_home()
        yield stk.coroutines.Return(self.pod_position_from_robot)

    def pod_detected(self):
        try:
            if not self.look_for_station_future.isRunning():
                self.pod_position_from_robot = self.look_for_station_future.value()
                self.cancel_search_home()
        except Exception as error_msg:
            pass
        

    def cancel_search_home(self):
        try:
            self.task.stop()
        except Exception as error_msg:
            self.logger.warning(error_msg)
        try:
            if self.search_future:
                self.search_future.cancel()
        except Exception as error_msg:
            self.logger.warning(error_msg)
        try:
            if self.search_future_track:
                self.search_future_track.cancel()
        except Exception as error_msg:
            self.logger.warning(error_msg)

        self.search_future = None
        self.search_future_track = None

        self.services.ALRecharge.stopAll()
        self.services.ALMotion.killMove()