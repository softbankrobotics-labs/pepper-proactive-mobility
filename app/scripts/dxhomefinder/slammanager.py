"""
SLAM manager
"""
# pylint: disable=E1101

__version__ = "0.1.0"

__copyright__ = "Copyright 2017, Softbank Robotics"
__author__ = 'mcaniot'
__email__ = 'mcaniot@softbankrobotics.com'

# Import qi
import qi

# stk libs
import stk.runner
import stk.events
import stk.services
import stk.logging
import stk.coroutines

import almath
import math

# Internal libs
import planeutils as pu

import move

RADIUS = 3.0

MOVE_CONFIG_LOW_SPEED = [["MaxVelXY", 0.1],["MaxAccXY", 0.1],["MaxVelTheta",0.5],["MaxAccTheta",0.325],["MaxJerkXY",0.5],["MaxJerkTheta",1.0]]

class SLAMManager(object):
    """
    Class for saving position
    """
    APP_ID = "com.softbankrobotics.SLAMManager"
    def __init__(self, session):
        self.session = session
        self.events = stk.events.EventHelper(self.session)
        self.services = stk.services.ServiceCache(self.session)
        self.logger = stk.logging.get_logger(self.session, self.APP_ID)
        self.navigation_future = None
        self.slam_pos_in_world = None # We don't know it at start

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
            if self.navigation_future:
                self.navigation_future.cancel()
        except Exception as error_msg:
            self.logger.warning(error_msg)
        self.navigation_future = None
        self.services.ALNavigation.stopNavigateTo()
        self.services.ALMotion.killMove()

    
    @stk.coroutines.async_generator
    def init_home(self):
        future_value = yield self.create_temp_map()
        if future_value:
            self.slam_pos_in_world = [0, 0]
        yield stk.coroutines.Return(future_value)

    def is_init(self):
        return self.slam_pos_in_world != None

    def uninit(self):
        self.slam_pos_in_world = None
        self.services.ALNavigation._stopTopoMapper()
        self.services.ALNavigation.stopLocalization()

    @stk.coroutines.async_generator
    def find_home(self):
        if self.slam_pos_in_world:
            robotPose = almath.Pose2D(
                     self.services.ALNavigation.getRobotPositionInMap()[0])
            yield stk.coroutines.Return([robotPose.x, robotPose.y, robotPose.theta])
        yield stk.coroutines.Return(None)

    def get_polar_coord(self, coord_home):
        x , y , theta = coord_home
        object_pos = pu.coords_to_pos(0, 0)
        robot_pose = pu.Pose(x_coord=x, y_coord=y, theta=theta)
        object_pos_in_r = robot_pose.get_relative_pos(object_pos)

        r_theta =  float(pu.get_pos_theta(object_pos_in_r))
        r = float(math.sqrt(math.pow(x,2) + math.pow(y,2)))
        return r, r_theta

    @stk.coroutines.async_generator
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
                                        MOVE_CONFIG_LOW_SPEED,
                                         _async=True)
            if value:
                self.slam_pos_in_world = [0.0,0.0]
                yield stk.coroutines.Return(True)
        yield stk.coroutines.Return(False)

    @stk.coroutines.async_generator
    def return_home(self, *args, **kwargs):
        on_home = False
        self.navigation_future = self.find_home()
        coord_home = yield self.navigation_future
        if coord_home:
            r, r_theta = self.get_polar_coord(coord_home)
            if r > 0.3:
                self.services.ALMotion.angleInterpolation(["HeadYaw", "HeadPitch"],
                                                            [0, -0.26],
                                                            2,
                                                            1,
                                                            _async=True)
                self.navigation_future = self.services.ALMotion.moveTo(
                                                    0,
                                                    0,
                                                    almath.modulo2PI(
                                                            r_theta),
                                                    _async=True)
                yield self.navigation_future
                if self.navigation_future.value():
                    move_object = move.Move(self.session, self.logger, r, 1.5)
                    self.navigation_future = move_object.run()
                    yield self.navigation_future
                    if self.navigation_future.value():
                        on_home = True
                        robot_pose = almath.Pose2D(coord_home)
                        center = almath.Pose2D(0.0, 0.0, 0.0)
                        poseDiff = robot_pose.diff(center)
                        self.aruco_future = self.services.ALMotion.moveTo(
                                                    0,
                                                    0,
                                                    almath.modulo2PI(
                                                    poseDiff.theta - r_theta),
                                                    _async=True)
                        yield self.aruco_future
            else:
                on_home = True
        else:
            # can't find slam position
            if self.services.DXHomeFinder:
                self.services.DXHomeFinder.reason_cant_go_home.setValue("no_target")
        if on_home:
            self.navigation_future = self.find_home()
            coord_home = yield self.navigation_future
            if coord_home:
                robot_pose = almath.Pose2D(coord_home)
                center = almath.Pose2D(0.0, 0.0, 0.0)
                poseDiff = robot_pose.diff(center)
                self.navigation_future = self.last_move(
                        poseDiff.x,
                        poseDiff.y,
                        poseDiff.theta)
                yield self.navigation_future
                if not self.navigation_future.value():
                    if self.services.DXHomeFinder:
                        self.services.DXHomeFinder.reason_cant_go_home.setValue("obstacle")
                yield stk.coroutines.Return(self.navigation_future.value())
            else:
                # can't find slam position
                if self.services.DXHomeFinder:
                    self.services.DXHomeFinder.reason_cant_go_home.setValue("no_target")
        if self.services.DXHomeFinder:
            if self.services.DXHomeFinder.reason_cant_go_home.value() == "unknown":
                self.services.DXHomeFinder.reason_cant_go_home.setValue("obstacle")
        yield stk.coroutines.Return(False)
                

    @stk.coroutines.async_generator
    def return_home_with_alnavigation(self):
        cpt = 0
        if self.slam_pos_in_world:
            for i in range(0, 100):
                robotPose = almath.Pose2D(
                    self.services.ALNavigation.getRobotPositionInMap()[0])
                center = almath.Pose2D(0.0, 0.0, 0.0)
                if robotPose.distance(center) > 0.3:
                    self.navigation_future = self.services.ALNavigation.navigateToInMap(
                                                                    [0.0, 0.0],
                                                                    _async=True)
                    yield self.navigation_future
                if robotPose.distance(center) < 0.3:
                    poseDiff = robotPose.diff(center)
                    if math.fabs(poseDiff.theta) > 0.2:
                        self.navigation_future = self.services.ALMotion.moveTo(
                                                        0,
                                                        0,
                                                        almath.modulo2PI(
                                                            poseDiff.theta),
                                                        _async=True)
                        yield self.navigation_future
                    else:
                        cpt += 1
                        if cpt > 3:
                            self.slam_pos_in_world = [0, 0]
                            yield stk.coroutines.Return(True)
                            return
        self.logger.info("End back home.")
        yield stk.coroutines.Return(False)

    @stk.coroutines.async_generator
    def create_temp_map(self):
        self.logger.info("Start mapping")
        # Start mapping
        yield self.services.ALNavigation._stopTopoMapper(_async=True)
        yield self.services.ALNavigation.stopLocalization(_async=True)
        value = False
        value = yield self.services.ALMotion.moveTo(0, 0, math.radians(360), _async=True)
        if value:
            yield self.services.ALNavigation._resetTopoMap(_async=True)
            yield self.services.ALNavigation.startLocalization(_async=True)
            yield self.services.ALNavigation._startTopoMapper(_async=True)
        self.logger.info("End mapping")
        yield stk.coroutines.Return(value)
