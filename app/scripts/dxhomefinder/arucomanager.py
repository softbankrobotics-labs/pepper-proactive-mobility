"""
ARuco manager
"""
# pylint: disable=E1101

__version__ = "0.1.0"

__copyright__ = "Copyright 2017, Softbank Robotics"
__author__ = 'mcaniot'
__email__ = 'mcaniot@softbankrobotics.com'

import qi

import json

# stk libs
import stk.runner
import stk.events
import stk.services
import stk.logging
import stk.coroutines

import math
import numpy
import almath

import move

DEFAULT_IDS = [128, 448]

DEFAULT_PARAMS_BIG = {
    "size": 0.20,
    "position": "floor",
    "ids": [128],
}

DEFAULT_PARAMS_SMALL = {
    "size": 0.10,
    "position": "floor",
    "ids": [448],
}


DEFAULT_PARAMS = {
    DEFAULT_IDS[0] : [DEFAULT_PARAMS_BIG, DEFAULT_PARAMS_SMALL],
    DEFAULT_IDS[1] : [DEFAULT_PARAMS_SMALL, DEFAULT_PARAMS_BIG]
}

DEFAULT_PARAMS_LIST = [DEFAULT_PARAMS_BIG, DEFAULT_PARAMS_SMALL]

MOVE_CONFIG_LOW_SPEED = [["MaxVelXY", 0.15],["MaxAccXY", 0.15],["MaxVelTheta",0.5],["MaxAccTheta",0.325],["MaxJerkXY",0.5],["MaxJerkTheta",1.0]]

MOVE_CONFIG_HIGH_SPEED = [["MaxVelXY", 0.25],["MaxAccXY", 0.3]]

DISTANCE_FIRST_APPROACH = 1.3

DISTANCE_FROM_SMALL_ARUCO = 0.61

class ARucoManager(object):
    """
    Class for saving position
    """
    APP_ID = "com.softbankrobotics.ARucoManager"
    def __init__(self, session):
        self.session = session
        self.events = stk.events.EventHelper(self.session)
        self.services = stk.services.ServiceCache(self.session)
        self.logger = stk.logging.get_logger(self.session, self.APP_ID)

        # Member variables
        self.aruco_pos_in_world = None # We don't know it at start
        self.aruco_future = None
        self.search_future = None
        self.subscriber_name = []
        self.event_name = []
        self.search_future_track = None
        self.future_sleep = None
        self.aruco_position_from_robot = {}

    ###########################################
    #            Public Method                #
    ###########################################

    @stk.coroutines.public_async_generator
    def init_home(self):
        "Initialize: go in front of ar code."
        value = True
        if not self.is_init():
            future_return = yield self.return_home()
            value = future_return
            self.logger.info(value)
        if value:
            self.services.PositionManager.init_position()
            self.aruco_pos_in_world = [0.0, 0.0]
            self.aruco_position_from_robot = {}
        yield stk.coroutines.Return(value)

    def is_init(self):
        return self.aruco_pos_in_world != None

    def uninit(self):
        self.aruco_pos_in_world = None
        self.aruco_position_from_robot = {}

    def cancel(self):
        "Cancel return home"
        try:
            if self.aruco_future:
                self.aruco_future.cancel()
        except Exception as error_msg:
            self.logger.warning(error_msg)
        self.aruco_future = None
        self.services.ALMotion.killMove()
        self.cancel_search_home()

    def need_approach_home(self):
        if self.services.PositionManager:
            if self.aruco_pos_in_world:
                dist_from_home = self.services.PositionManager.get_dist_from_robot(0,0)
                if dist_from_home > DISTANCE_FIRST_APPROACH + 0.2:
                    return True
        return False

    @stk.coroutines.async_generator
    def approach_home(self):
        x_coord, y_coord, theta = self.services.PositionManager.get_home_pos()
        coord_aruco = [x_coord, y_coord, theta]
        p2D_world2robot = almath.Pose2D(self.services.ALMotion.getRobotPosition(True))

        p2D_world2target = almath.Pose2D(coord_aruco)

        t_world2robot = almath.transformFromPose2D(p2D_world2robot)
        t_world2target = almath.transformFromPose2D(p2D_world2target)
        t_robot2target = t_world2robot.inverse() * t_world2target
        p6D_robot2target = almath.position6DFromTransform(t_robot2target)
        coord_aruco = list(p6D_robot2target.toVector())

        coord_home = [coord_aruco[0] + DISTANCE_FIRST_APPROACH * math.cos(coord_aruco[-1]),
                      coord_aruco[1] + DISTANCE_FIRST_APPROACH * math.sin(coord_aruco[-1]),
                      coord_aruco[-1] + math.pi]

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
        distance_big_aruco = None
        distance_small_aruco = None
        id_aruco = 128
        mode_search = False
        if not self.services.PositionManager:
            self.logger.info("no PositionManager")
            yield stk.coroutines.Return(None)

        if self.aruco_pos_in_world and not research_360_activated:
            distance_big_aruco = self.services.PositionManager.get_dist_from_robot(0, 0)
            distance_small_aruco = self.services.PositionManager.get_dist_from_robot(0.56, 0)
            if distance_big_aruco > 0.60:
                aruco_future = self.services.PositionManager.turn_toward_position(\
                                                    [0, 0],
                                                    _async=True)
                yield aruco_future
                aruco_future = self.services.ALTracker._lookAtWithEffector(
                        [distance_big_aruco, 0, 0],
                        2,
                        2,
                        0.1,
                        0,
                        _async=True)
                yield aruco_future
                id_aruco = 128
            elif distance_small_aruco > 0.40:
                aruco_future = self.services.PositionManager.turn_toward_position(\
                                                    [1.0, 0],
                                                    _async=True)
                yield aruco_future
                aruco_future = self.services.ALTracker._lookAtWithEffector(
                        [distance_small_aruco, 0, 0],
                        2,
                        2,
                        0.1,
                        0,
                        _async=True)
                yield aruco_future
                id_aruco = 448
            else:
                yield stk.coroutines.Return(None)
        else:
            self.logger.info("Lost mode")
            mode_search = True
        try:
            aruco_future = None
            if mode_search:
                aruco_future = yield self.search_home()
            else :
                print "DBG done moving, now scanning"
                aruco_future = yield self.services.DXAruco.get_home_position_in_world(
                        DEFAULT_PARAMS[id_aruco],
                        _async=True)
            if aruco_future:
                self.logger.info(aruco_future)
                if id_aruco in aruco_future:
                    self.logger.info("id_aruco")
                    self.logger.info(id_aruco)
                else:
                    self.logger.info("id_aruco")
                    [id_aruco] = [tmp for tmp in [128,448] if tmp != id_aruco]
                    self.logger.info(id_aruco)

                if id_aruco in aruco_future:
                    self.logger.info(aruco_future)
                    coord_aruco = aruco_future[id_aruco]
                    coord_home_pos = None
                    if len(coord_aruco) == 6:
                        coord_aruco = [coord_aruco[0],coord_aruco[1], coord_aruco[-1]]
                    coord_home_pos = coord_aruco
                    p2D_world2robot = almath.Pose2D(self.services.ALMotion.getRobotPosition(True))
                    p2D_world2target = almath.Pose2D(coord_aruco)
                    t_world2robot = almath.transformFromPose2D(p2D_world2robot)
                    t_world2target = almath.transformFromPose2D(p2D_world2target)
                    t_robot2target = t_world2robot.inverse() * t_world2target
                    p6D_robot2target = almath.position6DFromTransform(t_robot2target)
                    coord_aruco = list(p6D_robot2target.toVector())
                    x, y, theta = [coord_aruco[0], coord_aruco[1], coord_aruco[-1]]
    
                    coord_home = None
                    if id_aruco == 128:
                        theta += math.radians(-135)
                        coord_home = [
                                x,
                                y,
                                theta
                                ]
                        coord_home_pos = [coord_home_pos[0], coord_home_pos[1], coord_home_pos[-1] + math.radians(-135)]
                    else:
                        theta += math.radians(135)
                        coord_home = [
                                x - DISTANCE_FROM_SMALL_ARUCO * math.cos(theta),
                                y - DISTANCE_FROM_SMALL_ARUCO * math.sin(theta),
                                theta
                                ]
                        coord_home_pos = [coord_home_pos[0] - DISTANCE_FROM_SMALL_ARUCO * math.cos(coord_home_pos[-1] + math.radians(135)),
                                          coord_home_pos[1] - DISTANCE_FROM_SMALL_ARUCO * math.sin(coord_home_pos[-1] + math.radians(135)),
                                          coord_home_pos[-1] + math.radians(135)]
                    if coord_home_pos != None:
                        self.services.PositionManager.init_position_with_coord(coord_home_pos)
                        self.aruco_pos_in_world = [0.0, 0.0]
                    self.logger.info("coord " + repr(coord_home))
                    yield stk.coroutines.Return([coord_home,
                                                    id_aruco])
                    return
        except Exception as error_msg:
            self.logger.warning("No aruco with the id " + repr(id_aruco) + " found.")
            # Now continue at the next head angle
        yield stk.coroutines.Return(None)

    def get_polar_coord(self, coord_home):
        x , y , theta = coord_home
        r_theta = float(numpy.arctan(y/x))
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
                self.aruco_pos_in_world = [0.0,0.0]
                self.aruco_position_from_robot = {}
                yield stk.coroutines.Return(True)
        yield stk.coroutines.Return(False)

    @stk.coroutines.async_generator
    def return_home(self, research_360_activated=False):
        on_home = False
        id_aruco = None
        if self.need_approach_home() and not research_360_activated:
            self.aruco_future = self.approach_home()
            yield self.aruco_future
            if not self.aruco_future.value():
                if self.services.DXHomeFinder:
                    self.services.DXHomeFinder.reason_cant_go_home.setValue("obstacle")
                yield stk.coroutines.Return(False)
        self.aruco_future = self.find_home(research_360_activated)
        coord_home = yield self.aruco_future
        if coord_home:
            id_aruco = coord_home[1]
            coord_home = coord_home[0]
            r, r_theta = self.get_polar_coord(coord_home)
            if id_aruco == 128 or (id_aruco == 448 and r > 0.5):
                id_aruco = 128
                self.aruco_future = self.services.ALMotion.moveTo(
                                                    0,
                                                    0,
                                                    almath.modulo2PI(
                                                            r_theta),
                                                    MOVE_CONFIG_LOW_SPEED,
                                                    _async=True)
                yield self.aruco_future
                if self.aruco_future.value():
                    self.aruco_future = self.services.ALMotion.moveTo(
                                                    r,
                                                    0,
                                                    0,
                                                    MOVE_CONFIG_LOW_SPEED,
                                                    _async=True)
                    yield self.aruco_future
                    if self.aruco_future.value():
                        on_home = True
                        self.aruco_future = self.services.ALMotion.moveTo(
                                                    0,
                                                    0,
                                                    almath.modulo2PI(
                                                    coord_home[-1] - r_theta),
                                                    MOVE_CONFIG_LOW_SPEED,
                                                    _async=True)
                        yield self.aruco_future
            else:
                on_home = True
        else:
            # can't find aruco
            if self.services.DXHomeFinder:
                self.services.DXHomeFinder.reason_cant_go_home.setValue("no_target")
        if on_home:
            if id_aruco == 128:
                self.aruco_future = self.find_home()
                coord_home = yield self.aruco_future
                if coord_home:
                    id_aruco = coord_home[1]
                    coord_home = coord_home[0]
            if coord_home and id_aruco == 448:
                x, y, theta = coord_home
                self.aruco_future = self.last_move(
                        x,
                        y,
                        theta)
                yield self.aruco_future
                if not self.aruco_future.value():
                    if self.services.DXHomeFinder:
                        self.services.DXHomeFinder.reason_cant_go_home.setValue("obstacle")
                yield stk.coroutines.Return(self.aruco_future.value())
            else:
                # can't find aruco
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
        self.logger.info("search home")
        # launch aruco search
        self.subscribe_aruco_list(DEFAULT_PARAMS_LIST)
        try:
            self.search_future = self.search_routine()
            yield self.search_future
        except Exception as error_msg:
            self.logger.warning(error_msg)
        self.cancel_search_home()
        self.logger.info("end search home")
        yield stk.coroutines.Return(self.aruco_position_from_robot)

    def aruco_detected(self, value):
        json_value = json.loads(value)
        self.aruco_position_from_robot[json_value["id_aruco"]] = json_value["world2target"]
        self.cancel_all_future_search_home()

    def subscribe_aruco_list(self, aruco_list):
        if not self.services.DXAruco:
            return
        # Just to be sure
        self.unsubscribe_aruco_list()

        for param_aruco in aruco_list:
            sub_name = self.services.DXAruco.subscribe(param_aruco)
            self.subscriber_name.append(sub_name)
            for id_aruco in param_aruco["ids"]:
                ev_name = "DXAruco/%s/%s"%(sub_name, repr(id_aruco))
                self.event_name.append(ev_name)
                self.services.ALMemory.raiseEvent(ev_name, 0)
                self.events.connect(ev_name, self.aruco_detected)

    def unsubscribe_aruco_list(self):
        if self.event_name and isinstance(self.event_name, list):
            for tmp_name in list(self.event_name):
                try:
                    self.events.disconnect(tmp_name)
                except Exception as error_msg:
                    self.logger.warning("Cannot disconnect " + repr(tmp_name))
                try:
                    self.event_name.remove(tmp_name)
                except Exception as e:
                    self.logger.warning("cannot remove " + repr(tmp_name))
        if not self.services.DXAruco:
            return
        if self.subscriber_name and isinstance(self.subscriber_name, list):
            for tmp_name in list(self.subscriber_name):
                try:
                    self.services.DXAruco.unsubscribe(tmp_name)
                except Exception as error_msg:
                    self.logger.warning("Cannot unsubscribe " + repr(tmp_name))
                try:
                    self.subscriber_name.remove(tmp_name)
                except Exception as e:
                    self.logger.warning("cannot remove " + repr(tmp_name))


    def cancel_all_future_search_home(self):
        try:
            if self.search_future:
                self.search_future.cancel()
        except Exception as error_msg:
            self.logger.warning(error_msg)
        try:
            if self.future_sleep:
                self.future_sleep.cancel()
        except Exception as error_msg:
            self.logger.warning(error_msg)
        try:
            if self.search_future_track:
                self.search_future_track.cancel()
        except Exception as error_msg:
            self.logger.warning(error_msg)

    def cancel_search_home(self):
        self.cancel_all_future_search_home()
        self.unsubscribe_aruco_list()
        self.services.ALMotion.killMove()
