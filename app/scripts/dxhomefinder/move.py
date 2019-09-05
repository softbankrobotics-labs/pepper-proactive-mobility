#!/usr/bin/enginev python
# -*- coding: utf-8 -*-

__version__ = "0.1.0"

__copyright__ = "Copyright 2017, Softbank Robotics"
__author__ = 'mcaniot'
__email__ = 'mcaniot@softbankrobotics.com'

import almath
import qi

CONFIGURATIONS = {
    "full_speed" : {
        "acc": 0.30,
        "vcc": 0.25
    },
    "low_speed" : {
        "acc": 0.1,
        "vcc": 0.15
    }
}

ONE_SEC_TIME = 1000000
@qi.singleThreaded()
class Move(object):
    def __init__(self, session, logger, distance_to_do, slow_down_distance):
        # services declaration
        self.services = ["ALMotion", "ALMemory"]
        self.session = session
        self.logger = logger
        # fetching mandatory services
        for service_name in self.services:
            self.session.waitForService(service_name)
            setattr(self, service_name, self.session.service(service_name))

        self.distance_to_do = float(distance_to_do)
        self.move_failed_subscriber = self.ALMemory.subscriber("ALMotion/MoveFailed")
        self.future = None
        self.promise = None
        self.robot_position_begin = None
        self.error_odom = 0.3
        self.last_distance = 0
        self.cpt = 0
        self.slow_down_distance = slow_down_distance
        self.mode = "full_speed"
        self.task = qi.PeriodicTask()
        self.task.setCallback(self.move_routine)
        self.task.setUsPeriod(int(0.01*ONE_SEC_TIME))  # check every 0.1s
        self.check_variables()

    def check_variables(self):
        is_number_1 = (isinstance(self.distance_to_do, float) or isinstance(self.distance_to_do, int))
        is_number_2 = (isinstance(self.slow_down_distance, float) or isinstance(self.slow_down_distance, int))
        if not( is_number_2 and is_number_1):
            raise Exception("The class Main need 4 arguments. session, logger and two integers or floats")
        if self.distance_to_do <= self.slow_down_distance:
            self.mode = "low_speed"

    def init_variables(self):
        self.future = None
        self.promise = None
        self.robot_position_begin = None
        self.last_distance = 0
        self.cpt = 0
        self.mode = "full_speed"

    def run(self):
        self.logger.info("run")
        self.init_variables()
        self.check_variables()
        self.promise = qi.Promise(self._cancel)
        self.reason = None
        self.signal_id = self.move_failed_subscriber.signal.connect(self._on_move_failed)
        self.robot_position_begin  = almath.Pose2D(self.ALMotion.getRobotPosition(True))
        self.task.start(True)
        return self.promise.future()

    def move_routine(self):
        robot_position = almath.Pose2D(self.ALMotion.getRobotPosition(True))
        if self.robot_position_begin:
            distance_moved = self.robot_position_begin.distance(robot_position)
            self.cpt = 0
            if distance_moved >= self.distance_to_do:
                self.mode = "end_move"
                self._stop()
            elif (distance_moved <= (self.distance_to_do - self.slow_down_distance)) and self.mode != "low_speed":
                if self.mode == "full_speed":
                    self.future = self.ALMotion.move(CONFIGURATIONS[self.mode]["vcc"],0,0, _async=True)
                    self.mode = "full_speed_started"
            else :
                if self.mode  == "full_speed_started":
                    self.mode = "low_speed"
                if self.mode == "low_speed":
                    self.future = self.ALMotion.move(CONFIGURATIONS[self.mode]["vcc"],0,0, _async=True)
                    self.mode = "low_speed_started"

            if self.last_distance == distance_moved:
                self.cpt += 1
                if self.cpt >= 100:
                    self.reason = "can't move during 1 seconds"
                    self._stop()
            else:
                self.last_distance = distance_moved

    def _is_target_reached(self):
        self.logger.info("_is_target_reached")
        robot_position = almath.Pose2D(self.ALMotion.getRobotPosition(True))
        if self.robot_position_begin:
            distance_moved = self.robot_position_begin.distance(robot_position)
            if abs(distance_moved-self.distance_to_do) < self.error_odom:
                self.logger.info("target reached")
                return True
        self.logger.info("target not reached")
        return False

    def _is_reason_available(self):
        self.logger.info("_is_reason_available")
        if self.reason:
            return self._raise_move_failed(self.reason)
        return self._raise_move_failed("Unknown reason, move failed")

    def _raise_move_failed(self, reason):
        self.logger.warning("%s, move failed" % reason)

    def _on_move_failed(self, reason):
        self.logger.info("_on_move_failed %s" % reason)
        if not self.reason:
            self.logger.info("@@@ reason: %s" % reason)
            self.reason = reason
        self._stop()

    def _cancel(self, _):
        self.logger.info("_cancel")
        self._stop()
        self.logger.info("end _cancel")


    def _stop(self):
        try:
            if self.future:
                self.future.cancel()
        except Exception as error_msg:
            self.logger.warning(error_msg)
        self.ALMotion.move(0, 0, 0)
        self.ALMotion.stopMove(_async = True)
        self.task.stop()
        self.move_failed_subscriber.signal.disconnect(self.signal_id)
        if self._is_target_reached():
            self.promise.setValue(True)
        else:
            self._is_reason_available()
            self.promise.setValue(False)