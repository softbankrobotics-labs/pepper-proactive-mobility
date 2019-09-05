"""
Custom Slam for saving position and localization.
We create this Class for localize in a home but
it can be anywhere.
"""
# pylint: disable=E1101

__version__ = "0.1.0"

__copyright__ = "Copyright 2017, Softbank Robotics"
__author__ = 'mcaniot'
__email__ = 'mcaniot@softbankrobotics.com'

# Import qi
import qi
import almath

# Basic libs
import json
import math
import os
import time

# stk libs
import stk.runner
import stk.events
import stk.services
import stk.logging
import stk.coroutines

# Internal libs
import planeutils as pu

# Package ID
PACKAGE_ID = "proactive-mobility"
if stk.runner.is_on_robot():
    PATH_POSITION_FILE = ("/home/nao/.local/share/PackageManager/apps/" +
                         PACKAGE_ID + "/scripts/dxhomefinder/data/positions.txt")
else:
    PATH_POSITION_FILE = "data/positions.txt"

class PositionManager(object):
    """
    Class for saving position
    """
    APP_ID = "com.softbankrobotics.PositionManager"
    def __init__(self, qiapp):
        self.qiapp = qiapp
        self.session = qiapp.session
        self.events = stk.events.EventHelper(self.session)
        self.services = stk.services.ServiceCache(self.session)
        self.logger = stk.logging.get_logger(self.session, self.APP_ID)

        # Wait some services
        self.session.waitForService("ALMotion")
        self.session.waitForService("ALMemory")
        self.session.waitForService("ALPreferenceManager")

        # Position of the robot initial
        xw0, yw0, thetaw0 = self.services.ALMotion.getRobotPosition(True)
        self.home_frame = pu.Pose(x_coord=xw0, y_coord=yw0, theta=thetaw0)
        self.robot_in_home = self.home_frame.get_relative(self.home_frame)
        # Load saved position
        if os.path.exists(PATH_POSITION_FILE):
            self.__load_position_in_file()
        else:
            folder = os.path.dirname(PATH_POSITION_FILE)
            if not os.path.exists(folder):
                os.makedirs(folder)
            self.__save_position_in_file()

        # Save the time for checking if we need to relocalize or not
        # We need to relocalize each X minutes for safety and being
        # sur to know where we are.
        self.last_time_reloc = time.time()

        # periodic task
        self.relative_cumulated_displacement = almath.Pose2D(
                self.services.ALMotion._getCumulatedDisplacement())

    ###########################################
    #            Public Method                #
    ###########################################

    def cancel(self):
        "Cancel the current behavior and stop motion"
        self.services.ALMotion.killMove()
        self.logger.info("cancel")

    def get_odometric_position_error(self):
        cumulated_displacement = almath.Pose2D(
                self.services.ALMotion._getCumulatedDisplacement())
        last_cumulated_displasment = cumulated_displacement - self.relative_cumulated_displacement
        error_xy = last_cumulated_displasment.norm() * 0.03
        error_theta = last_cumulated_displasment.theta * 0.01
        return [ error_xy, error_theta ]

    def get_angle(self, position):
        "Get angle between the robot and a position in 2D Map"
        # Calculate position of the robot
        self.__recalculate_position()

        if isinstance(position, list) and len(position) == 2:
            # get object coordinate
            object_x = float(position[0])
            object_y = float(position[1])
            # get position of object in the home frame
            object_pos = pu.coords_to_pos(object_x, object_y)
            # get position of object in the robot frame
            object_pos_in_r = self.robot_in_home.get_relative_pos(
                                                                object_pos)
            # get angle between object and robot in rads
            angle_to_obj_in_rad = pu.get_pos_theta(object_pos_in_r)
            return math.degrees(angle_to_obj_in_rad)
        else:
            return -1

    def get_angle_from_robot(self, x_coord, y_coord):
        "Angle between position and the robot"
        # Calculate position of the robot
        self.__recalculate_position()
        object_pos = pu.coords_to_pos(x_coord, y_coord)
        object_pos_in_r = self.robot_in_home.get_relative_pos(object_pos)
        return float(pu.get_pos_theta(object_pos_in_r))

    def get_dist_from_robot(self, x_coord, y_coord):
        "Distance between position and the robot"
        # Calculate position of the robot
        self.__recalculate_position()
        object_pos = pu.coords_to_pos(x_coord, y_coord)
        return float(
                    pu.get_pos_to_pos_dist(object_pos,
                                           self.robot_in_home.pos))

    def get_orientation(self):
        "Get orientation of the robot"
        # Calculate position of the robot
        self.__recalculate_position()
        return pu.get_pos_theta(self.robot_in_home.orientation)

    def get_home_pos(self):
        "Get the position of the home"
        return self.home_frame.get_coord()

    def get_robot_pos(self):
        "Get the position of the robot in frame home"
        self.__recalculate_position()
        x_robot, y_robot, theta_robot = self.robot_in_home.get_coord()
        theta_robot = self.get_orientation()
        return x_robot, y_robot, theta_robot

    def init_position(self):
        "Init to position [0, 0]"
        # Initialize last time reloc
        self.last_time_reloc = time.time()

        # Get the robot position in world frame and initialize home frame
        # At this position
        xw0, yw0, thetaw0 = self.services.ALMotion.getRobotPosition(True)
        self.home_frame = pu.Pose(x_coord=xw0, y_coord=yw0, theta=thetaw0)
        self.robot_in_home = pu.Pose(x_coord=0, y_coord=0, theta=0)
        self.relative_cumulated_displacement = almath.Pose2D(
                self.services.ALMotion._getCumulatedDisplacement())
        # Save position in file
        self.__save_position_in_file()

    def init_position_with_coord(self, coord):
        "Init Position of the robot at the coordinate give in argument"
        # Initialize last time reloc
        self.last_time_reloc = time.time()

        # Get the robot position in world frame and initialize home frame
        # At this position
        xw0, yw0, thetaw0 = coord
        self.home_frame = pu.Pose(x_coord=xw0, y_coord=yw0, theta=thetaw0)
        self.robot_in_home = self.home_frame.get_relative(self.home_frame)
        self.relative_cumulated_displacement = almath.Pose2D(
                self.services.ALMotion._getCumulatedDisplacement())
        # Save position in file
        self.__save_position_in_file()

    @qi.nobind
    def on_start(self):
        "On start of the service"
        self.events.connect_decorators(self)
        self.logger.info(" on start")

    @qi.nobind
    def on_stop(self):
        "Cleanup"
        self.logger.info("Application finished.")
        self.__recalculate_position()
        self.events.clear()

    @stk.coroutines.public_async_generator
    def reset_angle(self):
        "Reset angle of the base in 0"
        # Calculate actual position
        self.__recalculate_position()
        # Get orientation of the robot
        robot_orientation_in_rad = self.get_orientation()
        # conversion in degrees, just for a better understanding
        robot_orientation_in_deg = math.degrees(robot_orientation_in_rad)
        # if the angle is between -10 and 10 degrees don't need to continue
        if robot_orientation_in_deg < 10 and robot_orientation_in_deg > -10:
            yield stk.coroutines.Return(True)
        else:
            # Force the robot to look in front
            self.services.ALMotion.angleInterpolation(["HeadYaw", "HeadPitch"],
                                                            [0, -0.26],
                                                            2,
                                                            1,
                                                            _async=True)

            # For returning at the orientation you just need to rotate to
            # -robot_orientation_in_rad
            value = yield self.services.ALMotion.moveTo(
                                0.0,
                                0.0,
                                -float(robot_orientation_in_rad),
                                _async=True)

            # For initialize the tracking, with that the robot look in front
            # and forget the last face tracked
            self.services.ALPeoplePerception.resetPopulation()
            yield stk.coroutines.Return(value)

    @stk.coroutines.async_generator
    @stk.logging.log_exceptions
    def reset_body(self):
        "Rotate the body for being in front of the human"
        # Get angle of HeadYaw
        [angle_head_in_rad] = yield (
                self.services.ALMotion.getAngles("HeadYaw", 1, _async=True))
        # conversion in degrees, just for a better understanding
        angle_head_in_deg = math.degrees(angle_head_in_rad)
        #if angle head is above +10 degrees or inferior -10 degrees
        if angle_head_in_deg > 10 or angle_head_in_deg < -10:
            # Turn the body and look in front, do it in parallel
            yield [self.services.ALMotion.moveTo(0, 0, angle_head_in_rad,
                   _async=True),
                   self.services.ALMotion.angleInterpolation(["HeadYaw", "HeadPitch"],
                                                             [0, -0.26],
                                                             2,
                                                             1,
                                                             _async=True)]

    @stk.coroutines.public_async_generator
    def turn_toward_position(self, position):
        # Turn toward this position
        x_coords = position[0]
        y_coords = position[1]


        # Get angle between the target and robot in rads
        angle_to_obj_in_rad = self.get_angle_from_robot(x_coords, y_coords)
        # Get angle in degrees for a better understanding
        angle_to_obj_in_deg = math.degrees(angle_to_obj_in_rad)

        # Variable for checking if the rotation is a sucess
        rotation_success = True
        # Turn towards the target position
        self.services.ALMotion.angleInterpolation(["HeadYaw", "HeadPitch"],
                                                            [0, -0.26],
                                                            2,
                                                            1,
                                                            _async=True)
        future_rotation = self.services.ALMotion.moveTo(0.0,
                                0.0,
                                angle_to_obj_in_rad, _async=True)
        yield future_rotation
        rotation_success = future_rotation.value()
        angle_condition = (angle_to_obj_in_deg > -1 and
                                                    angle_to_obj_in_deg < 1)
        # If the robot need to do a low angle, put rotation_success to True
        if angle_condition:
            rotation_success = True
        yield stk.coroutines.Return(rotation_success)

    @stk.coroutines.public_async_generator
    @stk.logging.log_exceptions
    def navigate_in_map(self, position):
        "Go to the position in a map"
        print "navigate in map begin."
        # Position to go
        x_coords = position[0]
        y_coords = position[1]

        self.logger.info("enter in navigate_in_map")

        # Turn toward the position
        print "start rotation"
        success_rotation = self.turn_toward_position(position)
        yield success_rotation
        print "end rotation"

        # Distance between the robot and the target position
        dist = self.get_dist_from_robot(x_coords, y_coords)

        # The distance between the robot and the object is superior at 10cm
        if dist >= 0.1 and success_rotation.value():
            # Move to the target position
            translation_success = yield self.services.ALNavigation.navigateTo(
                                                    dist,
                                                    0.0,
                                                    0.0,
                                                    _async=True)
            yield stk.coroutines.Return(translation_success)
        else:
            self.logger.info("end navigate_in_map : value True")
            yield stk.coroutines.Return(True)
        self.logger.info("end navigate_in_map : value False")
        yield stk.coroutines.Return(False)
        print "navigate in map end"

    @stk.coroutines.public_async_generator
    @stk.logging.log_exceptions
    def move_in_map(self, position):
        "Go to the position in a map"
        print "navigate in map begin."
        # Position to go
        x_coords = position[0]
        y_coords = position[1]

        self.logger.info("enter in navigate_in_map")

        # Turn toward the position
        print "start rotation"
        success_rotation = self.turn_toward_position(position)
        yield success_rotation
        print "end rotation"

        # Distance between the robot and the target position
        dist = self.get_dist_from_robot(x_coords, y_coords)

        # The distance between the robot and the object is superior at 10cm
        if dist >= 0.1 and success_rotation.value():
            # Move to the target position
            translation_success = yield self.services.ALMotion.moveTo(
                                                    dist,
                                                    0.0,
                                                    0.0,
                                                    _async=True)
            yield stk.coroutines.Return(translation_success)
        else:
            self.logger.info("end navigate_in_map : value True")
            yield stk.coroutines.Return(True)
        self.logger.info("end navigate_in_map : value False")
        yield stk.coroutines.Return(False)
        print "navigate in map end"

    @qi.nobind
    def stop(self):
        "Standard way of stopping the application."
        self.__recalculate_position()
        self.qiapp.stop()

    ###########################################
    #            Private Method               #
    ###########################################

    @qi.nobind
    def __load_position_in_file(self):
        "Load the position from the file on the robot"
        json_data = None
        with open(PATH_POSITION_FILE, 'r') as file:
            json_data_str = file.read()
            if json_data_str:
                json_data = json.loads(json_data_str)

        if json_data and (time.time() - json_data["timestamp"]) < 60*10:
            self.home_frame = pu.Pose(pos=complex(
                                    json_data["home_frame"]['pos']),
                                    orientation=complex(
                                    json_data["home_frame"]['orientation']))
            self.robot_in_home = pu.Pose(pos=complex(
                                  json_data["robot_in_home"]['pos']),
                                  orientation=complex(
                                  json_data["robot_in_home"]['orientation']))
            self.__save_position_in_file()
        else:
            self.init_position()

    @qi.nobind
    def __need_reloc(self):
        "Check if the robot need reloc"
        if time.time() - self.last_time_reloc > 60*10:
            return True
        else:
            return False

    @qi.nobind
    def __recalculate_position(self):
        "Recalculate actual position of the robot"
        x_coord, y_coord, theta = (
                                self.services.ALMotion.getRobotPosition(True))
        # Robot position in the world frame
        robot_in_world = pu.Pose(x_coord=x_coord,
                                 y_coord=y_coord,
                                 theta=theta)
        # Robot in the home frame
        self.robot_in_home = self.home_frame.get_relative(robot_in_world)
        # Save in a file on the robot
        self.__save_position_in_file()

    @qi.nobind
    def __save_position_in_file(self):
        "Save the position in the file on the robot"
        with open(PATH_POSITION_FILE, 'w') as file:
            json_data = {}
            json_data["timestamp"] = time.time()
            json_data["home_frame"] = {'pos' : repr(self.home_frame.pos),
                            'orientation': repr(self.home_frame.orientation)}
            json_data["robot_in_home"] = {
                    'pos' : repr(self.robot_in_home.pos),
                    'orientation': repr(self.robot_in_home.orientation)}
            file.write(json.dumps(json_data))

####################
# Setup and Run
####################

if __name__ == "__main__":
    stk.runner.run_service(PositionManager)

