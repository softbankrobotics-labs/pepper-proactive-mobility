"""
A service that runs in the background of life, and allows Pepper to go home.

Features
 - This service, which checks robot state and decides what to do
 - A webpage for configuration
 - Maybe, some dialogue triggers etc.

Logic:
 - at all time, basic awareness should be in "navigate" mode
 - unless you're too far
 - if alone, checks distance, then goes back
 - if robot seems lost, stop moving


Debug tip: when running this from your computer, you probably want to do this
on the robot first:

qicli call ALServiceManager.stopService dxProactiveMobility

"""

__version__ = "0.1.9"

__copyright__ = "Copyright 2017, Softbank Robotics"
__author__ = 'ekroeger and mcaniot'
__email__ = 'ekroeger@softbankrobotics.com and mcaniot@softbankrobotics.com'

import math
import time
import threading

import almath
import qi

import stk.runner
import stk.events
import stk.services
import stk.logging

import preferences
import translate
import tablet

TRACKING_MODE_NAVIGATE = "Navigate"

TIMEOUT_BEFORE_RETRY_GO_HOME = 60.0 # in seconds

NAV_EPSILON_POS = 0.1 # 10 cm
NAV_EPSILON_ANGLE_RAD = math.radians(10) # 10 degrees in radians

class StrollState(object):
    "Enum-like state inventory"
    UNDEFINED = "UNDEFINED"
    DEFINING_HOME = "DEFINING_HOME"
    AT_HOME = "AT_HOME"
    GO_ENGAGE = "GO_ENGAGE"
    WANDERING = "WANDERING"
    GOING_HOME = "GOING_HOME"
    SAFEGUARD_GOING_HOME = "SAFEGUARD_GOING_HOME"

# Behaviors
DEFINE_HOME_BEHAVIOR = "proactive-mobility/sethome"
GO_HOME_BEHAVIOR = "proactive-mobility/gohome"
GO_ENGAGE_BEHAVIOR = "proactive-mobility/goengage"

MAX_DISTANCE = 3.0
MAX_ERROR = 0.5

ENGAGEMENT_DISTANCE = 0.6 # in meters

# We need millisecond times for periodic tasks
ONE_MILLISECOND_IN_US = 1000
ONE_SECOND_IN_US = 1000 * ONE_MILLISECOND_IN_US
HUNDRED_SECOND_IN_US = 100 * ONE_MILLISECOND_IN_US

SHOULD_GO_HOME_KEY = "ProactiveMobility/ShouldGoHome"
ALLOW_ENGAGE_KEY = "ProactiveMobility/AllowGoEngage"
SHOULD_DEFINE_HOME_KEY = "ProactiveMobility/ShouldDefineHome"

TIME_RETRY_DEFINE_HOME = 10

NAVIGATE_RELATIVE_POSITION = [-0.5, 0.0, 0.3, 0.2, 0.007]

@qi.multiThreaded()
class GoEngageState(object):
    "State: going toxwards somebody"
    name = StrollState.GO_ENGAGE
    behavior = GO_ENGAGE_BEHAVIOR
    def __init__(self, session, services, logger, parent):
        # Helpers
        self.s = services
        self.events = stk.events.EventHelper(session)
        self.logger = logger
        # Other objects
        self.parent = parent
        # State
        self.running = False
        self.default_tracking_mode = None
        self.default_engaging_mode = None
        self.default_relative_pos = None

    #############################
    # Start / stop management

    def start(self):
        "Start state."
        if not self.running:
            self.running = True
            self.on_start()

    def stop(self):
        "Stop state."
        if self.running or not self.is_reset():
            self.running = False
            self.on_stop()

    #############################
    # Business logic

    def on_start(self):
        "Called when state starts"
        print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
        print "DBG setting tracking mode to navigate"
        print "DBG life is", self.s.ALAutonomousLife.focusedActivity()
        self.events.connect_decorators(self)

        self.default_tracking_mode = self.s.ALBasicAwareness.getTrackingMode()
        self.default_engaging_mode = self.s.ALBasicAwareness.getEngagementMode()
        self.default_relative_pos = self.s.ALTracker.getRelativePosition()

        self.s.ALBasicAwareness.setTrackingMode("Navigate")
        self.s.ALBasicAwareness.setEngagementMode("FullyEngaged")
        self.s.ALTracker.setRelativePosition(NAVIGATE_RELATIVE_POSITION)
        print "DBG set engagement mode to FullyEngaged"
        print "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"

    def is_reset(self):
        is_default_tracking = (self.s.ALBasicAwareness.getTrackingMode() == self.default_tracking_mode)
        is_default_engaging = (self.s.ALBasicAwareness.getEngagementMode() == self.default_engaging_mode)
        is_default_relative_pos = (self.s.ALTracker.getRelativePosition() == self.default_relative_pos)
        if is_default_tracking and is_default_engaging and is_default_relative_pos:
            return True
        else:
            return False

    def on_stop(self):
        "Called when state stops, for whatever reason."
        print "################################################"
        print "DBG stopping engage"
        if self.default_tracking_mode:
            self.s.ALBasicAwareness.setTrackingMode(self.default_tracking_mode)
        if self.default_engaging_mode:
            self.s.ALBasicAwareness.setEngagementMode(self.default_engaging_mode)
        if self.default_relative_pos:
            self.s.ALTracker.setRelativePosition(self.default_relative_pos)
        self.events.clear()

    def exit_to(self, target_state):
        "Helper: change state ONLY if we are still the docused state."
        if self.running and self.parent.state == self.name:
            self.parent.set_state(target_state)
        else:
            print "not changing state, because parent is", self.parent.state


    @stk.events.on("Launchpad/DistanceOfTrackedHuman")
    def on_distance_to_human(self, distance_to_human):
        "Callback to track human distance; can be an exit condition."
        print "DBG distance to human", distance_to_human #, tracking_mode
        if distance_to_human < 0:
            print "Back to wandering because nobody is around"
            self.exit_to(StrollState.WANDERING)
        elif distance_to_human < ENGAGEMENT_DISTANCE:
            print "Back to wandering because someone is very close"
            self.exit_to(StrollState.WANDERING)

    @stk.events.on("AutonomousLife/FocusedActivity")
    def on_focused_activity(self, activity_name):
        "Callback for life focused an activity (we may need to stop)"
        if activity_name != self.behavior:
            print "Oh, I lost focus, now it's", activity_name
            self.exit_to(StrollState.WANDERING)

@qi.multiThreaded()
class DXProactiveMobility(object):
    "NAOqi service example (set/get on a simple value)."
    APP_ID = "com.softbankrobotics.DXProactiveMobility"
    def __init__(self, qiapp):
        # generic activity boilerplate
        self.qiapp = qiapp
        self.session = qiapp.session
        self.events = stk.events.EventHelper(qiapp.session)
        self.s = stk.services.ServiceCache(qiapp.session)
        self.logger = stk.logging.get_logger(qiapp.session, self.APP_ID)

        # Wait some services
        self.session.waitForService("ALMotion")
        self.session.waitForService("DXHomeFinder") # NB it may not be there
        self.session.waitForService("ALBasicAwareness")
        self.session.waitForService("ALFaceDetection")

        # Internal variable
        self.last_time_at_home = time.time()
        self.started_solitary_wandering_time = time.time()
        self.state = StrollState.WANDERING
        self.life_state = "unknown"
        self.time_last_define_home = 0
        self.time_last_failed_go_gome = 0
        self.is_proactive_activated = False
        self.research_360_activated = False
        self.last_obstacle = None
        self.notifications = []

        # subscribe face detection
        self.s.ALFaceDetection.subscribe(self.APP_ID, 500 ,0)

        # periodic task
        self.update_task = qi.PeriodicTask()
        self.update_task.setCallback(self.update)
        self.update_task.setUsPeriod(ONE_SECOND_IN_US)

        # periodic task
        self.reset_body_task = qi.PeriodicTask()
        self.reset_body_task.setCallback(self.reset_angle)
        self.reset_body_task.setUsPeriod(HUNDRED_SECOND_IN_US)


        # State transitions
        self.changing_state_lock = threading.Lock()
        self.changing_state = False
        self.queued_state = None

        # configuration
        self.prefs = None

        # Sub objects
        self.go_engage_state = GoEngageState(qiapp.session, self.s,
                                             self.logger, self)
        self.tablet_helper = tablet.TabletHelper(self.session, self.s)
        self.configuration_reader = preferences.ConfigurationReader(\
            self.session, self.s, self.on_prefs)

    ########################################
    # Preferences

    def on_prefs(self, pref_dic):
        "Callback for when we have the preferences."
        #print "WARNING I AM CHANGING PREF" # Useful for tests
        #pref_dic[preferences.PREF_MAXDISTANCE] = 1.0
        print "DBG: I have preferences", pref_dic
        self.prefs = pref_dic
        # Some things require an action
        self.is_proactive_activated = self.prefs[preferences.PREF_ISACTIVE]
        # Stop update if needed
        if not self.is_proactive_activated and self.update_task.isRunning():
            self.update_task.stop()
        self.events.set(ALLOW_ENGAGE_KEY, self.is_proactive_activated)
        self.events.set(SHOULD_DEFINE_HOME_KEY, 0)
        self.events.set(SHOULD_GO_HOME_KEY, 0)
        self.s.DXHomeFinder.set_mode(pref_dic[preferences.PREF_TECHNOLOGY])
        self.tablet_helper.set_enabled_display(pref_dic[preferences.PREF_TABLETFEEDBACK])
        # Start update if needed
        if self.is_proactive_activated and not self.update_task.isRunning():
            self.update_task.start(True)

    ########################################
    # life cycle

    @qi.nobind
    def on_start(self):
        "Callback at start of service."
        self.events.connect_decorators(self)
        #self.update_task.start(True)

    @qi.nobind
    def on_stop(self):
        "Cleanup (add yours if needed)"
        self.s.ALFaceDetection.unsubscribe(self.APP_ID)
        if self.reset_body_task.isRunning():
            self.reset_body_task.stop()
        self.logger.info("DXProactiveMobility finished.")

    ########################################
    # Home management

    def handle_interrupted(self):
        "The behavior stopped for one reason."
        print "NB: activity was stopped, for some reason."
        if self.s.DXHomeFinder:
            self.s.DXHomeFinder.stop_going_home()


    def stop_awareness(self):
        "Helper - stop BA and force perception."
        try:
            self.s.ALBasicAwareness.stopAwareness()
        except RuntimeError:
            self.logger.info("Failed to stop awareness.")
        try:
            self.s.ALForcePerception.stop()
        except Exception as exc:
            self.logger.warning(exc)

    def start_awareness(self):
        "Helper - start BA and force perception."
        try:
            self.s.ALBasicAwareness.startAwareness()
        except RuntimeError:
            self.logger.info("Failed to stop awareness.")
        try:
            self.s.ALForcePerception.start()
        except Exception as exc:
            self.logger.warning(exc)

    def deactivate_proactive_mobility(self):
        # stop update if needed
        if self.update_task.isRunning():
            self.update_task.stop()
        #Deactivate proactivemobility
        self.is_proactive_activated = False
        self.events.set(ALLOW_ENGAGE_KEY, 0)
        self.events.set(SHOULD_DEFINE_HOME_KEY, 0)
        self.events.set(SHOULD_GO_HOME_KEY, 0)
        if self.s.DXHomeFinder:
            self.s.DXHomeFinder.uninit()

    def activate_proactive_mobility(self):
        # Check pref and activate if pref allows
        self.remove_notifications()
        self.configuration_reader = preferences.ConfigurationReader(\
            self.session, self.s, self.on_prefs)
    
    def remove_notifications(self):
        self.logger.warning("[activate_proactive_mobility] notifications to be removed: " + repr(self.notifications))
        if len(self.notifications) > 0:
            if self.s.ALNotificationManager:
                try:
                    for notification in self.notifications:
                        self.s.ALNotificationManager.remove(notification)
                except Exception as exc:
                    self.logger.warning("the notification with id " + repr(self.notifications) +" do not exist.")
        self.notifications = []

    def is_human(self):
        self.logger.info("is human")
        if self.s.ALTracker:
            robot_pose = almath.Pose2D(self.s.ALMotion.getRobotPosition(True))
            point_look_at = [
                [1 * math.cos(-robot_pose.theta) + 0 * math.sin(-robot_pose.theta), -1 * math.sin(-robot_pose.theta) + 0 * math.cos(-robot_pose.theta)],
                [1 * math.cos(-robot_pose.theta) + 1 * math.sin(-robot_pose.theta), -1 * math.sin(-robot_pose.theta) + 1 * math.cos(-robot_pose.theta)],
                [1 * math.cos(-robot_pose.theta) + (-1) * math.sin(-robot_pose.theta), -1 * math.sin(-robot_pose.theta) + (-1) * math.cos(-robot_pose.theta)]
                ]
            coord_to_check = [
                [robot_pose.x + point_look_at[0][0], robot_pose.y + point_look_at[0][1], 1.5],
                [robot_pose.x + point_look_at[1][0], robot_pose.y + point_look_at[1][1], 1.5],
                [robot_pose.x + point_look_at[2][0], robot_pose.y + point_look_at[2][1], 1.5],
                [robot_pose.x + point_look_at[0][0], robot_pose.y + point_look_at[0][1], 1.5]
                ]
            if self.last_obstacle:
                self.logger.info("last obstacle " + repr(self.last_obstacle))
                coord_to_check = [self.last_obstacle[:-1] + [1.0], self.last_obstacle[:-1] + [1.3]]
            for coord in coord_to_check:
                self.logger.info("enter in check human")
                self.s.ALTracker.lookAt(coord, 1, 0.2, 0)
                time.sleep(1.0)
                if len(self.s.ALMemory.getData("FaceDetected")) > 0:
                    self.logger.info("find human")
                    return True
        self.logger.info("no human")
        return False

    @stk.logging.log_exceptions
    def mark_here_as_home(self):
        "Pepper is now at home."
        self.logger.info("DBG - mark_here_as_home")
        self.tablet_helper.display("DEFINING_HOME")
        if self.s.DXHomeFinder:
            self.stop_awareness()
            time.sleep(1.0)
            if self.s.DXHomeFinder.set_current_pos_as_home():
                self.logger.info("successfully set home.")
                self.events.set(SHOULD_DEFINE_HOME_KEY, 0)
                self.events.set(ALLOW_ENGAGE_KEY,
                                self.prefs[preferences.PREF_ISACTIVE])
                self.tablet_helper.display("DEFINE_HOME_SUCCESS")
                if self.prefs[preferences.PREF_VOCALFEEDBACK]:
                    translate.say_audio_text(self.s.ALAnimatedSpeech,
                        self.s.ALTextToSpeech,
                        translate.DEFINE_HOME_SUCCESS)
                self.s.ALMotion.angleInterpolation(["HeadYaw", "HeadPitch"],
                                        [0, -0.26],
                                        2,
                                        1)
                self.logger.info("successfully set home.")
                if self.prefs[preferences.PREF_TABLETFEEDBACK]:
                    time.sleep(4)
            else:
                self.logger.info("Failed to define home!")
                self.reset_body_task.start(True)
                self.deactivate_proactive_mobility()
                self.tablet_helper.display("DEFINE_HOME_FAILURE")
                if self.prefs[preferences.PREF_VOCALFEEDBACK]:
                    translate.say_audio_text(self.s.ALAnimatedSpeech,
                        self.s.ALTextToSpeech,
                        translate.CANT_DEFINE_HOME,
                        self.s.DXHomeFinder.get_mode())
                if self.prefs[preferences.PREF_NOTIFICATIONFEEDBACK] and len(self.notifications) == 0:
                    notification = translate.notify(self.s.ALNotificationManager,
                        self.s.ALTextToSpeech,
                        translate.CANT_DEFINE_HOME,
                        self.s.DXHomeFinder.get_mode())
                    self.notifications.append(notification)
                if self.prefs[preferences.PREF_TABLETFEEDBACK]:
                    time.sleep(4)
            self.time_last_define_home = time.time()
            self.start_awareness()
            self.last_time_at_home = time.time()
        else:
            self.logger.info("No DXHomeFinder, not defining home.")

    @stk.logging.log_exceptions
    def go_home(self):
        "Goes home. should only be called from the associated app."
        self.set_state(StrollState.GOING_HOME, "from_behavior") # Normal
        self.stop_awareness()
        if not self.is_human():
            if self.s.DXHomeFinder:
                try:
                    self.s.DXHomeFinder.go_home(self.research_360_activated,_async=True)\
                        .then(self._on_go_home_done)
                except RuntimeError as exc:
                    self.logger.warning("Error going home: " + str(exc))
                    self.set_state(StrollState.WANDERING, "Exception")
            else:
                self.set_state(StrollState.WANDERING, "no_home_finder")
        else:
            self.set_state(StrollState.GO_ENGAGE, "human_found")

    def _on_go_home_done(self, future):
        "Callback when go_home is done."
        if future.hasError():
            self.time_last_failed_go_gome = time.time()
            self.reset_body_task.start(True)
            if self.state == StrollState.GOING_HOME:
                self.set_state(StrollState.WANDERING, "FutureError")
            else:
                self.logger.info("future has error, ignore 'cause state is " +\
                                    self.state)
        elif future.value():
            print "DBG successfully got home from future."
            self.research_360_activated = False
            self.s.ALMotion.angleInterpolation(["HeadYaw", "HeadPitch"],
                                        [0, -0.26],
                                        2,
                                        1)
            self.set_state(StrollState.AT_HOME, "future_success")
            self.last_time_at_home = time.time()
        else:
            print "DBG cant go home"
            do_loop = True
            last_speech_obstacle = 0
            stop_loop_reason = None
            while do_loop:
                if not self.is_human():
                    reason = self.s.DXHomeFinder.reason_cant_go_home.value()
                    if reason == "no_target":
                        self.tablet_helper.display("CANT_GO_TO_HOME_NOTARGET")
                    else:
                        self.tablet_helper.display("CANT_GO_TO_HOME_OBSTACLE")
                    if self.prefs[preferences.PREF_VOCALFEEDBACK] and time.time() - last_speech_obstacle > 15:
                            last_speech_obstacle = time.time()
                            translate.say_audio_text(self.s.ALAnimatedSpeech,
                                self.s.ALTextToSpeech,
                                translate.CANT_GO_TO_HOME,
                                reason)
                    if self.prefs[preferences.PREF_TABLETFEEDBACK]:
                        time.sleep(4)
                    if self.s.PositionManager:
                        if self.s.PositionManager.reset_angle():
                            stop_loop_reason = "reset_angle_done"
                            do_loop = False
                            break
                else:
                    stop_loop_reason = "human"
                    do_loop = False
                    break
            if stop_loop_reason != "human":
                self.time_last_failed_go_gome = time.time()
            self.s.ALMemory.raiseEvent("ALMotion/MoveFailed",0)
            self.set_state(StrollState.WANDERING, "future_failure")
        self.start_awareness()


    def get_state(self):
        "Accessor (for debug by qicli)"
        return self.state

    @stk.logging.log_exceptions
    def update(self):
        "Periodic update."
        home_defined = True
        if self.s.DXHomeFinder:
            if not self.s.DXHomeFinder.is_init():
                home_defined = False
                self.events.set(SHOULD_DEFINE_HOME_KEY, 1)
        if self.state == StrollState.WANDERING:
            if self.could_go_home() and home_defined:
                reasons = self.get_reasons_to_go_home()
                if reasons:
                    self.logger.info("Going home because %s" % reasons)
                    self.events.set(SHOULD_GO_HOME_KEY, 1)
        else:
            if self.state == StrollState.GO_ENGAGE:
                if not self.is_in_allowed_range():
                    self.events.set(ALLOW_ENGAGE_KEY, False)
                    # Maybe change it for a getting reason with is_in_allowed_range
                    # But easier to do like that
                    odom_error = self.s.PositionManager.get_odometric_position_error()
                    mode = "unknown"
                    if self.s.DXHomeFinder:
                        mode = self.s.DXHomeFinder.get_mode()
                    if odom_error[0] >= 0.5 or odom_error[1] >= 0.5 and mode != "slam":
                        self.research_360_activated = True
                        self.set_state(StrollState.WANDERING, "move_too_much")
                        time.sleep(3)
                        self.tablet_helper.display("CANT_REACH_HUMAN_DISTANCE")
                        if self.prefs[preferences.PREF_VOCALFEEDBACK]:
                            translate.say_audio_text(self.s.ALAnimatedSpeech,
                                self.s.ALTextToSpeech,
                                translate.CANT_REACH_HUMAN,
                                "distance")
                    else:
                        self.set_state(StrollState.WANDERING, "went_outside_range")
                        time.sleep(3)
                        self.tablet_helper.display("CANT_REACH_HUMAN_AREA")
                        if self.prefs[preferences.PREF_VOCALFEEDBACK]:
                            translate.say_audio_text(self.s.ALAnimatedSpeech,
                                self.s.ALTextToSpeech,
                                translate.CANT_REACH_HUMAN,
                                "area")
                    if self.prefs[preferences.PREF_TABLETFEEDBACK]:
                        time.sleep(4)
                else:
                    # self.started_solitary_wandering_time = time.time()
                    time_wandering = time.time() - self.started_solitary_wandering_time
                    if time_wandering > self.prefs[preferences.PREF_BOREDTIMEOUT]:
                        self.set_state(StrollState.WANDERING, "no_human_tracked")


    ###################################"
    # Criteria for going home

    def could_go_home(self):
        "In a state where going home would make sense."
        far_from_home = True
        if self.s.PositionManager:
            robot_x, robot_y, _ = self.s.PositionManager.get_robot_pos()
            odom_error = self.s.PositionManager.get_odometric_position_error()
            dist_from_home = math.hypot(robot_x, robot_y) + odom_error[0]
            robot_theta = self.s.PositionManager.get_orientation()
            if dist_from_home < 0.2 and abs(robot_theta) < math.radians(24):
                far_from_home = False
        return (self.life_state == "solitary") and \
               self.prefs[preferences.PREF_ISACTIVE] and far_from_home

    def is_lost(self):
        "Have I moved so much that my odometry is unreliable."
        if self.s.PositionManager:
            odom_error = self.s.PositionManager.get_odometric_position_error()
            return odom_error[0] > MAX_ERROR
        return False

    def is_still_home(self):
        "Are we still right at home"
        if self.s.PositionManager:
            robot_x, robot_y, theta = self.s.PositionManager.get_robot_pos()
            print "DBG HOME", robot_x, robot_y, math.degrees(theta)
            if max(abs(robot_x), abs(robot_y)) > NAV_EPSILON_POS:
                return False
            elif abs(theta) > NAV_EPSILON_ANGLE_RAD:
                return False
        return True

    def reset_angle(self):
        if self.s.PositionManager:
            is_success = self.s.PositionManager.reset_angle()
            if is_success:
                self.reset_body_task.stop()

    def is_in_allowed_range(self):
        "Am I still in the range around home defined by the preferences?"
        if self.s.PositionManager:
            robot_x, robot_y, _ = self.s.PositionManager.get_robot_pos()
            odom_error = self.s.PositionManager.get_odometric_position_error()
            dist_from_home = math.hypot(robot_x, robot_y) + odom_error[0]
            angle_range_degrees = self.prefs[preferences.PREF_MAXANGLE]
            max_angle_from_home_deg = angle_range_degrees / 2.0
            angle_degrees = math.degrees(math.atan2(robot_y, robot_x))
            #print "DBG x=%f y=%f angle=%f / %f" % (robot_x, robot_y,
            #                        angle_degrees,  max_angle_from_home_deg)

            # odom[0] => error in xy in m, odom[1] => error in theta in rad
            mode = "unknown"
            if self.s.DXHomeFinder:
                mode = self.s.DXHomeFinder.get_mode()
            if odom_error[0] >= 0.5 or odom_error[1] >= 0.5 and mode != "slam":
                # the robot move a lot and is lost
                return False
            if dist_from_home > self.prefs[preferences.PREF_MAXDISTANCE]:
                # Se're too far
                return False
            elif dist_from_home < 0.3:
                # Special radius very near the robot: if we're VERY close to
                # home ignore the angle
                return True
            elif abs(angle_degrees) > max_angle_from_home_deg:
                self.logger.info("Stepped outside allowed range angle!")
                return False
            else:
                return True
        else:
            self.logger.warning("No range angle!")
            return True

    def get_reasons_to_go_home(self):
        "Should I go home RIGHT NOW?"
        reasons = []
        time_wandering = time.time() - self.started_solitary_wandering_time
        print "DBG I've been wandering for", time_wandering
        
        time_since_try_to_go_home = time.time() - self.time_last_failed_go_gome
        if time_since_try_to_go_home > TIMEOUT_BEFORE_RETRY_GO_HOME:
            if time_wandering > self.prefs[preferences.PREF_BOREDTIMEOUT]:
                reasons.append("bored")
            if self.is_lost():
                reasons.append("lost")
        else:
            print "DBG I tried to go home less that one minute ago"
        return reasons

    ########################################
    # Engage people

    def go_engage(self):
        "Called by 'go engage' behavior"
        self.set_state(StrollState.GO_ENGAGE, "from_behavior")
        # Until we get close to that person.

    ########################################
    # Interactive activies (behaviors)

    def stop_activity(self, behavior_id):
        "Helper to start activity"
        if behavior_id == self.s.ALAutonomousLife.focusedActivity():
            self.s.ALAutonomousLife.stopFocus(_async=True)

    ########################################
    # State management

    def _set_state_internal(self, new_state, reason):
        "Set state - will handle all transition actions."
        if new_state == self.state:
            self.logger.info("State already is " + new_state)
        else:
            # Stop old state
            old_state = self.state
            if old_state == StrollState.GOING_HOME:
                self.stop_activity(GO_HOME_BEHAVIOR)
                self.events.set(SHOULD_GO_HOME_KEY, 0)
            elif old_state == StrollState.DEFINING_HOME:
                self.stop_activity(DEFINE_HOME_BEHAVIOR)
                self.events.set(SHOULD_DEFINE_HOME_KEY, 0)
            elif old_state == StrollState.GO_ENGAGE:
                print "done go engage"
                self.go_engage_state.stop()
                self.stop_activity(GO_ENGAGE_BEHAVIOR)
                self.started_solitary_wandering_time = time.time()
                print "now reset time after go engage"
            # start new state
            if new_state == StrollState.DEFINING_HOME:
                elapsed = time.time() - self.time_last_define_home
                if elapsed > TIME_RETRY_DEFINE_HOME:
                    self.events.set(SHOULD_DEFINE_HOME_KEY, 1)
            elif new_state == StrollState.AT_HOME:
                self.tablet_helper.display("AT_HOME")
                self.events.set(SHOULD_GO_HOME_KEY, 0)
                self.events.set(ALLOW_ENGAGE_KEY,
                                self.prefs[preferences.PREF_ISACTIVE])
                self.last_time_at_home = time.time()
                if self.prefs[preferences.PREF_TABLETFEEDBACK]:
                    time.sleep(4)
            elif new_state == StrollState.WANDERING:
                self.started_solitary_wandering_time = time.time()
            elif new_state == StrollState.GO_ENGAGE and \
              self.s.DXHomeFinder.is_init():
                self.tablet_helper.display("GO_ENGAGE")
                self.go_engage_state.start()
            elif new_state == StrollState.GOING_HOME:
                self.tablet_helper.display("GO_HOME")
                self.events.set(SHOULD_GO_HOME_KEY, 1)
            # Done setting state
            self.state = new_state
            self.logger.info("State transition done: %s -> %s (reason: %s)" % \
                (old_state, new_state, reason))

    def set_state(self, new_state, reason="unknown"):
        "Guarantees only the last state set is taken into account."
        if not self.is_proactive_activated:
            self.logger.info("No set state because of deactivate proactive mobility")
            return
        # First: check if we're already doing a transition.
        # If so, just queue it.
        # We actually always only keep the latest item in the queue
        with self.changing_state_lock:
            if self.changing_state:
                self.queued_state = (new_state, reason + "_queued")
                return
            self.changing_state = True
            self.queued_state = new_state, reason
        # Now, actually do the state change
        # 99% of the time we only have a simple item
        while True:
            # 1) remove the only item on the queue
            with self.changing_state_lock:
                if not self.queued_state:
                    self.changing_state = False
                    return
                new_state, reason = self.queued_state
                self.queued_state = None
            # 2) process that item (only ONE of this can be going on at the
            #    same state)
            self._set_state_internal(new_state, reason) # Takes some time
            # 3) go back and check if something else was added to that queue
            #    in the meantime.

    ########################################
    # Event callbacks

    @stk.events.on("BatteryTrapIsOpen")
    def on_hatch_open(self, hatch_open):
        "Callback for battery hatch: close hatch = define home."
        if not hatch_open:
            home_policy = self.prefs[preferences.PREF_HOMEPOLICY]
            if home_policy == preferences.HOMEPOLICY_HATCH_CLOSED:
                self.set_state(StrollState.DEFINING_HOME, "hatch")

    @stk.events.on("FaceDetected")
    def on_face_detected(self, value):
        self.logger.info(" focus what ??? " + repr(self.s.ALAutonomousLife.focusedActivity()))
        if value:
            self.started_solitary_wandering_time = time.time()
            if self.s.ALAutonomousLife.focusedActivity() != GO_ENGAGE_BEHAVIOR:
                self.logger.info("#### stop engage")
                self.go_engage_state.stop()
            if self.s.ALAutonomousLife.focusedActivity() == GO_ENGAGE_BEHAVIOR:
                self.logger.info("#### start engage")
                self.go_engage_state.start()
            elif self.is_in_allowed_range() and self.is_proactive_activated:
                self.events.set(ALLOW_ENGAGE_KEY, True)

    @stk.events.on("ALMotion/MoveFailed")
    def on_move_failed(self, value):
        self.logger.info("move failed " + repr(value))
        if value:
            if isinstance(value, list):
                if "Safety" in value:
                    obstacle_pose = almath.Pose2D(value[-1])
                    self.last_obstacle = [obstacle_pose.x, obstacle_pose.y, obstacle_pose.theta]
        else:
            self.last_obstacle = None

    @stk.events.on("AutonomousLife/State")
    def on_life_state(self, life_state):
        "Callback when AutonomousLife switches state."
        print "State is switching to", life_state
        if self.life_state == "disabled" and life_state == "solitary":
            self.activate_proactive_mobility()
        self.life_state = life_state
        if self.could_go_home():
            return # why?
        if life_state == "solitary":
            self.started_solitary_wandering_time = time.time()
            if not self.s.DXHomeFinder:
                self.logger.info("(no DXHomeFinder, not going home.)")
            elif not self.s.DXHomeFinder.is_init():
                self.set_state(StrollState.DEFINING_HOME, "needs_to_define")
            elif self.state == StrollState.GO_ENGAGE:
                self.logger.info("Going to engage, normal, don't go home")
            elif self.state == StrollState.SAFEGUARD_GOING_HOME:
                self.logger.info("I was interrupted going home, try again")
                self.set_state(StrollState.GOING_HOME, "after_safeguard") # !
            elif self.state != StrollState.AT_HOME:
                self.logger.info("(we just left home, not going home)")
                self.set_state(StrollState.WANDERING, "solitary_not_at_home")
        elif life_state == "safeguard":
            # Oh great, something terrible happened, like rolled over an ant
            if self.state == StrollState.GOING_HOME:
                self.set_state(StrollState.SAFEGUARD_GOING_HOME)
        elif life_state == "interactive":
            if self.state == StrollState.AT_HOME:
                self.last_time_at_home = time.time()
                if not self.is_still_home():
                    self.logger.info("Starting wandering - start timer")
                    self.set_state(StrollState.WANDERING,
                                   "interactive from home")
            elif self.state == StrollState.GO_ENGAGE:
                self.logger.info("I seem to have been close enough to " + \
                                 "trigger something")
                self.set_state(StrollState.WANDERING, "from_go_wandering")
        elif life_state == "disabled":
            self.deactivate_proactive_mobility()

####################
# Setup and Run
####################

if __name__ == "__main__":
    stk.runner.run_service(DXProactiveMobility)
