"""
Home Finder Service.


Can have several states:
 - going home - you should never move the orbot
 - not going home (and thus, possibly, moving around)


"""
__version__ = "0.1.0"

__copyright__ = "Copyright 2017, Softbank Robotics"
__author__ = 'ekroeger and mcaniot'
__email__ = 'ekroeger@softbankrobotics.com and mcaniot@softbankrobotics.com'

# Import qi
import qi

# stk libs
import stk.runner
import stk.events
import stk.services
import stk.logging
import stk.coroutines

# Internal libs
import slammanager as slamm
import podmanager as podm
import arucomanager as arucom

import math

ONE_SEC = 1000000

DEFAULT_ENGAGE_DISTANCE = 1.0

PACKAGE_ID = "home-finder"

@qi.multiThreaded() # TODO: take this out once this doesn't have time.sleep
class DXHomeFinder(object):
    "NAOqi service example (set/get on a simple value)."
    APP_ID = "com.softbankrobotics.DXHomeFinder"
    def __init__(self, qiapp):
        # generic activity boilerplate
        self.qiapp = qiapp
        self.session = qiapp.session
        self.events = stk.events.EventHelper(qiapp.session)
        self.services = stk.services.ServiceCache(qiapp.session)
        self.logger = stk.logging.get_logger(qiapp.session, self.APP_ID)
        # Property:
        self.going_home = qi.Property()
        self.going_home.setValue(False)
        self.reason_cant_go_home = qi.Property()
        self.reason_cant_go_home.setValue("unknown")
        
        self.session.waitForService("PositionManager")

        # Manager class
        self.position_manager = self.services.PositionManager
        self.slam_manager = slamm.SLAMManager(self.session)
        self.pod_manager = podm.PodManager(self.session)
        self.aruco_manager = arucom.ARucoManager(self.session)
        self.finders = {
            "slam": self.slam_manager,
            "pod": self.pod_manager,
            "aruco": self.aruco_manager
        }

        # Mode : simple, slam, qrcode, pod
        self.mode = "slam"
        # future_movement
        self.future_movement = None

    ##################################
    # Configuration

    @stk.coroutines.public_async_generator
    @stk.logging.log_exceptions
    def set_current_pos_as_home(self):
        "Home must be current position."
        self.position_manager.init_position()
        self.future_movement = self.finders[self.mode].init_home()
        yield self.future_movement
        init_value = self.future_movement.value()
        if init_value:
            self.position_manager.init_position()
        yield stk.coroutines.Return(init_value)

    ##################
    # Actions
    
    def is_init(self):
        return self.finders[self.mode].is_init()

    def uninit(self):
        return self.finders[self.mode].uninit()

    def set_mode(self, mode):
        "Change the navigation mode. (slam, aruco, pod)"
        if mode.lower() != self.mode:
            self.finders[self.mode].uninit()
            self.mode = mode.lower()
            self.future_movement = None
            self.logger.info("change the mode for : " + repr(mode))
        else:
            self.logger.info("same mode as before : " + repr(mode))
        return self.mode

    @stk.coroutines.public_async_generator
    @stk.logging.log_exceptions
    def go_home(self, research_360_activated=False):
        "Navigate to home, somehow."
        value = False
        if not self.going_home.value():
            self.reason_cant_go_home.setValue("unknown")
            self.going_home.setValue(True)
            if self.finders[self.mode].is_init():
                self.future_movement = self.finders[self.mode].return_home(research_360_activated)
                yield self.future_movement
                value = self.future_movement.value()
                if not value and not research_360_activated and self.reason_cant_go_home.value() == "no_target":
                    self.future_movement = self.finders[self.mode].return_home(True)
                    yield self.future_movement
                    value = self.future_movement.value()
                if value:
                    self.logger.info(repr(value))
                    self.position_manager.init_position()
            self.going_home.setValue(False)
            self.future_movement = None
        yield stk.coroutines.Return(value)

    def stop_going_home(self):
        "Cancel going home."
        self.going_home.setValue(False)
        try:
            if self.future_movement:
                self.future_movement.cancel()
        except Exception as error_msg:
            self.logger.warning(error_msg)
        self.future_movement = None
        self.finders[self.mode].cancel()
        

    ##################################
    # Info (helps decide what to do next)

    def get_robot_pos(self):
        """2D coord of the robot in frame home.
        Return a tiplet (x, y, theta) """
        return self.position_manager.get_robot_pos()

    @stk.logging.log_exceptions
    def get_home_world_pos(self):
        """X, Y, and angle position compared to home frame.

        Especially useful for debug visualisation.
        """

        return self.position_manager.get_home_pos()

    def get_lost_score(self):
        "How lost is Pepper? (helps decide if we allow move etc.)"
        pass

    def get_mode(self):
        return self.mode

    ####################################################
    # Life cycle


    @qi.nobind
    def on_stop(self):
        "Cleanup (add yours if needed)"
        self.logger.info("DXHomeFinder finished.")

####################
# Setup and Run
####################

if __name__ == "__main__":
    stk.runner.run_service(DXHomeFinder)

