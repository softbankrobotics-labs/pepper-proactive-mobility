# -*- coding: utf-8 -*-
"""
Created on Thu Mar 08 11:18:39 2018

@author: cdonnat, mcaniot
"""

import stk.runner
import stk.events
import stk.services
import stk.logging

#########################################
# Constants

PREF_DOMAIN = "com.softbankrobotics.proactive-mobility"
PACKAGE_UID = "proactive-mobility"

FEEDBACKS_ARRAY = {
    "AT_HOME": "home_ok.png",
    "CANT_GO_TO_HOME_NOTARGET": "look_for_home.png",
    "CANT_GO_TO_HOME_OBSTACLE": "obstacle.png",
    "CANT_REACH_HUMAN_AREA": "zone_human.png",
    "CANT_REACH_HUMAN_DISTANCE": "too_far.png",
    "CANT_REACH_HUMAN_OBSTACLE": "obstacle_human.png",
    "DEFINE_HOME_FAILURE": "home_failure.png",
    "DEFINE_HOME_SUCCESS": "home_success.png",
    "DEFINING_HOME": "define_home.png",
    "GO_ENGAGE": "go_engage.png",
    "GO_HOME": "go_home.png"
}

#######################################
# Tablet Feedbacks helper

class TabletHelper(object):
    def __init__(self, session, services, is_enabled = True):
        self.session = session
        self.services = services
        self.domain = PREF_DOMAIN
        self.is_display_enabled = is_enabled
        self.image_paths = None
        self.robot_ip = None
        if (self.services.ALTabletService):
            self._on_al_tablet_service()

    def _on_al_tablet_service(self):
        self.robot_ip = self.services.ALTabletService.robotIp()
    
    def display(self, content):
        print "[TabletHelper] tablet display %s" % content
        if self.is_display_enabled and self.services.ALTabletService:

            #Sets image_paths if ALTabletService connection is finally available
            if not self.robot_ip:
                self._on_al_tablet_service()
            picture = FEEDBACKS_ARRAY.get(content)
            if picture:
                print "[TabletHelper] tablet display picture %s" % picture
                self.services.ALTabletService.showWebview("http://%s/apps/%s?picture=%s" % (self.robot_ip, PACKAGE_UID, picture))

    def hide(self):
        print "[TabletHelper] tablet hide"
        if self.is_display_enabled and self.services.ALTabletService:
            self.services.ALTabletService.hideWebview()

    def set_enabled_display(self, is_enabled):
        self.is_display_enabled = is_enabled