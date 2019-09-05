# -*- coding: utf-8 -*-
"""
Created on Wed Feb 14 11:12:48 2018

@author: ekroeger, mcaniot
"""

import stk.runner
import stk.events
import stk.services
import stk.logging

#########################################
# Constants

PREF_DOMAIN = "com.softbankrobotics.proactive-mobility"

PREF_ISACTIVE = "IsActive"
PREF_TECHNOLOGY = "Technology"
PREF_MAXDISTANCE = "MaxDistance"
PREF_MAXANGLE = "MaxAngle"
PREF_BOREDTIMEOUT = "GoHomeTimeout"
PREF_HOMEPOLICY = "HomePolicy"
PREF_TABLETFEEDBACK = "TabletFeedback"
PREF_VOCALFEEDBACK = "VocalFeedback"
PREF_NOTIFICATIONFEEDBACK = "NotificationFeedback"

HOMEPOLICY_STARTUP = "startup"
HOMEPOLICY_HATCH_CLOSED = "hatch-closed"

#######################################
# Pseudo-bool helper type.

def pseudo_bool(raw_value):
    "Tries to coerce value to bool, allowing 'yes', 'on', 'true' and the like"
    try:
        value = int(raw_value)
        return bool(value)
    except ValueError:
        val_lower = str(raw_value).lower()
        if val_lower in ("on", "true", "yes"):
            return True
        elif val_lower in ("off", "false", "no"):
            return False
        else:
            raise ValueError("Couldn't convert value to bool: %s" % raw_value)


#######################################
# Now let's define our types, default values, and allowed ranges.


DEFAULTS = {
    PREF_ISACTIVE: False,
    PREF_TECHNOLOGY: "slam",
    PREF_MAXDISTANCE: 3.0,
    PREF_MAXANGLE: 180.0,
    PREF_BOREDTIMEOUT: 10.0,
    PREF_HOMEPOLICY: HOMEPOLICY_STARTUP,
    PREF_TABLETFEEDBACK: True,
    PREF_VOCALFEEDBACK: True,
    PREF_NOTIFICATIONFEEDBACK: True
}

TYPES = {
    PREF_ISACTIVE: pseudo_bool,
    PREF_TECHNOLOGY: str,
    PREF_MAXDISTANCE: float,
    PREF_MAXANGLE: float,
    PREF_BOREDTIMEOUT: float,
    PREF_HOMEPOLICY: str,
    PREF_TABLETFEEDBACK: pseudo_bool,
    PREF_VOCALFEEDBACK: pseudo_bool,
    PREF_NOTIFICATIONFEEDBACK: pseudo_bool
}

def is_valid(key, value):
    "Key-specific rules."
    if key == PREF_ISACTIVE:
        return value in [0, 1, True, False, "on", "off"]
    if key == PREF_TECHNOLOGY:
        return value in ["slam", "pod", "aruco"]
    if key == PREF_MAXDISTANCE:
        return 0.0 <= value <= 3.0
    if key == PREF_MAXANGLE:
        return 0.0 < value <= 360.0
    if key == PREF_BOREDTIMEOUT:
        return value == -1 or value >= 0
    if key == PREF_HOMEPOLICY:
        return value in [HOMEPOLICY_STARTUP, HOMEPOLICY_HATCH_CLOSED]
    if key == PREF_TABLETFEEDBACK:
        return value in [0, 1, True, False, "on", "off"]
    if key == PREF_VOCALFEEDBACK:
        return value in [0, 1, True, False, "on", "off"]
    if key == PREF_NOTIFICATIONFEEDBACK:
        return value in [0, 1, True, False, "on", "off"]

#######################################
# Tie it all in a single getter

def _get_valid_value(key, raw_value):
    "Check if the value is valid, and if not, return the default."
    wanted_type = TYPES[key]
    try:
        value = wanted_type(raw_value)
        # force strigns to be lower case
        if wanted_type == str:
            value = value.lower()
        if is_valid(key, value):
            return value
        else:
            return DEFAULTS[key]
    except ValueError:
        return DEFAULTS[key]
    except TypeError:
        return DEFAULTS[key]

#######################################
# Unit tests

def _test_pref_validation():
    "Check the logic for default values, and type casting."
    assert _get_valid_value(PREF_ISACTIVE, True) == True
    assert _get_valid_value(PREF_ISACTIVE, "Broccoli") == False
    assert _get_valid_value(PREF_ISACTIVE, "Yes") == True
    assert _get_valid_value(PREF_ISACTIVE, "On") == True
    assert _get_valid_value(PREF_ISACTIVE, "true") == True
    assert _get_valid_value(PREF_ISACTIVE, "false") == False
    assert _get_valid_value(PREF_TECHNOLOGY, None) == "slam"
    assert _get_valid_value(PREF_TECHNOLOGY, "aruco") == "aruco"
    assert _get_valid_value(PREF_TECHNOLOGY, "Broccoli") == "slam"
    assert _get_valid_value(PREF_MAXDISTANCE, None) == 3.0
    assert _get_valid_value(PREF_MAXDISTANCE, -1.0) == 3.0
    assert _get_valid_value(PREF_MAXDISTANCE, 2.0) == 2.0
    assert _get_valid_value(PREF_MAXDISTANCE, "2") == 2.0
    assert _get_valid_value(PREF_MAXDISTANCE, 5.0) == 3.0
    assert _get_valid_value(PREF_MAXANGLE, None) == 180.0
    assert _get_valid_value(PREF_MAXANGLE, 42) == 42.0
    assert _get_valid_value(PREF_BOREDTIMEOUT, 1000.0) == 1000.0
    assert _get_valid_value(PREF_BOREDTIMEOUT, -1.0) == -1.0
    assert _get_valid_value(PREF_BOREDTIMEOUT, None) == 10.0
    print "Pref validation checks all passed."

#######################################
# Link with preference manager

class ConfigurationReader(object):
    "Validator - returns dictionaries of prefs."
    def __init__(self, session, services, on_prefs_callback):
        self.session = session
        self.s = services
        self.events = stk.events.EventHelper(session)
        self.domain = PREF_DOMAIN
        self._last_prefs = None # to track what we notified
        self._on_prefs_callback = on_prefs_callback
        # This *may* be worth doing at another moment
        self.session.waitForService("ALPreferenceManager")
        self.read_values_and_notify()
        self.events.connect_decorators(self)

    def read_values_and_notify(self):
        "Get the preferences, and notify listeners if they changed."
        pref_dic = {}
        for key in DEFAULTS:
            raw_value = self.s.ALPreferenceManager.getValue(self.domain, key)
            pref_dic[key] = _get_valid_value(key, raw_value)
        if self._last_prefs != pref_dic:
            self._on_prefs_callback(pref_dic)
            self._last_prefs = pref_dic

    @stk.events.on("ALPreferenceManager.preferencesSynchronized")
    def on_preferences_synchronized(self):
        "Callback when preferences are synched with cloud (and maybe changed)."
        print "DBG Warning prefs synchronized!"
        self.read_values_and_notify()

def _debug_print_prefs():
    "Print to the log the current values on the robot (needs qi-url)."
    import pprint
    qiapp = stk.runner.init()
    services = stk.services.ServiceCache(qiapp.session)
    ConfigurationReader(qiapp.session, services, pprint.pprint)
    print "done."

if __name__ == "__main__":
    _test_pref_validation()
    _debug_print_prefs()
