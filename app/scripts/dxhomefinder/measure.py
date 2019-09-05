# -*- coding: utf-8 -*-
"""
Scripts for collecting tests in various measures

Created on Wed Jan 24 10:47:08 2018

@author: ekroeger
"""
import csv
import os
import math
import random
import time

# stk libs
import stk.runner
import stk.events
import stk.services
import stk.logging

# Internal libs
import arucomanager as arucom

class MeasureScenario(object):
    "Abstract class for scenarios"
    def __init__(self, measurer):
        self.measurer = measurer
        self.services = measurer.services

    def run(self, k):
        "Abstract function, to reimplement"
        assert False, "Override this in specific implementation."

class DummyScenario(MeasureScenario):
    "Dummy test: the robot doesn't do anything, did anything move?"
    def run(self, k):
        "Do nothing"
        pass

class BumperScenario(MeasureScenario):
    "Block between tests until someone hits the bumper - for testing."
    def run(self, k):
        "Block until a bumper is pressed."
        print "bump?"
        self.measurer.events.wait_for("LeftBumperPressed")
        print "bump!"

class ThereAndBackScenario(MeasureScenario):
    def go_there(self):
        assert False, "abstact class!"

    def go_back(self):
        assert False, "abstact class!"

    def run(self, k):
        go_result = self.go_there(k)
        before_return = time.time()
        back_result = self.go_back(k)
        result = {"return_time": time.time() - before_return}
        if go_result:
            result.update(go_result)
        if back_result:
            result.update(back_result)
        return result

class MoveScenario(ThereAndBackScenario):
    "Pepper moves, and returns to her place."
    def __init__(self, measurer, move_x=0.0, move_y=0.0, move_theta=0.0):
        ThereAndBackScenario.__init__(self, measurer)
        self.move_x = move_x
        self.move_y = move_y
        self.move_theta = move_theta
        self.result = {
            "move_x": self.move_x,
            "move_y": self.move_y,
            "move_theta": self.move_theta,
        }

    def go_there(self, k):
        "move by dx, dy, and theta, and then move back to original pos."
        if self.move_x or self.move_y:
            self.services.ALMotion.moveTo(self.move_x, self.move_y, 0)
            time.sleep(0.1)
        if self.move_theta:
            self.services.ALMotion.moveTo(0, 0, self.move_theta)
            time.sleep(0.1)
        return self.result

    def go_back(self, k):
        if self.move_theta:
            self.services.ALMotion.moveTo(0, 0, -self.move_theta)
            time.sleep(0.1)
        if self.move_x or self.move_y:
            self.services.ALMotion.moveTo(-self.move_x, -self.move_y, 0)
            time.sleep(0.1)


class SlamScenario(ThereAndBackScenario):
    "Pepper moves, and returns to her place."
    def __init__(self, measurer, move_x=0.0, move_y=0.0, move_theta=0.0):
        ThereAndBackScenario.__init__(self, measurer)
        #self.slam_manager = slamm.SLAMManager(self.session, 0)
        self.move_x = move_x
        self.move_y = move_y
        self.move_theta = move_theta
        print "setting mode..."
        self.services.DXHomeFinder.set_mode("slam_0")
        self.services.DXHomeFinder.set_current_pos_as_home()
        print "... finished."
        #self.slam_manager.init_home()
        self.result = {
            "move_x": self.move_x,
            "move_y": self.move_y,
            "move_theta": self.move_theta,
        }

    def go_there(self, k):
        "move by dx, dy, and theta, and then move back to original pos."
        if self.move_x or self.move_y:
            self.services.ALMotion.moveTo(self.move_x, self.move_y, 0)
            time.sleep(0.1)
        if self.move_theta:
            self.services.ALMotion.moveTo(0, 0, self.move_theta)
            time.sleep(0.1)

    def go_back(self, k):
        self.services.DXHomeFinder.go_home()


def split(low, high, steps):
    spread = high - low
    for i in range(steps):
        yield low + (float(i * spread) / (steps - 1))

#print list(split(0, 4, 3))

class ColinMaillardScenario(ThereAndBackScenario):
    "Pepper moves, and returns to her place."
    def __init__(self, measurer, direction=(-math.pi/2), advance=0.0,
                 move_theta=0.0, strategy="slam", steps=10):
        ThereAndBackScenario.__init__(self, measurer)
        #self.slam_manager = slamm.SLAMManager(self.session, 0)
        self.direction = direction
        # These will be decided later
        self.advance = None
        self.move_theta = None
        self.steps = []
        # Special: optionally we may define either as a range
        if isinstance(advance, list):
            low, high = advance
            for adv in split(low, high, steps):
                self.steps.append({
                    "advance": adv,
                    "move_theta": move_theta
                })
        elif isinstance(move_theta, list):
            low, high = move_theta
            for theta in split(low, high, steps):
                self.steps.append({
                    "advance": advance,
                    "move_theta": theta
                })
        else:
            # Simple, single step
            self.steps.append({
                    "advance": advance,
                    "move_theta": move_theta
                })
        self.next_steps = []
        print "steps", self.steps

        self.strategy = strategy
        if self.strategy == "slam":
            print "Strategy slam: setting mode..."
            self.services.DXHomeFinder.set_mode("slam_0")
            self.services.DXHomeFinder.set_current_pos_as_home()
            print "... finished."
        else:
            print "strategy =", self.strategy
        #self.slam_manager.init_home()

    def go_there(self, k):
        "Configurable go there"
        if not self.next_steps:
            # Copy
            self.next_steps = list(self.steps)
            random.shuffle(self.next_steps)
        step = self.next_steps.pop()
        # Save so that go_back also knows how to do this
        self.advance = step["advance"]
        self.move_theta = step["move_theta"]
        # this is the json we'll return
        result = {
            "direction": self.direction,
            "advance": self.advance,
            "move_theta": self.move_theta,
            "strategy": self.strategy
        }
        print "move plan:", result
        # Turn in the right direction
        time.sleep(0.1)
        self.services.ALMotion.moveTo(0.0, 0.0, self.direction)
        time.sleep(0.1)
        # Move forward
        self.services.ALMotion.moveTo(self.advance, 0.0, 0.0)
        time.sleep(0.1)
        # Spin on place
        self.services.ALMotion.moveTo(0.0, 0.0, self.move_theta)
        time.sleep(0.1)
        return result

    def go_back(self, k):
        if self.strategy == "slam":
            self.services.DXHomeFinder.go_home()
        elif self.strategy == "odometry":
            # Same operations in reverse order
            time.sleep(0.1)
            self.services.ALMotion.moveTo(0.0, 0.0, -self.move_theta)
            time.sleep(0.1)
            # move backwards
            self.services.ALMotion.moveTo(-self.advance, 0.0, 0.0)
            time.sleep(0.1)
            # get back in original position
            self.services.ALMotion.moveTo(0.0, 0.0, -self.direction)
            time.sleep(0.1)

# CSV helper

def save_dicts_to_csv(data, filepath):
    "Helper - saves a list of dicts to a clean csv file"
    headers = set()
    for data_dict in data:
        headers.update(data_dict.keys())
    headers = sorted(headers)
    with open(filepath, 'wb') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(headers)
        for row_dic in data:
            values = [row_dic.get(key, "") for key in headers]
            writer.writerow(values)
        csvfile.close()

class MovementMeasurer(object):
    "NAOqi service example (set/get on a simple value)."
    APP_ID = "com.softbankrobotics.MovementMeasures"
    def __init__(self, qiapp):
        # generic activity boilerplate
        self.qiapp = qiapp
        self.session = qiapp.session
        self.events = stk.events.EventHelper(qiapp.session)
        self.services = stk.services.ServiceCache(qiapp.session)
        self.logger = stk.logging.get_logger(qiapp.session, self.APP_ID)
        self.arcode_manager = arucom.ARucoManager(self.session)

    ####################################################
    # Making this a more effective testing framework

    def log_row(self, *args):
        "Helper for pretty printing"
        print "\t".join(map(str, args))

    def _iter_attempt_results(self, func, num_attempts):
        "general idea: look at code, do something, see if our pos changed."
        if not self.arcode_manager.init_home_pos():
            self.logger.info("No arucode in front of Pepper, abort!")
            return
        for i in range(num_attempts):
            func_result = func(i)
            delta = self.arcode_manager.compare_seen_mark_transform(realign=True)
            if delta:
                result = {
                    "attempt": i,
                    "time": time.strftime("%x %X"),
                    "error_x": delta.x,
                    "error_y": delta.y,
                    "error_theta": delta.theta,
                }
                if func_result:
                    # Assume it's a dictionary
                    result.update(func_result)
                yield result
            else:
                print "Warning: didn't see Arucode."

    @stk.logging.log_exceptions
    def multi_test_aruco(self, func, num_attempts):
        "general idea: look at code, do something, see if our pos changed."
        if not self.arcode_manager.init_home_pos():
            self.logger.info("No arucode in front of Pepper, abort!")
            return
        for i in range(num_attempts):
            func(i)
            delta = self.arcode_manager.compare_seen_mark_transform()
            if delta:
                self.log_row(i, delta.x, delta.y, delta.theta)
            else:
                self.logger.info("failed to compute delta.")

    ####################################################
    # Life cycle

    def run_scenario(self, scenario_class, num_attempts, **kwargs):
        "Run scenario and print the results to the log"
        scenario = scenario_class(self, **kwargs)
        self.multi_test_aruco(scenario.run, num_attempts)

    def get_folder(self):
        "determines the right data folder and makes sure it exists."
        if stk.runner.is_on_robot():
            folder = "/home/nao/homefinderdata/"
        else:
            folder = "../../stats/data/"
        if not os.path.exists(folder):
            os.makedirs(folder)
            print "created folder:" + folder
        return folder

    def measure_to_file(self, scenario_class, num_attempts, **kwargs):
        "Run scenario and save results to a CSV file"
        print "measure to file:", scenario_class.__name__
        folder = self.get_folder()
        filename = "%s_%s.csv" % (scenario_class.__name__,
                                  time.strftime("%Y-%m-%d-%H:%M:%S"))
        filepath = os.path.join(folder, filename)
        data = []
        scenario = scenario_class(self, **kwargs)
        for result in self._iter_attempt_results(scenario.run, num_attempts):
            print result
            data.append(result)
        save_dicts_to_csv(data, filepath)
        print "Finished writing to %s" % filename

    def on_start(self):
        "Callback on start - run measures"
        #self.run_scenario(MoveScenario, 5, move_x=0.1)
        #self.run_scenario(DummyScenario, 2)
        #self.measure_to_file(MoveScenario, 30, move_theta=math.pi / 4)
        #self.measure_to_file(SlamScenario, 2, move_y=-1.0, move_theta=math.pi / 4)
        #self.run_scenario(BumperScenario, 15)
        #self.measure_to_file(DummyScenario, 2)
        #self.arcode_manager.turn_towards_aruco()
        # More serious strategies
        #self.run_scenario(ColinMaillardScenario, 2, strategy="odometry", advance=1.0, move_theta=0.0)
        #self.measure_to_file(ColinMaillardScenario, 18, strategy="odometry",
        #                     advance=[0.0, 2.5], steps=6, move_theta=0.0)
        self.measure_to_file(ColinMaillardScenario, 18, strategy="odometry",
                             direction=0.0,
                             advance=0.0, steps=9, move_theta=[0.0, 2 * math.pi])
        self.logger.info("measures done.")
        self.qiapp.stop()

    def on_stop(self):
        "Cleanup (add yours if needed)"
        self.logger.info("MovementMeasures finished.")


if __name__ == "__main__":
    stk.runner.run_activity(MovementMeasurer)
