import os, platform
if platform.system() == "Linux" or platform.system() == "Darwin":
    os.environ["KIVY_VIDEO"] = "ffpyplayer"

from pysimbotlib.core import PySimbotApp, Robot
from kivy.config import Config

# Force the program to show user's log only for "info" level or more. The info log will be disabled.
Config.set('kivy', 'log_level', 'info')

import time

import random

import random
from pysimbotlib.core import Robot
from kivy.logger import Logger  # Kivy logging
    
class WalktoNearestFood(Robot):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.log_history = []
        self.stop_distance = 15

    def repeat(self, x, y):
        position = (x, y)
        self.log_history.append(position)

        if len(self.log_history) > 10:
            self.log_history.pop(0)

        repeated = sum(1 for p in self.log_history if p == position)

        if repeated >= 5:
            return True, "multi"
        elif repeated >= 3:
            return True, "minimal"
        else:
            return False, None

    def update(self):
        distances = self.distance()

        if len(self.log_history) > 100:
            self.log_history.pop(0)

        # log sensor checker
        # if R0 > self.stop_distance and min(distances) > self.stop_distance:
        #     self.move(100)
        # else:
        #     self.turn(45)

        # dont forget to set interval = 2
        # print(distances)
        # print("next")

        # results
        R0 = distances[0]      # Forward
        R0_45 = distances[1]   # Forward-right
        R0_90 = distances[2]   # Right
        R0_135 = distances[3]  # Back-right
        R0_180 = distances[4]  # Back
        R0_225 = distances[5]  # Back-left
        R0_270 = distances[6]  # Left
        R0_315 = distances[7]  # Forward-left

        front = R0
        fr = R0_45
        fl = R0_315

        x = round(self.pos[0], 2)
        y = round(self.pos[1], 2)

        print(self.pos)
        print(front)
        print(x, y)

        returner = self.repeat(x, y)

        is_repeat = returner[0]
        repeat_type = returner[1]
        # stuck handler
        if is_repeat:

            if repeat_type == "minimal":
                if fr < fl:
                    self.turn(-90)
                else:
                    self.turn(90)
                self.log_history.clear()
                return
            
            elif repeat_type == "multi":
                self.move(-10)
                if fr < fl:
                    self.turn(-115)
                else:
                    self.turn(115)
                self.log_history.clear()
                return

        # base code
        if front >= self.stop_distance and fl >= self.stop_distance and fr >= self.stop_distance:
            self.move(15)
            food_pos = self.smell_nearest()
            self.turn(food_pos /3)
            self.log_history.append(front)

        elif front >= self.stop_distance and (fr < self.stop_distance or fl < self.stop_distance):
            if fr < fl:
                self.turn(-15)
            else:
                self.turn(15)
            self.move(10)

        elif front < self.stop_distance:
            self.move(-5)
            if fr < fl:
                self.turn(-45)
            else:
                self.turn(45)

        else:
            if fr < fl:
                self.turn(-90)
            else:
                self.turn(90)


if __name__ == '__main__':
    app = PySimbotApp(robot_cls=WalktoNearestFood, num_robots=1, interval = 1/10000)
    app.run()