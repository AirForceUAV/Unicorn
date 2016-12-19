#!/usr/bin/evn python
# coding:utf-8

import time
import json
from config import config
from library import CancelWatcher, list_assign
from library import get_distance_metres, angle_heading_target
from library import Singleton, _angle
from attribute import Attribute


class Vehicle(Attribute):
    __metaclass__ = Singleton

    def __init__(self, mcu=None, ORB=None):
        super(Vehicle, self).__init__(mcu, ORB)

    def arm(self):
        if self._frame is 'HELI':
            return 0
        print "Waiting for arming..."
        self.channels[self.AIL[0]] = self.AIL[1]
        self.channels[self.ELE[0]] = self.ELE[3]
        self.channels[self.THR[0]] = self.THR[3]
        self.channels[self.RUD[0]] = self.RUD[3]
        self.send_pwm()
        time.sleep(2)
        self.disarm()
        pass

    def disarm(self):
        self.channels[self.AIL[0]] = self.AIL[2]
        self.channels[self.ELE[0]] = self.ELE[2]
        self.channels[self.THR[0]] = self.THR[3]
        self.channels[self.RUD[0]] = self.RUD[2]
        self.send_pwm()
        pass

    # def stall(self):
    #     self._log("stall")
    #     self.channels[self.THR[0]]=self.THR[1]
    #     if self._frame is 'HELI':
    #         self.channels[self.PIT[0]]=self.PIT_curve(self.THR[1])
    #     self.send_pwm()

    def takeoff(self, alt=5):
        if config.get_Baro()[0] <= 0:
            print 'Baro is closed'
            return 0
        print 'Takeoff to ', alt, 'm'

        if self.THR[5] < 0:
            self.channels[self.THR[0]] = int(self.THR[3] - self.THR[4] * 0.6)
        else:
            self.channels[self.THR[0]] = int(self.THR[1] + self.THR[4] * 0.6)
        self.channels[self.PIT[0]] = self.update_PIT(
            self.channels[self.THR[0]])
        self.send_pwm()
        time.sleep(2)
        # watcher = CancelWatcher()
        # while not watcher.IsCancel():
        #     currentAlt = self.get_alt()
        #     print 'Current Altitude', currentAlt
        #     if currentAlt is None:
        #         self.brake()
        #         return -1
        #     if currentAlt > alt * 0.9:
        #         print 'Reached Altitude'
        #         break

        self.brake()
        pass

    def land(self):
        if config.get_Baro()[0] <= 0:
            print 'Baro is closed'
            return 0
        print 'Landing... '

        if self.THR[5] < 0:
            self.channels[self.THR[0]] = int(self.THR[3] - self.THR[4] * 0.4)
        else:
            self.channels[self.THR[0]] = int(self.THR[1] + self.THR[4] * 0.4)
        self.channels[self.PIT[0]] = self.update_PIT(
            self.channels[self.THR[0]])
        self.send_pwm()
        Watcher = CancelWatcher()
        time.sleep(3)
        # preAlt = self.get_alt()
        # times = 0
        # while not watcher.IsCancel():
        #     currentAlt = self.get_alt()
        #     print 'Current Altitude', currentAlt
        #     if currentAlt is None:
        #         self.brake()
        #         return -1

        #     if abs(currentAlt - preAlt) < 0.2:
        #         times += 1
        #     else:
        #         times = 0
        #     if times >= 5:
        #         break
        #     if currentAlt > alt * 0.9:
        #         print 'Reached Altitude'
        #         break
        if not watcher.IsCancel():
            self.disarm()

    def THR_MID(self):
        list_assign(self.channels, self.channels_mid)

    def movement(self, att, sign=1):
        rate = self.gear[self.get_gear()] / 100.0
        variation = int(att[5] * att[4] * rate)
        self.channels[att[0]] = att[2] + sign * variation
        return self.channels[att[0]]

    def movement2(self, att, sign=1):
        rate = att[6] / 100.0
        variation = int(att[5] * att[4] * rate)
        self.channels[att[0]] = att[2] + sign * variation
        return self.channels[att[0]]

    def yaw_left(self):
        self._log('Turn Left...')
        if config.get_MCU()[0] > 0:
            self.THR_MID()
            self.movement2(self.RUD, -1)
            self.send_pwm()

    def yaw_right(self):
        self._log('Turn Right...')
        if config.get_MCU()[0] > 0:
            self.THR_MID()
            self.movement2(self.RUD)
            self.send_pwm()

    def forward(self, duration=None):
        self._log('Forward...')
        if config.get_MCU()[0] > 0:
            self.THR_MID()
            self.movement(self.ELE)
            self.send_pwm()
            if duration is not None:
                time.sleep(duration)
                self.brake()

    def brake(self):
        self._log('brake')
        if config.get_MCU()[0] > 0:
            duration = self.BD[self.get_gear()]
            list_assign(self.channels, self.channels_mid)
            self.send_pwm()
            time.sleep(duration)

    def yaw_left_brake(self):
        duration = self.mDuration()
        self._log('Yaw Left')
        if config.get_MCU()[0] > 0:
            self.THR_MID()
            self.movement2(self.RUD, -1)
            self.send_pwm()
            time.sleep(duration)
            self.brake()

    def yaw_right_brake(self):
        duration = self.mDuration()
        self._log('Yaw Right')
        if config.get_MCU()[0] > 0:
            self.THR_MID()
            self.movement2(self.RUD)
            self.send_pwm()
            time.sleep(duration)
            self.brake()

    def forward_brake(self):
        duration = self.mDuration()
        self._log('Forward')
        if config.get_MCU()[0] > 0:
            self.THR_MID()
            self.movement(self.ELE)
            self.send_pwm()
            time.sleep(duration)
            self.brake()

    def backward_brake(self):
        duration = self.mDuration()
        self._log('Backward')
        if config.get_MCU()[0] > 0:
            self.THR_MID()
            self.movement(self.ELE, -1)
            self.send_pwm()
            time.sleep(duration)
            self.brake()

    def roll_left_brake(self):
        duration = self.mDuration()
        self._log('Roll Left')
        if config.get_MCU()[0] > 0:
            self.THR_MID()
            self.movement(self.AIL, -1)
            self.send_pwm()
            time.sleep(duration)
            self.brake()

    def roll_right_brake(self):
        duration = self.mDuration()
        self._log('Roll Right')
        if config.get_MCU()[0] > 0:
            self.THR_MID()
            self.movement(self.AIL)
            self.send_pwm()
            time.sleep(duration)
            self.brake()

    def up_brake(self):
        duration = self.mDuration()
        self._log('Throttle Up')
        if config.get_MCU()[0] > 0:
            self.THR_MID()
            pwm = self.movement2(self.THR)
            self.channels[self.PIT[0]] = self.update_PIT(pwm)
            self.send_pwm()
            time.sleep(duration)
            self.brake()

    def down_brake(self):
        duration = self.mDuration()
        self._log('Throttle Down')
        if config.get_MCU()[0] > 0:
            self.THR_MID()
            pwm = self.movement2(self.THR, -1)
            self.channels[self.PIT[0]] = self.update_PIT(pwm)
            self.send_pwm()
            time.sleep(duration)
            self.brake()

    def mDuration(self):
        return self.MD

    def send_pwm(self):
        if not config.get_MCU()[0] > 0 or self.mcu is None:
            return 0
        self.mcu.send_pwm(self.channels)

    def isStop(self, origin, target, sign):
        diff = (360 + sign * (target - origin)) % 360
        if diff < 180 and diff > 5:
            return True
        else:
            return False

    def condition_yaw(self, heading=0, relative=True):
        """
        0<=heading<360
        """
        if heading < 0 or heading >= 360:
            self._log('0<=heading<360')
            return -1
        if relative and heading == 0:
            return 0

        current_heading = self.get_heading()
        watcher = CancelWatcher()
        # Relative angle to heading
        if relative:
            target_angle = (current_heading + heading) % 360
        else:
            # Absolute angle
            target_angle = heading

        direction = (360 + target_angle - current_heading) % 360
        if direction > 0 and direction < 180:
            is_cw = 1
            self._log('Turn right {}'.format(direction))
            self.yaw_right()
        elif direction >= 180 and direction < 360:
            is_cw = -1
            self._log('Turn left {}'.format(360 - direction))
            self.yaw_left()
        print "Target", target_angle
        while not watcher.IsCancel() and self.isStop(
                self.get_heading(), target_angle, is_cw):
            # self._log('Cur angle:{},Target
            # angle:{}'.format(self.get_heading(),target_angle))
            time.sleep(.01)
        self._log('Before Angle {}'.format(self.get_heading()))
        self.brake()
        self._log('After  Angle {}'.format(self.get_heading()))
        return 1

    def navigation(self, target):
        deviation = config.get_degree()[0]
        checktime = config.get_DD()[0]
        watcher = CancelWatcher()
        while not watcher.IsCancel():
            current_location = self.get_location()
            if current_location is None:
                self.brake()
                return -1
            distance = round(get_distance_metres(current_location, target), 2)
            self._log("Distance to Target {}m".format(distance))
            if distance < 3:
                self._log("Reached Target Waypoint!")
                break
                return 1
            angle = angle_heading_target(
                current_location, target, self.get_heading())
            if _angle(angle) > deviation:
                self.brake()
                self.condition_yaw(angle)
            self.forward()
            time.sleep(checktime)
        self.brake()
        return 0

    def RTL(self):
        target = self.get_home()
        if target is None:
            self._log("Home is None!")
            self.brake()
            return -1
        self.publish('Mode', 'RTL')

        self.navigation(target)
        # self.land()
        self.publish('Mode', 'Loiter')

    def Route(self, info):
        self.wp.Route(info)
        # self.Auto()

    def Auto(self):
        if self.wp.isNull():
            self._log('Waypoint is None')
            return -1
        self.publish('Mode', 'Auto')
        watcher = CancelWatcher()
        for point in self.wp.remain_wp():
            if watcher.IsCancel():
                self.publish('Mode', 'Loiter')
                return 0
            self.navigation(point)
            self.wp.add_number()

        self.publish('Mode', 'Loiter')
        self.wp.clear()

    def Guided(self):
        target = self.get_target()
        if target is None:
            self._log("Target is None!")
            self.brake()
            return -1
        self.publish('Mode', 'GUIDED')

        self.navigation(target)
        self.publish('Mode', 'Loiter')
        self.publish('Target', None)

    def Cancel(self):
        self._log("Cancel")
        CancelWatcher.Cancel = True
        time.sleep(1)
        self.brake()

if __name__ == "__main__":
    from uORB import uORB
    from library import Watcher
    mcu = None
    Watcher()
    ORB = uORB()
    # instancce of MCU module object
    if config.get_MCU()[0] > 0:
        from MCU_module import MCU
        mcu = MCU()

    if config.get_compass()[0] > 0:
        # instancce of compass module object
        from compass_module import Compass
        compass = Compass(ORB)

        compass.start()
        while not ORB.subscribe('Compass_State'):
            # print compass.get_heading()
            time.sleep(.5)

    if config.get_GPS()[0] > 0:
        from GPS_module import GPS      # instancce of GPS module object
        gps = GPS(ORB)

        gps.start()
        while not ORB.subscribe('GPS_State'):
            # print gps.get_num_stars()
            time.sleep(.5)

    if config.get_Baro()[0] > 0:
        from Baro import Baro
        baro = Baro(ORB)

        baro.start()
        while not ORB.subscribe('Baro_State'):
            # print gps.get_num_stars()
            time.sleep(.5)

    vehicle = Vehicle(mcu, ORB)
    # vehicle.arm()
    # vehicle.set_channels_mid()
    # vehicle.disarm()
    # vehicle.set_gear(2)
    # vehicle.takeoff(3)
    vehicle.yaw_left_brake()
    time.sleep(1)
    vehicle.yaw_right_brake()
    time.sleep(1)
    vehicle.roll_left_brake()
    time.sleep(1)
    vehicle.roll_right_brake()
    time.sleep(1)
    vehicle.forward_brake()
    time.sleep(1)
    vehicle.backward_brake()
    time.sleep(1)
    vehicle.up_brake()
    time.sleep(1)
    vehicle.down_brake()
    time.sleep(1)
    # vehicle.yaw_left()
    # vehicle.yaw_right()
    # vehicle.forward(5)

    # vehicle.condition_yaw(30)
    # vehicle.condition_yaw(300)

    # vehicle.set_target(0, 10)
    # vehicle.download()
    # print ORB.subscribe('Waypoint')
    # # vehicle.Auto()
    # vehicle.Guided()
    # vehicle.disarm()
