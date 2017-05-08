#!/usr/bin/python
#
# This file is part of IvPID.
# Copyright (C) 2015 Ivmech Mechatronics Ltd. <bilgi@ivmech.com>
#
# IvPID is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# IvPID is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# title           :test_pid.py
# description     :python pid controller test
# author          :Caner Durmusoglu
# date            :20151218
# version         :0.1
# notes           :
# python_version  :2.7
# dependencies    : matplotlib, numpy, scipy
#==============================================================================
import sys
sys.path.append('..')
from PID_Source.PID import PID
import time

import numpy as np
# from scipy.interpolate import spline
from lib.science import direction
from lib.config import config


class PIDC:

    def __init__(self):
        direct=config.direction

        self.P = direct['P']
        self.I = direct['I']
        self.D = direct['D']
        self.error = direct['error']   
        self.max_output = direct['maxoutput']         
        self.min = direct['min']
        self.i = 1
        self.output_max = 0  
        self.feedback_list = []
        self.time_list = []
        self.setpoint_list = []
        self.output_uni_list = []

    def yaw_pid(self, feedback, Target, origin):
        # def yaw_pid(self, feedback, Target, Decide):
        """Self-test PID class

        .. note::
            ...
            for i in range(1, END):
                pid.update(feedback)
                output = pid.output
                if pid.SetPoint > 0:
                    feedback += (output - (1/i))
                if i>9:
                    pid.SetPoint = 1
                time.sleep(0.02)
            ---
        """
        pid = PID(self.P, self.I, self.D)

        # pid.abs_angle = abs_angle
        # pid.origin = origin

        # pid.SetPoint = 0.0
        # pid.StartPoint = 0.0   # start ampling value ex:5
        pid.SetPoint = Target   # set a value ex:30
        # pid.SetPoint -= pid.StartPoint

        # pid.decide = Decide

        pid.setSampleTime(0.01)

        # if comp > 1.1 * pid.SetPoint:     # overshoot stop # for test
        #     output_uni = 0
        #     print('The feedback signal is overshoot')
        #     break

        # # no control effect with in 2 degree error
        if abs(feedback - pid.SetPoint) < self.error:
            output_uni = 0

        else:
            pid.update(feedback)
            # print 'i', self.i

            if self.i == 1:                  # when i exist # un while sentense
                # print 'output', pid.output
                self.output_max = pid.output


            output = pid.output

            direct = direction(origin, pid.SetPoint)
            if direct == 1:
                output = -1 * output
                        
        

            # uniformization # max position is 50
            # print self.output_max
            output_uni = self.max_output * output / self.output_max
            # time.sleep(0.02)

        if output_uni >= -1 * self.min and output_uni < 0:
            output_uni = -1 * self.min
        elif output_uni <= self.min and output_uni > 0:
            output_uni = self.min


        self.feedback_list.append(feedback)
        self.setpoint_list.append(pid.SetPoint)
        self.output_uni_list.append(output_uni)
        self.time_list.append(self.i)
        self.i += 1
        
        # print feedback, round(output_uni, 3)

        # print('feedback:%f  output:%f output_uni:%f  setpoint:%f' %
        #       (feedback, output, output_uni, pid.SetPoint))
        # print round(output_uni, 2)
        return round(output_uni, 3)

    def show(self, L=100):
        import matplotlib.pyplot as plt
        time_sm = np.array(self.time_list)
        time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)

        feedback_smooth = spline(
            self.time_list, self.feedback_list, time_smooth)
        # plt.plot(time_smooth, feedback_smooth)
        plt.plot(self.time_list, self.feedback_list)

        plt.plot(self.time_list, self.output_uni_list)

        plt.plot(self.time_list, self.setpoint_list)

        plt.xlim((0, L))
        plt.ylim((min(self.feedback_list) - 0.5, max(self.feedback_list) + 0.5))

        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID')

        plt.ylim((0, 180))

        plt.grid(True)
        plt.show()

if __name__ == "__main__":

    p = PIDC()

    # p.yaw_pid(0, 90,0)

    # p.yaw_pid(10, 90,0)
    # p.yaw_pid(20, 90,0)
    # p.yaw_pid(30, 90,0)
    # p.show()
