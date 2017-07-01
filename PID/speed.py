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

#title           :test_pid.py
#description     :python pid controller test
#author          :Caner Durmusoglu
#date            :20151218
#version         :0.1
#notes           :
#python_version  :2.7
#dependencies    : matplotlib, numpy, scipy
#==============================================================================

import sys
sys.path.append('..')
from PID_Source.PID_speed import PID

import time
import numpy as np




class PIDC:

    def __init__(self, P = 0.2,  I = 0.0, D= 0.0):
        self.P = P
        self.I = I
        self.D = D
        self.error = 0


        self.i = 0
        self.feedback_list = []
        self.output_list = []
        self.setpoint_list = []
        self.time_list = []



    
    def speed_pid(self, feedback, target):

        pid = PID(self.P, self.I, self.D)

        pid.SetPoint = target  # 1.0

        pid.setSampleTime(0.01)








        if abs(feedback - pid.SetPoint) < self.error:
            output = 0   # equle former value

        else:
            pid.update(feedback)

            

            output = pid.output


        # print feedback, pid.SetPoint, output

       
        time.sleep(0.02)


        self.feedback_list.append(feedback)
        self.setpoint_list.append(pid.SetPoint)
        self.output_list.append(output)
        self.time_list.append(self.i)
        self.i += 1

        return output


    def show(self, L=100):
        import matplotlib.pyplot as plt
        from scipy.interpolate import spline
        time_sm = np.array(time_list)
        time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
        feedback_smooth = spline(time_list, feedback_list, time_smooth)

        plt.plot(time_smooth, feedback_smooth)
        plt.plot(time_list, setpoint_list)
        plt.xlim((0, L))
        plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID')

        plt.ylim((1-0.5, 1+0.5))

        plt.grid(True)
        plt.show()

if __name__ == "__main__":

    p = PIDC(P = 0.3)

    print p.speed_pid(0.2, 1.0, 0)
    


    # p = PIDC(P=0.3)

    # p.speed_pid(0, 90,0)