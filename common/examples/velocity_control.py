#!/usr/bin/env python

from __future__ import print_function
from generic_control import pveControl
import time

if __name__ == '__main__':
    my_tester_obj = pveControl()
    time.sleep(1)
    print('\n\nSending command now\n\n')
    my_tester_obj.test_velocity_control()
    my_tester_obj.sim_for_x_secs(3)
    my_tester_obj.disconnect()
