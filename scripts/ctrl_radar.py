#!/usr/bin/env python

import rospy 
from marine_sensor_msgs.msg import RadarControlValue
import sys


if __name__ == '__main__':

    rospy.init_node('input_node')
    cmd_msg = RadarControlValue()
    pub = rospy.Publisher('/radar/HaloA/change_state', RadarControlValue, queue_size=10)  
    ######################################
    ### publish default configurations ###
    ######################################
    rospy.sleep(1)
    if len(sys.argv) > 1:
        cmd_msg.key = 'status'
        cmd_msg.value = sys.argv[1]
        pub.publish(cmd_msg)
        print('Transmitting...')
        rospy.sleep(0.5)
        cmd_msg.key = 'range'
        cmd_msg.value = sys.argv[2]
        pub.publish(cmd_msg)
        print 'Radar range: ', sys.argv[2]
    print "###############################"
    while not rospy.is_shutdown():
        key = raw_input(">> Enter 'key': " )

        ###########################################
        if key == 'status':
            print('Enter: transmit or standby')
            value = raw_input(">> Enter 'value': " )
            if value == 'transmit':
                cmd_msg.key = key
                cmd_msg.value = value
                pub.publish(cmd_msg)
                print('Transmitting...')
            elif value == 'standby':
                cmd_msg.key = key
                cmd_msg.value = value
                pub.publish(cmd_msg)
                print('Standby...')
        ###########################################
        if key == 'range':
            print('Enter the range in meter')
            value = raw_input(">> Enter 'value': " )
            if unicode(value, 'utf-8').isnumeric():
                cmd_msg.key = key
                cmd_msg.value = value
                pub.publish(cmd_msg)
                print 'New range: ', value
        ###########################################
        if key == 'sea_clutter':
            print('set to auto for best performance or a value between 0-100')
            value = raw_input(">> Enter 'value': " )
            if value == 'auto':
                cmd_msg.key = key
                cmd_msg.value = value
                pub.publish(cmd_msg)
                print 'sea_clutter is set to auto'

            elif unicode(value, 'utf-8').isnumeric():
                cmd_msg.key = key
                cmd_msg.value = value
                pub.publish(cmd_msg)
                print 'sea_clutter: ', value
        ###########################################
        if key == 'rain_clutter	':
            print('set to a value between 0-100')
            value = raw_input(">> Enter 'value': " )
            if unicode(value, 'utf-8').isnumeric():
                cmd_msg.key = key
                cmd_msg.value = value
                pub.publish(cmd_msg)
                print 'rain_clutter	: ', value
        ###########################################
        if key == 'noise_rejection	':
            print('set to 0 = "off", 1 = "low", 2 = "medium", 3 = "high"')
            value = raw_input(">> Enter 'value': " )
            if unicode(value, 'utf-8').isnumeric():
                cmd_msg.key = key
                cmd_msg.value = value
                pub.publish(cmd_msg)
                print 'noise_rejection	: ', value
        ###########################################
        if key == 'doppler_mode	':
            print('set to normal or approaching_only')
            value = raw_input(">> Enter 'value': " )
            if value=='normal' or value=='approaching_only':
                cmd_msg.key = key
                cmd_msg.value = value
                pub.publish(cmd_msg)
                print 'doppler_mode	: ', value

        