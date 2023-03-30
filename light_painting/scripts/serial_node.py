#!/usr/bin/env python3


#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rospy
from rosserial_python import SerialClient, RosSerialServer
from serial import SerialException
from time import sleep
import multiprocessing
import serial
import serial.tools.list_ports
import sys


if __name__ == "__main__":
   rospy.init_node("serial_node")
   UNO_VENDOR_ID = 0X2341
   rospy.loginfo("ROS Serial Python Node")
   port_name = None

   # Check for available serial ports and look for an Arduino with the specified vendor ID
   try:
       ports = list(serial.tools.list_ports.comports())
       for p in ports:
           rospy.loginfo(p.description)
           if p.vid == UNO_VENDOR_ID:
               port_name = p.device
               break
       if port_name is None:
           raise SerialException('No Arduino with specified vendor ID found')
   except Exception as e:
       rospy.logerr(str(e))
       sys.exit(1)


   baud = int(rospy.get_param('~baud', '57600'))


while not rospy.is_shutdown():
        if port_name is None:
            rospy.logwarn('No Arduino device found, retrying in 1 second...')
            sleep(1.0)
            continue

        rospy.loginfo("Connecting to %s at %d baud" % (port_name, baud))
        try:
            client = SerialClient(port_name, baud)
            client.port.open()  # Open the serial port
            client.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            sleep(1.0)
            continue
        except OSError:
            sleep(1.0)
            continue
        except:
            rospy.logwarn("Unexpected Error: %s", sys.exc_info()[0])
            client.port.close()
            sleep(1.0)
            continue
        finally:
            client.port.close()
