#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import tf

from velma_common.velma_interface import *
from geometry_msgs.msg import Point
from sv_effectors.srv import HeadAngles

def handle_moveHead(req):
    js = velma.getLastJointState()
    head_pan = js[1]["head_pan_joint"]
    head_tilt = js[1]["head_tilt_joint"]
    q_dest = (head_pan+req.alfa, head_tilt+req.beta)
    print q_dest
    velma.moveHead(q_dest, 0.1, start_time=0.1)
    velma.waitForHead()
    return 0

if __name__ == "__main__":

    rospy.init_node('ve_velma_head_srv', anonymous=True)

    rospy.sleep(1)

    velma = VelmaInterface("/velma_task_cs_ros_interface")
    
    print "waiting for init..."
    if not velma.waitForInit(timeout_s=20):
        print "could not initialize VelmaInterface"
        exit(1)
    print "init ok"

    print "sending head pan ENABLE command"
    velma.enableHP()
    velma.waitForHP()

    print "sending head pan START_HOMING command"
    velma.startHomingHP()
    velma.waitForHP()

    print "sending head tilt ENABLE command"
    velma.enableHT()
    velma.waitForHT()

    print "sending head tilt START_HOMING command"
    velma.startHomingHT()
    velma.waitForHT()

    print "moving head to position: 0"
    q_dest = (0,0)
    velma.moveHead(q_dest, 2.0, start_time=0.5)
    velma.waitForHead()

    print "Start"
    rospy.Service('moveHead', HeadAngles, handle_moveHead)
    rospy.spin()

