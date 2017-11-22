#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import tf

from velma_common.velma_interface import *
from geometry_msgs.msg import Point, Vector3Stamped
from std_msgs.msg import Header
from sv_effectors.srv import HeadAngles

def handle_moveHead(req):
    js = velma.getLastJointState()
    head_pan = js[1]["head_pan_joint"]
    head_tilt = js[1]["head_tilt_joint"]
    q_dest = (head_pan+req.alfa, head_tilt+req.beta)
    #print q_dest
    
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    
    state = geometry_msgs.msg.Vector3Stamped()
    state.vector.x = head_pan
    state.vector.y = head_tilt
    state.header = header
    
    destination = geometry_msgs.msg.Vector3Stamped()
    destination.vector.x = head_pan+req.alfa
    destination.vector.y = head_tilt+req.beta
    destination.header = header
    
    publish_head_state.publish(state)
    publish_head_destination.publish(destination)
    
    velma.moveHead(q_dest, 0.02, start_time=0)
    
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
    
    publish_head_state = rospy.Publisher('head_state', Vector3Stamped, queue_size=1)
    publish_head_destination = rospy.Publisher('head_destination', Vector3Stamped, queue_size=1)
    
    rospy.spin()

