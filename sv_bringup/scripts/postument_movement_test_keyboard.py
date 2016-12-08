#!/usr/bin/env python

from irpos import *
import os
import sys
import termios
import fcntl


def getch():
    fd = sys.stdin.fileno()

    oldterm = termios.tcgetattr(fd)
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)

    oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

    try:
        while 1:
            try:
                c = sys.stdin.read(1)
                break
            except IOError:
                pass
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)
    return c


def move_rel(irpos, time_from_start, rel_pose):
    print irpos.BCOLOR + "[IRPOS] Move relative to cartesian trajectory" + irpos.ENDC
    irpos.conmanSwitch([irpos.robot_name + 'mPoseInt'], [], True)

    actual_pose = irpos.get_cartesian_pose()

    # Transform poses to frames.
    actualFrame = pm.fromMsg(actual_pose)
    relativeFrame = pm.fromMsg(rel_pose)
    desiredFrame = actualFrame * relativeFrame
    pose = pm.toMsg(desiredFrame)

    cartesianGoal = CartesianTrajectoryGoal()
    cartesianGoal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(time_from_start), pose, Twist()))
    cartesianGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.1)
    irpos.pose_client.send_goal(cartesianGoal)


#MAIN
if __name__ == '__main__':

    half_pi = math.pi / 2
    irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
    irpos.move_to_joint_position([0, -half_pi, 0, 0, 3 * half_pi, -half_pi], 10.00)
    irpos.move_to_cartesian_pose(10.0, Pose(Point(0.93, 0.0, 1.145), Quaternion(0.0, 1.0, 0.0, 0.0)))
    print 'Start'

    irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156))

    speed = 0.01
    running = True

    while running:
        char = getch()
        char = ord(char)
        print char
        action = False

        if char == 113:  # q
            running = False
        elif char == 119:  # w
            irpos.stop_force_controller()
            rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 0.0, 0.0), PyKDL.Vector(0.1, 0.0, 0.0))
            move_rel(irpos, 5.0, pm.toMsg(rot))
        elif char == 115:  # s
            irpos.stop_force_controller()
            rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 0.0, 0.0), PyKDL.Vector(-0.1, 0.0, 0.0))
            move_rel(irpos, 5.0, pm.toMsg(rot))
        elif char == 97:  # a
            irpos.stop_force_controller()
            rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 0.0, 0.0), PyKDL.Vector(0.0, 0.1, 0.0))
            move_rel(irpos, 5.0, pm.toMsg(rot))
        elif char == 100:  # d
            irpos.stop_force_controller()
            rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 0.0, 0.0), PyKDL.Vector(0.0, -0.1, 0.0))
            move_rel(irpos, 5.0, pm.toMsg(rot))
        elif char == 10:  # backspace
            irpos.conmanSwitch([], [irpos.robot_name + 'mPoseInt'], True)
            irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
        elif char == 65:
            irpos.conmanSwitch([], [irpos.robot_name + 'mPoseInt'], True)
            irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(speed, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
        elif char == 66:
            irpos.conmanSwitch([], [irpos.robot_name + 'mPoseInt'], True)
            irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(-speed, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
        elif char == 67:
            irpos.conmanSwitch([], [irpos.robot_name + 'mPoseInt'], True)
            irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, speed, 0.0), Vector3(0.0, 0.0, 0.0)))
        elif char == 68:
            irpos.conmanSwitch([], [irpos.robot_name + 'mPoseInt'], True)
            irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, -speed, 0.0), Vector3(0.0, 0.0, 0.0)))

        print str(irpos.get_cartesian_pose())
