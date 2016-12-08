#!/usr/bin/env python

from irpos import *
import math

running = True
Kp = 0.8
Ki = 0.2
Kd = 0.0
max_speed = 0.05
max_speed_kw = max_speed * max_speed
min_speed = 0.001
max_acc = 0.001
x_int = 0.0
y_int = 0.0
z_int = 0.0
x_err_last = 0.0
y_err_last = 0.0
z_err_last = 0.0
Vx_old = 0.0
Vy_old = 0.0
Vz_old = 0.0


def callback(msg):
    global x_int
    global y_int
    global z_int
    global x_err_last
    global y_err_last
    global z_err_last
    global Vx_old
    global Vy_old
    global Vz_old
    str_msg = msg.data
    print str_msg
    str_msg = str_msg.replace("[", "").replace("]", "").replace(",", "")
    dx, dy, dz = str_msg.split()
    #print str(irpos.get_cartesian_pose())
    dx = float(dx)
    dy = float(dy)
    dz = float(dz)

    x_err = 0 - (dy)
    y_err = 0 + (dx)
    z_err = - 0.3 + (dz)
    x_int = 0.5 * x_int + x_err
    y_int = 0.5 * y_int + y_err
    z_int = 0.5 * z_int + z_err
    x_der = x_err - x_err_last
    y_der = y_err - y_err_last
    z_der = z_err - z_err_last
    x_err_last = x_err
    y_err_last = y_err
    z_err_last = z_err
    Vx = Kp * x_err + Ki * x_int + Kd * x_der
    Vy = Kp * y_err + Ki * y_int + Kd * y_der
    Vz = Kp * z_err + Ki * z_int + Kd * z_der

    open("log.csv", "a").write(str(x_err).replace('.', ',') + " " + str(y_err).replace('.', ',') + " " + str(z_err).replace('.', ',') + " ")
    open("log.csv", "a").write(str(Vx).replace('.', ',') + " " + str(Vy).replace('.', ',') + " " + str(Vz).replace('.', ',') + " ")

    S_cub = Vx * Vx + Vy * Vy + Vz * Vz
    if S_cub > max_speed_kw:
        pierw = math.sqrt(S_cub)
        Vx = max_speed * Vx / pierw
        Vy = max_speed * Vy / pierw
        Vz = max_speed * Vz / pierw

    if (dx == 0 and dy == 0 and dz == 0):
        Vx = 0
        Vy = 0
        Vz = 0

    open("log.csv", "a").write(str(Vx).replace('.', ',') + " " + str(Vy).replace('.', ',') + " " + str(Vz).replace('.', ',') + "\n")

    irpos.set_force_controller_goal(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(Vx, Vy, Vz), Vector3(0.0, 0.0, 0.0)))

    Vx_old = Vx
    Vy_old = Vy
    Vz_old = Vz

    # time.sleep(0.1)
    # irpos.stop_force_controller()


# MAIN
if __name__ == '__main__':

    half_pi = math.pi / 2
    irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
    irpos.move_to_joint_position([0, -half_pi, 0, 0, 3 * half_pi, -half_pi], 10.00)
    irpos.move_to_cartesian_pose(10.0, Pose(Point(0.93, 0.0, 1.145), Quaternion(0.0, 1.0, 0.0, 0.0)))
    print 'Start'
    irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156))
    irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(Vx, Vy, Vz), Vector3(0.0, 0.0, 0.0)))

    open("log.csv", "w").write("x_err y_err z_err Vx_org Vy_org Vz_org Vx_cub Vy_cub Vz_cub\n")

    #rospy.init_node('sv_system')
    rospy.Subscriber("/camera_postument/vr_camera_postument/postument_camera_object_position", String, callback, queue_size=1)
    rospy.spin()
