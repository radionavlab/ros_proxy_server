#!/usr/bin/python

import SocketServer, time
from multiprocessing import Process, Pipe
import json
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy import interpolate

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf

from app_pathplanner_interface.msg import PVATrajectory
from app_pathplanner_interface.msg import PVA_Stamped

class Point(): 
    '''
    A point has x, y, z, and t components
    '''
    def __init__(self, data):
        self.__dict__ = data
        
        assert 'x' in self.__dict__
        assert 'y' in self.__dict__
        assert 'z' in self.__dict__
        assert 'w' in self.__dict__
        assert 't' in self.__dict__

    def __str__(self):
        return json.dumps(self, default=lambda x: x.__dict__)


class Trajectory():
    '''
    A trajectory has a name and a list of points
    '''
    def __init__(self, json_data):
        self.__dict__ = json.loads(json_data)

        assert 'name' in self.__dict__
        assert 'points' in self.__dict__

        self.points = list(map(lambda p: Point(p), self.points))

    def __str__(self):
        return json.dumps(self, default=lambda x: x.__dict__)


class ProxyServerHandler(SocketServer.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            json_data = line.strip()
            trajectory = Trajectory(json_data)
            self.server.server_pipe.send(trajectory)

    def finish(self):
        self.request.close()

class ProxyServer(SocketServer.ForkingTCPServer, object):
    def __init__(self, server_info, handler, server_pipe):
        super(ProxyServer, self).__init__(server_info, handler)
        self.server_pipe = server_pipe


def create_pva_trajectory(trajectory):
    # Create new trajectory message
    trajectory_msg = PVATrajectory()
    trajectory_msg.pva = []

    # Break apart the points into np arrays
    x = np.array([point.x for point in trajectory.points])
    y = np.array([point.y for point in trajectory.points])
    z = np.array([point.z for point in trajectory.points])
    w = np.array([point.w for point in trajectory.points])
    t = np.array([(point.t - trajectory.points[0].t) / 1000.0 for point in trajectory.points])

    initial_time = t[0]
    final_time = t[-1]

    # S changes smooting
    # K is degree of spline curve
    tck, u = interpolate.splprep([x, y, z, w], s=0.5, k=5, u=t)
    sample_time_interval = 0.05
    # Be careful sampling at the endpoints [0, tf] because the spline is undefined
    time_samples = np.arange(2*sample_time_interval, final_time-2*sample_time_interval, sample_time_interval)
    pos_samples = interpolate.splev(time_samples, tck, der=0)
    vel_samples = interpolate.splev(time_samples, tck, der=1)
    acc_samples = interpolate.splev(time_samples, tck, der=2)

    # Fill in the header info
    trajectory_header       = std_msgs.msg.Header()
    trajectory_header.stamp = rospy.get_rostime()
    trajectory_msg.header   = trajectory_header

    # Create point messages
    for i in range(len(time_samples)):
        # Create PVA point
        point_msg = PVA_Stamped()
 
        # Create header
        point_header       = std_msgs.msg.Header()
        point_header.stamp = rospy.Time.from_sec(time_samples[i])
        point_msg.header   = point_header
 
        # Create pose message
        pos_msg = geometry_msgs.msg.Pose()
        # pos_msg.position.x, pos_msg.position.y, pos_msg.position.z = pos_samples[0][i], pos_samples[1][i], pos_samples[2][i]
        # Fucking coordinate systems
        pos_msg.position.x, pos_msg.position.y, pos_msg.position.z = pos_samples[1][i], -pos_samples[0][i], pos_samples[2][i]
        yaw_android = pos_samples[3][i]
        yaw = math.atan2(-math.cos(yaw_android), -math.sin(yaw_android))
        print yaw
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w  = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        point_msg.pos = pos_msg
 
        # Create velocity message
        vel_msg = geometry_msgs.msg.Twist()
        vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z = vel_samples[0][i], vel_samples[1][i], vel_samples[2][i]
        vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z = 0, 0, 0 # Assuming no change in angles
        point_msg.vel = vel_msg
 
        # Create acceleration message
        acc_msg = geometry_msgs.msg.Accel()
        acc_msg.linear.x, acc_msg.linear.y, acc_msg.linear.z = acc_samples[0][i], acc_samples[1][i], acc_samples[2][i]
        acc_msg.angular.x, acc_msg.angular.y, acc_msg.angular.z = 0, 0, 0 # Assuming no change in angles
        point_msg.acc = acc_msg
 
        # Append the PVA Point to the PVA Trajectory
        trajectory_msg.pva.append(point_msg)

    # Plotting
    plt.figure()

    # Position subplot
    plt.subplot(321)
    plt.title('Position (m)')
    plt.plot(pos_samples[0], pos_samples[1], x, y)
    axes = plt.gca()
    axes.set_xlim([-2.5, 2.5])
    axes.set_ylim([0,5])
    plt.grid(True)

    plt.subplot(322)
    plt.title('Altitude (m)')
    plt.plot(pos_samples[2])
    axes = plt.gca()
    axes.set_ylim([0,3])
    plt.grid(True)

    # Velocity Subplot
    plt.subplot(323)
    plt.title('Velocity (m/s)')
    plt.plot(vel_samples[0])
    plt.grid(True)

    plt.subplot(324)
    plt.title('Velocity (m/s)')
    plt.plot(vel_samples[1])
    plt.grid(True)

    # Acceleration subplot
    plt.subplot(325)
    plt.title('Acceleration (m/s^2)')
    plt.plot(acc_samples[0])
    plt.grid(True)

    plt.subplot(326)
    plt.title('Acceleration (m/s^2)')
    plt.plot(acc_samples[1])
    plt.grid(True)

    # plt.show()
 
    return trajectory_msg


def start_proxy_server(server_pipe):
    HOST, PORT = '', 8080
    server = ProxyServer((HOST, PORT), ProxyServerHandler, server_pipe)
    server.serve_forever()


def start_ros_node(ros_pipe):
    # Initialize the node
    rospy.init_node('trajectory_generator_proxy', anonymous=True)

    # Trajectory publisher
    PVA_publisher = rospy.Publisher('/phoenix/trajectory', PVATrajectory, queue_size=10)

    # Main loop
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # Print out everything in the pipe
        while True:
            if ros_pipe.poll() == True:
                trajectory = ros_pipe.recv()
                pva_trajectory = create_pva_trajectory(trajectory)
                PVA_publisher.publish(pva_trajectory)
            else:
                break

        rate.sleep()


def main():    
    # Communication pipe between server and ros node
    server_pipe, ros_pipe = Pipe()

    proxy_server_process = Process(target=start_proxy_server, args=(server_pipe,))
    proxy_server_process.start()

    ros_node_process = Process(target=start_ros_node, args=(ros_pipe,))
    ros_node_process.start()

    ros_node_process.join()
    proxy_server_process.join()


if __name__ == "__main__":
    main() 
