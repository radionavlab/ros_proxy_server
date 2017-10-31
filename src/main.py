#!/usr/bin/python

import SocketServer, time
from multiprocessing import Process, Queue
import json
import rospy
from app_pathplanner_interface.msg import PVATrajectory

data_queue = Queue()

class MyClientHandler(SocketServer.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            json_data = line.strip()
            data=json.loads(json_data)
            data_queue.put(data)

    def finish(self):
        self.request.close()

def start_proxy_server():
    HOST, PORT = '', 8080
    server = SocketServer.ForkingTCPServer((HOST, PORT), MyClientHandler)
    server.serve_forever()

def start_ros_node():
    rospy.init_node('trajectory_generator_proxy', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print data_queue.qsize()
        rate.sleep()

def main():
    proxy_server_process = Process(target=start_proxy_server)
    proxy_server_process.start()

    ros_node_process = Process(target=start_ros_node)
    ros_node_process.start()

    ros_node_process.join()
    proxy_server_process.join()


if __name__ == "__main__":
    main() 
