import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from telecom_interface.msg import Test

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy, qos_profile_system_default

import os

import random
import time

import threading
import sys
import tty
import termios


class PyPub(Node):
    def __init__(self, node_name, topic_name):
        super().__init__(node_name)

        self.node_name = node_name
        self.topic_name = topic_name

        self.history = [QoSHistoryPolicy.KEEP_LAST, QoSHistoryPolicy.KEEP_ALL]
        self.reliability = [QoSReliabilityPolicy.RELIABLE, QoSReliabilityPolicy.BEST_EFFORT]
        self.durability = [QoSDurabilityPolicy.TRANSIENT_LOCAL, QoSDurabilityPolicy.VOLATILE]
        self.liveliness = [QoSLivelinessPolicy.AUTOMATIC, QoSLivelinessPolicy.MANUAL_BY_TOPIC]

        self.history_idx = 0
        self.reliability_idx = 0
        self.durability_idx = 0
        self.liveliness_idx = 0   

        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,\
        depth=qos_profile_system_default.depth,\
        reliability=QoSReliabilityPolicy.RELIABLE,\
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,\
        liveliness=QoSLivelinessPolicy.AUTOMATIC,\
        deadline=qos_profile_system_default.deadline,\
        lifespan=qos_profile_system_default.lifespan,\
        liveliness_lease_duration=qos_profile_system_default.liveliness_lease_duration)

        self.print_status()

        key_press_thread = threading.Thread(target=self.detect_key_press)
        key_press_thread.daemon = True
        key_press_thread.start()        

    def timer_callback(self):
        msg = Test()
        msg.index = self.idx
        msg.data = random.random()
        msg.current_time = time.time()
        self.pub_.publish(msg)
        self.get_logger().info("publishing: %d, %f, %f " % (msg.index, msg.data, msg.current_time))
        self.idx += 1

    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def detect_key_press(self):
        while True:
            char = self.getch()

            if char == '1':
                self.history_idx = (self.history_idx + 1) % len(self.history)
                self.qos_profile.history = self.history[self.history_idx]   
                self.print_status()

            elif char == '2':
                self.reliability_idx = (self.reliability_idx + 1) % len(self.reliability)
                self.qos_profile.reliability = self.reliability[self.reliability_idx]   
                self.print_status()

            elif char == '3':
                self.durability_idx = (self.durability_idx + 1) % len(self.durability)
                self.qos_profile.durability = self.durability[self.durability_idx]   
                self.print_status() 

            elif char == '4':
                self.liveliness_idx = (self.liveliness_idx + 1) % len(self.liveliness)
                self.qos_profile.liveliness = self.liveliness[self.liveliness_idx]   
                self.print_status()  

            elif char == '5':
                self.qos_profile.depth = (self.qos_profile.depth + 1 ) % 11
                self.print_status()  

            elif char == '6':
                try:
                    sec = float(input("\n * Input lifespan length (sec) : "))
                    self.qos_profile.lifespan = rclpy.duration.Duration(seconds=sec)
                    self.print_status()  
                except:
                    print("\n * Invalid input.")

            elif char == '7':
                try:
                    sec = float(input("\n * Input deadline length (sec) : "))
                    self.qos_profile.deadline = rclpy.duration.Duration(seconds=sec)
                    self.print_status()  
                except:
                    print("\n * Invalid input.")

            elif char == '8':
                try:
                    sec = float(input("\n * Input liveliness_lease_duration length (sec) : "))
                    self.qos_profile.liveliness_lease_duration = rclpy.duration.Duration(seconds=sec)
                    self.print_status()  
                except:
                    print("\n * Invalid input.")

            elif char == 'q' or char == '\x03':
                print('* Terminated.')
                sys.exit(0)

            elif char == 's':
                self.pub_ = self.create_publisher(Test, self.topic_name, self.qos_profile)
                self.timer = self.create_timer(1.0, self.timer_callback)
                self.idx = 0
                print("\n Start publishing...")
                break

    def print_status(self):
        cl = os.system('cls' if os.name == 'nt' else 'clear')
        width = os.get_terminal_size().columns
        print('='*width, end='')
        print("\n{:^{}}".format('TOPIC PUBLISHER NODE', width))
        print('='*width, end='')
        print('\n * QoS Profile')
        print('   (1) history\t   : ' + str(self.qos_profile.history).split('.')[1])
        print('   (2) reliability : ' + str(self.qos_profile.reliability).split('.')[1])
        print('   (3) durability  : ' + str(self.qos_profile.durability).split('.')[1])
        print('   (4) liveliness  : ' + str(self.qos_profile.liveliness).split('.')[1])
        print('   (5) depth\t   : ' + str(self.qos_profile.depth))
        print('   (6) lifespan\t   : ' + str(int(str(self.qos_profile.lifespan).split(' ')[0]) / 1000000000) + ' sec')
        print('   (7) deadline\t   : ' + str(int(str(self.qos_profile.deadline).split(' ')[0]) / 1000000000) + ' sec')
        print('   (8) liveliness_lease_duration : ' + str(int(str(self.qos_profile.liveliness_lease_duration).split(' ')[0]) / 1000000000) + ' sec')

        print("\n [Press key '1~8' to set QoS profile.]")
        print(" [Press key 's' to start the Publisher node.]")
        print(" [Press key 'q' to quit.]")


def main(args=None):
    rclpy.init(args=args)

    node_name = "pub_test_node"
    topic_name = "test_topic"

    pub = PyPub(node_name, topic_name)

    try:
        rclpy.spin(pub)
    except KeyboardInterrupt:
        print("\n * Terminated.")

    # TODO : to shut down safely
    # pub.destroy_node()
    # rclpy.shutdown()

if __name__  == "__main__":
    main()