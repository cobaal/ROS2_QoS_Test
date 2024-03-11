import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from telecom_interface.msg import Test, TestCustom

from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy, qos_profile_system_default
from rclpy.qos_event import QoSEventHandler, SubscriptionEventCallbacks

import os

import random
import time
import datetime

import threading
import sys
import tty
import termios

import csv

import socket
import pickle
import struct

import ipaddress

class PySub(Node):
    def __init__(self, node_name, topic_name):
        super().__init__(node_name)

        self.node_name = node_name
        self.topic_name = topic_name

        self.history = [QoSHistoryPolicy.KEEP_LAST, QoSHistoryPolicy.KEEP_ALL]
        self.reliability = [QoSReliabilityPolicy.RELIABLE, QoSReliabilityPolicy.BEST_EFFORT]
        self.durability = [QoSDurabilityPolicy.TRANSIENT_LOCAL, QoSDurabilityPolicy.VOLATILE]
        self.liveliness = [QoSLivelinessPolicy.AUTOMATIC, QoSLivelinessPolicy.MANUAL_BY_TOPIC]

        self.msg_size = 0

        self.history_idx = 0
        self.reliability_idx = 0
        self.durability_idx = 0
        self.liveliness_idx = 0   

        self.savedData = []

        self.npkt = 1000

        self.qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,\
        depth=qos_profile_system_default.depth,\
        reliability=QoSReliabilityPolicy.RELIABLE,\
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,\
        liveliness=QoSLivelinessPolicy.AUTOMATIC,\
        deadline=qos_profile_system_default.deadline,\
        lifespan=qos_profile_system_default.lifespan,\
        liveliness_lease_duration=qos_profile_system_default.liveliness_lease_duration)

        self.event_callbacks = SubscriptionEventCallbacks(
            deadline=self.deadline_callback
        )

        self.print_status()

        key_press_thread = threading.Thread(target=self.detect_key_press)
        key_press_thread.daemon = True
        key_press_thread.start() 

    def listener_callback(self, msg):
        receivedTime = time.time()
        if self.msg_size == 0:
            _len = 8
        else:
            _len = len(msg.custom_data)
        self.get_logger().info("received msg [idx: %d, data length: %d, time: %f] received [time: %f, delay %f s]" % (msg.index, _len, msg.current_time, receivedTime, receivedTime-msg.current_time))
        self.savedData.append((msg.index, _len, msg.current_time, receivedTime))

    def testbed_listener_callback(self, msg):
        receivedTime = time.time()
        self.get_logger().info("received msg [idx: %d, testcase: %f, time: %f] received [time: %f, delay %f s]" % (msg.index, msg.data, msg.current_time, receivedTime, receivedTime-msg.current_time))
        self.savedData.append((msg.index, msg.data, msg.current_time, receivedTime, receivedTime - msg.current_time))

        if msg.index == self.npkt - 1:
            ack_msg = Int32()
            ack_msg.data = 1
            self.pub_ack.publish(ack_msg)

    def recvall(self, sock, n):
        data = bytearray()
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return bytes(data)

    def tcp_server(self, host):
        port = 1700

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) # disable Nagle's alogrithm
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        server_socket.bind((host, port))
        server_socket.listen(1)
        
        print(f"\n Listening on {host}:{port}")
        conn, addr = server_socket.accept()

        print(f"Connection from {addr}")
        try:
            while True:
                raw_msglen = self.recvall(conn, 4)
                if not raw_msglen:
                    break

                receivedTime = time.time()
                msglen = struct.unpack('>I', raw_msglen)[0]
                deserialized_msg = self.recvall(conn, msglen)
                
                # Deserialize the data
                msg = pickle.loads(deserialized_msg)
                if self.msg_size == 0:
                    _len = 8
                else:
                    _len = len(msg.custom_data)
                self.get_logger().info("received msg [idx: %d, data length: %d, time: %f] received [time: %f, delay %f s]" % (msg.index, _len, msg.current_time, receivedTime, receivedTime-msg.current_time))
                self.savedData.append((msg.index, msg.current_time, receivedTime, receivedTime - msg.current_time))

        except Exception as e:
            print(f"Connection failed: {e}")
        except KeyboardInterrupt:
            pass
        finally:
            conn.close()

    def deadline_callback(self, event):
        self.get_logger().info("Deadline missed!")

    def getch(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
        
    def msg_type(self, size):
        if size == 0:
            return Test
        else:
            return TestCustom

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
                try:
                    depth = int(input("\n * Input depth length : "))
                    self.qos_profile.depth = depth
                    self.print_status()  
                except:
                    print("\n * Invalid input.")

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

            elif char == '9':
                try:
                    m_size = int(input("\n * Input data size (Byte) : "))
                    self.msg_size = m_size
                    self.print_status()  
                except:
                    print("\n * Invalid input.")

            elif char == 'q' or char == '\x03':
                print('* Terminated.')
                sys.exit(0)

            elif char == 's':
                self.sub = self.create_subscription(self.msg_type(self.msg_size), self.topic_name, self.listener_callback, self.qos_profile, event_callbacks=self.event_callbacks)
                print("\n Start subscribing...")
                break

            elif char == 'x':
                self.sub = self.create_subscription(self.msg_type(self.msg_size), self.topic_name, self.testbed_listener_callback, self.qos_profile, event_callbacks=self.event_callbacks)
                print("\n Start subscribing...")

                self.pub_ack = self.create_publisher(Int32, 'ack_topic', self.qos_profile)
                break

            elif char == 't':
                host = input("\n *Enter the server IP address: ")
                try:
                    ipaddress.ip_address(host)
                    self.idx = 0
                    self.tcp_server(host)
                    break
                except ValueError:
                    print("\n * Invalid input.")

    def print_status(self):
        cl = os.system('cls' if os.name == 'nt' else 'clear')
        width = os.get_terminal_size().columns
        print('='*width, end='')
        print("\n{:^{}}".format('TOPIC SUBSCRIBER NODE 3.0', width))
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
        print('\n* Additional option')
        print('   (9) data_size [0 is 8 bytes]  : ' + str(self.msg_size) + ' byte(s)')
        
        print("\n [Press key '1~8' to set QoS profile.]")
        print(" [Press key '9' to set size of data.]")
        print(" [Press key 's' to start the Subscriber node.]")
        print(" [Press key 'x' to start experiment.]")
        print(" [Press key 't' to start TCP socket server.]")
        print(" [Press key 'q' to quit.]")

def main(args=None):
    rclpy.init(args=args)

    node_name = "sub_test_node"
    topic_name = "test_topic"

    sub = PySub(node_name, topic_name)

    try:
        rclpy.spin(sub)
    except KeyboardInterrupt:
        if sub.savedData:
            exit_input = input("\n\n * Do you want to save data? (Y/N): ").strip().upper()
            if exit_input != 'N':
                current_datetime = datetime.datetime.now()
                prefix_name = current_datetime.strftime("%Y-%m-%d_%H-%M-%S")
                filename = prefix_name + input(" * File name : " + prefix_name) + ".csv"
                with open(filename, 'w') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerows(sub.savedData)
                    print(" ** Saved to " + os.getcwd() + "/" + filename)

        print("\n * Terminated.")
        
    # sub.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()
