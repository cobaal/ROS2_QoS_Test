# ROS2_QoS_Test
1. Install

~~~
cd {workspace}/src
git clone https://github.com/cobaal/ROS2_QoS_Test.git

cd {workspace}
colcon build
colcon build --packages-select telecom_test_py 
source install/setup.bash
~~~

- colcon build error : setuotools
~~~
sudo apt install python3-pip
pip install setuptools==58.2.0
~~~

2. Usage (with SDP)

~~~
% PUBLISHER
ros2 run telecom_test_py pub_test_node 

% SUBSCRIBER
ros2 run telecom_test_py sub_test_node 
~~~

3. Usage (with Server-Client discovery)
~~~
% SERVER
fastdds discovery --server-id 0

% PUBLISHER
export ROS_DISCOVERY_SERVER={server_ip}:11811
ros2 run telecom_test_py pub_test_node --remap __node:=listener_discovery_server

% SUBSCRIBER
export ROS_DISCOVERY_SERVER={server_ip}:11811
ros2 run telecom_test_py sub_test_node --remap __node:=listener_discovery_server
~~~

4. Change link quality

~~~
sudo tc qdisc add dev [iface] root netem loss [0~100]%
sudo tc qdisc change dev [iface] root netem loss [0~100]%
sudo tc qdisc delete dev [iface] root netem loss [0~100]%
~~~

5. Change Heartbeat broadcast period

~~~
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE={path}/profiles.xml
~~~

6. Clock synchronization

~~~
sudo apt install ntp
sudo nano /etc/ntp.conf
~~~
- client
~~~
  # Use Ubuntu's ntp server as a fallback.
  # pool ntp.ubuntu.com
  server [master ip] iburst
~~~~
- local server
~~~
  server 127.127.1.0
~~~
- start
~~~
sudo systemctl restart ntp
watch -n 0.5 ntpq -p
sudo ufw allow ntp
~~~
