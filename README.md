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
sudo tc qdisc delete dev [iface] root netem loss [0~100]%
~~~

