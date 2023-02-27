# ROS2_QoS_Test
ROS2_QoS_Test

~~~
cd {workspace}/src
git clone https://github.com/cobaal/ROS2_QoS_Test.git

cd {workspace}
colcon build
colcon build --packages-select telecom_test_py 
source install/setup.bash

ros2 run telecom_test_py pub_test_node 
ros2 run telecom_test_py sub_test_node 

sudo tc qdisc add dev [iface] root netem loss [0~100]%
sudo tc qdisc delete dev [iface] root netem loss [0~100]%
~~~

