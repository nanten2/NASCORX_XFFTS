export ROS_IP=192.168.100.117
export ROS_MASTER_URI=http://192.168.100.183:11311
source /opt/ros/melodic/setup.bash
source /home/amigos/ros/devel/setup.bash
rosrun nascorx_xffts data_server.py
