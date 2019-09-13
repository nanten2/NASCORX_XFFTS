export ROS_MASTER_URI=http://172.20.0.21:11311
export ROS_IP=172.20.0.65

# pyenv
# -----
export PYENV_ROOT="$HOME/.pyenv"
export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init -)"

# ros
# ---
source /opt/ros/kinetic/setup.bash
source $HOME/ros/devel/setup.bash

# python path
export PYTHONPATH="/home/amigos/NASCORX_System-master/:$PYTHONPATH"
#python data_server.py
#python data_server2.py
#python data_server3.py
python data_server5.py
