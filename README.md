# ssm_gazebo

how to add the path


ros complette desktop install

sudo apt-get install ros-noetic-ros-controll
sudo apt-get install ros-noetic-ros-controllers


matlab install 
with aerodynamic
with ros
with co-simulation with gazebo --> robotic toolbox

proto buffer install 


---> run script generate_simulink_connect.m in the ssm_plugin
run buildPlugin.sh in the generated ssm_simulink_connect


*environment*

. /usr/share/gazebo/setup.sh
export IGN_IP=127.0.0.1
export SVGA_VGPU10=0
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ssm_gazebo/src/ssm_models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/ssm_gazebo/devel/lib
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/ssm_gazebo/src/ssm_simulink_connect/export