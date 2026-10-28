#ROS environment setup
source /opt/ros/jazzy/setup.zsh
export ROS_DOMAIN_ID=0

# Uncomment to use cycloneDDS as middleware
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Function for defining yourself as fastrtps client in case of connection issues
function fastrtps_client_setup(){
	export ROS_DISCOVERY_SERVER=127.0.0.1:11811
	export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/terraspiro-testing/fast_rtps_supercl_cfg_file.xml
	# restart daemon to apply changes
	ros2 daemon stop && ros2 daemon start;
}
