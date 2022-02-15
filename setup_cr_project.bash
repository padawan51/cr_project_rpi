#!/bin/bash

if [ "$USER" != "root" ]; then
	echo "Le mode root est requis"
else
	clear
	
	PI_BASHRC=/home/pi/.bashrc
	ROOT_BASHRC=/root/.bashrc
	NAME_RPI=$(hostname)
	WORKSPACE="/home/pi/ROS_WS2"
	WS_DIR="$WORKSPACE/climate_room_project"
	CURRENT_DIR=$(pwd)
	ROS_VERSION=$(rosversion -d)
	
	if [ 1 -eq 2 ]; then
	echo -n "Adresse IP du PC : "
	read IP_PC
	
	echo -n "Nom du PC : "
	read NAME_PC
	
	echo -n "Adresse IP du Raspberry Pi : "
	read IP_RPI
	
	# ________Modification du fichier /host/pi/.bashrc
	echo "####################" >> "$PI_BASHRC"
	echo source /opt/ros/$(rosversion -d)/setup.bash >> "$PI_BASHRC"
	echo >> "$PI_BASHRC"
	
	echo export ROS_MASTER_URI=http://"$IP_PC":11311 >> "$PI_BASHRC"
	echo export ROS_IP="$IP_RPI" >> "$PI_BASHRC"
	echo >> "$PI_BASHRC"
	
	echo alias rmuri=\'printenv ROS_MASTER_URI\' >> "$PI_BASHRC"
	echo alias stop=\'sudo shutdown -h now\' >> "$PI_BASHRC"
	echo alias root=\'"clear && sudo su"\' >> "$PI_BASHRC"
	echo alias src=\'source "$PI_BASHRC"\' >> "$PI_BASHRC"
	echo alias nano_bashrc=\'nano /home/pi/.bashrc\' >> "$PI_BASHRC"
	echo alias climate_room_dir=\'cd /home/pi/ROS_WS/climate_room_project\' >> "$PI_BASHRC"
	echo >> "$PI_BASHRC"
	# ________FIN de la modification du fichier /host/pi/.bashrc
	
	# ________Modification du fichier /root/.bashrc
	echo "####################" >> "$ROOT_BASHRC"
	echo source /opt/ros/$ROS_VERSION/setup.bash >> "$ROOT_BASHRC"
	echo >> "$ROOT_BASHRC"
	
	echo export ROS_MASTER_URI=http://"$IP_PC":11311 >> "$ROOT_BASHRC"
	echo export ROS_IP="$IP_RPI" >> "$ROOT_BASHRC"
	echo >> "$ROOT_BASHRC"
	
	echo cd /home/pi/ROS_WS/climate_room_project/ >> "$ROOT_BASHRC"
	echo source devel/setup.bash >> "$ROOT_BASHRC"
	echo >> "$ROOT_BASHRC"
	
	echo alias motor=\'"clear && rosrun project_node motor_mast_node"\' >> "$ROOT_BASHRC"
	echo alias sensor=\'"clear && rosrun project_node sensors_node"\' >> "$ROOT_BASHRC"
	echo alias rmuri=\'printenv ROS_MASTER_URI\' >> "$ROOT_BASHRC"
	echo alias stop=\'sudo shutdown -h now\' >> "$ROOT_BASHRC"
	echo alias src=\'source $ROOT_BASHRC\' >> "$ROOT_BASHRC"
	echo alias nano_bashrc=\'nano /root/.bashrc\' >> "$ROOT_BASHRC"
	echo alias climate_room_dir=\'cd /home/pi/ROS_WS/climate_room_project\' >> "$ROOT_BASHRC"
	echo alias climate_room_launch=\'roslaunch project_node climate_room_nodes.launch\' >> "$ROOT_BASHRC"
	echo >> "$ROOT_BASHRC"
	# ________Fin de la modification du fichier /root/.bashrc
	
	# ________Modification du fichier /etc/hosts
	echo "$IP_RPI	"    "$NAME_RPI" >> /etc/hosts
	echo "$IP_PC"			"$NAME_PC" >> /etc/hosts
	# ________FIN de la modification du fichier /etc/hosts
	fi
	
	# ________Installation de la bibliothèque pigpio
	if [ ! -d /home/pi/pigpio-master ]; then
		killall pigpiod
		cd /home/pi
		apt install python-setuptools python3-setuptools
		wget https://github.com/joan2937/pigpio/archive/master.zip
		unzip master.zip
		cd pigpio-master
		make
		sudo make install
	fi
	# ________FIN d'installation de la bibliothèque pigpio
	
	# ________Téléchargement du projet sur github
	if [ ! -d "$WORKSPACE" ] ; then
		mkdir "$WORKSPACE"
	else
		if [ -d "$WS_DIR" ]; then
			rm -r "$WS_DIR"/
		fi
	fi 
	
	#if [ 1 -eq 2 ]; then
	CR_GIT=cr_project_rpi
	cd "$WORKSPACE"
	
	git clone https://github.com/padawan51/"$CR_GIT".git
	mkdir -p "$WS_DIR"/src
	cd "$WS_DIR"/src
	catkin_init_workspace
	cd "$WS_DIR"
	catkin_make
	cd "$WS_DIR"/src
	#catkin_create_pkg custom_msgs roscpp
	#rm -r custom_msgs
	cp "$WORKSPACE"/"$CR_GIT"/custom_msgs/ "$WS_DIR"/src/custom_msgs/ -r
	cd "$WS_DIR"
	catkin_make
	#cd "$WS_DIR"/src
	#catkin_create_pkg project_node roscpp
	#rm -r project_node
	cp "$WORKSPACE"/"$CR_GIT"/project_node/ "$WS_DIR"/src/project_node/ -r
	#cd ../
	catkin_make
	chown -R pi:pi "$WORKSPACE"
	rm -r "$WORKSPACE"/"$CR_GIT"/
	#fi
	# ________FIN de téléchargement du projet sur github
fi