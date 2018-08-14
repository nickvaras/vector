#!/bin/bash

# VECTOR1 is ROS MASTER
#   Interface: is eth0
#   IP address is 10.66.171.2 


# A remote PC is ROS slave, the default interface is lo (change it to whatever physical interface is connected to sibot if working with real hardware), 
#    and the IP address is DHCP from VECTOR1 access point

if [ "$HOSTNAME" = vector1 ]; then
    export ROBOT_NETWORK=eth0
    export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')
    export ROS_MASTER_URI=http://vector1:11311/
else
    # This should be whatever physical interface is connected to the robot (ie wlan0, eth0, etc..)
    # we will try and find it and if nothing is found then we will default to lo
    robot_iface=$(ifconfig | awk '/10.66.171/ {print $1}' RS="\n\n")
    if [ ! -z "$robot_iface" ]
    then
        #We found an interface on the robot subnet so lets use this interface
        export ROBOT_NETWORK=$robot_iface
        export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')
        #now lets try to ping vector1 to see if it should be the master
        ping -q -c 1 -W 1 vector1 >/dev/null
        temp=$?
        if [ $temp -ne 0 ]; then
            #Could not find vector1 even though we are on the right network just make this PC master
            export ROS_MASTER_URI=http://$ROS_IP:11311/
        else
            #Found sibot1 so lets use vector1 and the master_uri 
            export ROS_MASTER_URI=http://vector1:11311/
        fi
    else
        #No interface on the vector1 network; default simulation settings
        export ROBOT_NETWORK=lo
        export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')
        export ROS_MASTER_URI=http://$ROS_IP:11311/            
    fi
fi

echo "ROS NETWORK is $ROBOT_NETWORK"
echo "ROS IP is $ROS_IP"
echo "ROS MASTER_URI is $ROS_MASTER_URI"
