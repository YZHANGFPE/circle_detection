#!/bin/bash
tmp=${ROS_MASTER_URI#*://}
ip=${tmp%:*}
echo $ip
sudo ntpdate $ip