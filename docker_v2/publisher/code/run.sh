#!/bin/bash 

#lancement de mes sv_publisher

 docker run -i  -v  /home/youssef/Documents/youssef_C/docker_v2/publisher/logdocker/sv1:/log --cap-add sys_nice --cap-add net_admin   --net macvlan_net   sv_1:latest &


