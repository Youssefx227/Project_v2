#!/bin/bash

#lancement subscribers

 docker run -i  -v  /home/youssef/Documents/youssef_C/docker_v2/subscriber/logdocker/sv1:/log --cap-add sys_nice --cap-add net_admin  --net macvlan_net   sv_s1:latest &
#sudo docker run -i  -v  /home/youssef/Documents/youssef_C/docker_v2/subscriber/logdocker/sv2:/log --cap-add sys_nice --cap-add net_admin   --net macvlan_net   sv_s2:latest &
#sudo docker run -i  -v  /home/youssef/Documents/youssef_C/docker_v2/subscriber/logdocker/sv3:/log --cap-add sys_nice --cap-add net_admin   --net macvlan_net  sv_s3:latest &
#sudo docker run -i  -v  /home/youssef/Documents/youssef_C/docker_v2/subscriber/logdocker/sv4:/log --cap-add sys_nice --cap-add net_admin   --net macvlan_net  sv_s4:latest &
#sudo docker run -i  -v  /home/youssef/Documents/youssef_C/docker_v2/subscriber/logdocker/sv5:/log --cap-add sys_nice --cap-add net_admin   --net macvlan_net  sv_s5:latest &

