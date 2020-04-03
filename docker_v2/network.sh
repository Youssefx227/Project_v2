#!/bin/bash
sudo docker network create -d macvlan --subnet=10.10.0.0/16 --gateway=10.10.0.1 -o macvlan_mode=bridge -o parent=eno1 macvlan_net

