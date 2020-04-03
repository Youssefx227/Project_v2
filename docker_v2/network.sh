#!/bin/bash
sudo docker network create -d macvlan --subnet=10.10.0.0/16 --gateway=10.10.0.1 -o macvlan_mode=bridge -o parent=ens35u1c4i2 macvlan_net

