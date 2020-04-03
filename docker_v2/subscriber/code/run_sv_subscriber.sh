#!/bin/bash

#lancement du subscriber 
# exécutable  #interface  #appid  #fichier_mesure_latence #durée de souscription [min]

./sv_subscriber eth0  0x4000 "/log/time_file.csv"   2

