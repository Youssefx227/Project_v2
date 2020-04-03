# Project_v2
You need to rebuild the entire lib-iec61850 before starting to make or launch anything else.

Publisher function : used to publish on a specified interface in the script run_sv_publisher.sh
Start/stop/exit command : when you launch the program,terminal ask you to press the button "A", so that the publication of SV begins.

Fonction du Subscriber :  used to subscribe to a specific stream, configurable in the script run_sv_publisher.sh
the program identify the stream (APPID, MAC,Dataset)
At each stop a log file is generated in a directory (example : logfiles)
log file contains the time between the acquisition of two consecutive SVs and the jitter

You can use docker to launch these programs into containers. please see the docker_v2 directory



