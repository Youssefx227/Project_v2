FROM ubuntu
MAINTAINER Youssef Mounkaila < youssefmounkaila [ at ] gmail.com >

ADD sv_subscriber /root
ADD /run_sv_subscriber.sh /root

RUN mkdir /log 


WORKDIR /root

RUN apt-get update
RUN apt-get install net-tools
#droits pour les dossier 


RUN chmod +x /root/run_sv_subscriber.sh

CMD  /run_sv_subscriber.sh






