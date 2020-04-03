FROM ubuntu
MAINTAINER Youssef Mounkaila < youssefmounkaila [ at ] gmail.com >

ADD sv_publisher /root
ADD run_sv_publisher.sh /root

RUN mkdir /log 


WORKDIR /root

RUN apt-get update
RUN apt-get install net-tools
#droits pour les dossier 


RUN chmod +x /root/run_sv_publisher.sh

CMD  /root/run_sv_publisher.sh

