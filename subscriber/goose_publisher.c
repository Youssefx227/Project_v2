/*
 * goose_publisher_example.c
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include <mms_value.h>
#include <goose_publisher.h>
#include <hal_thread.h>
#include <pthread.h>
/*
 * données partagées entre threads
*/
typedef struct data_{

    char* interface;
}data_;

void *goose_publish (void * donnees){

  /* appeler malloc pour demander de la mémoire
  si l'allocation a marché, notre pointeur contient une adresse
  L'allocation dynamique permet notamment de créer un entier/float/tableau dont
  la taille est déterminée par une variable au moment de l'exécution */
  data_* param = malloc(sizeof(data_));
  param = donnees;
  char* interface = malloc(sizeof(char));
   interface =(*param).interface;
  /* création d'une linkedlist*/
  LinkedList dataSetValues = LinkedList_create();
  /*ajout d'éléments*/
	LinkedList_add(dataSetValues,  MmsValue_newBoolean(true));

	CommParameters gooseCommParameters;

	gooseCommParameters.appId = 00;
	gooseCommParameters.dstAddress[0] = 0x01;
	gooseCommParameters.dstAddress[1] = 0x0c;
	gooseCommParameters.dstAddress[2] = 0xcd;
	gooseCommParameters.dstAddress[3] = 0x01;
	gooseCommParameters.dstAddress[4] = 0x00;
	gooseCommParameters.dstAddress[5] = 0x01;
	gooseCommParameters.vlanId = 0;
	gooseCommParameters.vlanPriority = 4;

  uint32_t timeAllowedToLive = 5;
  printf("creating goose publisher\n");
	/*
	 * Create a new GOOSE publisher instance. As the second parameter the interface
	 * name can be provided (e.g. "eth0" on a Linux system). If the second parameter
	 * is NULL the interface name as defined with CONFIG_ETHERNET_INTERFACE_ID in
	 * stack_config.h is used.
	 */
  GoosePublisher publisher = GoosePublisher_create(&gooseCommParameters, interface);

  GoosePublisher_setGoCbRef(publisher, "reference bloc de controle de goose");
  GoosePublisher_setTimeAllowedToLive(publisher,timeAllowedToLive);
  GoosePublisher_setDataSetRef(publisher, "Name of Dataset");
  GoosePublisher_setGoID(publisher, "IED sender identifier");
  GoosePublisher_setConfRev(publisher, 1);
  int j = 0,change_stnum;
 /* routine*/
	while(1){
		Thread_sleep(1000);

		if (GoosePublisher_publish(publisher, dataSetValues) == -1) {
			printf("Error sending message!\n");
		}
    j+=1;
    if(j>4){
    GoosePublisher_increaseStNum(publisher);
    LinkedList_add(dataSetValues,  MmsValue_newBoolean(false));
    change_stnum=0;
    j=0;
    }
	}

	GoosePublisher_destroy(publisher);
	LinkedList_destroyDeep(dataSetValues, (LinkedListValueDeleteFunction) MmsValue_delete);
  return 0;

}

// has to be executed as root!
int
main(int argc, char** argv)
{
    char* interface;

    if (argc > 1)

       interface = argv[1];

    else
       interface = "eth0";

    printf("Using interface %s\n", interface);

    pthread_t thread_goose_publish;
    /* création d'un bloc de données de thread de type data_ */
    data_ thread_data;
    thread_data.interface = interface;

    pthread_create(&thread_goose_publish,NULL,goose_publish,&thread_data);

    pthread_join(thread_goose_publish,NULL);

    return 0;
}
