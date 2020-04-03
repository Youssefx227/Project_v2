/* @Youssef Mounkaila Rte-France
	youssef.mounkaila@rte-france.com
 * sv_subscriber_example.c
 * programme pour l'acquistion de Sampled Values (SV) : subscriber
 */
#define _GNU_SOURCE
#include <iec61850_server.h>
#include <iec61850_model.h>
#include <sv_subscriber.h>
#include <hal_thread.h>
#include <static_model.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <pthread.h>
#include <mms_value.h>
#include <sched.h>
#include <pthread.h>
#include <goose_publisher.h>
#define _USE_MATH_DEFINES

#define gettid() syscall(__NR_gettid)

static bool running = true;
void sigint_handler(int signalId)
{
    running = 0;
}

#define SCHED_DEADLINE	6

 /*  utilisation des bons nombres pour l'appel système  */
 #ifdef __x86_64__
 #define __NR_sched_setattr	314
 #define __NR_sched_getattr	315
 #endif

 #ifdef __i386__
 #define __NR_sched_setattr	351
 #define __NR_sched_getattr	352
 #endif

 #ifdef __arm__
 #define __NR_sched_setattr	380
 #define __NR_sched_getattr	381
 #endif

 #define SUCCESS_MSG "Successfully set thread %lu to affinity to CPU %d\n"
 #define FAILURE_MSG "Failed to set thread %lu to affinity to CPU %d\n"


 #define print_error_then_terminate(en, msg) \
   do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)
 #define print_perror_then_terminate(msg) \
   do { perror(msg); exit(EXIT_FAILURE); } while (0)

long long int Ts_ms = 0;
int i =0;
struct timeval maintenant;
struct timeval debut_programme;
int position =0;
uint64_t TableauTimeStamp[4000*30];
int min=0,max=0;
/* ces champs sont utilisés pour le SCHED_DEADLINE */
 struct sched_attr {
    /* Taille de la structure */
	uint32_t size;
    /* Police (SCHED_*) */
	uint32_t sched_policy;
    /*flags*/
	uint64_t sched_flags;

	/* SCHED_NORMAL, SCHED_BATCH */
	uint32_t sched_nice; /* Valeur du nice (SCHED_OTHER,
SCHED_BATCH) */

/*Priorité statique (SCHED_FIFO,
SCHED_RR) */
	uint32_t sched_priority;

	/* SCHED_DEADLINE (nsec) */
	uint64_t sched_runtime;
	uint64_t sched_deadline;
	uint64_t sched_period;
 };
/*
 * fonction pour affecter les attributs
 */
 int sched_setattr(pid_t pid,
		  const struct sched_attr *attr,
		  unsigned int flags)
 {
	return syscall(__NR_sched_setattr, pid, attr, flags);
 }
/*
 * fonction pour récupérer les attributs
 */
 int sched_getattr(pid_t pid,
		  struct sched_attr *attr,
		  unsigned int size,
		  unsigned int flags)
 {
	return syscall(__NR_sched_getattr, pid, attr, size, flags);
 }


typedef struct data_{
   SVReceiver receiver;
   char* interface;
   uint16_t* appid;
}data_;

void min_max (long int* duree,int *min,int *max)
{
	long int val_min,val_max,i;
	val_min = duree[12000];
	val_max = duree[12000];
	int nb = 4000*20;
	for (i=12000; i<nb; i++)   // les valeurs de départs sont ignorées pour laisser le temps au scheduler deadline de s'exécuter convenablement
	{
		if (duree[i] < val_min)
		{
			val_min = duree[i];
		}
		else if (duree[i] > val_max)
		{
			val_max = duree[i];
		}
	}
	*min=val_min;
	*max=val_max;
}

/**
 *  la fonction quality prend en paramètre la qualité (entier) de la donnée reçu
 * elle renvoi une chaîne de caractère correpondant à la qualité
**/
char* quality (Quality Q){


    char* q = 0;
    switch(Q){

	  case  8192 :
		  q = "DERIVED";
		  break;
	  case  64 :
		  q = "DETAIL:FAILURE";
		  break;
	  case 16  :
		  q = "DETAIL:INACCURATE";
		  break;
	  case 512 :
		  q = "DETAIL:INACCURATE";
		  break;
	  case 256 :
		  q = "DETAIL:INCONSISTENT";
		  break;
	  case 128 :
		  q = "DETAIL:OLD_DATA";
		  break;
	  case 32:
		  q = "DETAIL:OSCILLATORY";
		  break;
	  case 8:
		  q = "DETAIL:OUT_OF_RANGE";
		  break;
	  case 4:
		  q = "DETAIL:OVERFLOW";
		  break;
	  case 4096 :
		  q = "OPERATOR_BLOCKED";
		  break;
	  case 1024 :
		  q = "SOURCE_SUBSTITUTED";
		  break;
	  case 2048 :
		  q = "TEST";
		  break;
	  case 0:
		  q = "VALIDITY:GOOD";
		  break;
	  case 2	:
		  q = "VALIDITY:INVALID";
		  break;
	  case 3	:
		  q = "VALIDITY:QUESTIONABLE";
		  break;
	  case 1	:
		  q = "VALIDITY:RESERVED";
		  break;
    }
    return(q);
}
/*
	foncton ajout délai
*/
void time_add_ns( struct timespec *t,int ns)
{

    t->tv_nsec += ns;

    if(t->tv_nsec > 1000000000) {
        t->tv_nsec -= 1000000000;
        t->tv_sec += 1;
    }

}

/*
void phasor_extractor (int x,int N){
	int X_re=0,X_im=0;
	int X_m[sizeof(x)]   = {0};
	int phase[sizeof(x)] = {0};
	int n=0;
	for(i=0;i<sizeof(buffer);i++){

		for(n=0;n<N;n++){
			X_Re[i] = (sqrt(2)/N)*x(2*M_PI*n/N*w)*cos(2*M_PI*n/N);
			X_im[i] = (sqrt(2)/N)*x(2*M_PI*n/N*w)*sin(2*M_PI*n/N);;
			X_m[i]  = sqrt(pow(X_Re[i],2)+pow(X_im[i],2));
			phase[i] = atan((-1)*X_im[i]/X_Re[i]);
		}

	}

	V_am = X_m[0];
	V_bm = X_m[1];
	V_cm = X_m[2];
	V_nm = X_m[3];

	I_am = X_m[4];
	I_bm = X_m[5];
	I_cm = X_m[6];
	I_nm = X_m[7];

}
*/
/* Callback handler pour les messages SV reçus */
static bool depart=true;
struct timeval before_usec;
uint64_t before; /* timestamp in microsecond */
Timestamp ts;
//Timestamp_clearFlags(&ts);
struct timeval timer_usec;
uint64_t timestamp_usec; /* timestamp in microsecond */

static void
svUpdateListener (SVSubscriber subscriber, void* parameter, SVSubscriber_ASDU asdu)
{

    //printf("svUpdateListener called\n");
    if(depart==true){
      gettimeofday(&before_usec, NULL);
      before  = ((uint64_t) before_usec.tv_sec) * 1000000 +
                       (uint64_t) before_usec.tv_usec;
      depart=false;
    }
    const char* svID = SVSubscriber_ASDU_getSvId(asdu);
    if (svID != NULL)
    //printf("  svID=(%s)\n", svID);

    //printf("  smpCnt:  %i\n",  SVSubscriber_ASDU_getSmpCnt(asdu));
  //  printf("  confRev: %u\n",  SVSubscriber_ASDU_getConfRev(asdu));

    /*
     * Accéder aux données requiert à priori une connaissance sur la structure de données sur laquelle on travaille.

     * Une valeur INT32 est encodée en 4 octets. on peut trouver la première valeur
     * INT 32 à l'octet de position 0, la seconde valeur à l'octet de position 4,
     * la troisième valeur à l'octet de position 8,etc.
     * Pour prévenir les erreurs due à la configuration,il faut relever la taille du bloc de
     * données des messages SV avant d'accéder aux données.
     */

    if (SVSubscriber_ASDU_getDataSize(asdu) >= 8) {

    /*
      printf("  ia: %i A\n", SVSubscriber_ASDU_getINT32(asdu, 0)/1000);
      printf("  quality: %s\n", quality(SVSubscriber_ASDU_getQuality(asdu,4)));
      fprintf("  ib: %i A\n", SVSubscriber_ASDU_getINT32(asdu, 8)/1000);
      printf("  quality: %s\n", quality(SVSubscriber_ASDU_getQuality(asdu, 12)));
      printf("  ic: %i A\n", SVSubscriber_ASDU_getINT32(asdu, 16)/1000);
      printf("  quality: %s\n", quality(SVSubscriber_ASDU_getQuality(asdu,20)));
      printf("  in: %i A\n", SVSubscriber_ASDU_getINT32(asdu,24)/1000);
      printf("  quality: %s\n", quality(SVSubscriber_ASDU_getQuality(asdu,28)));
      printf("  Va: %i V\n", SVSubscriber_ASDU_getINT32(asdu, 32)/100);
      printf("  quality: %s\n", quality(SVSubscriber_ASDU_getQuality(asdu,36)));
      printf("  Vb: %i V\n", SVSubscriber_ASDU_getINT32(asdu, 40)/100);
      printf("  quality: %s\n", quality(SVSubscriber_ASDU_getQuality(asdu,44)));
      printf("  Vc: %i V\n", SVSubscriber_ASDU_getINT32(asdu, 48)/100);
      printf("  quality: %s\n", quality(SVSubscriber_ASDU_getQuality(asdu,52)));
      printf("  Vn: %i V\n", SVSubscriber_ASDU_getINT32(asdu, 56)/100);
      printf("  quality: %s\n\n",quality( SVSubscriber_ASDU_getQuality(asdu,60)));
    */

    gettimeofday(&timer_usec, NULL);
    timestamp_usec = ((uint64_t) timer_usec.tv_sec) * 1000000 +
                     (uint64_t) timer_usec.tv_usec;

    //printf(" Timestamp: %lld\n",(long long int) timestamp_usec);
    //printf("time µs since the last sample : %lld\n",timestamp_usec-A);
    TableauTimeStamp[position] = (timestamp_usec-before );
    position+= 1;
    before  = timestamp_usec; // sauvegarde du timestamp précédent

  	}

}

//void *goose_publish (void * donnees){


/* récupération de l'identifiant de la thread*/
//  const pthread_t pid = pthread_self();
  /* choix du CPU */
//  const int core_id = 2;  // CPU3

  /* cpu_set_t: This data set is a bitset where each bit represents a CPU */
//  cpu_set_t cpuset;
  /* CPU_ZERO: This macro initializes the CPU set set to be the empty set */
//  CPU_ZERO(&cpuset);
  /* CPU_SET: This macro adds cpu to the CPU set set */
//  CPU_SET(core_id, &cpuset);
  /* pthread_setaffinity_np: La fonction pthread_setaffinity()  fixe  le masque du CPU_affinity de la thread au CPU pointé par cpuset. Si l'appel est un succes, et la thread n'est pas déjà entrain de tourner sur un des  CPU dans cpuset, alors il est migré vers un de ces CPUs */
/*  const int set_result = pthread_setaffinity_np(pid, sizeof(cpu_set_t), &cpuset);
  if (set_result != 0) {
    print_error_then_terminate(set_result, "pthread_setaffinity_np");
  }
*/
  /* Vérifie quel est l'actuel masque d'affinity qui est assigné à la thread */
  /* pthread_getaffinity_np: The pthread_getaffinity() function returns the CPU affinity mask of the  thread in the buffer pointed to by cpuset */
/*  const int get_affinity = pthread_getaffinity_np(pid, sizeof(cpu_set_t), &cpuset);
  if (get_affinity != 0) {
    print_error_then_terminate(get_affinity, "pthread_getaffinity_np");
  }

  char *buffer;
  // CPU_ISSET: Cette  macro retourne une valeur non nulle(true) si le cpu est un membre du jeu des CPU's, sinon zéro (false) .
  if (CPU_ISSET(core_id, &cpuset)) {

    const size_t needed = snprintf(NULL, 0, SUCCESS_MSG, pid, core_id);
    buffer = malloc(needed);
    snprintf(buffer, needed, SUCCESS_MSG, pid, core_id);
  } else {

    const size_t needed = snprintf(NULL, 0, FAILURE_MSG, pid, core_id);
    buffer = malloc(needed);
    snprintf(buffer, needed, FAILURE_MSG, pid, core_id);
  }
*/

  /* appeler malloc pour demander de la mémoire
  si l'allocation a marché, notre pointeur contient une adresse
  L'allocation dynamique permet notamment de créer un entier/float/tableau dont
  la taille est déterminée par une variable au moment de l'exécution */
/*    data_* param = malloc(sizeof(data_));
    param = donnees;
  	char* interface = malloc(sizeof(char));
  	interface =(*param).interface;
  */
  /* création d'une linkedlist*/
 //	LinkedList dataSetValues = LinkedList_create();
  /*ajout d'éléments*/
/*
 	LinkedList_add(dataSetValues,  MmsValue_newBoolean(true));

  	CommParameters gooseCommParameters;

	gooseCommParameters.appId = 0x3000;
	gooseCommParameters.dstAddress[0] = 0x01;
	gooseCommParameters.dstAddress[1] = 0x0c;
	gooseCommParameters.dstAddress[2] = 0xcd;
	gooseCommParameters.dstAddress[3] = 0x01;
	gooseCommParameters.dstAddress[4] = 0x00;
	gooseCommParameters.dstAddress[5] = 0x01;
	gooseCommParameters.vlanId = 0;
	gooseCommParameters.vlanPriority = 4;
    uint32_t timeAllowedToLive = 5 ; //ms
 	printf("creating goose publisher on interface :%s\n",interface);
  */
	/*
	 * Creation d'une nouvelle instance de publication d'un GOOSE. Comme second paramètre,le nom de
	 l'interface peut être fourni (e.g. "eth0" sur un système Linux). Si le second paramètre est nul
	 *le nom de l' interface est définit avec CONFIG_ETHERNET_INTERFACE_ID dans la stack_config.h.
	 */
/*	GoosePublisher publisher = GoosePublisher_create(&gooseCommParameters, interface);

	GoosePublisher_setGoCbRef(publisher, "This is a Goose_control_block_reference");
  	GoosePublisher_setTimeAllowedToLive(publisher,timeAllowedToLive);
  	GoosePublisher_setGoID(publisher, "This is a GOOSE");
  	GoosePublisher_setConfRev(publisher, 1);
	while(1){
		 usleep(250);
		if (GoosePublisher_publish(publisher, dataSetValues) == -1) {
			printf("Error sending message!\n");
		}

	}

	GoosePublisher_destroy(publisher);
	LinkedList_destroyDeep(dataSetValues, (LinkedListValueDeleteFunction) MmsValue_delete);
    pthread_exit(NULL);
}
*/
 //void *acquisition(void *donnees)
//{

  /* récupération de l'identifiant de la thread*/
//  const pthread_t pid = pthread_self();
  /* choix du CPU */
  //const int core_id = 2;  // CPU3

  /* cpu_set_t: This data set is a bitset where each bit represents a CPU */
//  cpu_set_t cpuset;
  /* CPU_ZERO: This macro initializes the CPU set set to be the empty set */
//  CPU_ZERO(&cpuset);
  /* CPU_SET: This macro adds cpu to the CPU set set */
//  CPU_SET(core_id, &cpuset);
  /* pthread_setaffinity_np: La fonction pthread_setaffinity()  fixe  le masque du CPU_affinity de la thread au CPU pointé par cpuset. Si l'appel est un succes, et la thread n'est pas déjà entrain de tourner sur un des  CPU dans cpuset, alors il est migré vers un de ces CPUs */
  //const int set_result = pthread_setaffinity_np(pid, sizeof(cpu_set_t), &cpuset);
  //if (set_result != 0) {
//    print_error_then_terminate(set_result, "pthread_setaffinity_np");
//  }

  /* Vérifie quel est l'actuel masque d'affinity qui est assigné à la thread */
  /* pthread_getaffinity_np: The pthread_getaffinity() function returns the CPU affinity mask of the  thread in the buffer pointed to by cpuset */
/*  const int get_affinity = pthread_getaffinity_np(pid, sizeof(cpu_set_t), &cpuset);
  if (get_affinity != 0) {
    print_error_then_terminate(get_affinity, "pthread_getaffinity_np");
  }

  char *buffer;
  // CPU_ISSET: Cette  macro retourne une valeur non nulle(true) si le cpu est un membre du jeu des CPU's, sinon zéro (false) .
  if (CPU_ISSET(core_id, &cpuset)) {

    const size_t needed = snprintf(NULL, 0, SUCCESS_MSG, pid, core_id);
    buffer = malloc(needed);
    snprintf(buffer, needed, SUCCESS_MSG, pid, core_id);
  } else {

   const size_t needed = snprintf(NULL, 0, FAILURE_MSG, pid, core_id);
    buffer = malloc(needed);
    snprintf(buffer, needed, FAILURE_MSG, pid, core_id);
  }*/

/*============= Deadline scheduler =============== */
 /* période de la tâche publication de 1s */
 /* paramètres du scheduler deadline      */

  //  unsigned int flags =0;
  //  struct sched_attr attr;
  //  attr.size = sizeof(struct sched_attr);
  //  attr.sched_policy = SCHED_DEADLINE;
  //  attr.sched_flags = 0;
  //  attr.sched_nice = -20;
  //  attr.sched_priority = 0;

      /* nanosecondes*/
/*
    attr.sched_deadline = 0.25e9;
    attr.sched_period  = 0.25e9;
    */
//    sched_setattr(gettid(), &attr, flags);


     /* allocation mémoire */
  //  data_* param = malloc(sizeof(data_));
  //  param  = donnees;
//    uint16_t * appid = malloc(sizeof(uint16_t));
//    appid  =(*param).appid;

    /* Créer un subscriber écoutant les messages SV  avec un APPID 4000h */
//    SVSubscriber subscriber = SVSubscriber_create(NULL,*appid);
//    SVSubscriber_setListener(subscriber, svUpdateListener, NULL);
    /* Connecter le subscriber au receiver */
//    SVReceiver_addSubscriber(param->receiver, subscriber);
  /*  struct timespec t;
    struct timeval avant_boucle,apres_boucle,debut_programme;
    long int duration ;
    int i=0;//j;
    int n=0;
    int taille = 40;
    long int duree [taille];
    gettimeofday(&debut_programme,NULL);
    */
    /* Commencer à écouter les messages SV - commence une nouvelle tâche de receiver en arrière-plan */
/*    SVReceiver_start(param->receiver);
    //clock_gettime(CLOCK_MONOTONIC, &t);
    signal(SIGINT, sigint_handler);
    gettimeofday(&debut_programme,NULL);
    while (running) {

    //  Thread_sleep(1);
      gettimeofday(&maintenant,NULL);
      if (maintenant.tv_sec - debut_programme.tv_sec > 5){ // supérieur à une durée fixé dans la variable taille
          int j;
*/
          /*création d'un fichier de stockage des valeurs */
      //    FILE *fichier;
      //  fichier = fopen("./timestamp_subscribe.csv", "w+");
          /* chemin du fichier dans le docker */
      // fichier = fopen("/log/temps_cycle.csv", "w+");

        //  if(fichier != NULL) {
            /*-- affichage après un certains temps des temps de cycle --*/
  /*          for (j=0;j<position;j++){
                fprintf(fichier,"%i",j);
                fprintf(fichier,"\t%ld\n",TableauTimeStamp[j]);
                //printf("\t%ld\n",TableauTimeStamp[j]);
            }
          fclose(fichier);
          }
          break;
        }
    }
*/


	   // gettimeofday(&apres_boucle,NULL);

	  //  duration  = (apres_boucle.tv_sec*1000000 + apres_boucle.tv_usec);
	  //  duration -= (avant_boucle.tv_sec*1000000 + avant_boucle.tv_usec);
	  //  duree [i] = duration;

	 //   if (apres_boucle.tv_sec - debut_programme.tv_sec > taille){ // supérieur à une durée fixée dans la variable taille
	    //  FILE *fichier;

	      /* enregistrement dans un dossier sur la machine physique */
	     // fichier = fopen("./temps_cycle_subscribe.csv", "w+");
	      /* docker */
	     // fichier = fopen("/log/temps_cycle_subscribe.csv", "w+");
/*	       if (fichier != NULL){
				for(j=0;j<taille;j++){
		    		printf ("4000 échantillons reçu en %lu us\n",duree[j]);

            printf("%lu \n",duree[j]);
		   	   	    fprintf (fichier," %lu\n",duree[j]);
		        }
		    	printf("\t");
		   	 	fprintf(fichier,"\t");
		    	fclose(fichier);
	        }
          */
	 //       break;
	 //   }

	  //  phasor_extractor(buffer,nb_sample);
    //  i+=1;
	  //  n=0;
  /* min_max(TableauTimeStamp,&min,&max);
    printf(" \n min : %i\t  max : %i\t",min,max);
    printf("jitter max : %i\n µs",max-250);
 */
    /* Arrête de l'écoute les messages SV */
//    SVReceiver_stop(param->receiver);
    /* Netoyyage et libération des ressources */
//    SVReceiver_destroy(param->receiver);
//    pthread_exit(NULL);
//}

int main(int argc, char** argv)
{

  /* choix du CPU */
  const int core_id = 2;  // CPU3
  /* récupération de l'identifiant de la thread*/
 const pthread_t pid = getpid();
  /* cpu_set_t: This data set is a bitset where each bit represents a CPU */
  cpu_set_t cpuset;
  /* CPU_ZERO: This macro initializes the CPU set set to be the empty set */
  CPU_ZERO(&cpuset);
  /* CPU_SET: This macro adds cpu to the CPU set set */
  CPU_SET(core_id, &cpuset);
  /* pthread_setaffinity_np: La fonction pthread_setaffinity()  fixe  le masque du CPU_affinity de la thread au CPU pointé par cpuset. Si l'appel est un succes, et la thread n'est pas déjà entrain de tourner sur un des  CPU dans cpuset, alors il est migré vers un de ces CPUs */
 const int set_result = pthread_setaffinity_np(pid, sizeof(cpu_set_t), &cpuset);
  if (set_result != 0) {
    print_error_then_terminate(set_result, "pthread_setaffinity_np");
  }

  /* Vérifie quel est l'actuel masque d'affinity qui est assigné à la thread */
  /* pthread_getaffinity_np: The pthread_getaffinity() function returns the CPU affinity mask of the  thread in the buffer pointed to by cpuset */
  const int get_affinity = pthread_getaffinity_np(pid, sizeof(cpu_set_t), &cpuset);
  if (get_affinity != 0) {
    print_error_then_terminate(get_affinity, "pthread_getaffinity_np");
  }

  char *buffer;
  // CPU_ISSET: Cette  macro retourne une valeur non nulle(true) si le cpu est un membre du jeu des CPU's, sinon zéro (false) .
  if (CPU_ISSET(core_id, &cpuset)) {

    const size_t needed = snprintf(NULL, 0, SUCCESS_MSG, pid, core_id);
    buffer = malloc(needed);
    snprintf(buffer, needed, SUCCESS_MSG, pid, core_id);
  } else {

    const size_t needed = snprintf(NULL, 0, FAILURE_MSG, pid, core_id);
    buffer = malloc(needed);
    snprintf(buffer, needed, FAILURE_MSG, pid, core_id);
  }

  SVReceiver receiver = SVReceiver_create();

  //uint16_t appid;

  if (argc > 1) {
      SVReceiver_setInterfaceId(receiver, argv[1]);
//     appid = strtol(argv[3],NULL,16);
//printf("Set interface id to subscribe: %s\n", argv[1]);
//	printf("Set interface id to publish goose: %s\n", argv[2]);
  }
  else {
      printf("Using interface eth0\n");
      SVReceiver_setInterfaceId(receiver, "eth0");
  }

    /* Créer un subscriber écoutant les messages SV  avec un APPID 4000h */
    SVSubscriber subscriber = SVSubscriber_create(NULL,0x4000);

    SVSubscriber_setListener(subscriber, svUpdateListener, NULL);
    /* Connecter le subscriber au receiver */
    SVReceiver_addSubscriber(receiver, subscriber);

    /* Commencer à écouter les messages SV - commence une nouvelle tâche de receiver en arrière-plan */
    SVReceiver_start(receiver);
    //clock_gettime(CLOCK_MONOTONIC, &t);
    signal(SIGINT, sigint_handler);

    gettimeofday(&debut_programme,NULL);
    while (running) {

    //  Thread_sleep(1);
      gettimeofday(&maintenant,NULL);
      if (maintenant.tv_sec - debut_programme.tv_sec > 5){ // supérieur à une durée fixé dans la variable taille
          int j;
          /*création d'un fichier de stockage des valeurs */
          FILE *fichier;
        fichier = fopen("./timestamp_subscribe.csv", "w+");
          /* chemin du fichier dans le docker */
      // fichier = fopen("/log/temps_cycle.csv", "w+");

          if(fichier != NULL) {
            /*-- affichage après un certains temps des temps de cycle --*/
            for (j=0;j<position;j++){
                fprintf(fichier,"%i",j);
                fprintf(fichier,"\t%ld\n",TableauTimeStamp[j]);
                //printf("\t%ld\n",TableauTimeStamp[j]);
            }
          fclose(fichier);
          }
          break;
        }

    }
    /* création d'un bloc de données de thread de type data_ */

    //   data_ thread_data;
    //  thread_data.receiver= receiver;
   //   thread_data.interface = argv[2];
   //   thread_data.appid = &appid;

    /* décalaration des  thread */
//    pthread_t thread_acquisition;
//    pthread_t thread_goose_publish;

    /* création de la thread d'acquisition*/
  //  pthread_create(&thread_acquisition,NULL,acquisition,&thread_data);
  //  pthread_create(&thread_goose_publish,NULL,goose_publish,&thread_data);

    /* le programme principal attend la fin de la tâche*/
  //  pthread_join(thread_acquisition,NULL);
//    pthread_join(thread_goose_publish,NULL);
/* Arrête de l'écoute les messages SV */
 SVReceiver_stop(receiver);
/* Netoyyage et libération des ressources */
 SVReceiver_destroy(receiver);
  //  return 0 ;
}
