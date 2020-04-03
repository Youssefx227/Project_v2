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
 #include <goose_publisher.h>


 #define nech 80 // nbre echantillon 
 #define fe 4000
 static bool running = true;

struct timeval maintenant;
struct timeval debut_programme;
struct timeval now;
uint64_t TableauTimeStamp[nech];
uint64_t t_between_ech [nech];
uint64_t n [nech];
float  ia [nech];
float  ib [nech];
float  ic [nech];
float  in [nech];
float  va [nech];
float  vb [nech];
float  vc [nech];
float  vn [nech];
float bufferFIR[nech];

int nLoopListener =0;
int min=0,max=0;
int i =0;
int k=0;
int position =0;
/* Callback handler pour les messages SV reçus */
bool depart=true;
struct timeval before_usec;
uint64_t before=0; /* timestamp in microsecond */
struct timeval timer_usec;
uint64_t timestamp_usec=0; /* timestamp in microsecond */


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


void sigint_handler(int signalId)
{
    running = 0;
}

void compute_DFT(float* input,float* phasor,int N){

	float sumreal,sumimag;
	float output_real;
    float output_imag ;
	float angle;
		/*reset*/
	    sumreal =0;
		sumimag =0;
		for(int n =0;n<N;n++){
			angle = (2*M_PI)/N;
			sumreal += input[n]*cos(n*angle);
			sumimag += input[n]*sin(n*angle);
		}
	output_real = 2*sumreal/N;
    output_imag = (-2)*sumimag/N;
    phasor[0] = sqrt(pow(output_real,2.0) + pow(output_imag,2.0));
    phasor[1] = atan2(output_imag,output_real) * (180.0/M_PI);
	//printf("phasor : %f -------  %f\n",phasor[0], phasor [1]);
} 
 
void FIR(float* buffer,int fc){

	float a = (float) fc/(float)fe;     
	int M= 3;           // coefficent du filtre 
	float w[M+1],h[M+1];
	float sum =0;

	for( int k=0;k<=M;k++){

		if((k-M/2)==0){

			h[k] =2*M_PI*a;
		}
		else{

			h[k] =  sin(2*M_PI*(k-M/2)*a)/(k-M/2);
		}
	
		//== hamming window == //
	  //  w[k] = 0.54-0.46*cos(2*M_PI*k/M);
		//== Blackman window ==//
		w[k] = 0.42  - 0.5*cos(2*M_PI*k/M) + 0.08*cos(4*M_PI*k/M);

		h[k] = h[k]*w[k];
		sum = sum + h[k]; 
	}
	for( int k=0;k<=M;k++){     // normaliser le filtre passe bas kernel pour avoir
		h[k] = h[k]/sum;        // un gain unitaire en DC
	}
	

	for(int i=0;i<nech;i++){   // produit de convolution du signal d'entrée et du filtre kernel
		for (int k=0;k<=M;k++){	
			if((i-k)>=0){
				bufferFIR[i] += h[k]*buffer[i-k];	
			}
			else{
				bufferFIR[i] += 0;	
			}
		}
	}
}

void decimate(float* buffer,float* buf_phasor_mod,float* buf_phasor_arg,int factorDecimation, int nloop,const char* phasor_name,const char* filename ,FILE* file)
{
	int fc = 1000 ; // fréquence de coupure
    int factor = factorDecimation;
    int i =0;
	int NbreSamples = nloop;
	float bufferDownsampled [nech/factor];
	float magnitude[nech/factor],phase[nech/factor];
	float phasor [2] = {0,0};
    if(NbreSamples==(nech-1)){
    	FIR (buffer,fc);  //filtrage numérique
    	FILE *doc;
    	char docname [200] = "./signal_filtré_";
    	strcat(docname,phasor_name);
    	strcat(docname,".csv");
        doc = fopen(docname, "w+r");
		while(i<=NbreSamples){
			fprintf(doc,"%s\t","signal de sortie filtré");
			fprintf(doc,"%s\n","signal d'origine");
			for(int j=0;j<(nech/factor);j++){
				bufferDownsampled [j] = bufferFIR[i];	
				fprintf(doc,"%f\t",bufferDownsampled [j]);
				fprintf(doc,"%f\n",buffer[i]);
				bufferFIR[i]=0;
				i+=factor;
			}
		}

		fclose(doc);
		compute_DFT(bufferDownsampled,phasor,(nech/factor));
		buf_phasor_mod[k] = phasor[0];
		buf_phasor_arg[k] = phasor[1];
		if(k==nech/factor){
			file = fopen(filename,"w+r");
			fprintf(file,"%s\n",phasor_name);
			fprintf(file,"%s\t","module");
			fprintf(file,"\t%s\n","argument");
			for(int i=0;i<k;i++){
				fprintf(file,"%f\t%f\n",buf_phasor_mod[i],buf_phasor_arg[i]);
			}
		    fclose(file);
		    k=0;
		}
		k+=1;
	}
	
}


void phasor_extract (){

	int factorDecimation =2;
	int i =0;
	const int size_buf= (nech/factorDecimation);
	float buf_phasor_mod_va[size_buf],buf_phasor_mod_vb[size_buf],buf_phasor_mod_vc[size_buf],buf_phasor_mod_vn[size_buf];
	float buf_phasor_arg_va[size_buf],buf_phasor_arg_vb[size_buf],buf_phasor_arg_vc[size_buf],buf_phasor_arg_vn[size_buf];
	float buf_phasor_mod_ia[size_buf],buf_phasor_mod_ib[size_buf],buf_phasor_mod_ic[size_buf],buf_phasor_mod_in[size_buf];
	float buf_phasor_arg_ia[size_buf],buf_phasor_arg_ib[size_buf],buf_phasor_arg_ic[size_buf],buf_phasor_arg_in[size_buf];
	FILE * file_va,*file_vb,*file_vc,*file_vn;
	FILE * file_ia,*file_ib, *file_ic,*file_in;
	decimate(va,buf_phasor_mod_va,buf_phasor_arg_va,factorDecimation,nLoopListener,"va phasor","./va_file",file_va);
	decimate(ia,buf_phasor_mod_ia,buf_phasor_arg_ia,factorDecimation,nLoopListener,"ia phasor","./ia_file",file_ia);
//	decimate(vb,buf_phasor_mod_vb,buf_phasor_arg_vb,factorDecimation,nLoopListener,"vb phasor","./vb_file",file_vb);
//	decimate(ib,buf_phasor_mod_ib,buf_phasor_arg_ib,factorDecimation,nLoopListener,"ib phasor","./ib_file",file_ib);
	/*
	decimate(vc,vc_magnitude,vc_phase,factorDecimation,nLoopListener,"vc_phasor");
	decimate(ic,ic_magnitude,ic_phase,factorDecimation,nLoopListener,"ic_phasor");
	decimate(vn,vn_magnitude,vn_phase,factorDecimation,nLoopListener,"vn_phasor");
	decimate(in,in_magnitude,in_phase,factorDecimation,nLoopListener,"in_phasor");        
	*/  
}


static void
svUpdateListener (SVSubscriber subscriber, void* parameter, SVSubscriber_ASDU asdu)
{
	/*
    //printf("svUpdateListener called\n");
    if(depart==true){
      gettimeofday(&before_usec, NULL);
      before  = ((uint64_t) before_usec.tv_sec) * 1000000 +
                 (uint64_t) before_usec.tv_usec;
      depart=false;
    }
    */
    /*
     * Accéder aux données requiert à priori une connaissance sur la structure de données sur laquelle on travaille.

     * Une valeur INT32 est encodée en 4 octets. on peut trouver la première valeur
     * INT 32 à l'octet de position 0, la seconde valeur à l'octet de position 4,
     * la troisième valeur à l'octet de position 8,etc.
     * Pour prévenir les erreurs due à la configuration,il faut relever la nech du bloc de
     * données des messages SV avant d'accéder aux données.
     */
    if (SVSubscriber_ASDU_getDataSize(asdu) >= 8) {

	  if(position>(nech-1)){
	    	position =0;
	  }

      n  [position] = SVSubscriber_ASDU_getSmpCnt(asdu);
      ia [position] = (float) (SVSubscriber_ASDU_getINT32(asdu, 0)/1000);
      ib [position] = (float) (SVSubscriber_ASDU_getINT32(asdu, 8)/1000);
      ic [position] = (float) (SVSubscriber_ASDU_getINT32(asdu, 16)/1000);
      in [position] = (float) (SVSubscriber_ASDU_getINT32(asdu, 24)/1000);
      va [position] = (float) (SVSubscriber_ASDU_getINT32(asdu, 32)/100);
      vb [position] = (float) (SVSubscriber_ASDU_getINT32(asdu, 40)/100);
      vc [position] = (float) (SVSubscriber_ASDU_getINT32(asdu, 48)/100);
      vn [position] = (float) (SVSubscriber_ASDU_getINT32(asdu, 56)/100);

     // gettimeofday(&timer_usec, NULL);
     // timestamp_usec = ((uint64_t) timer_usec.tv_sec) * 1000000 +
     //                (uint64_t) timer_usec.tv_usec;
    //  TableauTimeStamp[position] = timestamp_usec;
     // t_between_ech   [position] = (timestamp_usec - before);
      nLoopListener = position; //nombre de fois de l'appel de la fonction
      position += 1;
     // before  = timestamp_usec; // sauvegarde du timestamp précédent
	
    }
}

int main(int argc, char** argv)
{
  /* choix du CPU */
  const int core_id = 2;  // CPU3
  /* récupération de l'identifiant de la thread*/
 //const pthread_t pid = getpid();
  /* cpu_set_t: This data set is a bitset where each bit represents a CPU */
  cpu_set_t cpuset;
  /* CPU_ZERO: This macro initializes the CPU set set to be the empty set */
  CPU_ZERO(&cpuset);
  /* CPU_SET: This macro adds cpu to the CPU set set */
  CPU_SET(core_id, &cpuset);
  /* pthread_setaffinity_np: La fonction pthread_setaffinity()  fixe  le masque du CPU_affinity de la thread au CPU pointé par cpuset. Si l'appel est un succes, et la thread n'est pas déjà entrain de tourner sur un des  CPU dans cpuset, alors il est migré vers un de ces CPUs */
  sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);


  SVReceiver receiver = SVReceiver_create();
  uint16_t appid =0;
  const char* filename =0;
  if (argc > 1){
      SVReceiver_setInterfaceId(receiver, argv[1]);
      appid = strtol(argv[3],NULL,16);
      filename = argv[4];
  }
  else {
      printf("Using interface eth0\n");
      SVReceiver_setInterfaceId(receiver, "eth0");
  }

    /* Créer un subscriber écoutant les messages SV  avec un APPID */
    SVSubscriber subscriber = SVSubscriber_create(NULL,appid);

    SVSubscriber_setListener(subscriber, svUpdateListener, NULL);
    /* Connecter le subscriber au receiver */
    SVReceiver_addSubscriber(receiver, subscriber);

    /* Commencer à écouter les messages SV - commence une nouvelle tâche de receiver en arrière-plan */
    SVReceiver_start(receiver);

    signal(SIGINT, sigint_handler);

    //gettimeofday(&debut_programme,NULL);


    while (running) {

    	 /** fonction qui sous-échantillonne tous les signaux d'entrées (+ filtrage anti-repliement)
    	     diminue le taux d'échantillonnage  */
    	phasor_extract();
        //gettimeofday(&maintenant,NULL);
/*
        if (maintenant.tv_sec - debut_programme.tv_sec >5){ // supérieur à une durée fixé dans la variable nech
          int j =0;
          FILE *fichier;
          fichier = fopen(filename, "w+r");
        // fichier = fopen("/log/temps_cycle.csv", "w+");

          if(fichier != NULL) {
	        fprintf(fichier,"N° ech\t");
            fprintf(fichier,"Timestamp_ech\t");
            fprintf(fichier,"time_before_last_ech\t");
            fprintf(fichier,"current Ia \n");
            for (j=0;j<position;j++){
                  fprintf(fichier,"%ld",n[j]);
                  fprintf(fichier,"\t%ld",TableauTimeStamp[j]);
                  fprintf(fichier,"\t%ld",t_between_ech[j]);
                  fprintf(fichier,"\t%ld\n",ia[j]);
            }
            fclose(fichier);
          }
            break;
        }
*/
    }
/* Arrête de l'écoute les messages SV */
 SVReceiver_stop(receiver);
/* Netoyyage et libération des ressources */
 SVReceiver_destroy(receiver);
 return 0;
}
