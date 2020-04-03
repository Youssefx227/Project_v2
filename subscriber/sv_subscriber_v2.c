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


 #define TWO_PI_OVER_THREE  2*M_PI/3


 #define nech 80*2 // nombre échantillons
 #define fe 4000  
 #define f_50hz 50
 #define nbre_signal 1 /* nombre de subscriber*/

struct timeval maintenant;
struct timeval debut_programme;
struct timeval now;
const char* svid;
long long int TableauTimeStamp [nech];
long long int time_between_2_SV[nech];
uint64_t n [nech];
double  ia [nbre_signal][nech];
double  ib [nbre_signal][nech];
double  ic [nbre_signal][nech];
double  in [nbre_signal][nech];
double  va [nbre_signal][nech];
double  vb [nbre_signal][nech];
double  vc [nbre_signal][nech];
double  vn [nbre_signal][nech];
int nLoopListener =0;
int min=0,max=0;
int i =0;
int k=0;
int position[nbre_signal]={0};
/* Callback handler pour les messages SV reçus */
long long int before=0; /* timestamp in microsecond */
struct timespec timer_usec;
uint64_t timestamp_usec; /* timestamp in microsecond */
static bool stop = false;

void signal_catch_stop(){
 stop = true;
}
/* conversion in microseconds of timeval*/
static uint64_t TimeValToUSeconds(struct timespec* ts) 
{
   return(ts->tv_sec*1000000 + (ts->tv_nsec/1000));
}

void saving_in_comtrade(){

  FILE *signal;
  signal = fopen("./files/comtrade.csv","w+r");
  //signal = fopen("./comtrade.dat","w+r");
  for(int i=0;i<nech;i++){
    fprintf(signal,"%ld,",n[i]);
    fprintf(signal,"%s,",svid);
    fprintf(signal,"%lld,",TableauTimeStamp[i]);
   // fprintf(signal,"%ld,",t_between_ech[i]);
    for(int j=0;j<nbre_signal;j++){
      fprintf(signal,"%f,",va[j][i]);
      fprintf(signal,"%f,",vb[j][i]);
      fprintf(signal,"%f,",vc[j][i]);
      fprintf(signal,"%f,",ia[j][i]);
      fprintf(signal,"%f,",ib[j][i]);
      if(j<(nbre_signal-1)){
        fprintf(signal,"%f,",ic[j][i]);
      }
      else{ //enlève le symbole , quand on arrive en bout de ligne
          fprintf(signal,"%f",ic[j][i]);
      }
    }
      fprintf(signal,"\n");
  }
  fclose(signal);
}


void compute_DFT(double *input,double* phasor_mod,double* phasor_arg,int signal_length,int N){

	double sumreal=0,sumimag=0,sum1=0,sum2=0;
	double output_real[signal_length];
    double output_imag[signal_length];
    double output_real_f[signal_length];
    double output_imag_f[signal_length];
	double angle;

	for(int k=0;k<signal_length;k++){
	   	angle = 2*M_PI/(double)N;
        output_real_f[k] =  input[k]*cos(k*angle);
        output_imag_f[k]=   input[k]*sin(k*angle);
    }

  	output_real[0]=0;
  	output_imag[0]=0;


  	for (int k =0;k<signal_length;k++){

	    if(k<=N){
	       output_real[k] = (2/(double)N)*output_real_f[k]+ sum1;
	       output_imag[k] = (2/(double)N)*output_imag_f[k]+ sum2;
	       sum1 = output_real[k];
	       sum2 = output_imag[k];
	    }
    	else{
    		for (int i=(k-(N-1));i<k;i++){
    	   	    sumreal += output_real_f[i];
            	sumimag += output_imag_f[i];
    	    }
    	    output_real[k] = (2/(double)N)*sumreal;
          output_imag[k] = (-2/(double)N)*sumimag;
        }
        phasor_mod [k] = sqrt(output_real[k]*output_real[k] + output_imag[k]*output_imag[k]);
         if(k>0){
         	phasor_arg [k] = atan2(output_imag[k],output_real[k])*(180/M_PI);
        }
        else{
         	phasor_arg [k] = NAN;
         }
 		 sumreal =0;
 		 sumimag =0;
    }
}

void FIR(double *buffer,double* bufferFIR,int fc){

	double a = (double) fc/(double)fe;
	int M= 3;           // coefficent du filtre
	double w[M+1],h[M+1];
	double sum =0;
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

void decimate(double *buffer,double* phasor_mod,double* phasor_arg,int factorDecimation, int nloop,int length_buf,const char* phasor_name)
{
	int fc = 1000 ; // fréquence de coupure
  int i =0;
	int NbreSamples = nloop;
	double bufferDownsampled [length_buf];
	double bufferFIR[nech];
  if(NbreSamples==(nech-1)){
    	FIR (buffer,bufferFIR,fc);  //filtrage numérique
    	FILE *doc;
    	char docname [100] = "./files/signal_filtré_";
    	strcat(docname,phasor_name);
    	strcat(docname,".csv");
      doc = fopen(docname, "w+r");
	 	  while(i<=NbreSamples){
  			fprintf(doc,"%s\t","signal de sortie filtré");
  			fprintf(doc,"%s\n","signal d'origine");
  			for(int j=0;j<length_buf;j++){
  				bufferDownsampled [j] = bufferFIR[i];
          printf("%f\n",buffer[i]);
  				fprintf(doc,"%f\t",bufferDownsampled [j]);
  				fprintf(doc,"%f\n",buffer[i]);
  				bufferFIR[i]=0;
  				i+=factorDecimation;
  			}
		fclose(doc);
		}
		compute_DFT(bufferDownsampled,phasor_mod,phasor_arg,length_buf,(fe/(f_50hz*factorDecimation)));
	}
}

void phasor_extraction (){

	int factorDecimation =2;
	FILE * file_va =NULL,*file_vb=NULL,*file_vc=NULL,*file_vn=NULL;
	FILE * file_ia =NULL,*file_ib=NULL, *file_ic=NULL,*file_in=NULL;
  int length_buf= nech/factorDecimation;
  double va_mod[length_buf],vb_mod[length_buf],vc_mod[length_buf],vn_mod[length_buf];
  double va_arg [length_buf],vb_arg[length_buf],vc_arg[length_buf],vn_arg[length_buf];
  double ia_arg[length_buf],ib_arg[length_buf],ic_arg[length_buf],in_mod[length_buf];
  double ia_mod [length_buf],ib_mod[length_buf],ic_mod[length_buf],in_arg[length_buf];
  for(int i =0;i<nbre_signal;i++) {
    
   //  decimate(&(vn[i][nech]),vn_mod,vn_arg,factorDecimation,nLoopListener,length_buf);
   //  decimate(&(in[i][nech]),in_mod,in_arg,factorDecimation,nLoopListener,length_buf);
    // decimate(&(va[i][nech]),va_mod,va_arg,factorDecimation,nLoopListener,length_buf);
    // decimate(&(vb[i][nech]),vb_mod,vb_arg,factorDecimation,nLoopListener,length_buf);
    // decimate(&(vc[i][nech]),vc_mod,vc_arg,factorDecimation,nLoopListener,length_buf);
     decimate(&(ia[i][nech]),ia_mod,ia_arg,factorDecimation,nLoopListener,length_buf,"Ia");
    // decimate(&(ib[i][nech]),ib_mod,ib_arg,factorDecimation,nLoopListener,length_buf);
    // decimate(&(ic[i][nech]),ic_mod,ic_arg,factorDecimation,nLoopListener,length_buf);
    }


  FILE *phasor_csv;
  phasor_csv = fopen("./files/phasor_tab.csv","w+r");
  //char* en_tete1 ="va\t\tvb\t\tvc\t\tia\t\tib\t\tic";
   char* en_tete1 ="ia";
  fprintf(phasor_csv,"%s\n",en_tete1);
  for(int i=0;i<length_buf;i++){
	// fprintf(phasor_csv,"%f\t%f\t",va_mod[i],va_arg[i]);
	// fprintf(phasor_csv,"%f\t%f\t",vb_mod[i],vb_arg[i]);
	// fprintf(phasor_csv,"%f\t%f\t",vc_mod[i],vc_arg[i]);
	 fprintf(phasor_csv,"%f\t%f\n",ia_mod[i],ia_arg[i]);
	// fprintf(phasor_csv,"%f\t%f\t",ib_mod[i],ib_arg[i]);
	// fprintf(phasor_csv,"%f\t%f\n",ic_mod[i],ic_arg[i]);
    }
	fclose(phasor_csv);

}

int sig_num = 0;
static void
svUpdateListener (SVSubscriber subscriber, void* parameter, SVSubscriber_ASDU asdu)
{

    //printf("svUpdateListener called\n");
    clock_gettime(CLOCK_REALTIME,&timer_usec);
    /*
     * Accéder aux données requiert à priori une connaissance sur la structure de données sur laquelle on travaille.

     * Une valeur INT32 est encodée en 4 octets. on peut trouver la première valeur
     * INT 32 à l'octet de position 0, la seconde valeur à l'octet de position 4,
     * la troisième valeur à l'octet de position 8,etc.
     * Pour prévenir les erreurs due à la configuration,il faut relever la nech du bloc de
     * données des messages SV avant d'accéder aux données.
     */
    if (SVSubscriber_ASDU_getDataSize(asdu) >= 8 ){
     
	    /*commence à 1 pour le champ index du format .dat comtrade*/
      n[position[sig_num]] = position[sig_num]+1;
      svid   = SVSubscriber_ASDU_getSvId(asdu);
      ia [sig_num][position[sig_num]] = (double) (SVSubscriber_ASDU_getINT32(asdu, 0)/ 1000);
      ib [sig_num][position[sig_num]] = (double) (SVSubscriber_ASDU_getINT32(asdu, 8)/ 1000);
      ic [sig_num][position[sig_num]] = (double) (SVSubscriber_ASDU_getINT32(asdu, 16)/1000);
     // in [sig_num][position[sig_num]] = (double) (SVSubscriber_ASDU_getINT32(asdu, 24)/1000);
      va [sig_num][position[sig_num]] = (double) (SVSubscriber_ASDU_getINT32(asdu, 32)/100);
      vb [sig_num][position[sig_num]] = (double) (SVSubscriber_ASDU_getINT32(asdu, 40)/100);
      vc [sig_num][position[sig_num]] = (double) (SVSubscriber_ASDU_getINT32(asdu, 48)/100);
    //  vn [sig_num][position[sig_num]] = (double) (SVSubscriber_ASDU_getINT32(asdu, 56)/100);
      timestamp_usec = TimeValToUSeconds(&timer_usec);
      long long int Ts_us = timestamp_usec;
      TableauTimeStamp [position[sig_num]] = Ts_us;
      time_between_2_SV[position[sig_num]] =Ts_us - before;
      nLoopListener = position[sig_num]; //nombre de fois de l'appel de la fonction
      position[sig_num] += 1;
      if(position[sig_num]>(nech-1)){
  	    position[sig_num] =0;
  	}
     before  = Ts_us; // sauvegarde du timestamp précédent
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
  /* pthread_setaffinity_np: La fonction pthread_setaffinity()  fixe  le masque du CPU_affinity de la thread au CPU pointé par cpuset.
     Si l'appel est un succes, et la thread n'est pas déjà entrain de tourner sur un des  CPU dans cpuset, alors il est migré vers un de ces CPUs */
  sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);

  SVReceiver receiver = SVReceiver_create();
  const char* filename =0;
  double subscruption_time =0;
  char start;
  if (argc > 1){
  	/* interface réseau ou connecter le receiver si configuré */
      SVReceiver_setInterfaceId(receiver, argv[1]);
      printf("Set interface id: %s\n", argv[1]);
      /*récupère un nom de fichier si configuré*/
      filename = argv[3];
      subscruption_time = atof(argv[4])*60.0; // temps de publication [min --> sec]
  }
  else {
      printf("Using interface eth0\n");
      SVReceiver_setInterfaceId(receiver, "eth0");
  }

    SVSubscriber subscriber [nbre_signal];

    for(i=0;i<nbre_signal;i++){
      /* Créer un subscriber écoutant les messages SV  avec un APPID 0x4000 par défaut */
      subscriber[i] = SVSubscriber_create(NULL,0x4000);
      SVSubscriber_setListener(subscriber[i], svUpdateListener, NULL);
      /* Connecter le subscriber au receiver */
      SVReceiver_addSubscriber(receiver, subscriber[i]);
    }
    printf("\n Subsriber waiting for start command ... click A\n also CTRL_z to stop anytime in the program:\t");
    start = getchar();
    /* Commencer à écouter les messages SV - commence une nouvelle tâche de receiver en arrière-plan */
    SVReceiver_start(receiver);
    signal(SIGTSTP,signal_catch_stop);
    gettimeofday(&debut_programme,NULL);
    while (start =='A'){
    	saving_in_comtrade();
    	 /** fonction qui sous-échantillonne tous les signaux d'entrées (+ filtrage anti-repliement)
    	     diminue le taux d'échantillonnage  */
    	phasor_extraction();

        if (stop == true){  // dès qu'on stop on génère un fichier de log
          FILE * fichier_timestamp;
          fichier_timestamp = fopen(filename, "w+r");
          fprintf(fichier_timestamp," time_between_2_SV\t jitter\n");
          if(fichier_timestamp != NULL) {
           for(i=0;i<nech;i++){
             fprintf(fichier_timestamp,"%lld\t",time_between_2_SV[i]); // time between 2 consecutive SV
             fprintf(fichier_timestamp,"%lf\n",fabs(time_between_2_SV[i]-250)); // jitter
           } 
           fclose(fichier_timestamp);
          }
            char c;
            printf("pause... click B to continue or C to exit\t");
            /*message pour savoir ce que l'on fait après ce STOP*/
            c = getchar();
            if(c == 'B'){ /*si l'on appui sur B*/
              stop = false; /*on continue*/
            }
            else if(c == 'C'){ /*si l'on appui sur C*/
              printf(" \nexit.."); /*on sort du programme*/
              exit(0);
            }
        }
        gettimeofday(&maintenant,NULL);
        /*si le temps est écoulé, on arrête la publication*/
        if((maintenant.tv_sec - debut_programme.tv_sec) >= subscruption_time){exit(0);}   
    }
/* Arrête de l'écoute les messages SV */
 SVReceiver_stop(receiver);
/* Netoyyage et libération des ressources */
 SVReceiver_destroy(receiver);
 return 0;
}
