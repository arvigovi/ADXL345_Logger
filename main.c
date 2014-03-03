/* Hi Speed Logger Project - ADXL345 Config & Logging
* Author: Arvind Govindaraj (arvind.govindaraj@gmail.com)
* main.c
* Rev History:
* V0.3 - 02/24/2014
* V0.2 - 04/14/2013
* V0.1 - 03/06/2013
Comments: 
* Significant code from gpxlogger.c - part of the gpsd project
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdlib.h>
#include <bcm2835.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <string.h>
#include <syslog.h>
#include <math.h>
#include <errno.h>
#include <libgen.h>
#include <signal.h>
#include <assert.h>

#include "ADXL345.h"

#define ADXL_INT1_PIN RPI_GPIO_P1_11
#define ADXL_NoSamples 25

//global variables
static char *progname;
static FILE *logfile;
int i,j=0,temp; //loop counters for looging
int16_t ax[ADXL_NoSamples],ay[ADXL_NoSamples],az[ADXL_NoSamples];
struct timeval tv; //structure to hold current unix time
uint64_t timeus;

//function prototypes
void ADXL345_Log();

/* Return 0 if failed, 1 if passed*/
uint8_t HW_Config()
{
	unsigned char temp;
	
	if (!bcm2835_init())
        return 0;
		
	//GPIO init 
	bcm2835_gpio_fsel(ADXL_INT1_PIN, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_set_pud(ADXL_INT1_PIN, BCM2835_GPIO_PUD_OFF);
	

	//ADXL345 init
	if(!ADXL345_Init(ADXL345_SPI_COMM))
	{
		printf("ADXL345 Init Err \r\n");
		return 0;
	}

	return 1;
}

void HW_Close()
{
	//bcm2835_i2c_end();
	bcm2835_spi_end();
	bcm2835_close();
}

//this is the only exit point after logging intitiated successfully
static void quit_handler(int signum)
{
	/* don't clutter the logs on Ctrl-C */
    if (signum != SIGINT)
	syslog(LOG_INFO, "exiting, signal %d received", signum);
	HW_Close();
    exit(0);
}

void ADXL345_Log()
{

	//read samples from ADXL345 - the fprintf should not be in this loop because this is a fifo bulk read and can be quite fast. The fprintf time is not included in the delay calculation.
	for(i=0;i<ADXL_NoSamples;i++)
		ADXL345_GetXYZ_MB((ax+i),(ay+i),(az+i));
	
	gettimeofday(&tv,NULL);
	timeus = 1000000 * tv.tv_sec + tv.tv_usec;
	
	for(i=0;i<ADXL_NoSamples;i++)
		(void)fprintf(logfile,"%u,%f,%f,%f\r\n",timeus,(float)ax[i]*0.004,(float)ay[i]*0.004,(float)az[i]*0.004);
	
	j++; //keep track of total no of samples
}


int main(int argc, char **argv)
{
    int ch;
    bool daemonize = false;
    progname = argv[0];
    logfile = stdout;

	if(!HW_Config())
	{
		printf("HW Config Error \r\n");
		exit(0);
	}
	
    while ((ch = getopt(argc, argv, "dD:e:f:hi:lm:V")) != -1)
	{
		switch (ch) 
		{
		case 'd':
			openlog(basename(progname), LOG_PID | LOG_PERROR, LOG_DAEMON);
			daemonize = true;
			break;
       case 'f':       /* Output file name. */
            {
                char   *fname = NULL;
                time_t  t;
                size_t  s = 0;
                size_t fnamesize = strlen(optarg);

                t = time(NULL);
                while (s == 0) {
		    char *newfname = realloc(fname, fnamesize);
		    if (newfname == NULL) {
			syslog(LOG_ERR, "realloc failed.");
			goto bailout;
		    } else {
			fnamesize += 1024;
			fname = newfname;
		    }
		    s = strftime(fname, fnamesize-1, optarg, localtime(&t));
                }
		assert(fname != NULL); /* pacify splint */
                fname[s] = '\0';;
                logfile = fopen(fname, "w");
                if (logfile == NULL)
		    syslog(LOG_ERR,
			   "Failed to open %s: %s, logging to stdout.",
			   fname, strerror(errno));
	    bailout:
                free(fname);
                break;
		/*@+usedef@*/
            }
		}
	}

	
	if (daemonize && logfile == stdout) {
	syslog(LOG_ERR, "Daemon mode with no valid logfile name - exiting.");
	exit(1);
    }
	
    /* catch all interesting signals */
    (void)signal(SIGTERM, quit_handler);
    (void)signal(SIGQUIT, quit_handler);
    (void)signal(SIGINT, quit_handler);
	
    /* might be time to daemonize */
    if (daemonize) {
	/* not SuS/POSIX portable, but we have our own fallback version */
	if (daemon(0, 0) != 0)
	    (void) fprintf(stderr,"demonization failed: %s\n", strerror(errno));
    }
	
	//print header
	(void)fprintf(logfile,"Time(us),ax,ay,az\r\n");
	
	while(1)
	{
		ADXL345_Log();
		usleep(ADXL_NoSamples*10*1000);//sleep for 10ms / sample -> initial assumption the (ADXL_NoSamples)*fprintf statements don't use much time compared to (ADXL_NoSamples*10ms)
	}
	
	printf("main  exit\r\n");
    HW_Close();
    exit(0);
}