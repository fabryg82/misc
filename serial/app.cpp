#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

#include "serial.h"

void usage(void)
{
    printf("myapp [options]\n");
    printf("-p: generic param\n");
    printf("-h: print help\n");
    printf("\n");
    exit(0);
}

// Prototypes
static void sigHandler(int sig);
static long long int get_time_us();

// Global variable
static bool done = false;

int main(int argc, char* argv[])
{
    int param;

    // Catch CTRL-C
    //signal(SIGINT, sigHandler);
        
    // Parsing parameters
    if(argc == 1)
    {
        //usage();
    }

    while(1)
    {
        char c;

        static struct option long_options[] = {
            {"param", 1, 0, 'p'},
            {"help", 0, 0, 'h'},
            {0, 0, 0, 0}
        };

        c = getopt_long (argc, argv, "p:h", long_options, NULL);
        if(c == -1)     break;

        switch(c)
        {
        case 'p':
            param = atoi(optarg);
            break;
        case 'h':
            usage();
            break;
        default:
            printf ("?? getopt returned character code 0%o ??\n\n", c);
            return -1;
        }
    }
    if(optind < argc)
    {
        printf("Unknown parameter %s\n", argv[optind]);
        return -1;
    }

    // Validate parameters
    if(param < 0)
    {
        printf("Param must be positive\n");
        return -1;
    }
  
    SerialPort port("/dev/ttyUSB0");
    port.set_dtr(true, true);
    port.set_rts(true, true);

    while(1)
    {
        port.update_term_status();
        printf("DTR %d\n", port.get_dtr());
        printf("RTS %d\n", port.get_rts());
        printf("DCD %d\n", port.get_dcd());
        printf("DSR %d\n", port.get_dsr());
        printf("CTS %d\n", port.get_cts());
        printf("RI %d\n", port.get_ri());
	usleep(10000);
    }

//    while(!done)
//   {
//        printf("Time: %llu\n", get_time_us());
//        sleep(1);
//    }

        return 0;
}


void sigHandler(int sig)
{
    done = true;
    return;
}

inline 
long long int get_time_us()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long int)tv.tv_sec * 1000000 + tv.tv_usec;
}

