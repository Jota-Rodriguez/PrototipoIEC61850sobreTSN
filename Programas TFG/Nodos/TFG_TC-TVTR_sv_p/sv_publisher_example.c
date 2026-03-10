/**
    @file sv_publisher_example.c
    @author Joan Pau Rodriguez Sureda
    @brief Simulating the behaviour of a TVTR/TCTR. All the data generated is
    sended via a Sampled Values (SV) publisher based on IEC61850 standard.

    @note In this program the clock CLOCK_REALTIME is synchronized with the TSN
    switch.
    @note It also generates a .csv file with the computational and total time
    per cycle.
 */

#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <mqueue.h>
#include <sys/stat.h>
#include <MQTTClient.h>

#include "hal_thread.h"
#include "sv_publisher.h"
#include "../libraries/ts.h"

#define SEND_MQTT_CYCLES 2000
#define NUM_SAMPLES 1000000
#define SAMPLES_FILE "/home/joanpau/libiec61850-1.5/examples/TFG_TC-TVTR_sv_p/muestreo_ta_NM.csv"

#define CLIENTID "jp3252531"
#define MQTT_SERVER "tom.uib.es"
#define SUB_TOPIC "TFG_jprodriguez/SCADA/TC-TVTR/#"
#define PUB_TOPIC "TFG_jprodriguez/Maqueta/TC-TVTR"
#define QOS 1

#define QUEUE_NAME "/mq_TC-TVTR_XCBR"
#define QUEUE_PERMS ((int)(0644))
#define QUEUE_MAXMSG 16
#define QUEUE_MSGSIZE 1024

///////////////////////////////////////////////////////////////////////////////
///     SUBPROGRAMS AND GLOBAL VARIABLES DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

struct configParameters
{
    char interface[10];
    struct timespec cycle_time;
    char t_fault[10];
    bool prints_on;
};
typedef struct configParameters configParameters_t;

/****** GLOBAL VARIABLES *****************************************************/
static bool running = true;
static bool fault_done = false;
char ant_s_fault[10];
char s_fault[10];

configParameters_t c_parameters;
/* Max. and min. V/I values */
int max_volt = 240;
int max_curr = 60;
int min_volt = 220;
int min_curr = 40;
/* ASDU Data Points Value */
float VA, VB, VC, VN;
float IA, IB, IC, IN;

/*** SUBPROGRAMS *************************************************************/
static void initialize_parameters();
static void add_all_ASDU_datapoints(SVPublisher_ASDU asdu, int *data_points);
static void initialize_datapoints_values();
static void initialize_in_range(bool *in_range);
void sigint_handler(int signalId);
int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void connlost(void *context, char *cause);
static int set_config_parameters(int argc, char **argv);
static void set_datapoints_values(SVPublisher_ASDU asdu, int *data_points);
static inline void update_simulation(char *fault, char *buffer, char *ant_buffer, bool *in_range);
static void keep_values_in_range(bool *in_range);
static void usage(char *progname);
static void load_mqtt_payload(float *values);
static void print_send_SV_message();

///////////////////////////////////////////////////////////////////////////////
///     MAIN
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    /* Create, set callbacks, connect to MQTT Server for SCADA communication. */
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    int rc;
    int cycles = 0;

    if (rc = (MQTTClient_create(&client, MQTT_SERVER, CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to create client, return code %d\n", rc);
        return 1;
    }

    if ((MQTTClient_setCallbacks(client, client, connlost, msgarrvd, NULL)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to set callbacks, return code %d\n", rc);
        MQTTClient_destroy(&client);
        return 1;
    }

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        MQTTClient_destroy(&client);
        return 1;
    }

    /* Subscribing to the selected MQTT topics*/
    printf("Subscribing to topic %s\nfor client %s using QoS%d\n\n",
           SUB_TOPIC, CLIENTID, QOS);
    if ((rc = MQTTClient_subscribe(client, SUB_TOPIC, QOS)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to subscribe, return code %d\n", rc);
    }

    /* Setting up variables for the message queue */
    char buffer[QUEUE_MSGSIZE], ant_buffer[QUEUE_MSGSIZE];
    strncpy(buffer, "cerrado", 8);
    strncpy(ant_buffer, "cerrado", 6);
    mqd_t mq;
    
    /* Define the attributes for the mqd_t object */
    struct mq_attr attr;
    attr.mq_flags = 0;
    attr.mq_maxmsg = QUEUE_MAXMSG;
    attr.mq_msgsize = QUEUE_MSGSIZE;

    /* Create message queue */
    mq = mq_open(QUEUE_NAME, O_CREAT | O_RDONLY | O_NONBLOCK, QUEUE_PERMS, &attr);
    if (mq == -1)
    {
        perror("Failed to open a message queue.");
        exit(1);
    }

    /*
     * Obtain different arguments to parameterise the program.
     * These parameters are the interface,the trasmission period and
     * printing mode.
     */
    initialize_parameters();
    set_config_parameters(argc, argv);

    /*
     * Define the SV parameters for the publisher. Then we create the
     * publisher with the interface and parameters defined previously.
     */
    CommParameters svCommParameters;
    svCommParameters.appId = 0x4000;
    /* Stardard IEC61850 Multicast SV Adress */
    svCommParameters.dstAddress[0] = 0x01;
    svCommParameters.dstAddress[1] = 0x0c;
    svCommParameters.dstAddress[2] = 0xcd;
    svCommParameters.dstAddress[3] = 0x01;
    svCommParameters.dstAddress[4] = 0x00;
    svCommParameters.dstAddress[5] = 0x04;
    svCommParameters.vlanId = 0;
    svCommParameters.vlanPriority = 7;

    SVPublisher svPublisher = SVPublisher_create(&svCommParameters, c_parameters.interface);

    if (svPublisher)
    {
        int ASDU_Data_Points[8];
        bool p_in_range[8]; // [0]VA; [1]VB;[2] VC;[3] VN;
                            // [4]IA; [5]IB;[6] IC;[7] IN;

        /* Create ASDU and add data points */
        SVPublisher_ASDU asdu1 = SVPublisher_addASDU(svPublisher, "MSVCB01", "PhsMeas1", 1);
        add_all_ASDU_datapoints(asdu1, ASDU_Data_Points);

        SVPublisher_setupComplete(svPublisher);

        /* Variables initial values */
        initialize_datapoints_values();

        /* Initialize simulation range values */
        initialize_in_range(p_in_range);

        /*
         * Calculation of the next instant of time to send a message synchronized
         * with the cycle time.
         */
        struct timespec t_next = get_next_time(c_parameters.cycle_time);
        smart_sleep(t_next);

        //int x = 0;
        //struct timespec *time_values;
        //time_values = malloc(2 * NUM_SAMPLES * sizeof(struct timespec)); -> Para tiempo de compuatcion y total
        //time_values = malloc(NUM_SAMPLES * sizeof(struct timespec));
        while (running /* && x < NUM_SAMPLES s*/)
        {
            struct timespec ts1;
            clock_gettime(CLOCK_REALTIME, &ts1);

            /* update the values in the SV ASDUs */
            set_datapoints_values(asdu1, ASDU_Data_Points);

            /*
             * Update the message variables values.
             */
            update_simulation(c_parameters.t_fault, buffer, ant_buffer, p_in_range);

            /*
             * Every cicles checks if the upper or lower limit of
             * voltage or current has been exceeded in order to
             * correct the values in the next iteration and keep
             * them within the defined range.
             */
            keep_values_in_range(p_in_range);

            /* Send the SV message */
            SVPublisher_publish(svPublisher);

            /* Print all the data sent on the SV message */
            c_parameters.prints_on ? print_send_SV_message() : NULL;

            /* Update the sample counters */
            SVPublisher_ASDU_increaseSmpCnt(asdu1);

            /*
             * Every 2000 cycles sends a MQTT message containing all
             * simulation values to the SCADA.
             */
            if (cycles == SEND_MQTT_CYCLES)
            {
                float actual_values[8];

                load_mqtt_payload(actual_values);

                pubmsg.payload = (void *)actual_values;
                pubmsg.payloadlen = sizeof(actual_values);
                pubmsg.qos = QOS;
                pubmsg.retained = 0;

                c_parameters.prints_on ? printf("MQTT -> Message Sended.\n") : 0;
                
                if ((rc = MQTTClient_publishMessage(client, PUB_TOPIC, &pubmsg, NULL)) != MQTTCLIENT_SUCCESS)
                {
                    printf("Failed to publish message, return code %d\n", rc);
                }
            }

            /* Receiving mq message */
            ssize_t bytes_read;
            strncpy(ant_buffer, buffer, 8);
            bytes_read = mq_receive(mq, buffer, QUEUE_MSGSIZE, NULL);
            //printf("%s, %s, %i\n", buffer, s_fault, fault_done);

            cycles = (cycles % SEND_MQTT_CYCLES) + 1;

            struct timespec ts2;
            clock_gettime(CLOCK_REALTIME, &ts2);

            /*
             * Calculation of the next instant of time to send a message synchronized
             * with the cycle time.
             */
            t_next = get_next_time(c_parameters.cycle_time);
            smart_sleep(t_next);

            //time_values[2 * x] = ts1;
            //time_values[2 * x + 1] = ts2;

            //time_values[x] = ts1;
            //x++;
        }

        /* Open and Write/Create a file to upload time samples of SVs messages. */
        /*FILE *fp;

        fp = fopen(SAMPLES_FILE, "w");

        if (fp == NULL)
        {
            printf("Failed to open file: %s.\n", SAMPLES_FILE);
            return 1;
        }

        fprintf(fp, "%s;%s\n", "CYCLES", "TA1");

        char buf1[16];
        for (int i = 0; i < x; i++)
        {
            timespec_string_print(time_values[i], buf1, sizeof(buf1));
            fprintf(fp, "%i;%s\n", i, buf1);
        }

        fclose(fp);*/

        SVPublisher_destroy(svPublisher);
    }
    else
    {
        printf("Failed to create SV publisher\n");
    }
    
    /* Disconnect and destroy from the MQTT server*/
    if ((rc = MQTTClient_disconnect(client, 30000)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to disconnect, return code %d\n", rc);
    }

    MQTTClient_destroy(&client);

    /* Close and unlink from message mqueue */
    if (mq_close(mq) == -1)
    {
        perror("Failed to close message queue.");
        exit(1);
    }

    if (mq_unlink(QUEUE_NAME) == -1)
    {
        perror("Failed to unlink message queue.");
        exit(1);
    }

    return 0;
}

///////////////////////////////////////////////////////////////////////////////
///     SUBPROGRAMS DEFINITION
///////////////////////////////////////////////////////////////////////////////

static void initialize_parameters()
{
    strcpy(c_parameters.interface, "enp2s0");
    c_parameters.cycle_time.tv_sec = 0;
    c_parameters.cycle_time.tv_nsec = 250000;
    strcpy(c_parameters.t_fault, "all");
    c_parameters.prints_on = false;
    strcpy(s_fault, "stop");
    strcpy(ant_s_fault, "stop");
}

static void add_all_ASDU_datapoints(SVPublisher_ASDU asdu, int *data_points)
{
    for (int i = 0; i < 8; i++)
    {
        data_points[i] = SVPublisher_ASDU_addFLOAT(asdu);
    }
}

static void initialize_datapoints_values()
{
    VA = 230.1;
    VB = 229.5;
    VC = 231.2;
    VN = 0.0;
    IA = 55.2;
    IB = 55.3;
    IC = 54.8;
    IN = 0.0;
}

static void initialize_in_range(bool *in_range)
{
    for (int i = 0; i < sizeof(in_range) / sizeof(in_range[0]); i++)
    {
        in_range[i] = true;
    }
}

void sigint_handler(int signalId)
{
    running = 0;
}

int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    printf("MQTT -> T: %s ", topicName);
    printf("M: %.*s\n", message->payloadlen, (char *)message->payload);

    if (strcmp(topicName, "TFG_jprodriguez/SCADA/TC-TVTR/type") == 0)
    {
        strncpy(c_parameters.t_fault, message->payload, message->payloadlen);
        c_parameters.t_fault[message->payloadlen] = '\0';
    }
    else if (strcmp(topicName, "TFG_jprodriguez/SCADA/TC-TVTR/state") == 0)
    {
        strncpy(ant_s_fault, s_fault, strlen(s_fault));
        ant_s_fault[strlen(s_fault)] = '\0';

        strncpy(s_fault, message->payload, message->payloadlen);
        s_fault[message->payloadlen] = '\0';
    }

    printf("t_fault: %s\n", c_parameters.t_fault);

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);

    return 1;
}

void connlost(void *context, char *cause)
{
    printf("\nConnection lost\n");
    if (cause)
        printf("     cause: %s\n", cause);
}

static int set_config_parameters(int argc, char **argv)
{
    char *progname;
    int c;
    progname = strrchr(argv[0], '/');
    progname = progname ? 1 + progname : argv[0];

    while (EOF != (c = getopt(argc, argv, "i:m:t:d:ph")))
    {
        switch (c)
        {
        case 'i':
            strcpy(c_parameters.interface, optarg);
            break;
        case 't':
            if (strspn(optarg, "0123456789") == strlen(optarg))
            {
                c_parameters.cycle_time = timespec_from_ns(atoi(optarg));
            }
            else
            {
                printf("Invalid transmission period.\n");
                return -1;
            }
            break;
        case 'p':
            c_parameters.prints_on = true;
            break;
        case 'h':
            usage(progname);
            return 0;
        case '?':
        default:
            usage(progname);
            return -1;
        }
    }

    /* Print all the parameters defined on the arguments */
    printf("Using interface %s\n", c_parameters.interface);
    printf("Tx T: ");
    timespec_print(c_parameters.cycle_time);
    printf("Type of fault simulation: %s\n", c_parameters.t_fault);
    printf("Message prints: %s\n", c_parameters.prints_on ? "ON" : "OFF");
}

static void set_datapoints_values(SVPublisher_ASDU asdu, int *data_points)
{
    SVPublisher_ASDU_setFLOAT(asdu, data_points[0], VA);
    SVPublisher_ASDU_setFLOAT(asdu, data_points[1], VB);
    SVPublisher_ASDU_setFLOAT(asdu, data_points[2], VC);
    SVPublisher_ASDU_setFLOAT(asdu, data_points[3], VN);
    SVPublisher_ASDU_setFLOAT(asdu, data_points[4], IA);
    SVPublisher_ASDU_setFLOAT(asdu, data_points[5], IB);
    SVPublisher_ASDU_setFLOAT(asdu, data_points[6], IC);
    SVPublisher_ASDU_setFLOAT(asdu, data_points[7], IN);
    
}

static inline void update_simulation(char *fault, char *buffer, char *ant_buffer, bool *in_range)
{
    if (strcmp(s_fault, "stop") == 0 && strcmp(buffer, "cerrado") == 0)
    {
        for (int i = 0; i < sizeof(in_range) / sizeof(in_range[0]); i++)
        {
            switch (i)
            {
            case 0:
                if (in_range[i])
                {
                    VA += 0.9f;
                }
                else
                {
                    VA -= 1.1f;
                }
                break;
            case 1:
                if (in_range[i])
                {
                    VB += 0.7f;
                }
                else
                {
                    VB -= 1.2f;
                }
                break;
            case 2:
                if (in_range[i])
                {
                    VC += 0.8f;
                }
                else
                {
                    VC -= 1.3f;
                }
                break;
            case 4:
                if (in_range[i])
                {
                    IA += 0.8f;
                }
                else
                {
                    IA -= 0.5f;
                }
                break;
            case 5:
                if (in_range[i])
                {
                    IB += 0.9f;
                }
                else
                {
                    IB -= 0.7f;
                }
                break;
            case 6:
                if (in_range[i])
                {
                    IC += 0.8f;
                }
                else
                {
                    IC -= 0.6f;
                }
                break;
            }
        }
    }
    else if (strcmp(s_fault, "stop") == 0 && strcmp(buffer, "abierto") == 0)
    {
        VA = 230.1;
        VB = 229.5;
        VC = 231.2;
        VN = 0;

        IA = 55.2;
        IB = 54.8;
        IC = 50.0;
        IN = 0;
    }
    else if (strcmp(s_fault, "start") == 0 && strcmp(buffer, "cerrado") == 0 && !fault_done)
    {
        if (strcmp(fault, "voltage") == 0)
        {
            printf("a\n");
            VA += 100.0f;
            VB += 100.0f;
            VC += 100.0f;
            VN += 200.0f;
        }
        else if (strcmp(fault, "current") == 0)
        {
            printf("b\n");
            IA += 100.0f;
            IB += 100.0f;
            IC += 100.0f;
            IN += 40.0f;
        }
        else if (strcmp(fault, "all") == 0)
        {
            VA += 100.0f;
            VB += 100.0f;
            VC += 100.0f;
            VN += 200.0f;
            IA += 100.0f;
            IB += 100.0f;
            IC += 100.0f;
            IN += 40.0f;
        }
        fault_done = true;
    }
    else if (strcmp(s_fault, "start") == 0 && strcmp(buffer, "abierto") == 0)
    {
        VA = 0;
        VB = 0;
        VC = 0;
        VN = 0;

        IA = 0;
        IB = 0;
        IC = 0;
        IN = 0;

        fault_done = false;
    }
}

static void keep_values_in_range(bool *in_range)
{
    for (int i = 0; i < sizeof(in_range) / sizeof(in_range[0]); i++)
    {
        switch (i)
        {
        case 0:
            if (VA >= max_volt)
            {
                in_range[i] = false;
            }
            else if (VA <= min_volt)
            {
                in_range[i] = true;
            }
            break;
        case 1:
            if (VB >= max_volt)
            {
                in_range[i] = false;
            }
            else if (VB <= min_volt)
            {
                in_range[i] = true;
            }
            break;
        case 2:
            if (VC >= max_volt)
            {
                in_range[i] = false;
            }
            else if (VC <= min_volt)
            {
                in_range[i] = true;
            }
            break;
        case 4:
            if (IA >= max_curr)
            {
                in_range[i] = false;
            }
            else if (IA <= min_curr)
            {
                in_range[i] = true;
            }
            break;
        case 5:
            if (IB >= max_curr)
            {
                in_range[i] = false;
            }
            else if (IB <= min_curr)
            {
                in_range[i] = true;
            }
            break;
        case 6:
            if (IC >= max_curr)
            {
                in_range[i] = false;
            }
            else if (IC <= min_curr)
            {
                in_range[i] = true;
            }
            break;
        }
    }
}

static void load_mqtt_payload(float *values)
{
    values[0] = VA;
    values[1] = VB;
    values[2] = VC;
    values[3] = VN;
    values[4] = IA;
    values[5] = IB;
    values[6] = IC;
    values[7] = IN;
}

static void usage(char *progname)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            " -i name    interface to use(default 'eth0')\n"
            " -t number  message tx period(default 500 ms).\n"
            " -p         enables the printing of SV message contents(default off)."
            " -h         prints this message.\n",
            progname);
}

static void print_send_SV_message()
{
    printf("////////////////////////////////////////////////////////////////\n");
    printf("///NEW MESSAGE SENT/////////////////////////////////////////////\n");
    printf("/// ***Device: PMU****************************************** ///\n");
    printf("///Voltaje Fase A:\n");
    printf("///   Magnitud: %f\n", VA);
    printf("///Voltaje Fase B:\n");
    printf("///   Magnitud: %f\n", VB);
    printf("///Voltaje Fase C:\n");
    printf("///   Magnitud: %f\n", VC);
    printf("///Voltaje Neutro:\n");
    printf("///   Magnitud: %f\n", VN);
    printf("///Corriente Fase A:\n");
    printf("///   Magnitud: %f\n", IA);
    printf("///Corriente Fase B:\n");
    printf("///   Magnitud: %f\n", IB);
    printf("///Corriente Fase C:\n");
    printf("///   Magnitud: %f\n", IC);
    printf("///Corriente Neutro:\n");
    printf("///   Magnitud: %f\n", IN);
    printf("////////////////////////////////////////////////////////////////\n\n");
}