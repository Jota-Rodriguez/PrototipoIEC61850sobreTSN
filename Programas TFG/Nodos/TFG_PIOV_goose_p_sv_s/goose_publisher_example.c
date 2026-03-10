/**
    @file goose_publisher_example.c
    @author Joan Pau Rodriguez Sureda
    @brief Simulating the behaviour of a PIOV. Receives data via a Sampled
    Values (SV) subscriber, analizes this data and sends the results using a
    Generic Object Oriented Substation Event (GOOSE) publisher based on
    IEC61850 standard.

    @note In this program the clock CLOCK_REALTIME is synchronized with the TSN
    switch.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <pthread.h>
#include <MQTTClient.h>

#include "mms_value.h"
#include "goose_publisher.h"
#include "hal_thread.h"
#include "sv_subscriber.h"
#include "../libraries/ts.h"

#define CLIENTID "jp3252532"
#define MQTT_SERVER "tom.uib.es"
#define TOPIC "TFG_jprodriguez/Maqueta/PIOV"
#define QOS 1

#define NUM_SAMPLES 100001
#define SAMPLES_FILE "/home/joanpau/libiec61850-1.5/examples/TFG_PIOV_gooseP_sv_s/11-06-25_muestreo_ms_1.txt"

#define PULSATING_GOOSE_MESSAGE 0
#define BURST_GOOSE_MESSAGE 1

///////////////////////////////////////////////////////////////////////////////
///      SUBPROGRAMS AND GLOBAL VARIABLES DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

struct configParameters
{
    char *interface;
    struct timespec cycle_time;
    int prints_on;
};
typedef struct configParameters configParameters_t;

/*** GLOBAL VARIABLES ********************************************************/
static bool running = true;

configParameters_t c_parameters;
/* File variables */
int x;
struct timespec *time_values;
/* Time and clock variables */
struct timespec t_next;
/* MQTT Variables */
MQTTClient client;
/* Thread variables */
static bool th_HP_running = false;
static bool th_LP_running = true;
pthread_mutex_t mutex;
pthread_cond_t tx;
pthread_t t1, t2;
/* Max. and min. V/I values */
static const float max_volt = 250.0;
/* ASDU Data Variables */
static float VA, VB, VC, VN;
/* Goose Control block name*/
char *piov_gcb = "IDE.PIOV/LLN0$GO$gcbPIOV";
/* GOOSE Publishers */
GoosePublisher publisherHP, publisherLP;
static int sqNum = 0;
static int stNum = 0;
/* GPDU Data Variables */
MmsValue *MmsV1;
Timestamp ts;
LinkedList dataSetValues;
static bool hay_alarma = false;
static bool ant_hay_alarma = false;

/*** SUBPROGRAMS *************************************************************/
static void initialize_parameters(struct configParameters *parameters);
void sigint_handler(int signalId);
int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void connlost(void *context, char *cause);
static int set_config_parameters(int argc, char **argv, struct configParameters *parameters);
static void svUpdateListener(SVSubscriber subscriber, void *parameter, SVSubscriber_ASDU asdu);
void *th_goose_message_HP(void *arg);
void *th_goose_message_LP(void *arg);
static void usage(char *progname);
static void print_received_SV_message(SVSubscriber_ASDU asdu);
static void print_sended_GOOSE_message(int type);

///////////////////////////////////////////////////////////////////////////////
///     MAIN
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    //time_values = malloc(2 * NUM_SAMPLES * sizeof(struct timespec));

    /* Create, set callbacks, connect to MQTT Server for SCADA communication. */
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    int rc;

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

    char *payload;
    payload = hay_alarma ? "true\0" : "false\0";
    pubmsg.payload = (void *)payload;
    pubmsg.payloadlen = (int)strlen(payload);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;
    if ((rc = MQTTClient_publishMessage(client, TOPIC, &pubmsg, NULL)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to publish message, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }

    /*
     * Obtain different arguments to parameterise the program.
     * These parameters are the interface,the trasmission period and
     * printing mode.
     */
    initialize_parameters(&c_parameters);
    set_config_parameters(argc, argv, &c_parameters);

    /*
     * Creating the SV Receiver who will listen to the incoming SV messages
     * from the ethernet interface.
     */
    SVReceiver receiver = SVReceiver_create();
    SVReceiver_setInterfaceId(receiver, c_parameters.interface);

    /* Create a subscriber listening to SV messages with APPID 0x4000 */
    SVSubscriber subscriber = SVSubscriber_create(NULL, 0x4000);

    /* Install a callback handler for the subscriber */
    SVSubscriber_setListener(subscriber, svUpdateListener, NULL);

    /* Connect the subscriber to the receiver */
    SVReceiver_addSubscriber(receiver, subscriber);

    /*
     * Define the GOOSE parameters for the publisher. Then we create the
     * publisher with the interface and parameters defined previously.
     */
    CommParameters gooseCommParameters;
    /* Stardard IEC61850 Multicast GOOSE Adress */
    gooseCommParameters.appId = 1000;
    gooseCommParameters.dstAddress[0] = 0x01;
    gooseCommParameters.dstAddress[1] = 0x0c;
    gooseCommParameters.dstAddress[2] = 0xcd;
    gooseCommParameters.dstAddress[3] = 0x01;
    gooseCommParameters.dstAddress[4] = 0x00;
    gooseCommParameters.dstAddress[5] = 0x01;
    gooseCommParameters.vlanId = 0;
    gooseCommParameters.vlanPriority = 6;
    publisherHP = GoosePublisher_create(&gooseCommParameters, c_parameters.interface);

    /* Creating a second Publisher for messages with less priority*/
    gooseCommParameters.vlanPriority = 5;
    publisherLP = GoosePublisher_create(&gooseCommParameters, c_parameters.interface);

    if (publisherHP && publisherLP)
    {
        /* Set up the GOOSE Publisher parameters */
        GoosePublisher_setGoCbRef(publisherHP, piov_gcb);
        GoosePublisher_setGoCbRef(publisherLP, piov_gcb);
        GoosePublisher_setConfRev(publisherHP, 1);
        GoosePublisher_setConfRev(publisherLP, 1);
        GoosePublisher_setDataSetRef(publisherHP, "IDE.PIOV/LLN0$GO.HP$PIOV");
        GoosePublisher_setDataSetRef(publisherLP, "IDE.PIOV/LLN0$GO.LP$PIOV");
        GoosePublisher_setTimeAllowedToLive(publisherHP, timespec_get_ns(c_parameters.cycle_time) / NSEC_PER_MSEC);
        GoosePublisher_setTimeAllowedToLive(publisherLP, timespec_get_ns(c_parameters.cycle_time) / NSEC_PER_MSEC);

        /* Create and load the GOOSE mesages dataset.*/
        dataSetValues = LinkedList_create();

        MmsV1 = MmsValue_newBoolean(hay_alarma);

        LinkedList_add(dataSetValues, MmsV1);

        /* Start listening to SV messages - starts a new receiver background thread */
        SVReceiver_start(receiver);

        if (SVReceiver_isRunning(receiver))
        {
            /* Initialization of the GOOSE Publisher threads, mutex and conditional varaiable*/
            if (pthread_mutex_init(&mutex, NULL) != 0)
            {
                printf("Failed to initialize mutex.\n");
                return -1;
            }

            if (pthread_cond_init(&tx, NULL) != 0)
            {
                printf("Failed to initialize conditional variable.\n");
                return -1;
            }

            if (pthread_create(&t1, NULL, th_goose_message_LP, NULL) != 0)
            {
                printf("Failed to create LP thread.");
                return -1;
            }

            if (pthread_create(&t2, NULL, th_goose_message_HP, NULL) != 0)
            {
                printf("Failed to create HP thread.");
                return -1;
            }

            pthread_join(t1, NULL);
            pthread_join(t2, NULL);

            /* Stop listening to SV messages */
            SVReceiver_stop(receiver);
        }
        else
        {
            printf("Failed to start SV subscriber. Reason can be that the Ethernet interface doesn't exist or root permission are required.\n");
        }
        /* Destory GOOSE dataset and Publishers */
        LinkedList_destroyDeep(dataSetValues, (LinkedListValueDeleteFunction)MmsValue_delete);
        GoosePublisher_destroy(publisherHP);
        GoosePublisher_destroy(publisherLP);
    }
    else
    {
        printf("Failed to create GOOSE publisher. Reason can be that the Ethernet interface doesn't exist or root permission are required.\n");
    }

    /* Disconnect and destroy from the MQTT server*/
    if ((rc = MQTTClient_disconnect(client, 30000)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to disconnect, return code %d\n", rc);
    }

    /* Open and Write/Create a file to upload time samples of SVs messages. */
    /*FILE *fp;

    fp = fopen(SAMPLES_FILE, "w");

    if (fp == NULL)
    {
        printf("Failed to open file: %s.\n", SAMPLES_FILE);
        return 1;
    }

    printf("a");

    fprintf(fp, "%s;%s\n", "CYCLES", "TIME");

    char buf1[16], buf2[16];
    struct timespec total;
    for (int i = 1; i < x; i++)
    {
        total = timespec_from_ns(timespec_diff(time_values[2 * i - 2], time_values[2 * i]));
        timespec_string_print(total, buf2, sizeof(buf2));
        fprintf(fp, "%i;%s\n", i, buf2);
    }

    fclose(fp);*/

    MQTTClient_destroy(&client);

    /* Destroy threads, mutex and condition variable. */
    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&tx);
    pthread_exit(NULL);

    return 0;
}

///////////////////////////////////////////////////////////////////////////////
///     SUBPROGRAMS DEFINITION
///////////////////////////////////////////////////////////////////////////////
static void initialize_parameters(struct configParameters *parameters)
{
    parameters->interface = "enp2s0";
    parameters->cycle_time.tv_sec = 1;
    parameters->cycle_time.tv_nsec = 0;
    parameters->prints_on = 0;
}

void sigint_handler(int signalId)
{
    running = 0;
    pthread_cancel(t1);
    pthread_cancel(t2);
}

int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
}

void connlost(void *context, char *cause)
{
    printf("\nConnection lost\n");
    if (cause)
        printf("     cause: %s\n", cause);
}

static int set_config_parameters(int argc, char **argv, struct configParameters *parameters)
{
    char *progname;
    int c;
    progname = strrchr(argv[0], '/');
    progname = progname ? 1 + progname : argv[0];

    while (EOF != (c = getopt(argc, argv, "i:m:t:d:p:h")))
    {
        switch (c)
        {
        case 'i':
            parameters->interface = optarg;
            break;

        case 't':
            if (strspn(optarg, "0123456789") == strlen(optarg))
            {
                parameters->cycle_time = timespec_from_ns(atol(optarg));
            }
            else
            {
                printf("Invalid transmission period.\n");
                return -1;
            }
            break;

        case 'p':
            if (strspn(optarg, "01234") == strlen(optarg))
            {
                parameters->prints_on = atoi(optarg);
            }
            else
            {
                printf("Invalid type of print.\n");
                return -1;
            }
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
    printf("Using interface %s\n", parameters->interface);
    printf("Tx T: ");
    timespec_print(parameters->cycle_time);
    printf("Message prints: ");
    switch (parameters->prints_on)
    {
    case 0:
        printf("OFF\n");
        break;

    case 1:
        printf("TYPE 1. Tx Message content.\n");
        break;

    case 2:
        printf("TYPE 2. Rx Message content.\n");
        break;

    case 3:
        printf("TYPE 3. Tx/Rx Message content.\n");
        break;

    case 4:
        printf("TYPE 4. Thread managing.\n");
        break;

    default:
        break;
    }
}

static void svUpdateListener(SVSubscriber subscriber, void *parameter, SVSubscriber_ASDU asdu)
{
    //if (c_parameters.prints_on == 4)
        //printf("SV --> Message Received\n");
    /*
     * Access to the data requires a priori knowledge of the data set.
     * To prevent damages due configuration, must check the length of the
     * data block of the SV message before accessing the data.
     */
    if (SVSubscriber_ASDU_getDataSize(asdu) > 0)
    {
        VA = SVSubscriber_ASDU_getFLOAT32(asdu, 0);
        VB = SVSubscriber_ASDU_getFLOAT32(asdu, 4);
        VC = SVSubscriber_ASDU_getFLOAT32(asdu, 8);
        VN = SVSubscriber_ASDU_getFLOAT32(asdu, 12);
    }
    else
    {
        printf("El ASDU esta vacio.");
    }

    if (c_parameters.prints_on == 2 || c_parameters.prints_on == 3)
        print_received_SV_message(asdu);

    /* Analize data to detect any problem in the system. */
    ant_hay_alarma = hay_alarma;

    if (VA > max_volt || VB > max_volt || VC > max_volt || VA == 0 || VB == 0 || VC == 0 || VN > 0)
    {
        hay_alarma = true;
    }
    else
    {
        hay_alarma = false;
    }

    /*
     * If the state of the system changes starts the process of interrumpting
     * the LP GOOSE Publisher Thread and stating the HP GOOSE Publisher Thread.
     */
    if (ant_hay_alarma != hay_alarma && !th_HP_running)
    {
        if (c_parameters.prints_on == 4)
            printf("SV --> Alarm ||| VA: %f  VB: %f  VC: %f  VN: %f\n", VA, VB, VC, VN);
        pthread_mutex_lock(&mutex);
        if (c_parameters.prints_on == 4)
            printf("SV -> Locked\n");

        pthread_cancel(t1);
        th_LP_running = false;
        pthread_cond_signal(&tx);

        pthread_mutex_unlock(&mutex);
        if (c_parameters.prints_on == 4)
            printf("SV -> Unlocked\n");
    }
}

void *th_goose_message_HP(void *arg)
{
    pthread_mutex_lock(&mutex);
    if (c_parameters.prints_on == 4)
        printf("HP -> Locked 1\n");

    while (running /*&& x < NUM_SAMPLES*/)
    {
        /*
         * Waits until the state of the system changes, then sends 4 HP
         * GOOSE messages with T = 1ms. Ater thath returns to waiting.
         */
        th_HP_running = false;
        if (c_parameters.prints_on == 4)
            printf("HP -> Waiting ...\n");
        pthread_cond_wait(&tx, &mutex);
        th_HP_running = true;
        if (c_parameters.prints_on == 4)
            printf("HP -> Exit waiting.\n");

        struct timespec t = timespec_from_ns(1000000);

        MmsValue_setBoolean(MmsV1, hay_alarma);

        sqNum = 0;
        stNum++;
        GoosePublisher_increaseStNum(publisherHP);

        if (GoosePublisher_publish(publisherHP, dataSetValues) == -1)
        {
            printf("Error sending GOOSE message!\n");
        }

        MQTTClient_message pubmsg = MQTTClient_message_initializer;
        char *payload;
        payload = hay_alarma ? "true\0" : "false\0";
        pubmsg.payload = (void *)payload;
        pubmsg.payloadlen = (int)strlen(payload);
        pubmsg.qos = QOS;
        pubmsg.retained = 0;
        int rc;
        if ((rc = MQTTClient_publishMessage(client, TOPIC, &pubmsg, NULL)) != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to publish message, return code %d\n", rc);
            exit(EXIT_FAILURE);
        }

        if (c_parameters.prints_on == 4)
            printf("HP --> Message Sended. -> hay_alarma: %i\n", hay_alarma);

        if (c_parameters.prints_on == 1 || c_parameters.prints_on == 3)
        {
            print_sended_GOOSE_message(BURST_GOOSE_MESSAGE);
        }
        sqNum++;

        for (int i = 0; i < 3; i++)
        {
            if (c_parameters.prints_on == 4)
            {
                printf("HP -> Going to sleep ");
                timespec_print(t);
                printf("HP -> Unlocking 1\n");
            }
            pthread_mutex_unlock(&mutex);

            //struct timespec ts2;
            //clock_gettime(CLOCK_REALTIME, &ts2);

            t_next = get_next_time(t);
            smart_sleep(t_next);

            //struct timespec ts1;
            //clock_gettime(CLOCK_REALTIME, &ts1);

            pthread_mutex_lock(&mutex);
            if (c_parameters.prints_on == 4)
                printf("HP -> Locked 2\n");

            MmsValue_setBoolean(MmsV1, hay_alarma);

            if (GoosePublisher_publish(publisherHP, dataSetValues) == -1)
            {
                printf("Error sending GOOSE message!\n");
            }

            if (c_parameters.prints_on == 4)
                printf("HP --> Message Sended\n");

            if (c_parameters.prints_on == 1 || c_parameters.prints_on == 3)
                print_sended_GOOSE_message(BURST_GOOSE_MESSAGE);

            //time_values[2 * x] = ts1;
            //time_values[2 * x + 1] = ts2;    

            sqNum++;
        }
        GoosePublisher_setSqNum(publisherLP, sqNum);
        GoosePublisher_setStNum(publisherLP, stNum);

        //x++;

        if (pthread_create(&t1, NULL, th_goose_message_LP, NULL) != 0)
        {
            printf("Failed to create LP thread.\n");
        }
    }
    if (c_parameters.prints_on == 4)
        printf("HP -> Unlocking 2\n");
    pthread_mutex_unlock(&mutex);
}

void *th_goose_message_LP(void *arg)
{
    /*
     * Sends LP GOOSE messages with a T = 1s, while the system state doesn't
     * change.
     */
    pthread_mutex_lock(&mutex);
    if (c_parameters.prints_on == 4)
        printf("LP -> Locked 1\n");
    while (running)
    {
        th_LP_running = true;
        if (c_parameters.prints_on == 4)
            printf("LP -> Unlocking 1\n");
        pthread_mutex_unlock(&mutex);
        if (c_parameters.prints_on == 4)
            printf("LP -> Going to sleep\n");

        t_next = get_next_time(c_parameters.cycle_time);
        smart_sleep(t_next);

        pthread_mutex_lock(&mutex);
        if (c_parameters.prints_on == 4)
            printf("LP -> Locked 2\n");

        if (GoosePublisher_publish(publisherLP, dataSetValues) == -1)
        {
            printf("Error sending GOOSE message!\n");
        }

        MQTTClient_message pubmsg = MQTTClient_message_initializer;
        char *payload;
        payload = hay_alarma ? "true\0" : "false\0";
        pubmsg.payload = (void *)payload;
        pubmsg.payloadlen = (int)strlen(payload);
        pubmsg.qos = QOS;
        pubmsg.retained = 0;
        int rc;
        if ((rc = MQTTClient_publishMessage(client, TOPIC, &pubmsg, NULL)) != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to publish message, return code %d\n", rc);
            exit(EXIT_FAILURE);
        }

        if (c_parameters.prints_on == 4)
            printf("LP --> Message Sended\n");

        if (c_parameters.prints_on == 1 || c_parameters.prints_on == 3)
        {
            print_sended_GOOSE_message(PULSATING_GOOSE_MESSAGE);
        }

        sqNum++;
    }
    if (c_parameters.prints_on == 4)
        printf("LP -> Unlocking 2\n");
    pthread_mutex_unlock(&mutex);
}

static void usage(char *progname)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            " -i name    interface to use(default 'eth0')\n"
            "               [normal]: no sinchronized with switch gate opening(default)\n"
            "               [tsn]: sinchronized with switch gate opening\n"
            " -t number  message tx period(default 500 ms)\n"
            " -p         enables the printing of SV message contents(default off).\n"
            "               [0]: prints disabled\n"
            "               [1]: prints enabled. Tx message content\n"
            "               [2]: prints enabled. Rx message content\n"
            "               [3]: prints enabled. Tx/Rx message content\n"
            "               [4]: prints enabled. Thread managing\n"
            " -h         prints this message\n",
            progname);
}

static void print_received_SV_message(SVSubscriber_ASDU asdu)
{
    printf("   /////////////////////////////////////////////////////////////\n");
    printf("   //////svUpdateListener FUNCTION CALLED///////////////////////\n");
    printf("   /// ***Device: IDE2************************************** ///\n");

    const char *svID = SVSubscriber_ASDU_getSvId(asdu);

    if (svID != NULL)
        printf("   ///svID: %s\n", svID);

    if (SVSubscriber_ASDU_hasDatSet(asdu))
    {
        printf("   ///Data Set: %s\n", SVSubscriber_ASDU_getDatSet(asdu));
    }
    else
    {
        printf("   ///No contiene DatSet.\n");
    }

    printf("   ///Sample Counter: %i\n", SVSubscriber_ASDU_getSmpCnt(asdu));
    printf("   ///confRev: %u\n", SVSubscriber_ASDU_getConfRev(asdu));

    if (SVSubscriber_ASDU_hasRefrTm(asdu))
    {
        printf("   ///Refresh Time: %ld ms\n", SVSubscriber_ASDU_getRefrTmAsMs(asdu));
    }
    else
    {
        printf("   ///No contiene RefrTm.\n");
    }

    printf("   ///Sample Synchronization: %i\n", SVSubscriber_ASDU_getSmpSynch(asdu));

    if (SVSubscriber_ASDU_hasSmpRate(asdu))
    {
        printf("   ///Samp Rate: %d\n", SVSubscriber_ASDU_getSmpRate(asdu));
    }
    else
    {
        printf("   ///No contiene SmpRate.\n");
    }

    if (SVSubscriber_ASDU_hasSmpMod(asdu))
    {
        printf("   ///Sample Mode: %d\n", SVSubscriber_ASDU_getSmpMod(asdu));
    }
    else
    {
        printf("   ///No contiene SmpMod.\n");
    }
    printf("   ///Data Size: %i\n", SVSubscriber_ASDU_getDataSize(asdu));
    printf("   /// ***DATA ********************************************* ///\n");
    printf("   ///Mag. VA: %f\n", VA);
    printf("   ///Mag. VB: %f\n", VB);
    printf("   ///Mag. VC: %f\n", VC);
    printf("   ///Mag. VN: %f\n", VN);
    printf("   /////////////////////////////////////////////////////////////\n\n");

    if (hay_alarma)
    {
        printf("   *************************************************************\n");
        printf("   ****         ALARM. VOLTAGE VALUES OUT OF RANGE.         ****\n");
        printf("   *************************************************************\n");
    }
}

static void print_sended_GOOSE_message(int type)
{
    printf("////////////////////////////////////////////////////////////////\n");
    if (type == PULSATING_GOOSE_MESSAGE)
    {
        printf("/// GOOSE PULSATING MESSAGE ////////////////////////////////////\n");
    }
    else if (type == BURST_GOOSE_MESSAGE)
    {
        printf("//////GOOSE EVENT DATA UPDATE///////////////////////////////////\n");
    }
    printf("/// ***Device: IDE2***************************************** ///\n");
    printf("//////Alarm: %s\n", hay_alarma ? "YES" : "NO");
    printf("//////sqNum: %i\n", sqNum);
    printf("//////stNum: %i\n", stNum);
    printf("////////////////////////////////////////////////////////////////\n\n");
}