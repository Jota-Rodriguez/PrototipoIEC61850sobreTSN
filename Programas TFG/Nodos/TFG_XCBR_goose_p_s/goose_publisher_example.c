/**
    @file goose_publisher_example.c
    @author Joan Pau Rodriguez Sureda
    @brief Simulating the behaviour of a XCBR. Receives data via a Generic
    Object Oriented Substation Event (GOOSE) subscriber, analizes this data and
    sends the results using a GOOSE publisher based on IEC61850 standard.

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
#include <mqueue.h>
#include <sys/types.h>
#include <pthread.h>
#include <sys/stat.h>
#include <MQTTClient.h>

#include "mms_value.h"
#include "goose_receiver.h"
#include "goose_subscriber.h"
#include "goose_publisher.h"
#include "hal_thread.h"
#include "linked_list.h"
#include "../libraries/ts.h"

#define CLIENTID "jp3252535"
#define MQTT_SERVER "tom.uib.es"
#define SUB_TOPIC "TFG_jprodriguez/SCADA/XCBR"
#define PUB_TOPIC "TFG_jprodriguez/Maqueta/XCBR"
#define QOS 1

#define NUM_SAMPLES 5000
#define SAMPLES_FILE "/home/joanpau/libiec61850-1.5/examples/TFG_XCBR_goose_p_s/muestreo_ta_LP_NM_XCBR_.csv"

#define QUEUE_NAME "/mq_TC-TVTR_XCBR"
#define QUEUE_PERMS ((int)(0644))
#define QUEUE_MAXMSG 16
#define QUEUE_MSGSIZE 1024

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
/* File Variables */
int x;
struct timespec *time_values;
/* Mqueue variables */
mqd_t mq;
char msg[QUEUE_MSGSIZE];
/* Time and clock variables*/
struct timespec t_next;
/* MQTT Variables */
MQTTClient client;
/* Thread variables */
static bool th_HP_running = false;
static bool th_LP_running = true;
pthread_mutex_t mutex;
pthread_cond_t tx;
pthread_t t1, t2;
/* Goose Control block names */
char *cswi_gcb = "IDE.CSWI/LLN0$GO$gcbCSWI";
char *xcbr_gcb = "IDE.XCBR/LLN0$GO$gcbXCBR";
/* GOOSE Publishers */
GoosePublisher publisherHP, publisherLP;
static int sqNum = 0;
static int stNum = 0;
/* GPDU Data Variables */
MmsValue *MmsV1;
Timestamp ts;
LinkedList dataSetValues;
static bool xcbr_state = false;
static bool ant_xcbr_state = false;
static int8_t t_alarma = 0;

/*** SUBPROGRAMS *************************************************************/
static void initialize_parameters(struct configParameters *parameters);
void sigint_handler(int signalId);
int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message);
void connlost(void *context, char *cause);
static int set_config_parameters(int argc, char **argv, struct configParameters *parameters);
static void printParseErrorCode(GooseSubscriber self);
static void gooseListener(GooseSubscriber subscriber, void *parameter);
void *th_goose_message_HP(void *arg);
void *th_goose_message_LP(void *arg);
static void usage(char *progname);
static void print_received_GOOSE_message(GooseSubscriber subscriber, MmsValue *values);
static void print_GOOSE_DataSet(MmsValue *values);
static void print_sended_GOOSE_message(int type);

///////////////////////////////////////////////////////////////////////////////
///     MAIN
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    time_values = malloc(NUM_SAMPLES * sizeof(struct timespec));

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

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    printf("Subscribing to topic %s\nfor client %s using QoS%d\n\n",
           SUB_TOPIC, CLIENTID, QOS);
    if ((rc = MQTTClient_subscribe(client, SUB_TOPIC, QOS)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to subscribe, return code %d\n", rc);
    }
    char *payload;
    payload = xcbr_state ? "true\0" : "false\0";
    pubmsg.payload = (void *)payload;
    pubmsg.payloadlen = (int)strlen(payload);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;
    if ((rc = MQTTClient_publishMessage(client, PUB_TOPIC, &pubmsg, NULL)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to publish message, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }

    /* Define the attributes for the mqd_t object */
    struct mq_attr attr;
    attr.mq_flags = 0;
    attr.mq_maxmsg = QUEUE_MAXMSG;
    attr.mq_msgsize = QUEUE_MSGSIZE;

    /* Create message queue */
    mq = mq_open(QUEUE_NAME, O_WRONLY | O_NONBLOCK, QUEUE_PERMS, &attr);
    if (mq == (mqd_t)-1)
    {
        perror("Failed to open a message queue.");
        exit(1);
    }

    /*
     * Obtain different arguments to parameterise the program.
     * These parameters are the interface,the trasmission period and
     * printing mode.
     */
    initialize_parameters(&c_parameters);
    set_config_parameters(argc, argv, &c_parameters);

    /*
     * Creating the GOOSE Receiver who will listen to the incoming GOOSE messages
     * from the ethernet interface. The second Receiver is for the virtual
     * connection with PIOC node.
     */
    GooseReceiver receiver = GooseReceiver_create();
    GooseReceiver_setInterfaceId(receiver, c_parameters.interface);

    /* Create a subscriber listening to especific GCB */
    GooseSubscriber subscriber = GooseSubscriber_create(cswi_gcb, NULL);

    /* Stardard IEC61850 Multicast GOOSE Adress */
    uint8_t dstMac[6] = {0x01, 0x0c, 0xcd, 0x01, 0x00, 0x01};
    GooseSubscriber_setDstMac(subscriber, dstMac);
    GooseSubscriber_setAppId(subscriber, 1000);

    /* Install a callback handler for the subscriber */
    GooseSubscriber_setListener(subscriber, gooseListener, NULL);

    /* Connect the subscriber to the receiver */
    GooseReceiver_addSubscriber(receiver, subscriber);

    CommParameters gooseCommParameters;

    gooseCommParameters.appId = 1000;
    gooseCommParameters.dstAddress[0] = dstMac[0];
    gooseCommParameters.dstAddress[1] = dstMac[1];
    gooseCommParameters.dstAddress[2] = dstMac[2];
    gooseCommParameters.dstAddress[3] = dstMac[3];
    gooseCommParameters.dstAddress[4] = dstMac[4];
    gooseCommParameters.dstAddress[5] = dstMac[5];
    gooseCommParameters.vlanId = 0;
    gooseCommParameters.vlanPriority = 6;
    publisherHP = GoosePublisher_create(&gooseCommParameters, c_parameters.interface);

    /* Creating a second Publisher for messages with less priority*/
    gooseCommParameters.vlanPriority = 5;
    publisherLP = GoosePublisher_create(&gooseCommParameters, c_parameters.interface);

    if (publisherHP && publisherLP)
    {
        /* Set up the GOOSE Publisher parameters */
        GoosePublisher_setGoCbRef(publisherHP, xcbr_gcb);
        GoosePublisher_setGoCbRef(publisherLP, xcbr_gcb);
        GoosePublisher_setConfRev(publisherHP, 1);
        GoosePublisher_setConfRev(publisherLP, 1);
        GoosePublisher_setDataSetRef(publisherHP, "IDE.XCBR/LLN0$GO.HP$XCBR");
        GoosePublisher_setDataSetRef(publisherLP, "IDE.XCBR/LLN0$GO.LP$XCBR");
        GoosePublisher_setTimeAllowedToLive(publisherHP, timespec_get_ns(c_parameters.cycle_time) / NSEC_PER_MSEC);
        GoosePublisher_setTimeAllowedToLive(publisherLP, timespec_get_ns(c_parameters.cycle_time) / NSEC_PER_MSEC);

        /* Create and load the GOOSE mesages dataset.*/
        dataSetValues = LinkedList_create();

        MmsV1 = MmsValue_newBoolean(xcbr_state);

        LinkedList_add(dataSetValues, MmsV1);

        /* Start listening to GOOSE messages - starts a new receiver background thread */
        GooseReceiver_start(receiver);

        if (GooseReceiver_isRunning(receiver))
        {
            /* Initialization of the GOOSE Publisher threads, mutex and conditional varaiable */
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

            if (pthread_create(&t2, NULL, th_goose_message_HP, NULL) != 0)
            {
                printf("Failed to create HP thread.");
                return -1;
            }

            if (pthread_create(&t1, NULL, th_goose_message_LP, NULL) != 0)
            {
                printf("Failed to create LP thread.");
                return -1;
            }

            pthread_join(t1, NULL);
            pthread_join(t2, NULL);

            /* Open and Write/Create a file to upload time samples of SVs messages. */
            /*FILE *fp;

            fp = fopen(SAMPLES_FILE, "w");

            if (fp == NULL)
            {
                printf("Failed to open file: %s.\n", SAMPLES_FILE);
                return 1;
            }

            fprintf(fp, "%s;%s\n", "CYCLES", "TA1");

            char buf1[32];
            for (int i = 0; i < x; i++)
            {
                timespec_string_print(time_values[i], buf1, sizeof(buf1));
                fprintf(fp, "%i;%s\n", i, buf1);
            }

            fclose(fp);*/

            /* Stop listening to GOOSE messages */
            GooseReceiver_stop(receiver);
        }
        else
        {
            printf("Failed to start GOOSE subscriber. Reason can be that the Ethernet interface doesn't exist or root permission are required.\n");
        }
        /* Destory GOOSE dataset and Publishers*/
        LinkedList_destroyDeep(dataSetValues, (LinkedListValueDeleteFunction)MmsValue_delete);
        GoosePublisher_destroy(publisherHP);
        GoosePublisher_destroy(publisherLP);
    }
    else
    {
        printf("Failed to create GOOSE publisher. Reason can be that the Ethernet interface doesn't exist or root permission are required.\n");
    }

    /* Disconnect from the MQTT server*/
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
    char *s_state;

    pthread_mutex_lock(&mutex);
    if (c_parameters.prints_on == 4)
        printf("MQTT -> Locked\n");

    printf("MQTT -> T: %s ", topicName);
    printf("M: %.*s\n", message->payloadlen, (char *)message->payload);

    ant_xcbr_state = xcbr_state;

    if (strcmp((char *)message->payload, "close") == 0)
    {
        xcbr_state = false;
        s_state = "cerrado";
        snprintf(msg, sizeof(msg), "%s", s_state);
    }
    else if (strcmp((char *)message->payload, "open") == 0)
    {
        xcbr_state = true;
        s_state = "abierto";
        snprintf(msg, sizeof(msg), "%s", s_state);
    }

    if (mq_send(mq, msg, strlen(msg), 0) == -1)
    {
        printf("Failed to send mqueue message.1\n");
    }

    if (c_parameters.prints_on == 4)
        printf("MQTT -> Mq message: %s\n", s_state);

    pthread_mutex_unlock(&mutex);
    if (c_parameters.prints_on == 4)
        printf("MQTT -> Unlocked\n");

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

static void printParseErrorCode(GooseSubscriber self)
{

    int error = GooseSubscriber_getParseError(self);

    printf("   //////ParseError code: %i.\n", error);

    switch (error)
    {

    case 0:
        printf("   //////No hay errores.");
        break;

    case 1:
        printf("   //////ERROR UNKNOWN TAG.");
        break;

    case 2:
        printf("   //////ERROR TAGCODE.");
        break;

    case 3:
        printf("   //////ERROR SUBLEVEL.");
        break;

    case 4:
        printf("   //////ERROR OVERFLOW.");
        break;

    case 5:
        printf("   //////ERROR UNDERFLOW.");
        break;

    case 6:
        printf("   //////ERROR TYPE MISMATCH.");
        break;

    case 7:
        printf("   //////ERROR LENGTH MISMATCH.");
        break;

    case 8:
        printf("   //////ERROR PADDING.");
        break;
    }

    if (GooseSubscriber_isValid(self))
    {
        printf(" VALID message.\n");
    }
    else
    {
        printf(" INVALID message.\n");
    }
}

static void gooseListener(GooseSubscriber subscriber, void *parameter)
{
    if (c_parameters.prints_on == 4)
        printf("GL --> Message Received. %s -> ", GooseSubscriber_getGoCbRef(subscriber));

    MmsValue *values = GooseSubscriber_getDataSetValues(subscriber);

    if (c_parameters.prints_on == 2 || c_parameters.prints_on == 3)
        print_received_GOOSE_message(subscriber, values);

    MmsValue *aux = MmsValue_getElement(values, 0);

    t_alarma = MmsValue_toInt32(aux);
    if (c_parameters.prints_on == 4)
        printf("t_alarma: %i\n", t_alarma);

    /* Analize data to detect any problem in the system. */
    ant_xcbr_state = xcbr_state;

    if (t_alarma > 0)
    {
        xcbr_state = true;
        snprintf(msg, sizeof(msg), "abierto");
        if (c_parameters.prints_on == 4)
            printf("MAIN -> Mq message: abierto\n");
    }
    else if (t_alarma == 0)
    {
        xcbr_state = false;
        snprintf(msg, sizeof(msg), "cerrado");
        if (c_parameters.prints_on == 4)
            printf("MAIN -> Mq message: cerrado\n");
    }

    if (mq_send(mq, msg, strlen(msg), 0) == -1)
    {
        printf("Failed to send mqueue message.2\n");
    }

    /*
     * If the state of the system changes starts the process of interrumpting
     * the LP GOOSE Publisher Thread and stating the HP GOOSE Publisher Thread.
     */
    if (ant_xcbr_state != xcbr_state && !th_HP_running)
    {
        if (c_parameters.prints_on == 4)
            printf("GL --> Alarm ||| hay_alarma: %i xcbr_state: %s\n", t_alarma, xcbr_state ? "Abierto" : "Cerrado");
        pthread_mutex_lock(&mutex);
        if (c_parameters.prints_on == 4)
            printf("GL -> Locked\n");

        pthread_cancel(t1);
        th_LP_running = false;
        pthread_cond_signal(&tx);

        pthread_mutex_unlock(&mutex);
        if (c_parameters.prints_on == 4)
            printf("GL -> Unlocked\n");
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

        MmsValue_setBoolean(MmsV1, xcbr_state);

        sqNum = 0;
        stNum++;
        GoosePublisher_increaseStNum(publisherHP);

        if (GoosePublisher_publish(publisherHP, dataSetValues) == -1)
        {
            printf("Error sending GOOSE message!\n");
        }

        MQTTClient_message pubmsg = MQTTClient_message_initializer;
        char *payload;
        payload = xcbr_state ? "true\0" : "false\0";
        pubmsg.payload = (void *)payload;
        pubmsg.payloadlen = (int)strlen(payload);
        pubmsg.qos = QOS;
        pubmsg.retained = 0;
        int rc;
        if ((rc = MQTTClient_publishMessage(client, PUB_TOPIC, &pubmsg, NULL)) != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to publish message, return code %d\n", rc);
            exit(EXIT_FAILURE);
        }

        if (c_parameters.prints_on == 4)
            printf("HP --> Message Sended. -> xcbr_state: %i\n", xcbr_state);

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

            t_next = get_next_time(t);
            smart_sleep(t_next);

            pthread_mutex_lock(&mutex);
            if (c_parameters.prints_on == 4)
                printf("HP -> Locked 2\n");

            //struct timespec ts1;
            //clock_gettime(CLOCK_REALTIME, &ts1);

            MmsValue_setBoolean(MmsV1, xcbr_state);

            if (GoosePublisher_publish(publisherHP, dataSetValues) == -1)
            {
                printf("Error sending GOOSE message!\n");
            }
            if (c_parameters.prints_on == 4)
                printf("HP --> Message Sended\n");

            if (c_parameters.prints_on == 1 || c_parameters.prints_on == 3)
            {
                print_sended_GOOSE_message(BURST_GOOSE_MESSAGE);
            }

            //time_values[x] = ts1;
            //x++;

            sqNum++;
        }
        GoosePublisher_setSqNum(publisherLP, sqNum);
        GoosePublisher_setStNum(publisherLP, stNum);

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
    while (running /*&& x < NUM_SAMPLES*/)
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

        struct timespec ts1;
        clock_gettime(CLOCK_REALTIME, &ts1);

        if (GoosePublisher_publish(publisherLP, dataSetValues) == -1)
        {
            printf("Error sending GOOSE message!\n");
        }

        MQTTClient_message pubmsg = MQTTClient_message_initializer;
        char *payload;
        payload = xcbr_state ? "true\0" : "false\0";
        pubmsg.payload = (void *)payload;
        pubmsg.payloadlen = (int)strlen(payload);
        pubmsg.qos = QOS;
        pubmsg.retained = 0;
        int rc;
        if ((rc = MQTTClient_publishMessage(client, PUB_TOPIC, &pubmsg, NULL)) != MQTTCLIENT_SUCCESS)
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

        //time_values[x] = ts1;
        //x++;
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

static void print_received_GOOSE_message(GooseSubscriber subscriber, MmsValue *values)
{
    printf("   /////////////////////////////////////////////////////////////\n");
    printf("   ///Function GooseListener called/////////////////////////////\n");
    printf("   //////HEADER********************************************* ///\n");
    printf("   ///VlanTag: %s\n", GooseSubscriber_isVlanSet(subscriber) ? "found" : "NOT found");
    if (GooseSubscriber_isVlanSet(subscriber))
    {
        printf("   ///VlanID: %u. VlanPriority: %u.\n", GooseSubscriber_getVlanId(subscriber), GooseSubscriber_getVlanPrio(subscriber));
    }
    printf("   ///AppID: %i.\n", GooseSubscriber_getAppId(subscriber));
    uint8_t MACsrc[6];
    GooseSubscriber_getSrcMac(subscriber, MACsrc);
    printf("   ///SrcMac: %02X:%02X:%02X:%02X:%02X:%02X\n", MACsrc[0], MACsrc[1], MACsrc[2], MACsrc[3], MACsrc[4], MACsrc[5]);
    uint8_t MACdst[6];
    GooseSubscriber_getDstMac(subscriber, MACdst);
    printf("   ///SrcMac: %02X:%02X:%02X:%02X:%02X:%02X\n", MACdst[0], MACdst[1], MACdst[2], MACdst[3], MACdst[4], MACdst[5]);
    printf("   //////APDU*********************************************** ///\n");
    printf("   ///GOOSE ID: %s.\n", GooseSubscriber_getGoId(subscriber));
    printf("   ///GOOSE Control block: %s.\n", GooseSubscriber_getGoCbRef(subscriber));
    printf("   ///Data Set: %s.\n", GooseSubscriber_getDataSet(subscriber));
    printf("   ///ConfRev: %u.\n", GooseSubscriber_getConfRev(subscriber));
    printf("   ///Needs Comission: %s\n", GooseSubscriber_needsCommission(subscriber) ? "true" : "false");
    printf("   ///Simulation: %s\n", GooseSubscriber_isTest(subscriber) ? "true" : "false");
    printf("   ///StNum: %u. SqNum: %u.\n", GooseSubscriber_getStNum(subscriber), GooseSubscriber_getSqNum(subscriber));
    printf("   ///Time Allowed To Live: %u.\n", GooseSubscriber_getTimeAllowedToLive(subscriber));
    uint64_t timestamp = GooseSubscriber_getTimestamp(subscriber);
    printf("   ///Timestamp: %u.%u.\n", (uint32_t)(timestamp / 1000), (uint32_t)(timestamp % 1000));

    printParseErrorCode(subscriber);

    char buffer[1024];
    MmsValue_printToBuffer(values, buffer, 1024);

    printf("   //////DATA*********************************************** ///\n");
    printf("   ///allData: %s\n", buffer);
    printf("   /////////////////////////////////////////////////////////////\n\n");

    printf("   *************************************************************\n");
    switch (t_alarma)
    {
    case 1:
        printf("   ****         ALARM. VOLTAGE VALUES OUT OF RANGE.         ****\n");
        break;
    case 2:
        printf("   ****         ALARM. CURRENT VALUES OUT OF RANGE.         ****\n");
        break;
    case 3:
        printf("   ****    ALARM. VOLTAGE & CURRENT VALUES OUT OF RANGE.    ****\n");
        break;
    }
    printf("   *************************************************************\n");
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
    printf("//////XCBR State: %s\n", xcbr_state ? "Abierto" : "Cerrado");
    printf("//////sqNum: %i\n", sqNum);
    printf("//////stNum: %i\n", stNum);
    printf("////////////////////////////////////////////////////////////////\n\n");
}
