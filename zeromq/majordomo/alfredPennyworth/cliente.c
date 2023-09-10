//  MDP/Client
//  Connects REQ socket to tcp://localhost:5559
//  Sends REQUEST to server, expects REPLY back

/*
Publisher will send a particular payload according on parameter passed to it:
- no parameter: payload dimension increased every message;
- integer parameter: payload dimension fixed (dim(parameter)).
*/

/*
A REQUEST command consists of a multipart message of 4 or more frames, 
formatted on the wire as follows:
Frame 0: Empty (zero bytes, invisible to REQ application)
Frame 1: “MDPC01” (six bytes, representing MDP/Client v0.1)
Frame 2: Service name (printable string)
Frames 3+: Request body (opaque binary)
*/
#include "zhelpers.h"
#include "cJSON.h"
#include "utils.h"
#define REQUESTS 100

struct timespec timespec_start, timespec_end;

void telemetry(void *publisher_tel, int count, double rtt){
    //todo
    // Creazione dell'oggetto JSON
    cJSON *root = cJSON_CreateObject();

    // Inserimento dei dati nella coppia
    cJSON_AddNumberToObject(root, "count", count);
    cJSON_AddNumberToObject(root, "rtt", rtt);

    s_sendmore (publisher_tel, "TELEMETRY"); //envelope
    s_send (publisher_tel, cJSON_Print(root)); //content
    
    // Deallocazione della memoria
    cJSON_Delete(root);
}

int main (int argc, char **argv) 
{
    int count = 0;
    void *context = zmq_ctx_new ();

    //  Socket to talk to server
    void *requester = zmq_socket (context, ZMQ_REQ);
    zmq_connect (requester, "tcp://localhost:5559");

    // Prepare publisher for telemetry
    void *context_tel = zmq_ctx_new ();
    void *publisher_tel = zmq_socket (context_tel, ZMQ_PUB);
    zmq_bind (publisher_tel, "tcp://*:5565");

    char *message = NULL;
    for ( ; ; ) {
        /*message = (char*)malloc((count + 1) * sizeof(char)); // +1 for the null terminator
        for(int i=0; i<count; i++) message[i] = (char) (rand() % (0x7e - 0x20) + 0x20);
        count=count+1000;*/
        if (argc == 1) {
            message = (char*)malloc((count + 1) * sizeof(char)); // +1 for the null terminator
            for(int i=0; i<count; i++){
                message[i] = (char) (rand() % (0x7e - 0x20) + 0x20);
            }
            count=count+1000;
        } else {
            message = (char *)malloc((atoi(argv[1]) + 1) * sizeof(char));
            for(int i=0; i<atoi(argv[1]); i++){
                message[i] = (char) (rand() % (0x7e - 0x20) + 0x20);
            }
        }

        zmq_msg_t part;
        zmq_msg_init_size (&part, sizeof("MDPC01"));
        memcpy (zmq_msg_data (&part), "MDPC01", sizeof("MDPC01"));
        zmq_msg_send (&part, requester, ZMQ_SNDMORE);
        printf("Send message: %.*s\n", (int)sizeof("MDPC01"), "MDPC01");
        zmq_msg_init_size (&part, sizeof("Service"));
        memcpy (zmq_msg_data (&part), "Service", sizeof("Service"));
        zmq_msg_send (&part, requester, ZMQ_SNDMORE);
        if (argc == 1) {
            zmq_msg_init_size (&part, (count + 1) * sizeof(char));
            memcpy (zmq_msg_data (&part), message, (count + 1) * sizeof(char));
        } else {
            zmq_msg_init_size (&part, (atoi(argv[1]) + 1) * sizeof(char));
            memcpy (zmq_msg_data (&part), message, (atoi(argv[1]) + 1) * sizeof(char));
        }
        
        zmq_msg_send (&part, requester, 0);

        clock_gettime(CLOCK_MONOTONIC, &timespec_start);
        int time = -1;
        
        int more;
        size_t more_size = sizeof (more);
        do {
            /* Create an empty ØMQ message to hold the message part */
            zmq_msg_t part;
            int rc = zmq_msg_init (&part);
            assert (rc == 0);

            /* Block until a message is available to be received from socket */
            rc = zmq_msg_recv (&part, requester, 0);
            if (time == -1) {
                /*Because of multipart message, we get the time just when receive
                the header of the response*/
                clock_gettime(CLOCK_MONOTONIC, &timespec_end);
                time = 0;
            }
            //assert (rc != -1);
            void* data = zmq_msg_data(&part);
            size_t size = zmq_msg_size(&part);
            printf("Received message: %.*s\n", (int)size, (char*)data);

            /* Determine if more message parts are to follow */
            rc = zmq_getsockopt (requester, ZMQ_RCVMORE, &more, &more_size);
            assert (rc == 0);
            zmq_msg_close (&part); 
            if (!more) printf("\n");
        } while (more);
        free(message);
        struct timespec timespec_diff;
        sub_timespec(timespec_start, timespec_end, &timespec_diff);
        telemetry(publisher_tel, count, timespec_diff.tv_sec + (timespec_diff.tv_nsec / 1000000000.0f));
        //if (timespec_end.tv_nsec - timespec_start.tv_nsec >= 0) 
        //    telemetry(publisher_tel, count, (timespec_end.tv_nsec - timespec_start.tv_nsec)/1000);
    }
    zmq_close (requester);
    zmq_ctx_destroy (context);
    return 0;
}