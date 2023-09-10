//  MDP/Client
//  Connects REQ socket to tcp://localhost:5559
//  Sends REQUEST to server, expects REPLY back

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

int main (void) 
{
    void *context = zmq_ctx_new ();

    //  Socket to talk to server
    void *requester = zmq_socket (context, ZMQ_REQ);
    zmq_connect (requester, "tcp://localhost:5559");

    // Prepare publisher for telemetry
    void *context_tel = zmq_ctx_new ();
    void *publisher_tel = zmq_socket (context_tel, ZMQ_PUB);
    zmq_bind (publisher_tel, "tcp://*:5565");

    int request_nbr;
    for (request_nbr = 0; request_nbr != REQUESTS; request_nbr++) {
        zmq_msg_t part;
        zmq_msg_init_size (&part, sizeof("MDPC01"));
        memcpy (zmq_msg_data (&part), "MDPC01", sizeof("MDPC01"));
        zmq_msg_send (&part, requester, ZMQ_SNDMORE);
        printf("Send message: %.*s\n", (int)sizeof("MDPC01"), "MDPC01");
        zmq_msg_init_size (&part, sizeof("Service"));
        memcpy (zmq_msg_data (&part), "Service", sizeof("Service"));
        zmq_msg_send (&part, requester, ZMQ_SNDMORE);
        zmq_msg_init_size (&part, sizeof("request"));
        memcpy (zmq_msg_data (&part), "request", sizeof("request"));
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
        if (timespec_end.tv_nsec - timespec_start.tv_nsec >= 0) 
            telemetry(publisher_tel, 0, (timespec_end.tv_nsec - timespec_start.tv_nsec)/1000);
    }
    zmq_close (requester);
    zmq_ctx_destroy (context);
    return 0;
}