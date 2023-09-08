//  Hello World worker
//  Connects REP socket to tcp://localhost:5560
//  Expects "Hello" from client, replies with "World"

#include "zhelpers.h"
#include <unistd.h>

int main (void) 
{
    void *context = zmq_ctx_new ();

    //  Socket to talk to clients
    void *responder = zmq_socket (context, ZMQ_REP);
    zmq_connect (responder, "tcp://localhost:5560");

    while (1) {
        int more;
        size_t more_size = sizeof (more);
        do {
            /* Create an empty ØMQ message to hold the message part */
            zmq_msg_t part;
            int rc = zmq_msg_init (&part);
            assert (rc == 0);

            /* Block until a message is available to be received from socket */
            rc = zmq_msg_recv (&part, responder, 0);
            //assert (rc != -1);
            void* data = zmq_msg_data(&part);
            size_t size = zmq_msg_size(&part);
            printf("Received message: %.*s\n", (int)size, (char*)data);

            /* Determine if more message parts are to follow */
            rc = zmq_getsockopt (responder, ZMQ_RCVMORE, &more, &more_size);
            assert (rc == 0);
            zmq_msg_close (&part); 
            if (!more) printf("\n");
        } while (more);

        //do some stuffs
        sleep (1);

        //  Send reply back to client
        zmq_msg_t part;
        zmq_msg_init_size (&part, sizeof("MDPC01"));
        memcpy (zmq_msg_data (&part), "MDPC01", sizeof("MDPC01"));
        zmq_msg_send (&part, responder, ZMQ_SNDMORE);
        zmq_msg_init_size (&part, sizeof("Service"));
        memcpy (zmq_msg_data (&part), "Service", sizeof("Service"));
        zmq_msg_send (&part, responder, ZMQ_SNDMORE);
        zmq_msg_init_size (&part, sizeof("reply"));
        memcpy (zmq_msg_data (&part), "reply", sizeof("reply"));
        zmq_msg_send (&part, responder, 0);
    }
    //  We never get here, but clean up anyhow
    zmq_close (responder);
    zmq_ctx_destroy (context);
    return 0;
}