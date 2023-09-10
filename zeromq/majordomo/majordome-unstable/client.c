#include "zhelpers.h"
#include <unistd.h>
#include <errno.h>

int main (void){

    //  Prepare our context and publisher
    void *context = zmq_ctx_new ();
    void *request = zmq_socket (context, ZMQ_REQ);
    int rc = zmq_connect (request, "tcp://127.0.0.1:5563");
    printf("%s\n",strerror(errno));
    assert (rc == 0);

    /*
    Basically, it takes a little time (a few milliseconds) for the connection to be set up, 
    and in that time lots of messages can be lost. 
    The publisher needs to sleep a little before starting to publish, 
    or (better) it needs to explicitly synchronize with the subscriber (maybe soon - lol).
    */
    s_sleep(3000); //IMPORTANT!!!

    while (1) {

        /*
        Frame 0: Empty (zero bytes, invisible to REQ application)
        The REQ socket will silently create frame 0 for outgoing requests, 
        and remove it for replies before passing them to the calling application.
        Frame 1: “MDPC01” (six bytes, representing MDP/Client v0.1)
        Frame 2: Service name (printable string)
        Frames 3+: Request body (opaque binary)
        */
        zmq_msg_t part;
        zmq_msg_init_size (&part, 6);
        memcpy (zmq_msg_data (&part), "MDPC01", 6);
        zmq_msg_send (&part, request, ZMQ_SNDMORE);
        zmq_msg_init_size (&part, 0);
        memcpy (zmq_msg_data (&part), "Service", 7);
        zmq_msg_send (&part, request, ZMQ_SNDMORE);
        zmq_msg_init_size (&part, 0);
        memcpy (zmq_msg_data (&part), "prova1", 6);
        zmq_msg_send (&part, request, 0);

        int more;
        size_t more_size = sizeof (more);
        do {
            /* Create an empty ØMQ message to hold the message part */
            zmq_msg_t part;
            int rc = zmq_msg_init (&part);
            assert (rc == 0);
            /* Block until a message is available to be received from socket */
            rc = zmq_msg_recv (&part, request, 0);
            assert (rc != -1);
            void* data = zmq_msg_data(&part);
            size_t size = zmq_msg_size(&part);
            printf("Received message: %.*s\n", (int)size, (char*)data);
            /* Determine if more message parts are to follow */
            rc = zmq_getsockopt (request, ZMQ_RCVMORE, &more, &more_size);
            assert (rc == 0);
            zmq_msg_close (&part); 
        } while (more);
        //s_recv(request)

        s_sleep (250);
    }
    //  We never get here, but clean up anyhow
    zmq_close (request);
    zmq_ctx_destroy (context);
    return 0;
}