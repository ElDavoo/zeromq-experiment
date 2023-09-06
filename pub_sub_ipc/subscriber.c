#include "zhelpers.h"

int main (void){
    //  Prepare our context and subscriber
    void *context = zmq_ctx_new ();
    void *subscriber = zmq_socket (context, ZMQ_SUB);
    zmq_connect (subscriber, "ipc://#1");
    zmq_setsockopt (subscriber, ZMQ_SUBSCRIBE, "PING", 1);

    void *context_pong = zmq_ctx_new ();
    void *publisher_pong = zmq_socket (context_pong, ZMQ_PUB);
    zmq_bind (publisher_pong, "ipc://#2");

    /*
    Basically, it takes a little time (a few milliseconds) for the connection to be set up, 
    and in that time lots of messages can be lost. 
    The publisher needs to sleep a little before starting to publish, 
    or (better) it needs to explicitly synchronize with the subscriber (maybe soon - lol).
    */
    s_sleep(3000);
    
    while (1) {
        //  Read envelope with address
        char *address = s_recv (subscriber);
        //  Read message contents
        char *contents = s_recv (subscriber);
        printf ("[%s] %s\n", address, contents);
        //printf ("[%s] \n", address);
        free (address);
        free (contents);
        s_sendmore (publisher_pong, "Pong");
        s_send (publisher_pong, " pong");
    }
    //  We never get here, but clean up anyhow
    zmq_close (subscriber);
    zmq_ctx_destroy (context);
    return 0;
}








/*#include <czmq.h>

int main(int argc, char ** argv) {
    zsock_t *socket = zsock_new_sub("tcp://127.0.0.1:5555", "TOPIC");
    assert(socket);
    zsock_t *socket2 = zsock_new_pub("tcp://127.0.0.1:5556");
    assert(socket2);

    char *topic;
    char *frame;
    zmsg_t *msg;
    
    while(!zsys_interrupted) {
        //zsys_info("Recv on %s", topic);
        int rc = zsock_recv(socket, "sm", &topic, &msg);//riceve uno string message
                                                        //questa funzione è bloccante, ovvero
                                                        //rimane in attesa di un messaggio
        assert(rc == 0);
        zsys_info("Recv on %s", topic);
        while(frame = zmsg_popstr(msg)) { //poppa le stringhe dal mess.
            zsys_info("> %s", frame);
            free(frame);
        }
        zsock_send(socket2, "ss", "TOPIC2", "MESSAGE PART");
    }
    free(topic);
    zmsg_destroy(&msg);

    zsock_destroy(&socket);
    return 0;
}*/