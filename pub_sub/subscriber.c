#include <czmq.h>

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
        while(frame = zmsg_popstr(msg)) { //poppa le stringhe dal mess.
            zsys_info("> %s", frame);
            free(frame);
        }
    }
    free(topic);
    zmsg_destroy(&msg);

    zsock_destroy(&socket);
    return 0;
}