#include <czmq.h>

int main(int argc, char ** argv) {
    zsock_t *socket = zsock_new_pub("tcp://127.0.0.1:5555"); //localhost non funge
    assert(socket);

    while(!zsys_interrupted) {
        zsys_info("Publishing");
        zsock_send(socket, "sss", "TOPIC", "MESSAGE PART", "ANOTHER");//invia un messaggio con 3 stringhe
        zclock_sleep(1000);
    }

    zsock_destroy(&socket);
    return 0;
}