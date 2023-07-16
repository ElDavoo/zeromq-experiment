#include "zhelpers.h"
#include <unistd.h>
#include "cJSON.h"

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

int main (void){
    int count = 0;

    //  Prepare our context and publisher
    void *context = zmq_ctx_new ();
    void *publisher = zmq_socket (context, ZMQ_PUB);
    zmq_bind (publisher, "tcp://*:5563");

    // Prepare Pong context and subscriber
    void *context_pong = zmq_ctx_new ();
    void *subscriber_pong = zmq_socket (context_pong, ZMQ_SUB);
    zmq_connect (subscriber_pong, "tcp://127.0.0.1:5564");
    zmq_setsockopt (subscriber_pong, ZMQ_SUBSCRIBE, "Pong", 1);

    // Prepare publisher for telemetry
    void *context_tel = zmq_ctx_new ();
    void *publisher_tel = zmq_socket (context_tel, ZMQ_PUB);
    zmq_bind (publisher_tel, "tcp://*:5565");

    /*
    Basically, it takes a little time (a few milliseconds) for the connection to be set up, 
    and in that time lots of messages can be lost. 
    The publisher needs to sleep a little before starting to publish, 
    or (better) it needs to explicitly synchronize with the subscriber (maybe soon - lol).
    */
    s_sleep(3000); //IMPORTANT!!!

    while (1) {

        char* message = (char*)malloc((count + 1) * sizeof(char)); // +1 for the null terminator
        for(int i=0; i<count; i++){
            message[i] = 'A';
        }
        count=count+1000;

        s_sendmore (publisher, "PING"); //envelope
        s_send (publisher, message); //content
        clock_t start_time = clock();
        printf ("Pingo\n");

        //  Read envelope with address
        char *address = s_recv (subscriber_pong);
        //  Read message contents
        char *contents = s_recv (subscriber_pong);
        clock_t end_time = clock();
        printf ("[%s] %s. RTT: %2f\n", address, contents, (double)(end_time - start_time) / CLOCKS_PER_SEC * 1000);
        telemetry(publisher_tel, count, (double)(end_time - start_time) / CLOCKS_PER_SEC * 1000);
        free (address);
        free (contents);
        free(message);
        s_sleep (250);
    }
    //  We never get here, but clean up anyhow
    zmq_close (publisher);
    zmq_ctx_destroy (context);
    return 0;
}





























/*#include <czmq.h>

int main(int argc, char ** argv) {
    zsock_t *socket = zsock_new_pub("tcp://127.0.0.1:5555"); //localhost non funge
    assert(socket);
    zsock_t *socket2 = zsock_new_sub("tcp://127.0.0.1:5556", "TOPIC2");
    assert(socket2);

    zsock_send(socket, "ss", "TOPIC", "Init");

    char *topic;
    char *frame;
    zmsg_t *msg;

    while(!zsys_interrupted) {
        zsys_info("Publishing");
        zsock_send(socket, "ss", "TOPIC", "ME");//invia un messaggio con nome topic + stringa
        zclock_sleep(3000);

        int rc = zsock_recv(socket2, "sm", &topic, &msg);
        assert(rc == 0);
        zsys_info("Recv on %s", topic);
        while(frame = zmsg_popstr(msg)) { //poppa le stringhe dal mess.
            zsys_info("> %s", frame);
            free(frame);
        }
    }

    zsock_destroy(&socket);
    return 0;
}*/