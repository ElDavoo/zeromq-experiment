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

#define SIZE 1000
//char message[SIZE + 1];
// a timespec struct
struct timespec timespec_start, timespec_end;

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
        
        /*
        // Get a random printable character between 0x20 and 0x7e
        char r = (char) (rand() % (0x7e - 0x20) + 0x20);

        for(int i=0; i<SIZE; i++){
            message[i] = r;
        }*/

        char* message = (char*)malloc((count + 1) * sizeof(char)); // +1 for the null terminator
        for(int i=0; i<count; i++){
            //message[i] = 'A';
            message[i] = (char) (rand() % (0x7e - 0x20) + 0x20);
        }
        count=count+1000;
        char *address = NULL;
        char *contents = NULL;
        printf ("Pingo\n");
        s_sendmore (publisher, "PING"); //envelope
        s_send (publisher, message); //content
        clock_gettime(CLOCK_MONOTONIC, &timespec_start);
        //  Read envelope with address
        address = s_recv (subscriber_pong);
        //  Read message contents
        contents = s_recv (subscriber_pong);
        clock_gettime(CLOCK_MONOTONIC, &timespec_end);
        printf ("[%s] %s. RTT: %ld us\n", address, contents, (timespec_end.tv_nsec - timespec_start.tv_nsec)/1000);
        if (timespec_end.tv_nsec - timespec_start.tv_nsec >= 0) 
            telemetry(publisher_tel, count, (timespec_end.tv_nsec - timespec_start.tv_nsec)/1000);
        free (address);
        free (contents);
        free (message);
        s_sleep (10);
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