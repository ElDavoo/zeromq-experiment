/*
 * Copyright (C) 2023 by Antonio Solida e Davide Palma
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
Publisher will send a particular payload according on parameter passed to it:
- no parameter: payload dimension increased every message;
- integer parameter: payload dimension fixed (dim(parameter)).
*/

#include "zhelpers.h"
#include <unistd.h>
#include "cJSON.h"
#include "utils.h"

// a timespec struct
struct timespec timespec_start, timespec_end;

int main (int argc, char **argv){
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

    char *message = NULL;
    while (1) {

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
        struct timespec timespec_diff;
        sub_timespec(timespec_start, timespec_end, &timespec_diff);
        telemetry(publisher_tel, count, timespec_diff.tv_sec + (timespec_diff.tv_nsec / 1000000000.0f));
        /*printf ("[%s] %s. RTT: %ld us\n", address, contents, (timespec_end.tv_nsec - timespec_start.tv_nsec)/1000);
        if (timespec_end.tv_nsec - timespec_start.tv_nsec >= 0) 
            telemetry(publisher_tel, count, (timespec_end.tv_nsec - timespec_start.tv_nsec)/1000);
        */
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