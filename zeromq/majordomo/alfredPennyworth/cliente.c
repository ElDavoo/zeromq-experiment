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

//  MDP/Client

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

struct timespec timespec_start, timespec_end;

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
        // printf("Send message: %.*s\n", (int)sizeof("MDPC01"), "MDPC01");
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
            // printf("Received message: %.*s\n", (int)size, (char*)data);

            /* Determine if more message parts are to follow */
            rc = zmq_getsockopt (requester, ZMQ_RCVMORE, &more, &more_size);
            assert (rc == 0);
            zmq_msg_close (&part); 
            // if (!more) printf("\n");
        } while (more);
        free(message);
        struct timespec timespec_diff;
        sub_timespec(timespec_start, timespec_end, &timespec_diff);
        telemetry(publisher_tel, count, (timespec_diff.tv_sec * 1000000000.0f) + timespec_diff.tv_nsec - (10 *1000*1000) );
    }
    zmq_close (requester);
    zmq_ctx_destroy (context);
    return 0;
}