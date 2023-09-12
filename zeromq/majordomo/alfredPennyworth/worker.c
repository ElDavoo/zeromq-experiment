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

//  Worker

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
            // assert (rc != -1);
            // void* data = zmq_msg_data(&part);
            // size_t size = zmq_msg_size(&part);
            // printf("Received message: %c\n", (int)size, data);

            /* Determine if more message parts are to follow */
            rc = zmq_getsockopt (responder, ZMQ_RCVMORE, &more, &more_size);
            assert (rc == 0);
            zmq_msg_close (&part); 
            // if (!more) printf("\n");
        } while (more);

        //do some stuffs
        s_sleep(10);

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