#include <time.h>
#include "utils.h"
#include "cJSON.h"
#include "zhelpers.h"

void sub_timespec(struct timespec t1, struct timespec t2, struct timespec *td){
    td->tv_nsec = t2.tv_nsec - t1.tv_nsec;
    td->tv_sec  = t2.tv_sec - t1.tv_sec;
    if (td->tv_sec > 0 && td->tv_nsec < 0){
        td->tv_nsec += 1000000000; //NS_PER_SECOND
        td->tv_sec--;
    }
    else if (td->tv_sec < 0 && td->tv_nsec > 0){
        td->tv_nsec -= 1000000000; //NS_PER_SECOND
        td->tv_sec++;
    }
}

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