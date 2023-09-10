#include <time.h>
#include "utils.h"

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