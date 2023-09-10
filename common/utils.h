#ifndef common__h
#define common__h

void sub_timespec(struct timespec t1, struct timespec t2, struct timespec *td);
void telemetry(void *publisher_tel, int count, double rtt);

#endif