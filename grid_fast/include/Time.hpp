#ifndef TIME_H
#define TIME_H
#include <sys/time.h>

class Time {
public:
  struct timeval tv;
  Time() { reset(); }
  void reset() { gettimeofday(&tv, NULL); }
  float get_since() {
    struct timeval t2;
    gettimeofday(&t2, NULL);
    return (t2.tv_sec - tv.tv_sec) + 0.000001 * (t2.tv_usec - tv.tv_usec);
  }
};

#endif
