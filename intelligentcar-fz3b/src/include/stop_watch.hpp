#pragma once

#include <any>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <thread>
#include <sys/time.h>

class StopWatch
{
public:
  // timer for benchmark time useage
  void tic()
  {
    struct timeval tp;
    gettimeofday(&tp, NULL);
    _sec = tp.tv_sec;
    _usec = tp.tv_usec;
  }

  double toc()
  { // return ms
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return (((double)(tp.tv_sec - _sec)) * 1000.0 + (tp.tv_usec - _usec) / 1000.0);
  }

private:
  double _sec, _usec;
};
