#pragma once
#include <unistd.h>
#include <chrono>
#include <thread>
#include <iostream>

namespace pr {
namespace time {

inline void sleep(int s) {
    using namespace std::chrono;
    std::this_thread::sleep_for(seconds(s));
}
inline void msleep(int ms) {
    using namespace std::chrono;
    std::this_thread::sleep_for(milliseconds(ms));
}
inline void usleep(int us) {
    using namespace std::chrono;
    std::this_thread::sleep_for(microseconds(us));
}


typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::microseconds microseconds;

static Clock::time_point t0 = Clock::now();

inline void tic() {
 t0 = Clock::now();
}

inline void toc() {
    Clock::time_point t1 = Clock::now();
    microseconds us = std::chrono::duration_cast<microseconds>(t1 - t0);
    std::cout <<"Elapsed time is "<< us.count()/1000.0 << " milliseconds\n";
}

} // namespace pr::time
} // namespace pr
