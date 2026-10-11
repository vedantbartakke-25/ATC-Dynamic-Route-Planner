#ifndef HIGH_RES_TIMER_HPP
#define HIGH_RES_TIMER_HPP

#include <chrono>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#ifdef ERROR
#undef ERROR
#endif

class HighResTimer {
private:
    LARGE_INTEGER start_time;
    static double get_frequency_reciprocal() {
        LARGE_INTEGER freq;
        QueryPerformanceFrequency(&freq);
        return 1000.0 / static_cast<double>(freq.QuadPart); // Multiplier to get milliseconds directly
    }

public:
    void start() {
        QueryPerformanceCounter(&start_time);
    }

    double elapsed_ms() const {
        LARGE_INTEGER end_time;
        QueryPerformanceCounter(&end_time);
        static double mult = get_frequency_reciprocal();
        return static_cast<double>(end_time.QuadPart - start_time.QuadPart) * mult;
    }
};

#else

class HighResTimer {
private:
    std::chrono::steady_clock::time_point start_time;

public:
    void start() {
        start_time = std::chrono::steady_clock::now();
    }

    double elapsed_ms() const {
        auto end_time = std::chrono::steady_clock::now();
        double elapsed_ns = std::chrono::duration<double, std::nano>(end_time - start_time).count();
        return elapsed_ns / 1000000.0;
    }
};

#endif

#endif // HIGH_RES_TIMER_HPP
