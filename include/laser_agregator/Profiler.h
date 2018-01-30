#pragma once

#include <iostream>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <atomic>
#include <string>

#include "laser_agregator/scope_exit.h"
#include "laser_agregator/ringbuffer.h"


class Timer{
public:
    void start(){
        m_StartTime = std::chrono::high_resolution_clock::now();
        m_running = true;
    }

    bool stop(){
        //only stop if it was running otherwise it was already stopped before
        if (m_running){
            m_EndTime = std::chrono::high_resolution_clock::now();
            m_running = false;
            return true;
        }else{
            return false; //returns that it failed to stop the timer because it was already stopped before
        }
    }

    double elapsed_ms(){
        std::chrono::time_point<std::chrono::high_resolution_clock> endTime;

        if(m_running){
            endTime = std::chrono::high_resolution_clock::now();
        }else{
            endTime = m_EndTime;
        }

        return std::chrono::duration_cast<std::chrono::milliseconds>(endTime - m_StartTime).count();
    }

    double elapsed_s(){
        return elapsed_ms() / 1000.0;
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_StartTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_EndTime;
    bool m_running = false;
};



class Profiler{
public:
    Profiler(){};
    void start_time( std::string name ){
        // std::thread::id thread_id= std::this_thread::get_id();
        // std::stringstream ss;
        // ss << thread_id;
        // std::string thread_id_string = ss.str();
        // std::string full_name=name+ thread_id_string;
        std::string full_name=name;

        timers[full_name].start();
    }

    void stop_time(std::string name){
        // std::thread::id thread_id= std::this_thread::get_id();
        // std::stringstream ss;
        // ss << thread_id;
        // std::string thread_id_string = ss.str();
        // std::string full_name=name+ thread_id_string;
        std::string full_name=name;

        //get elapsed time for that timer and register it into the timings
        if(timers[full_name].stop()){  //we manage to stop is correctly and it was no stopped before
            double time_elapsed=timers[full_name].elapsed_ms();
            timings[full_name].push(time_elapsed);
        }
    }

    std::unordered_map<std::string, ringbuffer<float,100> > timings;  //contains the last N timings of the registers
    std::unordered_map<std::string, Timer> timers;  //contains the timers for the registers
private:


};



#define TIME_SCOPE_2(name, profiler)\
    profiler->start_time(name); \
    SCOPE_EXIT{profiler->stop_time(name);};

#define TIME_START_2(name,profiler) \
    profiler->start_time(name);

#define TIME_END_2(name,profiler) \
    profiler->stop_time(name);
