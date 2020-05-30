#include <iostream>
#include <random>
#include <thread>
#include <future>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */

template <typename T>
T MessageQueue<T>::receive()
{
    std::unique_lock<std::mutex> lck_guard(_mutex);
    _cond.wait(lck_guard, [this] { return !_queue.empty(); });
    T element = std::move(_queue.back());
    _queue.pop_back();
    return element; // the element will not be copied due to RVO in C++.
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // simulate some work
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lck_guard(_mutex);
    _queue.emplace_back(std::move(msg));
    _cond.notify_one();
}

/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
    _queue = std::make_shared<MessageQueue<TrafficLightPhase>>();
}

TrafficLight::~TrafficLight()
{
    
}

void TrafficLight::waitForGreen()
{
    while(true) {
       TrafficLightPhase phase = _queue->receive();
       if(phase == TrafficLightPhase::green) {
           return;
       }
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}


void TrafficLight::simulate()
{
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{
    /* Reference: https://stackoverflow.com/questions/19665818/generate-random-numbers-using-c11-random-library */
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<int> dist(4, 6);

    /// Cycle duration will be in the range [4, 6].
    auto cycle_duration = dist(mt);

    /// Current time point.
    std::chrono::system_clock::time_point last_timePoint = std::chrono::system_clock::now();
    while (true)
    {
        /// Sleep for 1 msec.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        /// Compute the cycle duration.
        int durationInSeconds = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - last_timePoint).count();
        if (durationInSeconds >= cycle_duration)
        {
            // Toggle current Traffic Light Phase.
            std::unique_lock<std::mutex> _uLock(_mutex);
            _currentPhase = _currentPhase == TrafficLightPhase::green ? TrafficLightPhase::red : TrafficLightPhase::green;
            TrafficLightPhase message = _currentPhase;
            _uLock.unlock();

            // Sending the message.
            std::future<void> _ftr = std::async(std::launch::async, &MessageQueue<TrafficLightPhase>::send, _queue, std::move(message));
            _ftr.wait();

            // Reset the cycle duration and update the last_timestamp;
            cycle_duration = dist(mt);
            last_timePoint = std::chrono::system_clock::now();
        }
    }
}