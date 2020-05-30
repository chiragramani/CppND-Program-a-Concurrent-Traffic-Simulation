#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include <mutex>
#include <deque>
#include <condition_variable>
#include "TrafficObject.h"

enum class TrafficLightPhase
{
    red,
    green
};

// forward declarations to avoid include cycle
class Vehicle;

template <class T>
class MessageQueue
{
public:
    void send(T &&msg);
    T receive();

private:
    std::deque<T> _queue;
    std::mutex _mutex;
    std::condition_variable _cond;
};

class TrafficLight : public TrafficObject
{
public:
    // constructor / desctructor
    TrafficLight();
    ~TrafficLight();

    // getters / setters
    void setCurrentPhase(TrafficLightPhase currentPhase);

    // typical behaviour methods
    void waitForGreen();
    void simulate() override;
    TrafficLightPhase getCurrentPhase();

private:
    // typical behaviour methods

    std::condition_variable _condition;
    std::mutex _mutex;
    void cycleThroughPhases();
    TrafficLightPhase _currentPhase;
    std::shared_ptr<MessageQueue<TrafficLightPhase>> _queue;
};

#endif