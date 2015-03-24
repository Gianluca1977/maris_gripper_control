#ifndef SIMULATOR_H
#define SIMULATOR_H

#define NUM_MOT     3

#include "motor_simulator.h"

class Simulator
{
    MotorSimulator Motors[NUM_MOT];

public:
    Simulator(int []);
};

#endif // SIMULATOR_H
