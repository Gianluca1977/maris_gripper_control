#include "simulator.h"

Simulator::Simulator()
{
    for(int i = 0; i < NUM_MOT; i++) Motors[i].setID(nodeIds[i]);
}
