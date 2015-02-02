#include "supervisor.h"


bool Supervisor::GLOBALSTOP = false;

Supervisor::Supervisor()
{
}

Supervisor::~Supervisor()
{
    GLOBALSTOP = true;
}
