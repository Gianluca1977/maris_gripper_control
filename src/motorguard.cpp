#include "motorguard.h"

MotorGuard::MotorGuard() : StateMachine(MotorConfigurator::ST_MAX_STATES), motor_index(0)
{
}

MotorGuard::MotorGuard(Motor (&motor)[]) : StateMachine(MotorConfigurator::ST_MAX_STATES), GripperMotors(motor), motor_index(0)
{

}
