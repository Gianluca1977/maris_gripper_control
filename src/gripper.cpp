#include "gripper.h"
#include "controller.h"
#include <iostream>

/*
Gripper::Gripper()
{    
    jointReduction = 3000 * 14 * 20 * 33 / 12;
    safeJointOffset = 10000;			//encoder pulse
}
*/

Gripper::Gripper(int nodeIds[])
{
    for(int i = 0; i < NUM_MOT; i++) Motors[i].setID(nodeIds[i]);

    jointReduction = 3000 * 14 * 20 * 33 / 12;
    safeJointOffset = 10000;			//encoder pulse
}

void Gripper::Open(int vel)
{

}

void Gripper::Close(int vel)
{

}

void Gripper::disableMotors()
{
    for(int i = 0; i < NUM_MOT; i++) Motors[i].disable();
}

void Gripper::enableMotors()
{
    for(int i = 0; i < NUM_MOT; i++) Motors[i].enable();
}

bool Gripper::isOperative()
{
    for(int i = 0; i < NUM_MOT; i++) if(!Motors[i].Operational) return false;
    return true;
}

void Gripper::stop()
{
    for(int i = 0; i < NUM_MOT; i++) Motors[i].stop();
}

bool Gripper::checkStopped()
{
    for(int i = 0; i < NUM_MOT; i++) if(!Motors[i].checkStopped()) return false;
    return true;
}

// Update Motors States
void Gripper::updateStates(TPCANMsg msg) {

    static std::vector<bool> auxSetTactile(NUM_MOT,false);
    bool allSet = true;

    double sensorsOut[3];

    int nodeID = msg.ID & NODE_ID_MASK;
    int cmdID = msg.ID & COB_ID_MASK;

    for(int i = 0; i < NUM_MOT; i++) {
        if(Motors[i].ID == nodeID) {
            if(cmdID == TPDO3_COBID) {	//0x380(896D) - trace response
                long tmp_curr = msg.DATA[5];
                Motors[i].Current = (tmp_curr << 8) + msg.DATA[4];

                Motors[i].PositionGrad = pcanData2Double(msg,0)*360/jointReduction;

                //KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "States Update", "ID = %X,  curr = %X%X, pos = %X%X%X%X", Motors[i].ID, msg.DATA[5], msg.DATA[4], msg.DATA[3], msg.DATA[2], msg.DATA[1], msg.DATA[0] );

            }//if(cmdID == TPDO3_COBID)
            else if(cmdID == TPDO2_COBID) {	//0x280(640D) - TxPDO2 response
                if(msg.DATA[0] == 0x2B){
                    Motors[i].Velocity = pcanData2Double(msg,1)*360/jointReduction;
                    //    KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "States Update", "ID = %X,  vel = %X%X%X%X", Motors[i].ID, msg.DATA[4], msg.DATA[3], msg.DATA[2], msg.DATA[1]);
                } else if(msg.DATA[0] == 0x40){
                    Motors[i].PositionGrad = pcanData2Double(msg,1)*360/jointReduction;
                    //   KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, "States Update", "ID = %X,  pos = %X%X%X%X", Motors[i].ID, msg.DATA[4], msg.DATA[3], msg.DATA[2], msg.DATA[1]);
                }
            }//else if(cmdID == TPDO2_COBID)
            else if(cmdID == TPDO1_COBID) {//0x180(384D) - TxPDO1 (statusWord)

                Motors[i].stateUpdate(msg.DATA);

                if(Motors[i].stateChanged())
                    switch(Motors[i].State & STATUS_WORD_MASK){
                    case SWITCH_ON_DISABLED:
                            KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motor %d Switch On Disabled.", Motors[i].ID);
                        break;
                    case READY_TO_SWITCH_ON:
                            KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motor %d Ready to Switch On.", Motors[i].ID);
                        break;
                    case SWITCHED_ON:
                            KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motor %d Switched On.", Motors[i].ID);
                        break;
                    case OPERATION_ENABLED:
                            KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motor %d Operation Enabled.", Motors[i].ID);
                        break;
                    case FAULT_STATE:
                            KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motors %d Fault Detected. Reset Motors.", Motors[i].ID);
                        break;
                    case QUICKSTOP:
                        break;
                    default:
                        break;
                    }

            } else if(cmdID == BOOTUP_COBID) {	//0x700 (1972D) - Boot up message
                Motors[i].BootUp = true;
                KAL::DebugConsole::Write(LOG_LEVEL_NOTICE, TRIGGERTASK_NAME, "Motors %d booted up.", Motors[i].ID);
            }
        }//if(Motors[i].ID == nodeID){
    }//for(int i=0; i<NUM_MOT;i++){

    /*
    // Response from sensors
    if(cmdID == TPDO3_COBID && nodeID > 15) {
        NUM_MOT = nfingers;
        for(int i=0;i < NUM_MOT;i++) {
            if(Hand[i].ForceSensors.nodeId == nodeID) {

                long long int tmp_sens = msg.DATA[1];
                tmp_sens = (tmp_sens << 8 ) + msg.DATA[0];
                sensorsOut[0] = tmp_sens;

                tmp_sens = msg.DATA[3];
                tmp_sens = (tmp_sens << 8 ) + msg.DATA[2];
                sensorsOut[1] = tmp_sens;

                tmp_sens = msg.DATA[5];
                tmp_sens = (tmp_sens << 8 ) + msg.DATA[4];
                sensorsOut[2] = tmp_sens;

                if(setTactOffset && !auxSetTactile.at(i)) {
                    Hand[i].ForceSensors.xOffset = sensorsOut[0];
                    Hand[i].ForceSensors.yOffset = sensorsOut[1];
                    Hand[i].ForceSensors.zOffset = sensorsOut[2];
                    auxSetTactile.at(i) = true;
                }
                Hand[i].ForceSensors.x = sensorsOut[0] - Hand[i].ForceSensors.xOffset;
                Hand[i].ForceSensors.y = sensorsOut[1] - Hand[i].ForceSensors.yOffset;
                Hand[i].ForceSensors.z = sensorsOut[2] - Hand[i].ForceSensors.zOffset;
            }
            if(!auxSetTactile.at(i))
                allSet = false;
        }//for(int i=0;i < NUM_MOT;i++)
        if(allSet) {
            setTactOffset = false;
            auxSetTactile.assign(NUM_MOT,false);
        }
    }//if(cmdID == TPDO3_COBID && nodeID > 15)
    */
}//void updateStates


/*
  * Check if we are interested on that node
  * return true if the node is in the JointIDs_ vector
  * false otherwise
  */
bool Gripper::hasID(int nodeID)
{
    bool yep = false;
    for(unsigned int i=0;i < NUM_MOT;i++) {
        if(Motors[i].ID == nodeID) {
            yep = true;
            break;
        }
    }
    /*
    if(!yep) {
        for(unsigned int i=0;i < nfingers;i++) {
            if(Hand[i].ForceSensors.nodeId==nodeID) {
                yep = true;
                break;
            }
        }
    }*/
    return yep;
}

double Gripper::pcanData2Double(TPCANMsg msg, int offset)
{
    long long int tmp_data = msg.DATA[3 + offset];
    tmp_data = (tmp_data << 8) + msg.DATA[2 + offset];
    tmp_data = (tmp_data << 8) + msg.DATA[1 + offset];
    tmp_data = (tmp_data << 8) + msg.DATA[0 + offset];


    if(tmp_data & SIGN_MASK) {
        //DEBUG("changing pos sign..\n");
        //DEBUG("~(tmp_data-1) = 0x%X\n",(( VALUE_MASK & ~tmp_data) +1));
        return -1.0*(double)(( VALUE_MASK & ~tmp_data ) +1);
    }
    else return (double) tmp_data;
}

void Gripper::saveMaxLimit(int node, long pos)
{
    if(node !=-1) for(unsigned int k =0; k<NUM_MOT;k++) Motors[k].MaxPosGrad = pos;
    else for(unsigned int k =0; k<NUM_MOT;k++) Motors[k].MaxPosGrad = Motors[k].PositionGrad;
}

double Gripper::roundToSignificant(double num, int significant){

    if(num == 0) return 0;

    double d = ceil(log10(num < 0 ? -num: num));
    int power = significant - (int)d;

    double magnitude = pow(10,power);
    long shifted = round(num*magnitude);
    return shifted/magnitude;
}
