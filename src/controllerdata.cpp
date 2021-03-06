#include <libconfig.h++>

#include "main.h"
#include "controllerdata.h"
#include "interface_data.h"

using namespace libconfig;

bool ControllerData::armPresent;
bool ControllerData::TcpActive = false;
int ControllerData::nodeIds[NUM_MOT];

ControllerData::sockTCP_t ControllerData::sockTCP;
ControllerData::homeConfig_t ControllerData::homeConfig;
ControllerData::operConfig_t ControllerData::operConfig;

int ControllerData::finger_conf_num;
double ControllerData::finger_confs[MAX_CONF][NUM_MOT];

bool ControllerData::doHome;
bool ControllerData::fault;
bool ControllerData::tcpDoHome;
bool ControllerData::tcpIsHomeDone;
bool ControllerData::emerg_stop;

int ControllerData::argc;
char** ControllerData::argv;
std::string ControllerData::nodeName;

WF::BinarySemaphore ControllerData::RequestSem;
WF::BinarySemaphore ControllerData::StatusSem;

ControllerData::ControllerData()
{
    bool lookup = true;
    Config conf_;
    std::string doc;
    doc = "./gripper_config.conf";
    try {
      conf_.readFile(doc.c_str());
    }
    catch(ParseException& e) {
        std::cout << "Parse exception when reading " << doc.c_str() << std::endl;
        std::cout << "Line " << e.getLine() << " error: " << e.getError() << std::endl;
        //return false;
    }
    try {
        conf_.lookupValue("arm-present",armPresent);

        conf_.lookupValue("sockTCP.active", sockTCP.active);
        conf_.lookupValue("sockTCP.portno", sockTCP.port);

        Setting& ids = conf_.lookup("joints.nodeIds");

        for(int i = 0; i < ids.getLength(); i++){
            nodeIds[i] = ids[i];
        }

        conf_.lookupValue("homing-config.safe-curr-limit", homeConfig.safe_curr_limit);
        conf_.lookupValue("homing-config.peak-curr", homeConfig.peak_curr);
        conf_.lookupValue("homing-config.cont-curr", homeConfig.cont_curr);
        conf_.lookupValue("homing-config.max-speed", homeConfig.max_speed);

        conf_.lookupValue("operation-config.safe-curr-limit", operConfig.safe_curr_limit);
        conf_.lookupValue("operation-config.peak-curr", operConfig.peak_curr);
        conf_.lookupValue("operation-config.cont-curr", operConfig.cont_curr);
        conf_.lookupValue("operation-config.max-speed", operConfig.max_speed);
        conf_.lookupValue("operation-config.max-acc", operConfig.max_acc);
        conf_.lookupValue("operation-config.max-deacc", operConfig.max_deacc);

        if(armPresent) std::cout << "arm is present!" << std::endl;
        else std::cout << "arm is NOT present!" << std::endl;

        if(sockTCP.active) {
            printf("A server TCP will be created. Port: %d\n", sockTCP.port);
            //printf(" Addr: %s \n", sockTCP.addr.c_str());
            std::cout << "sizeof(SystemRequest_2) = " <<  sizeof(SystemRequest) << std::endl;
            std::cout << "sizeof(SystemStatus) = " <<  sizeof(SystemStatus) << std::endl << std::endl;
        }

        std::cout << "Homing config: " << std::endl;
        std::cout << "safe-curr-limit: " << homeConfig.safe_curr_limit << std::endl;
        std::cout << "peak-curr: " << homeConfig.peak_curr << std::endl;
        std::cout << "cont-curr: " << homeConfig.cont_curr << std::endl;
        std::cout << "max-speed: " << homeConfig.max_speed << std::endl << std::endl;

        std::cout << "operation config: " << std::endl;
        std::cout << "safe-curr-limit: " << operConfig.safe_curr_limit << std::endl;
        std::cout << "peak-curr: " << operConfig.peak_curr << std::endl;
        std::cout << "cont-curr: " << operConfig.cont_curr << std::endl;
        std::cout << "max-acc: " << operConfig.max_acc << std::endl;
        std::cout << "max-deacc: " << operConfig.max_deacc << std::endl;
        std::cout << "max-speed: " << operConfig.max_speed << std::endl << std::endl;

        for(unsigned int i = 0; i < NUM_MOT; i++)
            std::cout <<"motor [" << i <<"]: " << nodeIds[i] << std::endl;

        conf_.lookupValue("finger-confs.num-conf", finger_conf_num);
        std::cout << std::endl << finger_conf_num << " gripper configuration selected" << std::endl;

        for(int j = 0; j < finger_conf_num; j++ ){

              std::stringstream tmp_str;

              tmp_str << "finger-confs.conf" << j;

              std::cout << std::endl << "Reading " << tmp_str.str() << std::endl;

              Setting& ids = conf_.lookup(tmp_str.str());

              for(int i = 0; i < ids.getLength(); i++){
                  int tmp = ids[i];
                  finger_confs[j][i] = tmp;
                  std::cout << "finger_confs[" << j << "][" << i << "] = " <<  finger_confs[j][i] << std::endl;
              }
        }

        lookup = true;
    }
    catch(SettingException& e) {
            std::cout << "Setting Exception path: " << e.getPath() <<" what: " << e.what() << std::endl;
        //return false;
        }
    catch(ConfigException& e) {
            std::cout << "Config Exception. What: "<< e.what() << std::endl;
      //      return false;
    }

    //printf("new\n\n");
    /*
    action_done = false;
    action_free = false;	//control the GraspActionInterface->executeGrasp main while
    action_perc_complete = 0;
    */
    emerg_stop = false;
    //isHomeDone = false;
    doHome = false;
    //action_force = 0;
    //auxInternalCall = -1;
    fault = false;
    tcpDoHome = false;
    tcpIsHomeDone = false;
    //setTactOffset = false;

    //return lookup;
}

ControllerData::ControllerData(int arg_c, char **arg_v, std::string node_Name)
{
    argc = arg_c;
    argv = arg_v;
    nodeName = node_Name;
    ControllerData();
}
