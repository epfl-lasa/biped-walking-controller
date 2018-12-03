#include "DesVelocityReader.h"

DesVelocityReader::DesVelocityReader(string moduleName, int VelocityCmdType, Eigen::Vector3d  init_vel)
                                                    : moduleName_(moduleName)
                                                    , VelocityCmdType_(VelocityCmdType)
                                                    , init_vel_(init_vel){}


bool DesVelocityReader::initReader(){


    //Opening the port for the Keyboard input cmds  moduleName
    std::string KeyboardOutputs_port ="/";
                KeyboardOutputs_port += moduleName_;
                KeyboardOutputs_port += "/keyboardOutputs:o";


    KeyboardCmd_port_In.open("/KeyboardInputCmds:i");

    if(!Network::connect(KeyboardOutputs_port.c_str(), KeyboardCmd_port_In.getName().c_str()))
    {
        printf(" Unable to connect to the KeyboardCmdsReaderModule port");
        return false;
    }

    // Reading of measurement
    keyboardValues  = KeyboardCmd_port_In.read(); 
    incr_vel_.resize(keyboardValues->size());
    incr_vel_.setZero();

    des_com_vel_.resize(keyboardValues->size());
    des_com_vel_.setZero();

    std::cout << "Initial Velocity vx:" << init_vel_(0) << " vy:" << init_vel_(1) <<  " wz:" << init_vel_(2) << std::endl;
    des_com_vel_(0) = init_vel_(0);
    des_com_vel_(1) = init_vel_(1);
    des_com_vel_(2) = init_vel_(2);   

    return true; 
}


void DesVelocityReader::stop(){

    KeyboardCmd_port_In.close();
    return ;
}


void DesVelocityReader::read(){


        // Extract the read value
        keyboardValues  = KeyboardCmd_port_In.read();         
        incr_vel_(0) = keyboardValues->get(0).asDouble();
        incr_vel_(1) = keyboardValues->get(1).asDouble();
        incr_vel_(2) = keyboardValues->get(2).asDouble();

        // This is a stopping condition
        if (isnan(incr_vel_(0)))
            des_com_vel_(0) = NAN;
        else{
                if(VelocityCmdType_ == 1) {   
                    // Add increments
                    des_com_vel_(0) = des_com_vel_(0) + incr_vel_(0);
                    des_com_vel_(1) = des_com_vel_(1) + incr_vel_(1);
                    des_com_vel_(2) = des_com_vel_(2) + incr_vel_(2);

                    // Truncate velocities
                    // forward and backward walking 
                    // -----------------------------         
                    if(des_com_vel_(0) >= 0.1 && incr_vel_(0) > 0.0) { des_com_vel_(0) = 0.1; }
                    if(des_com_vel_(0) <= -0.1 && incr_vel_(0) < 0.0) {  des_com_vel_(0) = -0.1;  }


                    // lateral walking 
                    // ---------------
                    if(des_com_vel_(1) >= 0.1 && incr_vel_(1) > 0.0) { des_com_vel_(1) = 0.1; }
                    if(des_com_vel_(1) <= -0.1 && incr_vel_(1) < 0.0) {  des_com_vel_(1) = -0.1;  }

                    // rotation
                    // --------
                    if(des_com_vel_(2) >= 0.15 && incr_vel_(2) > 0.0) { des_com_vel_(2) = 0.15; }
                    if(des_com_vel_(2) <= -0.15 && incr_vel_(2) < 0.0) {  des_com_vel_(2) = -0.15; }

                }
                else if(VelocityCmdType_ == 0) {
                    
                    // Fix COM velocity
                    des_com_vel_(0) = 0.005;
                    des_com_vel_(1) = 0.0;
                    des_com_vel_(2) = 0.0;
                }   
        }
}
    

