#include "DesVelCommandThread.h"

DesVelCommandThread::DesVelCommandThread(int period, std::string _moduleName, std::string _robotName)
                                                    : RateThread(period)
                                                    , moduleName(_moduleName)
                                                    , robotName(_robotName){ ThreadPeriod = period;}


bool DesVelCommandThread::threadInit(){

    // initialization of the counter
    CycleCounter  = 0;
    start_time = yarp::os::Time::now();


    //Opening the port for the Keyboard input cmds  moduleName
    std::string KeyboardOutputs_port ="/";
                KeyboardOutputs_port += moduleName;
                KeyboardOutputs_port += "/keyboardOutputs:o";

    if(KeyboardCtrl && !iskeypportActive)
    {
        KeyboardCmd_port_In.open("/KeyboardInputCmds:i");

        if(!Network::connect(KeyboardOutputs_port.c_str(), KeyboardCmd_port_In.getName().c_str()))
        {
            printf(" Unable to connect to the KeyboardCmdsReaderModule port");
            return false;
        }
        // Reading of measurement
        keyboardValues  = KeyboardCmd_port_In.read(); 
        alpha_velo.resize(keyboardValues->size());
        alpha_velo.setZero();
        iskeypportActive = true;
    }
    
    // Current hack
    des_COM_velo.setZero(3);
    des_COM_velo.setZero();
    
}


void DesVelCommandThread::threadRelease(){

}


void DesVelCommandThread::run(){

}
    

