#include <iostream>
#include <iomanip>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/BufferedPort.h>

#include <ncurses.h>

#include "KeyboardReaderSender.h"

using namespace std;
using namespace yarp::os;


int main(int argc, char **argv)
{
    //initialize the network
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(5)) 
    {
        std::cerr << "YARP network is not available" << std::endl;
        return -1;
    }

    // creation of a ressource finder object
    yarp::os::ResourceFinder rf; // = yarp::os::ResourceFinder::getResourceFinderSingleton();
    
    rf.setVerbose(true);                                        //logs searched directories
    rf.setDefaultConfigFile("KeyboardCommandsReader.ini");      //default config file name.
    rf.configure(argc, argv);
    
    if (rf.check("help")) 
    {
        std::cout<< "Possible parameters" << std::endl << std::endl;
        std::cout<< "\t--context          :Where to find a user defined .ini file within $ICUB_ROOT/app e.g. /KeyboardCommandsReader/conf" << std::endl;
        std::cout<< "\t--rate             :Period used by the module. Default set to 10ms." << std::endl;
        std::cout<< "\t--robot            :Robot name (icubSim or icub). Set to icub by default." << std::endl;
        std::cout<< "\t--moduleName       :name of the module ." << std::endl;
        std::cout<< "\t--period           :name of the keyboardReader ." << std::endl;
        std::cout<< "\t--local            :Prefix of the ports opened by the module. Set to the module name by default, i.e. KeyboardCommandsReader." << std::endl;
        return 0;
    }

    // ===================================================
    std::string moduleName = rf.check("name", Value("CpBalanceWalkingController"), "Looking for module name").asString();
    std::string robotName  = rf.check("robot", Value("icub"), "Looking for robot name").asString();
    int ReaderThreadPeriod = rf.check("period", Value(40), "Looking for controller period").asInt();
    double modulePeriod = rf.check("modulePeriod", Value(0.25), "Looking for module period").asDouble();
    double variation = rf.check("variation", Value(0.001), "Looking for velocity variation (increment)").asDouble();

    //int ReaderThreadPeriod = rf.find("period").asInt();

   // ====================================================
    KeyboardReaderSender KeyReader(ReaderThreadPeriod, moduleName);

    KeyReader.start();
    KeyReader.increment = variation;

    while(KeyReader.ch !=27 )
    {

        //
    }

    endwin();

    KeyReader.stop();

    cout<<"Main returning..."<<endl;

    
    return 0;

}
