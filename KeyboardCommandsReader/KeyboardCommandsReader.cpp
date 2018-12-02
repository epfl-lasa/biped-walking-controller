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

//#include "KeyboardCmdsReaderModule.h"

using namespace std;
using namespace yarp::os;

class KeyboardCommandsReader : public yarp::os::RateThread 
{
    
    private: 

        int ThreadPeriod;                       // thread period
        // Forwards Dynamics
        

    public:
        std::string moduleName;
        std::string robotName;

        int ch;

        double increment;

        double vx_factor;
        double vy_factor;
        double wz_factor;

        // port to stream the value of the coefficients
        // Creation of port for the CoM
        BufferedPort<yarp::sig::Vector> VeloFactors_Port;
        //yarp::sig::Vector& output_veloCoef;

        yarp::sig::Vector VeloFactors;
        


        KeyboardCommandsReader(int period, std::string KeyboardModule)  : RateThread(period)
                                                                        , moduleName(KeyboardModule)
                                                                        , VeloFactors(3) {ThreadPeriod = period; }
        //~KeyboardCommandsReader();

        bool threadInit()
        {
            initscr();
            raw();
            keypad(stdscr, TRUE);
            noecho();
            nodelay(stdscr, TRUE);

            printw("Write something (Esc to escape): ");

            vx_factor = 0.0;
            vy_factor = 0.0;
            wz_factor = 0.0;

            increment = 0.0;

            VeloFactors.resize(3, 0.0);

            // opening the port 
            std::string veloCoefPortName = "/";
            veloCoefPortName += moduleName;
            veloCoefPortName += "/keyboardOutputs:o";

            VeloFactors_Port.open(veloCoefPortName.c_str());

            // yarp::sig::Vector &output_veloCoef = VeloFactors_Port.prepare();            
            return true;
        };

        void threadRelease()
        {

            // closing the port
            //VeloFactors_Port.interrupt();
            VeloFactors_Port.close();

            endwin();
            return;
        };

        void run()
        {
            
            yarp::sig::Vector &output_veloCoef = VeloFactors_Port.prepare();

            // 
            if((ch = getch())!= ERR)
            {
                if(ch !=27) // ((ch = getch()) !=27 && KeyboardCtrl) //((c= getch())!=27)
                {

                    switch(ch)
                        {
                            case KEY_UP:

                                vx_factor = increment;

                            break;

                            case KEY_DOWN:
                                           
                                     vx_factor = -increment;
                            break;

                            case KEY_LEFT:
                                           
                                    vy_factor = increment;
                            break;

                            case KEY_RIGHT:    
                                            
                                    vy_factor = -increment;
                            break;

                            case 68:  //  CLOCKWISE
                                            
                                    wz_factor = increment;
                            break;

                            case 100: // CLOCKWISE
                                            
                                    wz_factor = increment;
                            break;

                            case 65:   // ANTICLOCKWISE   
                                            
                                    wz_factor = -increment;
                            break;

                            case 97:   // ANTICLOCKWISE
                                            
                                    wz_factor = -increment;
                            break;

                            default:
                                    {   
                                        vx_factor = 0.0; //vx_factor;
                                        vy_factor = 0.0; //vy_factor;
                                        wz_factor = 0.0; //wz_factor;
                                    }
                        }

                    refresh();
                    //printw("Write something (Esc to escape): %d", vx_factor);
                }
                else
                {
                    //VeloFactors_Port.interrupt();
                    endwin();
                    return;
                }
            }
            else
            {
                
                vx_factor = 0.0; //vx_factor;
                vy_factor = 0.0; //vy_factor;
                wz_factor = 0.0; //wz_factor;
                refresh();
            }
            

            VeloFactors[0] = vx_factor;
            VeloFactors[1] = vy_factor;
            VeloFactors[2] = wz_factor;

            output_veloCoef = VeloFactors;

            VeloFactors_Port.write();

        };
        
};


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
    rf.setDefaultConfigFile("KeyboardCommandsReader.ini");        //default config file name.
    //rf.setDefaultContext("KeyboardCommandsReader");             //when no parameters are given to the module this is the default context
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
    double variation = rf.check("variation", Value(0.005), "Looking for module period").asDouble();

    //int ReaderThreadPeriod = rf.find("period").asInt();

   // ====================================================
    KeyboardCommandsReader KeyReader(ReaderThreadPeriod, moduleName);

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
