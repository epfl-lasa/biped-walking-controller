#include <iostream>
#include <iomanip>
#include <math.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/BufferedPort.h>
#include <ncurses.h>

using namespace std;
using namespace yarp::os;

class KeyboardReaderSender : public yarp::os::RateThread 
{
    
    private: 

        int ThreadPeriod;                       // thread period
        

    public:
        std::string moduleName;
        std::string robotName;

        int ch;

        double increment;

        double vx_factor;
        double vy_factor;
        double wz_factor;

        // port to stream the value of the coefficients
        BufferedPort<yarp::sig::Vector> VeloFactors_Port;
        yarp::sig::Vector VeloFactors;        


        KeyboardReaderSender(int period, std::string KeyboardModule)  : RateThread(period)
                                                                        , moduleName(KeyboardModule)
                                                                        , VeloFactors(3) {ThreadPeriod = period; }

        bool threadInit();

        void threadRelease();

        void run();
        
};
