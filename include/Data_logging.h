#ifndef Data_logging_H
#define Data_logging_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>

#include <yarp/sig/Vector.h>

// #include "CpStanceFootPose.h"
//#include "Discrete_CP_LIP_Model.h"
// #include "CpFootTrajectories.h"
// #include "VelocitiesSetPoints.h"

#include "TemplateModels.h"
#include "PatternsGenerator.h"


using namespace std;
// 
class Data_logging
{

    public:

        double SimTime;

        std::string logPatterns;
        std::string logFeetTrajectories;
        std::string logJointsPosValues;
        std::string logFeetForceTorques;

        // std::ofstream flog_Pattern(const char *fileName);
        // std::ofstream flog_FeetTraj(const char *fileName);
        // std::ofstream flog_JtsPos(const char *fileName);
        // std::ofstream flog_F_FT(const char *fileName);

        std::ofstream *flog_Pattern;
        std::ofstream *flog_FeetTraj;
        std::ofstream *flog_JtsPos;
        std::ofstream *flog_F_FT;


        Data_logging();

        ~Data_logging();

        void InitializeLogger();

        void Write_Data(double SamplingTime,
                        int Cycle_counter,
                        CpStanceFootPose *CoPref,
                        Discrete_CP_LIP_Model *DMod,
                        CpFootTrajectories *FtTraj,
                        VelocitiesSetPoints *VeloRef,
                        yarp::sig::Vector left_commands,
                        yarp::sig::Vector right_commands,
                        yarp::sig::Vector left_encoders,
                        yarp::sig::Vector right_encoders,
                        VectorXd left_Foot_FT,
                        VectorXd right_Foot_FT,
                        VectorXd CoM_position,
                        VectorXd TestData);

        void Close_files();
};

 
#endif // Data_logging_H
