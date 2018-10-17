
#include "Data_logging.h"

Data_logging::Data_logging()
{
	
}

Data_logging::~Data_logging()
{
	Data_logging::Close_files();

    if (flog_Pattern) {
        delete flog_Pattern;
        flog_Pattern = 0;
    }

    if (flog_FeetTraj) {
        delete flog_FeetTraj;
        flog_FeetTraj = 0;
    }

    if (flog_JtsPos) {
        delete flog_JtsPos;
        flog_JtsPos = 0;
    }

    if (flog_F_FT) {
        delete flog_F_FT;
        flog_F_FT = 0;
    }

}

void Data_logging::InitializeLogger()
{
    logPatterns 		= "./PatternsData.txt";
    logFeetTrajectories = "./FeetTrajectoriesData.txt";
    logJointsPosValues  = "./JointsPosData.txt";
    logFeetForceTorques = "./FeetForceTorquesData.txt";

	

    flog_Pattern  = new std::ofstream(logPatterns.c_str());
    flog_FeetTraj = new std::ofstream(logFeetTrajectories.c_str());
    flog_JtsPos   = new std::ofstream(logJointsPosValues.c_str());
    flog_F_FT     = new std::ofstream(logFeetForceTorques.c_str());

	SimTime = 0.0;
}


void Data_logging::Write_Data(double SamplingTime,
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
                              VectorXd TestData)
{

    // Write the data
	SimTime = SamplingTime * Cycle_counter;

	   // Patterns [ZMP_ref, ZMP, CoM, CP_ref, CP]
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    *flog_Pattern << SimTime <<"    " ;
    // x
    *flog_Pattern << CoPref->CoPRefX <<"    "<<  DMod->OutputX <<"    "<< (DMod->StatesX)(0)<<"   "<< (DMod->ZMP_X)<<"  ";
    // y
    *flog_Pattern << CoPref->CoPRefY <<"    "<<  DMod->OutputY <<"    "<< (DMod->StatesY)(0)<<"   "<< (DMod->ZMP_Y)<<"  ";
    // Rotation
    *flog_Pattern << CoPref->CoPRefR <<"    "<<  DMod->OutputR <<"    "<< (DMod->StatesR)(1)<<"   ";
    // Reference Capture Point
    // x and y
    *flog_Pattern << CoPref->xCP_ref <<"    "<<  CoPref->yCP_ref <<"   ";
    // reference velocity
    *flog_Pattern << VeloRef->VxAbs(0) <<"    "<<  VeloRef->VyAbs(0) <<"   "<< VeloRef->WzAbs(0) <<"   "<<std::endl;


    
    // Feet trajectories : [x y z]_left, [x y z]_right
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    *flog_FeetTraj << SimTime <<"   ";
    // left
    *flog_FeetTraj <<  FtTraj->XTrajLeftFoot <<"  "<< FtTraj->YTrajLeftFoot<<"   "<< FtTraj->ZTrajLeftFoot<<"   "<< FtTraj->AngTrajLeftFoot<<"   ";
    // right
    *flog_FeetTraj <<  FtTraj->XTrajRightFoot<<"  "<< FtTraj->YTrajRightFoot<<"   "<< FtTraj->ZTrajRightFoot<<"   "<< FtTraj->AngTrajRightFoot<<"   ";
    // actual CoM position
    *flog_FeetTraj <<  CoM_position(0) <<"  "<< CoM_position(1) <<"   "<<  CoM_position(2) <<"   ";
    // 
    *flog_FeetTraj <<  FtTraj->StanceIndicator[0] <<"  "<< TestData(0) <<"   "<<  TestData(1) <<"   "<< std::endl;


    // Legs Joints positions
    // ~~~~~~~~~~~~~~~~~~~~~
    *flog_JtsPos << SimTime <<" ";
    // left commands
    *flog_JtsPos << left_commands[0] <<"    "<< left_commands[1] <<"    "<< left_commands[2] <<"    "<< left_commands[3] <<"    "<< left_commands[4] <<"    "<<left_commands[5] <<" ";
    // right commands
    *flog_JtsPos << right_commands[0] <<"   "<< right_commands[1] <<"   "<< right_commands[2] <<"   "<< right_commands[3] <<"   "<< right_commands[4] <<"   "<<right_commands[5] <<"    ";
    // left encoders
    *flog_JtsPos << left_encoders[0] <<"    "<< left_encoders[1] <<"    "<< left_encoders[2] <<"    "<< left_encoders[3] <<"    "<< left_encoders[4] <<"    "<<left_encoders[5] <<" ";
    // right encoders
    *flog_JtsPos << right_encoders[0] <<"   "<< right_encoders[1] <<"   "<< right_encoders[2] <<"   "<< right_encoders[3] <<"   "<< right_encoders[4] <<"   "<<right_encoders[5] <<"    "<< std::endl;


    // Feet Force/Torques [fx fy fz mx my mz]_left/right
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    *flog_F_FT << SimTime <<"   ";
    // left force/torques
    *flog_F_FT << left_Foot_FT[0] <<"   "<< left_Foot_FT[1] <<" "<< left_Foot_FT[2] <<" "<< left_Foot_FT[3] <<" "<< left_Foot_FT[4] <<" "<<left_Foot_FT[5] <<"  ";
    // right forces/torques
    *flog_F_FT << right_Foot_FT[0] <<"  "<< right_Foot_FT[1] <<"    "<< right_Foot_FT[2] <<"    "<< right_Foot_FT[3] <<"    "<< right_Foot_FT[4] <<"    "<<right_Foot_FT[5] <<" "<< std::endl;
}

void Data_logging::Close_files()
{
	// close the files
    flog_Pattern->close();
    flog_FeetTraj->close();
    flog_JtsPos->close();
    flog_F_FT->close();

    // // nullify the pointers
    // if (flog_Pattern) {
    //     delete flog_Pattern;
    //     flog_Pattern = 0;
    // }

    // if (flog_FeetTraj) {
    //     delete flog_FeetTraj;
    //     flog_FeetTraj = 0;
    // }

    // if (flog_JtsPos) {
    //     delete flog_JtsPos;
    //     flog_JtsPos = 0;
    // }

    // if (flog_F_FT) {
    //     delete flog_F_FT;
    //     flog_F_FT = 0;
    // }

}
