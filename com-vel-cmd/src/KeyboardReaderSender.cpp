#include "KeyboardReaderSender.h"


bool KeyboardReaderSender::threadInit()
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
}

void KeyboardReaderSender::threadRelease()
{

    // closing the port
    //VeloFactors_Port.interrupt();
    VeloFactors_Port.close();

    endwin();
    return;
}

void KeyboardReaderSender::run()
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
                        std::cout << "vx_factor" << vx_factor << std::endl;

                    break;

                    case KEY_DOWN:
                                   
                             vx_factor = -increment;
                             std::cout << "vx_factor" << vx_factor << std::endl;
                    break;

                    case KEY_LEFT:
                                   
                            vy_factor = increment;
                            std::cout << "vy_factor" << vy_factor << std::endl;
                    break;

                    case KEY_RIGHT:    
                                    
                            vy_factor = -increment;
                            std::cout << "vy_factor" << vy_factor << std::endl;
                    break;

                    case 98:
                            vx_factor = NAN;
                            std::cout << "Pressed (b) key! Stop Ctrl!" << std::endl;
                    break; 

                    case 68:  //  CLOCKWISE
                                    
                            wz_factor = increment;
                            std::cout << "wz_factor" << wz_factor << std::endl;
                    break;

                    case 100: // CLOCKWISE
                                    
                            wz_factor = increment;
                            std::cout << "wz_factor" << wz_factor << std::endl;
                    break;

                    case 65:   // ANTICLOCKWISE   
                                    
                            wz_factor = -increment;
                            std::cout << "wz_factor" << wz_factor << std::endl;
                    break;

                    case 97:   // ANTICLOCKWISE
                                    
                            wz_factor = -increment;
                            std::cout << "wz_factor" << wz_factor << std::endl;
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
        // IDK why you are doing this, when I remove it, I get it now!
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

}       