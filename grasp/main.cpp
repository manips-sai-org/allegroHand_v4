#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>  //_getch
#include <string.h>
#include <pthread.h>
#include "canAPI.h"
#include "rDeviceAllegroHandCANDef.h"
#include "RockScissorsPaper.h"
#include <BHand/BHand.h>

#include "RedisClient.h"
#include <Eigen/Core>
#include <boost/algorithm/clamp.hpp>

#define PEAKCAN (1)

typedef char    TCHAR;
#define _T(X)   X
#define _tcsicmp(x, y)   strcmp(x, y)

using namespace std;
using namespace Eigen;

/////////////////////////////////////////////////////////////////////////////////////////
// Redis client and control setup 
RedisClient redis_client;
const string hostname = "127.0.0.1";
const int port = 6379;
const struct timeval& timeout = {1, 500000};

// Read 
const string ALLEGRO_CONTROL_MODE = "allegroHand::controller::control_mode";
const string ALLEGRO_TORQUE_COMMANDED = "allegroHand::controller::joint_torques_commanded";
const string ALLEGRO_POSITION_COMMANDED = "allegroHand::controller::joint_positions_commanded";
const string ALLEGRO_PALM_ORIENTATION = "allegroHand::controller::palm_orientation";
const string ALLEGRO_GRASP_KP = "allegroHand::controller::grasp_kp";
const string ALLEGRO_GRASP_KV = "allegroHand::controller::grasp_kv";

// Write
const string ALLEGRO_CURRENT_POSITIONS = "allegroHand::sensors::joint_positions";
const string ALLEGRO_CURRENT_VELOCITIES = "allegroHand::sensors::joint_velocities";
const string ALLEGRO_DRIVER_READY = "allegroHand::controller::driver_ready";


// Setup control modes 
const int TORQUE_MODE = 0;
const int POSITION_MODE = 1;
const int GRAVITY_MODE = 2; 
int control_mode = GRAVITY_MODE;  // initialize starting control mode
bool driver_ready = false;  // initialize driver not ready 


VectorXd joint_positions = VectorXd::Zero(MAX_DOF);
VectorXd joint_velocities = VectorXd::Zero(MAX_DOF);
MatrixXd R_palm = MatrixXd::Identity(3, 3);
double R_palm_c[] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};
VectorXd joint_torques_commanded = VectorXd::Zero(MAX_DOF);
VectorXd joint_positions_commanded = VectorXd::Zero(MAX_DOF);

// Initial default position control gain values 
MatrixXd kp_pos = MatrixXd::Identity(4, 4);
MatrixXd kv_pos = MatrixXd::Identity(4, 4);

const bool average_filter = true;  // moving average filter for velocity 
int buffer_size = 10;  
int buffer_counter = 0;  
MatrixXd velocity_buffer = MatrixXd::Zero(MAX_DOF, buffer_size);

double tau_max = 0.65;  // safety saturation (0.7 is hardware limits)


/////////////////////////////////////////////////////////////////////////////////////////
// for CAN communication
const double delT = 0.003;
int CAN_Ch = 0;
bool ioThreadRun = false;
pthread_t        hThread;
int recvNum = 0;
int sendNum = 0;
double statTime = -1.0;
AllegroHand_DeviceMemory_t vars;

double curTime = 0.0;

/////////////////////////////////////////////////////////////////////////////////////////
// for BHand library
BHand* pBHand = NULL;
double q[MAX_DOF];
double q_des[MAX_DOF];
double tau_des[MAX_DOF];
double cur_des[MAX_DOF];
/* Added */
double q_prev[MAX_DOF]; 
double dq[MAX_DOF];  
double gravity_torque[MAX_DOF];

// USER HAND CONFIGURATION
const bool RIGHT_HAND = false;
const int HAND_VERSION = 4;

const double tau_cov_const_v4 = 1200.0; // 1200.0 for SAH040xxxxx

/////////////////////////////////////////////////////////////////////////////////////////
// functions declarations
char Getch();
void PrintInstruction();
void MainLoop();
bool OpenCAN();
void CloseCAN();
int GetCANChannelIndex(const TCHAR* cname);
bool CreateBHandAlgorithm();
void DestroyBHandAlgorithm();
void ComputeTorque();

/////////////////////////////////////////////////////////////////////////////////////////
// Read keyboard input (one char) from stdin
char Getch()
{
    /*#include <unistd.h>   //_getch*/
    /*#include <termios.h>  //_getch*/
    char buf=0;
    struct termios old={0};
    fflush(stdout);
    if(tcgetattr(0, &old)<0)
        perror("tcsetattr()");
    old.c_lflag&=~ICANON;
    old.c_lflag&=~ECHO;
    old.c_cc[VMIN]=1;
    old.c_cc[VTIME]=0;
    if(tcsetattr(0, TCSANOW, &old)<0)
        perror("tcsetattr ICANON");
    if(read(0,&buf,1)<0)
        perror("read()");
    old.c_lflag|=ICANON;
    old.c_lflag|=ECHO;
    if(tcsetattr(0, TCSADRAIN, &old)<0)
        perror ("tcsetattr ~ICANON");
    printf("%c\n",buf);
    return buf;
}

/////////////////////////////////////////////////////////////////////////////////////////
// CAN communication thread
static void* ioThreadProc(void* inst)
{
    int id;
    int len;
    unsigned char data[8];
    unsigned char data_return = 0;
    int i;

    // Initialize redis
    redis_client.connect(hostname, port, timeout);
    redis_client.set(ALLEGRO_CONTROL_MODE, std::to_string(control_mode));  
    redis_client.setEigenMatrixJSON(ALLEGRO_TORQUE_COMMANDED, joint_torques_commanded);
    redis_client.setEigenMatrixJSON(ALLEGRO_PALM_ORIENTATION, R_palm);

    // Initial default position control gain values 
    kp_pos = MatrixXd::Constant(4, 4, 5.0);
    kp_pos.row(3).setConstant(2.5);
    kv_pos = MatrixXd::Constant(4, 4, 0.09);

    for (int i = 0; i < MAX_DOF; i++) {
        joint_positions(i) = q[i];  // startup value is 0 for q (default) : this is redundant 
    }
    redis_client.setEigenMatrixJSON(ALLEGRO_CURRENT_POSITIONS, joint_positions);
    redis_client.setEigenMatrixJSON(ALLEGRO_CURRENT_VELOCITIES, joint_velocities);
    redis_client.setEigenMatrixJSON(ALLEGRO_POSITION_COMMANDED, joint_positions);  
    redis_client.setEigenMatrixJSON(ALLEGRO_GRASP_KP, kp_pos);
    redis_client.setEigenMatrixJSON(ALLEGRO_GRASP_KV, kv_pos);  
    redis_client.set(ALLEGRO_DRIVER_READY, "0");  // set to driver not ready (false)


    // Create read and write callback
    redis_client.createReadCallback(0);  
    redis_client.addIntToReadCallback(0, ALLEGRO_CONTROL_MODE, control_mode);
    redis_client.addEigenToReadCallback(0, ALLEGRO_TORQUE_COMMANDED, joint_torques_commanded);
    redis_client.addEigenToReadCallback(0, ALLEGRO_POSITION_COMMANDED, joint_positions_commanded);
    redis_client.addEigenToReadCallback(0, ALLEGRO_PALM_ORIENTATION, R_palm);
    redis_client.addEigenToReadCallback(0, ALLEGRO_GRASP_KP, kp_pos);
    redis_client.addEigenToReadCallback(0, ALLEGRO_GRASP_KV, kv_pos);

    redis_client.createWriteCallback(0);
    redis_client.addEigenToWriteCallback(0, ALLEGRO_CURRENT_POSITIONS, joint_positions);
    redis_client.addEigenToWriteCallback(0, ALLEGRO_CURRENT_VELOCITIES, joint_velocities);

    // Initial starting behavior
    pBHand->SetMotionType(eMotionType_GRAVITY_COMP);

    while (ioThreadRun)
    {
        /* wait for the event */
        while (0 == get_message(CAN_Ch, &id, &len, data, FALSE))
        {
            //            printf(">CAN(%d): ", CAN_Ch);
            //            for(int nd=0; nd<len; nd++)
            //                printf("%02x ", data[nd]);
            //            printf("\n");

            switch (id)
            {
            case ID_RTR_HAND_INFO:
            {
                printf(">CAN(%d): AllegroHand hardware version: 0x%02x%02x\n", CAN_Ch, data[1], data[0]);
                printf("                      firmware version: 0x%02x%02x\n", data[3], data[2]);
                printf("                      hardware type: %d(%s)\n", data[4], (data[4] == 0 ? "right" : "left"));
                printf("                      temperature: %d (celsius)\n", data[5]);
                printf("                      status: 0x%02x\n", data[6]);
                printf("                      servo status: %s\n", (data[6] & 0x01 ? "ON" : "OFF"));
                printf("                      high temperature fault: %s\n", (data[6] & 0x02 ? "ON" : "OFF"));
                printf("                      internal communication fault: %s\n", (data[6] & 0x04 ? "ON" : "OFF"));
            }
                break;
            case ID_RTR_SERIAL:
            {
                printf(">CAN(%d): AllegroHand serial number: SAH0%d0 %c%c%c%c%c%c%c%c\n", CAN_Ch, HAND_VERSION
                       , data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
            }
                break;
            case ID_RTR_FINGER_POSE_1:
            case ID_RTR_FINGER_POSE_2:
            case ID_RTR_FINGER_POSE_3:
            case ID_RTR_FINGER_POSE_4:
            {
                int findex = (id & 0x00000007);

                vars.enc_actual[findex*4 + 0] = (short)(data[0] | (data[1] << 8));
                vars.enc_actual[findex*4 + 1] = (short)(data[2] | (data[3] << 8));
                vars.enc_actual[findex*4 + 2] = (short)(data[4] | (data[5] << 8));
                vars.enc_actual[findex*4 + 3] = (short)(data[6] | (data[7] << 8));
                data_return |= (0x01 << (findex));
                recvNum++;

                //                printf(">CAN(%d): Encoder[%d] Count : %6d %6d %6d %6d\n"
                //                    , CAN_Ch, findex
                //                    , vars.enc_actual[findex*4 + 0], vars.enc_actual[findex*4 + 1]
                //                    , vars.enc_actual[findex*4 + 2], vars.enc_actual[findex*4 + 3]);

                if (data_return == (0x01 | 0x02 | 0x04 | 0x08))
                {                    
                    // convert encoder count to joint angle
                    for (i=0; i<MAX_DOF; i++)
                    {
                        q_prev[i] = q[i];  // added to save previous joint position 
                        q[i] = (double)(vars.enc_actual[i])*(333.3/65536.0)*(3.141592/180.0);
                    }

                    // print joint angles
                    //                    for (int i=0; i<4; i++)
                    //                    {
                    //                        printf(">CAN(%d): Joint[%d] Pos : %5.1f %5.1f %5.1f %5.1f\n"
                    //                            , CAN_Ch, i, q[i*4+0]*RAD2DEG, q[i*4+1]*RAD2DEG, q[i*4+2]*RAD2DEG, q[i*4+3]*RAD2DEG);
                    //                    }

                    /* Updated code for velocity filter, torque mode option, and safety */

                    // Update velocity filter
                    if (average_filter)
                    {
                        for (int i = 0; i < MAX_DOF; i++)
                        {
                            velocity_buffer(i, buffer_counter) = (q[i] - q_prev[i]) / delT;
                            dq[i] = velocity_buffer.row(i).sum() / buffer_size;  
                        }

                        // Update buffer counter (modulus is another approach)
                        if (buffer_counter == buffer_size - 1) {
                            buffer_counter = 0;
                        } else {
                            buffer_counter += 1;
                        }
                    }
                    else
                    {
                        for (int i = 0; i < MAX_DOF; i++)
                        {
                            dq[i] = (q[i] - q_prev[i]) / delT;
                        }
                    }

                    // Transfer kinematics information to variables 
                    for (int i = 0; i < MAX_DOF; i++)
                    {
                        joint_positions(i) = q[i];
                        joint_velocities(i) = dq[i];
                    }

                    // Debug outputs 
                    if (sendNum % 1000 == 0)
                    {
                        // std::cout << "Control Mode: " << control_mode << std::endl;
                        for (int i = 0; i < 4; i++) 
                        {
                            // std::cout << std::setprecision(2) << joint_positions.segment(4 * i, 4).transpose() << std::endl;
                            // std::cout << std::setprecision(2) << joint_velocities.segment(4 * i, 4).transpose() << std::endl;

                        }
                    }

                    // Read and write redis keys 
                    redis_client.executeWriteCallback(0);  // write (joint positions, joint velocities)
                    redis_client.executeReadCallback(0);  // read (control mode, commanded torques, commanded positions, palm orientation)

                    // Obtain gravity torques 
                    R_palm.transposeInPlace();  // row-major ordering enforcement; used for gravity compensation 
                    R_palm.resize(9, 1);
                    for (int i = 0; i < 9; i++) {
                        R_palm_c[i] = R_palm(i);
                    }

                    pBHand->SetOrientation(R_palm_c);  // for gravity vector direction wrt palm
                    pBHand->SetJointPosition(q);
                    pBHand->SetJointDesiredPosition(q);  // enforcing 0 position error to only extract gravity torque
                    pBHand->UpdateControl(0);
                    pBHand->GetJointTorque(gravity_torque);
                    // gravity_torque[12] = 0;  // artifact from previous driver

                    if (sendNum < 20)  // wait 20 cycles for CAN communication to update joint positions    
                    {
                        for (int i = 0; i < MAX_DOF; i++)
                        {
                            tau_des[i] = 0;  // stiction will hold the hand in place; gravity torque used here will assume q = 0, which may not be q current
                        }
                    }
                    else if (driver_ready == false)
                    {
                        redis_client.setEigenMatrixJSON(ALLEGRO_POSITION_COMMANDED, joint_positions);  // set the current hand configuration as the current desired position 
                        driver_ready = true;
                        redis_client.set(ALLEGRO_DRIVER_READY, "1");  // std::stoi() from the controller side 
                    }
                    else if (control_mode == TORQUE_MODE)
                    {
                        for (int i = 0; i < MAX_DOF; i++)
                        {
                            tau_des[i] = joint_torques_commanded(i) + gravity_torque[i]; 
                        }
                        redis_client.setEigenMatrixJSON(ALLEGRO_POSITION_COMMANDED, joint_positions);  // set the current hand configuration as the current desired position 
                    }
                    else if (control_mode == POSITION_MODE)
                    {
                        // pBHand->SetJointPosition(q);
                        // q_des = redis key value
                        // pBHand->SetJointDesiredPosition(q_des);  // enforcing 0 position error to only extract gravity torque
                        // pBHand->UpdateControl(0);
                        for (int i = 0; i < MAX_DOF; i++)
                        {
                            tau_des[i] = - kp_pos(i) * (q[i] - joint_positions_commanded(i)) - kv_pos(i) * dq[i] + gravity_torque[i];  
                        }
                    }
                    else  
                    {
                        for (int i = 0; i < MAX_DOF; i++)
                        {
                            tau_des[i] = gravity_torque[i];
                        }
                        redis_client.setEigenMatrixJSON(ALLEGRO_POSITION_COMMANDED, joint_positions);  // set the current hand configuration as the current desired position 
                    }

                    // Torque saturation
                    for (int i = 0; i < MAX_DOF; i++)
                    {
                        tau_des[i] = boost::algorithm::clamp(tau_des[i], -tau_max, tau_max);
                    }

                    /* Previous implementation of position-only torque control (private library) */
                    // ComputeTorque();  

                    // convert desired torque to desired current and PWM count
                    for (int i=0; i<MAX_DOF; i++)
                    {
                        cur_des[i] = tau_des[i];
                        if (cur_des[i] > 1.0) cur_des[i] = 1.0;
                        else if (cur_des[i] < -1.0) cur_des[i] = -1.0;
                    }

                    // send torques
                    for (int i=0; i<4;i++)
                    {
                        vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+0]*tau_cov_const_v4);
                        vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+1]*tau_cov_const_v4);
                        vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+2]*tau_cov_const_v4);
                        vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+3]*tau_cov_const_v4);

                        command_set_torque(CAN_Ch, i, &vars.pwm_demand[4*i]);
                        //usleep(5);
                    }
                    sendNum++;
                    curTime += delT;
                    data_return = 0;
                }
            }
                break;
            case ID_RTR_IMU_DATA:
            {
                printf(">CAN(%d): AHRS Roll : 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
                printf("               Pitch: 0x%02x%02x\n", data[2], data[3]);
                printf("               Yaw  : 0x%02x%02x\n", data[4], data[5]);
            }
                break;
            case ID_RTR_TEMPERATURE_1:
            case ID_RTR_TEMPERATURE_2:
            case ID_RTR_TEMPERATURE_3:
            case ID_RTR_TEMPERATURE_4:
            {
                int sindex = (id & 0x00000007);
                int celsius = (int)(data[0]      ) |
                              (int)(data[1] << 8 ) |
                              (int)(data[2] << 16) |
                              (int)(data[3] << 24);
                printf(">CAN(%d): Temperature[%d]: %d (celsius)\n", CAN_Ch, sindex, celsius);
            }
                break;
            default:
                printf(">CAN(%d): unknown command %d, len %d\n", CAN_Ch, id, len);
                /*for(int nd=0; nd<len; nd++)
                    printf("%d \n ", data[nd]);*/
                //return;
            }
        }
    }
    return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Application main-loop. It handles the commands from rPanelManipulator and keyboard events
void MainLoop()
{
    std::cout << "Starting Allegro Hand Driver :)" << std::endl;

    bool bRun = true;

    // Set control gains (if using the private library)
    // pBHand->SetGainsEx(kp_default, kv_default); // kp_default and kv_default are arrays of doubles of size 16

    // Main loop 
    int cnt = 0;
    while (bRun) 
    {
        // Output necessary information to terminal here        
        cnt += 1;
    }

}

/////////////////////////////////////////////////////////////////////////////////////////
// Compute control torque for each joint using BHand library
void ComputeTorque()
{
    if (!pBHand) return;
    pBHand->SetJointPosition(q); // tell BHand library the current joint positions
    pBHand->SetJointDesiredPosition(q_des);
    pBHand->UpdateControl(0);
    pBHand->GetJointTorque(tau_des);

//    static int j_active[] = {
//        0, 0, 0, 0,
//        0, 0, 0, 0,
//        0, 0, 0, 0,
//        1, 1, 1, 1
//    };
//    for (int i=0; i<MAX_DOF; i++) {
//        if (j_active[i] == 0) {
//            tau_des[i] = 0;
//        }
//    }
}

/////////////////////////////////////////////////////////////////////////////////////////
// Open a CAN data channel
bool OpenCAN()
{
#if defined(PEAKCAN)
    CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
#elif defined(IXXATCAN)
    CAN_Ch = 1;
#elif defined(SOFTINGCAN)
    CAN_Ch = 1;
#else
    CAN_Ch = 1;
#endif
    printf(">CAN(%d): open\n", CAN_Ch);

    int ret = command_can_open(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR command_can_open !!! \n");
        return false;
    }

    // initialize CAN I/O thread
    ioThreadRun = true;
    /*int ioThread_error = */pthread_create(&hThread, NULL, ioThreadProc, 0);
    printf(">CAN: starts listening CAN frames\n");

    // query h/w information
    printf(">CAN: query system information\n");
    ret = request_hand_information(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR request_hand_information !!! \n");
        command_can_close(CAN_Ch);
        return false;
    }
    ret = request_hand_serial(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR request_hand_serial !!! \n");
        command_can_close(CAN_Ch);
        return false;
    }

    // set periodic communication parameters(period)
    printf(">CAN: Comm period set\n");
    short comm_period[3] = {3, 0, 0}; // millisecond {position, imu, temperature}
    ret = command_set_period(CAN_Ch, comm_period);
    if(ret < 0)
    {
        printf("ERROR command_set_period !!! \n");
        command_can_close(CAN_Ch);
        return false;
    }

    // servo on
    printf(">CAN: servo on\n");
    ret = command_servo_on(CAN_Ch);
    if(ret < 0)
    {
        printf("ERROR command_servo_on !!! \n");
        command_set_period(CAN_Ch, 0);
        command_can_close(CAN_Ch);
        return false;
    }

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Close CAN data channel
void CloseCAN()
{
    printf(">CAN: stop periodic communication\n");
    int ret = command_set_period(CAN_Ch, 0);
    if(ret < 0)
    {
        printf("ERROR command_can_stop !!! \n");
    }

    if (ioThreadRun)
    {
        printf(">CAN: stoped listening CAN frames\n");
        ioThreadRun = false;
        int status;
        pthread_join(hThread, (void **)&status);
        hThread = 0;
    }

    printf(">CAN(%d): close\n", CAN_Ch);
    ret = command_can_close(CAN_Ch);
    if(ret < 0) printf("ERROR command_can_close !!! \n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Load and create grasping algorithm
bool CreateBHandAlgorithm()
{
    if (RIGHT_HAND)
        pBHand = bhCreateRightHand();
    else
        pBHand = bhCreateLeftHand();

    if (!pBHand) return false;
    pBHand->SetMotionType(eMotionType_NONE);
    pBHand->SetTimeInterval(delT);
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Destroy grasping algorithm
void DestroyBHandAlgorithm()
{
    if (pBHand)
    {
#ifndef _DEBUG
        delete pBHand;
#endif
        pBHand = NULL;
    }
}

////////////////////////////////////////////////////////////////////////////////////////
// Print program information and redis instructions
void PrintInstruction()
{
    printf("--------------------------------------------------\n");
    printf("Allegro Hand: ");
    if (RIGHT_HAND) printf("Right Hand, v%i.x\n\n", HAND_VERSION); else printf("Left Hand, v%i.x\n\n", HAND_VERSION);

    printf(">> Default initialization:\n");
    printf("   Control: Gravity compensation\n");
    printf("   Palm orientation: identity\n\n\n");

    printf(">> Control Modes:\n");
    printf("   0: Torque control \n");
    printf("   1: Position control\n");
    printf("   2: Gravity compensation\n\n\n");

    printf(">> Redis key to switch control: \n   set \"allegroHand::controller::control_mode\" mode_#\n\n");
    printf(">> Redis key to SET joint positions: \n   set \"allegroHand::controller::joint_positions_commanded\" \"[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]\"\n\n");
    printf(">> Redis key to GET joint positions: \n   get \"allegroHand::sensors::joint_positions\"\n\n");
    printf(">> Redis key to set kp: \n   set \"allegroHand::controller::grasp_kp\" \"[[5.000000,5.000000,5.000000,5.000000],[5.000000,5.000000,5.000000,5.000000],[5.000000,5.000000,5.000000,5.000000],[2.500000,2.500000,2.500000,2.500000]]\"\n\n");
    printf(">> Redis key to set kv: \n   set \"allegroHand::controller::grasp_kv\" \"[[0.090000,0.090000,0.090000,0.090000],[0.090000,0.090000,0.090000,0.090000],[0.090000,0.090000,0.090000,0.090000],[0.090000,0.090000,0.090000,0.090000]]\"\n\n\n");
    // printf(">> Redis key to set joint torques: \n   set \"allegroHand::controller::joint_torques_commanded\" \"[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]\"\n\n");
    // printf(">> Redis key to set palm orientation: \n   set \"allegroHand::controller::palm_orientation\" \"[[0,0,1],[0,1,0],[-1,0,0]]\"\n\n");
    
    printf("--------------------------------------------------\n\n");

    printf(">> PREDEFINED GRASPS \n\n");
    printf("set \"allegroHand::controller::control_mode\" 1\n\n");
    printf("<HOME> \nset \"allegroHand::controller::joint_positions_commanded\" \"[0.094000,0.264248,0.001331,0.905473,0.062134,0.211345,0.122759,1.074300,0.069945,0.417897,0.051660,1.041635,1.280231,0.145660,0.261052,0.395795]\"\n\n");
    // printf("<READY> \nset \"allegroHand::controller::joint_positions_commanded\" \"[0.044559,0.657735,0.275965,0.908136,0.115747,0.763629,0.135364,0.903431,0.060359,0.731675,0.105539,1.031250,1.200877,0.334371,0.538792,0.038967]\"\n\n");
    printf("<BONNIE/CLYDE_BOTTLE> \nset \"allegroHand::controller::joint_positions_commanded\" \"[0.203978,1.342276,0.850173,0.530005,-0.087698,1.424116,0.468758,1.063294,-0.129239,1.354526,0.643000,0.879998,1.432282,0.048820,0.388250,0.894644]\"\n\n");
    // printf("<CLYDE_CAP> \nset \"allegroHand::controller::joint_positions_commanded\" \"[0.256703,0.610779,0.621875,1.624809,-0.058051,0.624271,0.460414,1.186497,-0.215783,0.734426,0.295226,0.557876,1.110338,0.417098,0.831622,0.576428]\"\n\n");
    printf("<READY_4_CAP> \nset \"allegroHand::controller::joint_positions_commanded\" \"[-0.534798,0.681523,0.140956,1.011900,0.199540,0.705134,0.069235,0.951008,0.104918,0.128085,0.371563,0.619567,0.985892,0.674245,0.564534,0.287770]\"\n\n");
    printf("<CAP> \nset \"allegroHand::controller::joint_positions_commanded\" \"[-0.233447,0.610779,0.621875,1.348756,0.045713,0.624271,0.460414,1.186497,0.233270,0.514294,-0.017131,0.525212,1.110338,0.417098,0.831622,0.576428]\"\n\n");
    // printf("<CLYDE_LIGHTBULB> \nset \"allegroHand::controller::joint_positions_commanded\" \"[0.003906,1.040215,0.305967,1.161022,-0.036925,0.803484,0.420116,1.236914,0.004438,0.189864,0.180811,0.714455,1.269402,0.584505,0.642734,0.393398]\"\n\n");
    
    printf("--------------------------------------------------\n\n");

}

/////////////////////////////////////////////////////////////////////////////////////////
// Get channel index for Peak CAN interface
int GetCANChannelIndex(const TCHAR* cname)
{
    if (!cname) return 0;

    if (!_tcsicmp(cname, _T("0")) || !_tcsicmp(cname, _T("PCAN_NONEBUS")) || !_tcsicmp(cname, _T("NONEBUS")))
        return 0;
    else if (!_tcsicmp(cname, _T("1")) || !_tcsicmp(cname, _T("PCAN_ISABUS1")) || !_tcsicmp(cname, _T("ISABUS1")))
        return 1;
    else if (!_tcsicmp(cname, _T("2")) || !_tcsicmp(cname, _T("PCAN_ISABUS2")) || !_tcsicmp(cname, _T("ISABUS2")))
        return 2;
    else if (!_tcsicmp(cname, _T("3")) || !_tcsicmp(cname, _T("PCAN_ISABUS3")) || !_tcsicmp(cname, _T("ISABUS3")))
        return 3;
    else if (!_tcsicmp(cname, _T("4")) || !_tcsicmp(cname, _T("PCAN_ISABUS4")) || !_tcsicmp(cname, _T("ISABUS4")))
        return 4;
    else if (!_tcsicmp(cname, _T("5")) || !_tcsicmp(cname, _T("PCAN_ISABUS5")) || !_tcsicmp(cname, _T("ISABUS5")))
        return 5;
    else if (!_tcsicmp(cname, _T("7")) || !_tcsicmp(cname, _T("PCAN_ISABUS6")) || !_tcsicmp(cname, _T("ISABUS6")))
        return 6;
    else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS7")) || !_tcsicmp(cname, _T("ISABUS7")))
        return 7;
    else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS8")) || !_tcsicmp(cname, _T("ISABUS8")))
        return 8;
    else if (!_tcsicmp(cname, _T("9")) || !_tcsicmp(cname, _T("PCAN_DNGBUS1")) || !_tcsicmp(cname, _T("DNGBUS1")))
        return 9;
    else if (!_tcsicmp(cname, _T("10")) || !_tcsicmp(cname, _T("PCAN_PCIBUS1")) || !_tcsicmp(cname, _T("PCIBUS1")))
        return 10;
    else if (!_tcsicmp(cname, _T("11")) || !_tcsicmp(cname, _T("PCAN_PCIBUS2")) || !_tcsicmp(cname, _T("PCIBUS2")))
        return 11;
    else if (!_tcsicmp(cname, _T("12")) || !_tcsicmp(cname, _T("PCAN_PCIBUS3")) || !_tcsicmp(cname, _T("PCIBUS3")))
        return 12;
    else if (!_tcsicmp(cname, _T("13")) || !_tcsicmp(cname, _T("PCAN_PCIBUS4")) || !_tcsicmp(cname, _T("PCIBUS4")))
        return 13;
    else if (!_tcsicmp(cname, _T("14")) || !_tcsicmp(cname, _T("PCAN_PCIBUS5")) || !_tcsicmp(cname, _T("PCIBUS5")))
        return 14;
    else if (!_tcsicmp(cname, _T("15")) || !_tcsicmp(cname, _T("PCAN_PCIBUS6")) || !_tcsicmp(cname, _T("PCIBUS6")))
        return 15;
    else if (!_tcsicmp(cname, _T("16")) || !_tcsicmp(cname, _T("PCAN_PCIBUS7")) || !_tcsicmp(cname, _T("PCIBUS7")))
        return 16;
    else if (!_tcsicmp(cname, _T("17")) || !_tcsicmp(cname, _T("PCAN_PCIBUS8")) || !_tcsicmp(cname, _T("PCIBUS8")))
        return 17;
    else if (!_tcsicmp(cname, _T("18")) || !_tcsicmp(cname, _T("PCAN_USBBUS1")) || !_tcsicmp(cname, _T("USBBUS1")))
        return 18;
    else if (!_tcsicmp(cname, _T("19")) || !_tcsicmp(cname, _T("PCAN_USBBUS2")) || !_tcsicmp(cname, _T("USBBUS2")))
        return 19;
    else if (!_tcsicmp(cname, _T("20")) || !_tcsicmp(cname, _T("PCAN_USBBUS3")) || !_tcsicmp(cname, _T("USBBUS3")))
        return 20;
    else if (!_tcsicmp(cname, _T("21")) || !_tcsicmp(cname, _T("PCAN_USBBUS4")) || !_tcsicmp(cname, _T("USBBUS4")))
        return 21;
    else if (!_tcsicmp(cname, _T("22")) || !_tcsicmp(cname, _T("PCAN_USBBUS5")) || !_tcsicmp(cname, _T("USBBUS5")))
        return 22;
    else if (!_tcsicmp(cname, _T("23")) || !_tcsicmp(cname, _T("PCAN_USBBUS6")) || !_tcsicmp(cname, _T("USBBUS6")))
        return 23;
    else if (!_tcsicmp(cname, _T("24")) || !_tcsicmp(cname, _T("PCAN_USBBUS7")) || !_tcsicmp(cname, _T("USBBUS7")))
        return 24;
    else if (!_tcsicmp(cname, _T("25")) || !_tcsicmp(cname, _T("PCAN_USBBUS8")) || !_tcsicmp(cname, _T("USBBUS8")))
        return 25;
    else if (!_tcsicmp(cname, _T("26")) || !_tcsicmp(cname, _T("PCAN_PCCBUS1")) || !_tcsicmp(cname, _T("PCCBUS1")))
        return 26;
    else if (!_tcsicmp(cname, _T("27")) || !_tcsicmp(cname, _T("PCAN_PCCBUS2")) || !_tcsicmp(cname, _T("PCCBUS2")))
        return 27;
    else
        return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Program main
int main(int argc, TCHAR* argv[])
{
    PrintInstruction();

    memset(&vars, 0, sizeof(vars));
    memset(q, 0, sizeof(q));
    memset(q_des, 0, sizeof(q_des));
    memset(tau_des, 0, sizeof(tau_des));
    memset(cur_des, 0, sizeof(cur_des));
    /* Added */
    memset(q_prev, 0, sizeof(q_prev));
    memset(dq, 0, sizeof(dq));
    memset(gravity_torque, 0, sizeof(gravity_torque));
    curTime = 0.0;

    if (CreateBHandAlgorithm() && OpenCAN())
        MainLoop();

    CloseCAN();
    DestroyBHandAlgorithm();

    return 0;
}