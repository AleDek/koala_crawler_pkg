
#include <chrono>
#include <thread>
#include <iostream>
#include <boost/thread.hpp>

#include "KoalaCommander.h"

// #include "ros/ros.h" //include ros header file
// #include "std_msgs/Float32.h"  //header file of the message to publish
// #include "std_msgs/Int32.h"


using namespace std;

struct crawler_feedback fb;
struct crawler_setpoint sp;
float force_global_sp;

mutex sem;
KoalaCommander  master("can0", 1000000);
int running;

void payloadMenu(void){
    int func =0;
    
    float ff;
    int ii;
    cout << "PAYLOAD MODULE FUNCTIONS:\n 1 - probe height\n 2 - brush speed\n 3 - brush height\n 4 - light\n 5 - laser\n 6 - fan\n 7 - enable thermal"; 
    cin>>func;
    switch(func) {
            case 1 :cout << "insert probe height(float32)"; 
                    cin>>ff;
                    master.setSensorProbeHeight(ff);
                    break;     
            case 2 :cout << "insert brush speed (float32)"; 
                    cin>>ff;
                    master.setSensorBrushSpeed(ff);
                    break;      
            case 3 :cout << "insert brush height (float32)"; 
                    cin>>ff;
                    master.setSensorBrushHeight(ff);
                    break;   
            case 4 :cout << "insert light value (uint8)"; 
                    cin>>ii;
                    master.setSensorLight(ii);
                    break;         
            case 5 :cout << "insert laser value (uint8)"; 
                    cin>>ii;
                    master.setSensorLaser(ii);
                    break;
            case 6 :cout << "insert fan value (uint8)"; 
                    cin>>ii;
                    master.setSensorFan(ii);
                    break;
            case 7 :cout << "enable thermal data (uint8)"; 
                    cin>>ii;
                    master.setSensorThermalStream(ii);
                    break;
        }
}

void inputTask(void){
    char cmd_ch;
    int aux_i;
    bool ninteen_on = false;
    float aux_f;
    
    while(1){
        cin>>cmd_ch;
        sem.lock();
        switch(cmd_ch) {
            case 's' : cout << "insert servo position (float32)"; 
                        cin>> aux_f;
                        //sem.lock();
                        if(aux_f > 45.0) ninteen_on = true;
                        sp.left.pivot= aux_f;
                        sp.center.pivot = aux_f;
                        sp.right.pivot = aux_f;
                        //sem.unlock();
                     break;     
            case 'f' : cout << "insert force setpoint (float32)"; 
                        cin>>aux_f;
                        //sem.lock();
                        force_global_sp = aux_f;
                        //sem.unlock();
                     break;     
            case 'c' : cout << "insert clamp position setpoint (int32)"; 
                        cin>>aux_f;
                        //sem.lock();
                        sp.left.clampPosition = aux_f;
                        sp.right.clampPosition =  aux_f;
                        //sem.unlock();
                     break;   
            case 'v' :  cout << "insert wheel velocity setpoint (int32)"; 
                        cin>> aux_i;
                        //sem.lock();
                        if(ninteen_on){
                                sp.left.wheelVelocity =  -aux_i;
                                sp.center.wheelVelocity = aux_i;
                                sp.right.wheelVelocity = aux_i;   
                        }
                        else{
                                sp.left.wheelVelocity =  aux_i;
                                sp.center.wheelVelocity = aux_i;
                                sp.right.wheelVelocity = aux_i;
                        }
                        //sem.unlock();
                     break;   
            case 'l' :  cout << "insert led value (uint8)"; 
                        cin>>aux_i;
                        master.setLeds(aux_i,aux_i,aux_i);
                     break;         
            case 'o' :  master.offsetForceSensor();
                     break;
            case 'k' : cout << "insert force kp (float32)"; 
                        cin>>aux_f;
                        master.setGainLeft(aux_f);
                        master.setGainRight(aux_f);
                     break;  
            case 'd' : cout << "insert force deadzone (float32)"; 
                        cin>>aux_f;
                        master.setDeadzoneLeft(aux_f);
                        master.setDeadzoneRight(aux_f);
                     break;  
            case 'r' :  master.readControlParam();
                     break;
            case 'p' :  payloadMenu();
                     break;
            case 'x' :  running = 0;
                     break;
        }
        sem.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void printDialog(void){
    cout<<"Koala CAN commander: \n s - set servo\n l - set led\n o - offset force sensor \n f - set force setpoint \nc - set clamp position\n  v - MC3001 set vel \n k - set force kp\n d - set force deadzone\n r - read control params\n p - payload menu \n x - shutdown\n";
}

int main()
{   
    
//     ros::init(argc, argv,"ros_topic_publisher");
//     ros::NodeHandle nh;
    //Publish an integer on topic called number
//     ros::Publisher topic_pub = nh.advertise<std_msgs::Float32>("/crawler/force_center", 10); 
//     ros::Publisher setpoint_pub = nh.advertise<std_msgs::Float32>("/crawler/force_setpoint", 10); 
//     std_msgs::Float32 msg; 
//     std_msgs::Float32 msg_sp; 

    char dummy;
    float crawler_weight = 4200.0;
    memset(&fb, 0, sizeof(struct crawler_feedback));
    memset(&sp, 0, sizeof(struct crawler_setpoint));
    force_global_sp = -crawler_weight;
    float total_force;
    int numRPDOs =0;
    int expectedRPDOs=3;
    // force pid variables
    float _pid_e =0.00f;
    float _pid_e_s =0.00f;
    float _pid_e_max = 150.0;
    float _pid_e_int =0.00f;
    float _pid_kp = 0.1;
    float _pid_ki = 0.02;
    float _pid_dt = 100e-3;
    float _pid_u =0.00f;
    float clamp_max_range = (491520.0/360.0)*70.0; //70 deg

    
    master.can_init();

    cout<<"Koala CAN master: enter a char to start the nodes"<<endl;
    cin>>dummy;
    running = 1;
    
    master.clamp_homing_procedure();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    master.all_hardware_initialization(); //set nodes operational
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    master.offsetForceSensor();
    master.sendSYNC();
        
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));


   boost::thread consoleInput_th(inputTask);

    //master.readControlParam(); // deprecated because no more pid on nucleos

    while(running){ 
        /*SYNC can communication*/
        master.sendSYNC();
        /*read feedback PDOs*/
        numRPDOs = master.read_PDO_feedback();
        //cout<< "frame ricevuti: "<<numRPDOs<<endl; 
        //if(numRPDOs != 3*expectedRPDOs) { cout<< "AIUTO AIUTOO !! si e perso un frame\n"; };
        master.getFeedback(fb);
        sem.lock();
        master.printFeedback();
        
        /*compute control/planning action*/
        // msg.data = fb.center.force;
        // msg_sp.data = sp.left.force;
        // topic_pub.publish(msg);
        // setpoint_pub.publish(msg_sp);
        total_force = (((fb.left.force)+(fb.center.force)+(fb.right.force)-crawler_weight)/3.00);
        // _pid_e = force_global_sp -((fb.left.force+fb.center.force+fb.right.force-crawler_weight)/3.00);
        //         /* soglia */
        // if(_pid_e >= _pid_e_max) 
        //         _pid_e_s = _pid_e - _pid_e_max;
        // if(_pid_e <= -_pid_e_max) 
        //         _pid_e_s = _pid_e + _pid_e_max;
        // else if(fabs(_pid_e) < _pid_e_max) 
        //         _pid_e_s =0.0;


        // _pid_e_int+=_pid_e*_pid_dt;
        // if(_pid_e_int< 0.0) _pid_e_int =0.0;
        // else if(_pid_e_int> clamp_max_range/_pid_ki) _pid_e_int =clamp_max_range/_pid_ki;
        // _pid_u = _pid_kp*_pid_e + _pid_ki*_pid_e_int;
        // sp.left.clampPosition =_pid_u);
        // sp.right.clampPosition =_pid_u);
        

        /*send setpoint PDOS*/
       
        master.setSetpoint(sp);
        sem.unlock();
        master.write_PDO_setpoint();
        master.printSetpoint();
        master.printSensorSetpoint();
        printf("total force: %f \n",total_force);
        printDialog();


        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

    master.all_hardware_deinit();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    master.can_shutdown();
  
    return 0;
}



