
#include <chrono>
#include <thread>
#include <iostream>
#include <boost/thread.hpp>

#include "KoalaCommander.h"

using namespace std;

struct crawler_feedback fb;
struct crawler_setpoint sp;

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
                    break;     
            case 2 :cout << "insert brush speed (float32)"; 
                    cin>>ff;
                    break;      
            case 3 :cout << "insert brush height (float32)"; 
                    cin>>ff;
                    break;   
            case 4 :cout << "insert light value (uint8)"; 
                    cin>>ii;
                    break;         
            case 5 :cout << "insert laser value (uint8)"; 
                    cin>>ii;
                    break;
            case 6 :cout << "insert fan value (uint8)"; 
                    cin>>ii;
                    break;
            case 7 :cout << "enable thermal data (uint8)"; 
                    cin>>ii;
                    break;
        }
}

void inputTask(void){
    char cmd_ch;
    int aux_i;
    float aux_f;
    
    while(1){
        cin>>cmd_ch;
        sem.lock();
        switch(cmd_ch) {
            case 's' : cout << "insert servo position (float32)"; 
                        cin>> aux_f;
                        //sem.lock();
                        sp.left.pivot= aux_f;
                        sp.center.pivot = aux_f;
                        sp.right.pivot = aux_f;
                        //sem.unlock();
                     break;     
        //     case 'f' : cout << "insert force setpoint (float32)"; 
        //                 cin>>aux_f;
        //                 //sem.lock();
        //                 sp.left.force = aux_f;
        //                 sp.right.force = aux_f;
        //                 //sem.unlock();
        //              break;      
            case 'v' :  cout << "insert velocity setpoint (int32)"; 
                        cin>> aux_i;
                        //sem.lock();
                        sp.left.wheelVelocity =  aux_i;
                        sp.center.wheelVelocity = aux_i;
                        sp.right.wheelVelocity = aux_i;
                        //sem.unlock();
                     break;   
            case 'l' :  cout << "insert led value (uint8)"; 
                        cin>>aux_i;
                        master.setLeds(aux_i,aux_i,aux_i);
                     break;         
            case 'o' :  master.offsetForceSensor();
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
    cout<<"Koala CAN commander: \n s - set servo\n l - set led\n o - offset force sensor \n f - set force setpoint \n v - MC3001 set vel \n p - payload menu \n x - shutdown\n";
}

int main()
{   
    
    char dummy;
    memset(&fb, 0, sizeof(struct crawler_feedback));
    memset(&sp, 0, sizeof(struct crawler_setpoint));
    int numRPDOs =0;
    int expectedRPDOs=3;

    
    master.can_init();

    cout<<"Koala CAN master: enter a char to start the nodes"<<endl;
    cin>>dummy;
    running = 1;
    master.all_hardware_initialization();
    //master.sendSYNC();
        
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    //boost::thread consoleInput_th(inputTask);
     master.readControlParam();
     master.printFeedback();

    while(running){ 
        /*SYNC can communication*/
        //master.sendSYNC();
        /*read feedback PDOs*/
       


        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

    master.all_hardware_deinit();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    master.can_shutdown();
  
    return 0;
}



