#include <chrono>
#include <thread>
#include <iostream>

#include "KoalaCommander.h"

using namespace std;


int main()
{   
    
    char dummy;

    KoalaCommander  master("can0", 1000000);
    master.can_init();

    cout<<"Koala CAN master: enter a char to start the nodes"<<endl;
    cin>>dummy;
    master.all_hardware_initialization();
    master.sendSYNC();
        
    cout<<"Koala CAN master: enter a char to start reading continuously"<<endl;
    cin>>dummy;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    int numRPDOs =0;
    int expectedRPDOs=2;

    struct crawler_feedback fb;
    struct crawler_setpoint sp;
    memset(&fb, 0, sizeof(struct crawler_feedback));
    memset(&sp, 0, sizeof(struct crawler_setpoint));

    while(1){ 
        /*SYNC can communication*/
        master.sendSYNC();

        /*read feedback PDOs*/
        numRPDOs = master.read_PDO_feedback();
        if(numRPDOs != expectedRPDOs) { cout<< "AIUTO AIUTOO !! si e perso un frame\n"; };
        master.getFeedback(fb);
        master.printFeedback();
        //cout<<"force[L]= "<<fb.left.force<<endl;

        /*compute control/planning action*/
        //sp = f(fb,u)
        
        /*send setpoint PDOS*/
        //master.setSetpoint(sp);
        //master.write_PDO_setpoint();
        //master.printSetpoint();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }

    master.all_hardware_deinit();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    master.can_shutdown();
  
    return 0;
}
