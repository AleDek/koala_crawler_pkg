#ifndef _KOALA_COMMANDER
#define _KOALA_COMMANDER

#include <chrono>
#include <thread>
#include <iostream>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/fcntl.h>
#include <sys/poll.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <signal.h>

#define RESET_NETWORK 1


#define CLAMP_DEG_2_TICK (491520.0f/360.0f)    //16384*30 = 491520 consider gearbox reduction
#define WHEEL_RAD_TO_TICK (1.0f) //(16384.0f/(2.0f*M_PI)) //TODO check and set gb ratio in MC3001 config
#define WHEEL_RADS_TO_TICKS (1.0f)//TODO verificare 

using namespace std;

enum DataType {U8, U16, U32};



struct crawler_feedback{
    struct left{
        float pivot; //servo pivot in degrees
        float force; //measured force in grams
        int32_t wheelVelocity;
        int32_t wheelPosition;
        float clampPosition;  //clamp position in degrees
        int range[4];  //[rear_low front_low rear_high front_hign]
        uint8_t led;
        float deadzone; //deprecated
        float force_kp;  //deprecated
    } left;
    struct center{
        float pivot; //servo pivot in degrees
        float force; //measured force in grams
        int32_t wheelVelocity;
        int32_t wheelPosition;
        int range[4]; //[rear_low front_low rear_high front_hign]
        uint8_t led;
    } center;
    struct right{
        float pivot; //servo pivot in degrees
        float force; //measured force in grams
        int32_t wheelVelocity;
        int32_t wheelPosition;
        float clampPosition;  //clamp position in degrees
        int range[4]; //[rear_low front_low rear_high front_hign]
        uint8_t led;
        float deadzone; //deprecated
        float force_kp; //deprecated
    } right;

};

struct crawler_setpoint{
    struct left{
        float clampPosition; //clamp position in degrees
        float pivot;
        float wheelVelocity;
    } left;
    struct center{
        float pivot;
        float wheelVelocity;
    } center;
    struct right{
        float clampPosition; //clamp position in degrees
        float pivot;
        float wheelVelocity;
    } right;

};

struct sensor_setpoint{
    float probe_height;
    float brush_speed;
    float brush_height; // may be unnecessary
    uint8_t light;
    uint8_t laser;
    uint8_t fan;
    uint8_t enable_thermal_stream;
};


void watch_dog_timer(int sig);

class KoalaCommander {

private :

    //SDO commands bits for first byte
    #define CMD_U8  0b00101111  //0x2F
    #define CMD_U16 0b00101011   //2B
    #define CMD_U32 0b00100011  //0x23
    
    uint8_t CMD_sel[3] = {CMD_U8, CMD_U16, CMD_U32};
    
    /*Special address description flags for CAN_ID*/
    #define CAN_EFF_FLAG 0x80000000U
    #define CAN_RTR_FLAG 0x40000000U
    #define CAN_ERR_FLAG 0x20000000U
    /* can device config var */
    char candev[6];
    int bitrate;

    //socketcan vars
    int ret;
    int s, nbytes,i;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    int flags;


    struct crawler_feedback Feedback;
    struct crawler_setpoint Setpoint;
    struct sensor_setpoint  SensorSetpoint;
    float left_clamp_offset_deg;
    float right_clamp_offset_deg;
    float clamp_deg_to_tick;
    float wheel_rad_to_tick;
    float wheel_rads_to_ticks;



    void setFilter(uint16_t cobid ,uint16_t mask);
    int setNodeOperational(uint8_t node_id);
    int setNodePreOperational(uint8_t node_id);
    
    uint32_t read_SDO(short node_id, short index, uint8_t subindex);
    int write_SDO(short node_id, short index, short subindex, DataType dType, long data); //NB ATTENZIONE ! funzione appezzottata !!!!! (spreca messaggi can nell attesa del suo)

    //void read_PDO_blocking(void); //while blocking loop, deprecated because of the non-blocking reading
       
    //nucleoF3 functions and setpoint ids [2 4 6]
    int send_nucleo_PDO(uint8_t dest_node_id, float servo_set,float force_set);
    void setLed(uint8_t node_id);
    void setCam(uint8_t node_id);
    
    //MC3001 functions and setpoint ids [1 3 5]
    int enableWheel(uint8_t node_id);
    int disableWheel(uint8_t node_id);
    int setWheelOperatingMode(uint8_t node_id, int8_t opmode); //useful opmode: 1=pos 3=vel 8=CSP 9=CSV (int8)"; 
    int send_MC3001_PDO_wheel(uint8_t dest_node_id, uint16_t controlword ,int32_t vel_set); //TODO rename to velocity
    int send_MC3001_PDO_clamp(uint8_t dest_node_id, uint16_t controlword ,int32_t pos_set);
    

public : /// api functions
    KoalaCommander(const char* device, int _baud);

    int can_init(void);
    void can_shutdown(void);

    void all_hardware_initialization(void);
    void all_hardware_deinit(void);
    int clamp_homing_procedure(void);

    int sendSYNC(void);

    int read_PDO_feedback(void); //flush all recieved pdos and fill feedback data NB read while there are no more frame to read
    void write_PDO_setpoint(void);

    void setLeds(uint8_t led_L,uint8_t led_C,uint8_t led_R);
    void offsetForceSensor(void);

    void readControlParam(void); // ATTENZIONE USARE IL MENO POSSIBILE
    void setGainLeft(float kp);
    void setGainRight(float kp);
    void setDeadzoneLeft(float d);
    void setDeadzoneRight(float d);

    void setSensorProbeHeight(float h);
    void setSensorBrushHeight(float h);
    void setSensorBrushSpeed(float h);
    void setSensorLight(uint8_t h);
    void setSensorLaser(uint8_t h);
    void setSensorFan(uint8_t h);
    void setSensorThermalStream(uint8_t on);

    void getFeedback( struct crawler_feedback &fb){ fb = Feedback; }
    void setSetpoint( struct crawler_setpoint &set){ Setpoint = set; }

    void printFeedback(void);
    void printSetpoint(void);
    void printSensorSetpoint(void);

    

}; //end class

#endif