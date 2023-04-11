#include "KoalaCommander.h"

int wd_alarm ;

void watch_dog_timer(int sig){
    wd_alarm = 1;
}

KoalaCommander::KoalaCommander(const char* device, int _baud){
    strcpy(candev,device);
    bitrate = _baud;

    memset(&frame, 0, sizeof(struct can_frame));
    memset(&Feedback, 0, sizeof(struct crawler_feedback));
    memset(&Setpoint, 0, sizeof(struct crawler_setpoint));
    memset(&SensorSetpoint, 0, sizeof(struct sensor_setpoint));

    //TODO initialize setpoint values to home value (specially for servos !!)
    left_clamp_offset_deg = 0.0f;
    right_clamp_offset_deg = 0.0f;
    clamp_deg_to_tick = CLAMP_DEG_2_TICK;
    wheel_rad_to_tick = WHEEL_RAD_TO_TICK;
    wheel_rads_to_ticks = WHEEL_RADS_TO_TICKS;

    signal(SIGALRM, watch_dog_timer);

}


int KoalaCommander::can_init(void){

    memset(&frame, 0, sizeof(struct can_frame));
    
    if(RESET_NETWORK){
        char buf[128]={0};
        sprintf(buf,"sudo ifconfig %s down",candev);
        system(buf);
        
        sprintf(buf,"sudo ip link set %s type can bitrate %d",candev,bitrate);
        system(buf);

        sprintf(buf,"sudo ifconfig %s up",candev);
        system(buf);

    }
 
    printf("This is a can receive demo,can0 with %d bps!\r\n",bitrate);

    /*Create socket*/
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)   return -1;
    
    /*Specify can0 device*/
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);
    if (ret < 0) return -1;

    // set non blocking readings
    flags = fcntl(s,F_GETFL,0);
    if(flags == -1) return -1;
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
    
    /*Bind the socket to can0*/
    addr.can_family = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) return -1;

    /* set frame buffer size */
    int  bufsize = 800 * 1024;
    setsockopt(s, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize)); 
    setsockopt(s, SOL_SOCKET, SO_SNDBUF, &bufsize, sizeof(bufsize)); 

    return 1;
}

void KoalaCommander::setFilter(uint16_t cobid ,uint16_t mask){
    /*Define receive filter rules,we can set more than one filter rule!*/
    struct can_filter rfilter[1];
    rfilter[0].can_id = cobid;//Standard frame id !
    rfilter[0].can_mask = mask;//CAN_SFF_MASK;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
}

 //NB ATTENZIONE ! funzione appezzottata !!!!! (spreca messaggi can nell attesa del suo)
uint32_t KoalaCommander::read_SDO(short node_id, short index, uint8_t subindex){ 

    // struct can_filter rfilter;
    // rfilter.can_id = 0x580+node_id;//Standard frame id !
    // rfilter.can_mask = CAN_SFF_MASK;//CAN_SFF_MASK;
    // setsockopt(s, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS, &rfilter, sizeof(rfilter));
    int maxwait = 10;
    /*assembly  message data! */
    frame.can_id = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x40;
    frame.data[1] = index;
    frame.data[2] = index>>8;
    frame.data[3] = subindex;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;
    
    /*Send message out */
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)){
         return -1; // trasmission error
    }
    uint32_t result =0;
    int c =0;
    alarm(1); // watchdog for waiting response
    do{
        nbytes = read(s, &frame, sizeof(frame));
        //printf("rx ID: %04x \n",frame.can_id);
        if(nbytes > 0){
            //printf("raw: %02x %02x %02x %02x %02x %02x %02x %02x  \n",frame.data[0] , frame.data[1] , frame.data[2] , frame.data[3], frame.data[4] , frame.data[5] , frame.data[6] , frame.data[7] );
            result =  (frame.data[4] | frame.data[5] << 8 | frame.data[6] << 16 | frame.data[7] << 24);
            c++;
        }
        
    }while((frame.can_id != 0x580+node_id) && (c <= maxwait) && !wd_alarm);
    if(wd_alarm) wd_alarm =0;
    if(frame.can_id == 0x580+node_id) return result;
    else return -1;
}

int KoalaCommander::write_SDO(short node_id, short index, short subindex, DataType dType, long data)
{   
    //struct can_frame frame2send;
  
    /*assembly  message data! */
    frame.can_id = 0x600 + node_id;
    frame.can_dlc = 8;
    frame.data[0] = CMD_sel[dType];//0x23;//0x2F;
    frame.data[1] = index;
    frame.data[2] = index>>8;
    frame.data[3] = subindex;
    frame.data[4] = data >> 0;
    frame.data[5] = data >> 8;
    frame.data[6] = data >> 16;
    frame.data[7] = data >> 24;

    /*Send message out */
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        return -1; // trasmission error
    }
    
    /// get response frame
    nbytes = read(s, &frame, sizeof(frame));
    //check response frame is OK (?)

    return 1;
}

void KoalaCommander::can_shutdown(void){
    /*Close can0 device and destroy socket!*/
    close(s);
    //system("sudo ifconfig can0 down");
    char buf[128]={0};
    sprintf(buf,"sudo ifconfig %s down",candev);
    system(buf);
}

/*
void KoalaCommander::read_PDO_blocking(){ //deprecated
	float force_set= 0.0;
	float force_fb = 0.0;

	while(1){
	    //cout<<"Koala CAN commander: \n s - set servo\n l - set led\n c - set camera \n f - set force setpoint \n g - get force value\n e - enable MC3001 \n d - disable MC3001 \n m - MC3001 set operation modde \n v - MC3001 set vel \n p - MC3001 set pos \n\n";
	    //cin>>cmd_ch;
        nbytes = read(s, &frame, sizeof(frame));
        if(nbytes > 0) {
            //if(!(frame.can_id&CAN_EFF_FLAG))

			if(frame.can_id == 0x182){ //NODE 2 (nucleo Left side) TPDO1 force fb
			   	uint32_t force_actual =	(frame.data[0] | frame.data[1] << 8 | frame.data[2] << 16 | frame.data[3] << 24);
			   	//uint32_t force_actual   =	(frame.data[4] | frame.data[5] << 8 | frame.data[6] << 16 | frame.data[7] << 24);
			   	//force_set = *((float*)&force_setpoint);
			   	Feedback.left.force = *((float*)&force_actual);
			   	printf("force control[L]: actual=%f \n",Feedback.left.force);
			}
            if(frame.can_id == 0x282){ //NODE 2 (nucleo Left side) TPDO2 tofs readings
                Feedback.left.range[0]  =   (int)(frame.data[0] | frame.data[1] << 8 );
                Feedback.left.range[1]  =   (int)(frame.data[2] | frame.data[3] << 8 );
                Feedback.left.range[2]  =   (int)(frame.data[4] | frame.data[5] << 8 );
                Feedback.left.range[3]  =   (int)(frame.data[6] | frame.data[7] << 8 );
               
                printf("ranging sensors[L]: [%d, %d, %d, %d]\n",Feedback.left.range[0] ,Feedback.left.range[1] ,Feedback.left.range[2] ,Feedback.left.range[3]);
            }

			if(frame.can_id == 0x184){ //NODE 4 (nucleo Center side) TPDO1 force fb
			   	uint32_t force_setpoint =	(frame.data[0] | frame.data[1] << 8 | frame.data[2] << 16 | frame.data[3] << 24);
			   	uint32_t force_actual   =	(frame.data[4] | frame.data[5] << 8 | frame.data[6] << 16 | frame.data[7] << 24);
			   	force_set = *((float*)&force_setpoint);
			   	force_feedback[1] = *((float*)&force_actual);
			   	printf("force control[C]: set=%f fb=%f \n",force_set,force_feedback[2]);
			}

			if(frame.can_id == 0x186){ //NODE 6 (nucleo Right side) TPDO1 force fb
			   	uint32_t force_setpoint =	(frame.data[0] | frame.data[1] << 8 | frame.data[2] << 16 | frame.data[3] << 24);
			   	uint32_t force_actual   =	(frame.data[4] | frame.data[5] << 8 | frame.data[6] << 16 | frame.data[7] << 24);
			   	force_set = *((float*)&force_setpoint);
			   	force_feedback[2] = *((float*)&force_actual);
			   	printf("force control[R]: set=%f fb=%f \n",force_set,force_feedback[3]);
			}
        }
        //yeild();
	}
}
*/

int KoalaCommander::read_PDO_feedback(void){
  
    int recived_count =0;
    do{
        nbytes = read(s, &frame, sizeof(frame));
        if(nbytes > 0) {
            //if(!(frame.can_id&CAN_EFF_FLAG))
        // LEFT //
            /*NUCLEO[L] id=2 */
            //NODE 2 (nucleo Left side) TPDO1 force fb
            if(frame.can_id == 0x182){ 
                uint32_t force_actual = (frame.data[0] | frame.data[1] << 8 | frame.data[2] << 16 | frame.data[3] << 24);
                Feedback.left.force= *((float*)&force_actual);
                recived_count+=1;
            }
            //NODE 2 (nucleo Left side) TPDO2 tofs readings
            if(frame.can_id == 0x282){ 
                Feedback.left.range[0]  =   (int)(frame.data[0] | frame.data[1] << 8 );
                Feedback.left.range[1]  =   (int)(frame.data[2] | frame.data[3] << 8 );
                Feedback.left.range[2]  =   (int)(frame.data[4] | frame.data[5] << 8 );
                Feedback.left.range[3]  =   (int)(frame.data[6] | frame.data[7] << 8 );
                recived_count+=1;           
            }
            /*MC3001_WHEEL[L] id=1 */
            //NODE 1 (MC3001 left side) TPDO4 pos_actual vel_actual
            if(frame.can_id == 0x481){ 
                uint32_t pos_actual = (frame.data[0] | frame.data[1] << 8 | frame.data[2] << 16 | frame.data[3] << 24);
                uint32_t vel_actual = (frame.data[4] | frame.data[5] << 8 | frame.data[6] << 16 | frame.data[7] << 24);
                Feedback.left.wheelPosition = (*((int32_t*)&pos_actual))/wheel_rad_to_tick;
                Feedback.left.wheelVelocity = (*((int32_t*)&vel_actual))/wheel_rads_to_ticks;
                recived_count++;
            }
            /*MC3001_CLAMP[L] id=7 */
            //NODE 8 (MC3001 left side) TPDO4 pos_actual vel_actual
            if(frame.can_id == 0x287){ 
                uint32_t statusworld = (frame.data[0] | frame.data[1] << 8);
                uint32_t pos_actual = (frame.data[2] | frame.data[3] << 8 | frame.data[4] << 16 | frame.data[5] << 24);
                Feedback.left.clampPosition = (*((int32_t*)&pos_actual))/clamp_deg_to_tick;
                recived_count++;
            }
        // CENTER //
            /*NUCLEO[C] id=4 */
            //NODE 4 (nucleo center side) TPDO1 force fb
            if(frame.can_id == 0x184){ 
                uint32_t force_actual = (frame.data[0] | frame.data[1] << 8 | frame.data[2] << 16 | frame.data[3] << 24);
                Feedback.center.force= *((float*)&force_actual);
                recived_count+=1;
            }
            //NODE 4 (nucleo center side) TPDO2 tofs readings
            if(frame.can_id == 0x284){ 
                Feedback.center.range[0]  =   (int)(frame.data[0] | frame.data[1] << 8 );
                Feedback.center.range[1]  =   (int)(frame.data[2] | frame.data[3] << 8 );
                Feedback.center.range[2]  =   (int)(frame.data[4] | frame.data[5] << 8 );
                Feedback.center.range[3]  =   (int)(frame.data[6] | frame.data[7] << 8 );
                recived_count+=1;           
            }
            /*MC3001[C] id=3 */
            //NODE 3 (MC3001 center side) TPDO4 pos_actual vel_actual
            if(frame.can_id == 0x483){ 
                uint32_t pos_actual = (frame.data[0] | frame.data[1] << 8 | frame.data[2] << 16 | frame.data[3] << 24);
                uint32_t vel_actual = (frame.data[4] | frame.data[5] << 8 | frame.data[6] << 16 | frame.data[7] << 24);
                Feedback.center.wheelPosition = (*((int32_t*)&pos_actual))/wheel_rad_to_tick;
                Feedback.center.wheelVelocity = (*((int32_t*)&vel_actual))/wheel_rads_to_ticks;
                recived_count++;
            }
        // RIGHT //
            /*NUCLEO[R] id=6 */
            //NODE 6 (nucleo Right side) TPDO1 force fb
            if(frame.can_id == 0x186){ 
                uint32_t force_actual = (frame.data[0] | frame.data[1] << 8 | frame.data[2] << 16 | frame.data[3] << 24);
                Feedback.right.force= *((float*)&force_actual);
                recived_count+=1;
            }
            //NODE 6 (nucleo Right side) TPDO2 tofs readings
            if(frame.can_id == 0x286){ 
                Feedback.right.range[0]  =   (int)(frame.data[0] | frame.data[1] << 8 );
                Feedback.right.range[1]  =   (int)(frame.data[2] | frame.data[3] << 8 );
                Feedback.right.range[2]  =   (int)(frame.data[4] | frame.data[5] << 8 );
                Feedback.right.range[3]  =   (int)(frame.data[6] | frame.data[7] << 8 );
                recived_count+=1;           
            }
            /*MC3001[R] id=5 */
            //NODE 5 (MC3001 Right side) TPDO4 pos_actual vel_actual
            if(frame.can_id == 0x485){ 
                uint32_t pos_actual = (frame.data[0] | frame.data[1] << 8 | frame.data[2] << 16 | frame.data[3] << 24);
                uint32_t vel_actual = (frame.data[4] | frame.data[5] << 8 | frame.data[6] << 16 | frame.data[7] << 24);
                Feedback.right.wheelPosition = (*((int32_t*)&pos_actual))/wheel_rad_to_tick;
                Feedback.right.wheelVelocity = (*((int32_t*)&vel_actual))/wheel_rads_to_ticks;
                recived_count++;
            }
            /*MC3001_CLAMP[L] id=8 */
            //NODE 8 (MC3001 left side) TPDO4 pos_actual vel_actual
            if(frame.can_id == 0x288){ 
                uint32_t statusworld = (frame.data[0] | frame.data[1] << 8);
                uint32_t pos_actual = (frame.data[2] | frame.data[3] << 8 | frame.data[4] << 16 | frame.data[5] << 24);
                Feedback.right.clampPosition = (*((int32_t*)&pos_actual))/clamp_deg_to_tick;
                recived_count++;
            }
        }

    }while(nbytes>0);
    return recived_count; 
}



int KoalaCommander::send_nucleo_PDO(uint8_t dest_node_id, float servo_set,float force_set){
    uint32_t data = 0;
	/*assembly  message data! */
    frame.can_id = 0x200+dest_node_id;
    frame.can_dlc = 8;
    data = *((uint32_t*)&servo_set); /*servo pos at 0x6000 01 32b*/
    frame.data[0] = data >> 0;
    frame.data[1] = data >> 8;
    frame.data[2] = data >> 16;
    frame.data[3] = data >> 24;
    data = *((uint32_t*)&force_set); /*force set at 0x6200 01 32b*/
    frame.data[4] = data >> 0;
    frame.data[5] = data >> 8;
    frame.data[6] = data >> 16;
    frame.data[7] = data >> 24;

    /*Send message out */
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        return -1; // trasmission error
    }
    return 1;
}

int KoalaCommander::send_MC3001_PDO_wheel(uint8_t dest_node_id, uint16_t controlword ,int32_t vel_set){
    uint32_t data = 0;
    /*assembly  message data! */
    frame.can_id = 0x400+dest_node_id;
    frame.can_dlc = 6;
    frame.data[0] = controlword >> 0;
    frame.data[1] = controlword >> 8;
    data = *((uint32_t*)&vel_set); /*force set at 0x6200 01 32b*/
    frame.data[2] = vel_set >> 0;
    frame.data[3] = vel_set >> 8;
    frame.data[4] = vel_set >> 16;
    frame.data[5] = vel_set >> 24;

    /*Send message out */
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        return -1; // trasmission error
    }
    return 1;
}

int KoalaCommander::send_MC3001_PDO_clamp(uint8_t dest_node_id, uint16_t controlword ,int32_t pos_set){
    uint32_t data = 0;
    /*assembly  message data! */
    frame.can_id = 0x300+dest_node_id;
    frame.can_dlc = 6;
    frame.data[0] = controlword >> 0;
    frame.data[1] = controlword >> 8;
    data = *((uint32_t*)&pos_set); /*force set at 0x6200 01 32b*/
    frame.data[2] = pos_set >> 0;
    frame.data[3] = pos_set >> 8;
    frame.data[4] = pos_set >> 16;
    frame.data[5] = pos_set >> 24;

    /*Send message out */
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        return -1; // trasmission error
    }
    return 1;
}

void KoalaCommander::write_PDO_setpoint(void){
    send_nucleo_PDO(2, Setpoint.left.pivot, 0);  //NUCLEO[L] id 2
    send_nucleo_PDO(4, Setpoint.center.pivot, 0);                  //NUCLEO[C] id 4
    send_nucleo_PDO(6, Setpoint.right.pivot, 0);//NUCLEO[R] id 6

    send_MC3001_PDO_wheel(1, 0x000F ,(int)(Setpoint.left.wheelVelocity*wheel_rads_to_ticks));    //MC3001[L] id 1
    send_MC3001_PDO_wheel(3, 0x000F ,-(int)(Setpoint.center.wheelVelocity*wheel_rads_to_ticks));  //MC3001[C] id 3
    send_MC3001_PDO_wheel(5, 0x000F ,-(int)(Setpoint.right.wheelVelocity*wheel_rads_to_ticks));   //MC3001[R] id 5

    send_MC3001_PDO_clamp(7, 0x000F ,(int)(Setpoint.left.clampPosition*clamp_deg_to_tick));   //MC3001[clamp L] id 8send_MC3001_PDO
    send_MC3001_PDO_clamp(8, 0x000F ,(int)(Setpoint.right.clampPosition*clamp_deg_to_tick));   //MC3001[clamp R] id 8send_MC3001_PDO

    Feedback.left.pivot = Setpoint.left.pivot;  //update last setpoint in feedback for reading current servo val if needed
    Feedback.center.pivot = Setpoint.center.pivot;
    Feedback.right.pivot = Setpoint.right.pivot;
}

int KoalaCommander::setNodeOperational(uint8_t node_id){
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x01;
    frame.data[1] = node_id;
    /*Send message out */
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        return -1; // trasmission error
    }
    return 1;
}

int KoalaCommander::setNodePreOperational(uint8_t node_id){
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x02;
    frame.data[1] = node_id;
    /*Send message out */
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        return -1; // trasmission error
    }
    return 1;  
}

int KoalaCommander::sendSYNC(void){
    frame.can_id = 0x080;
    frame.can_dlc = 0;
    /*Send message out */
    nbytes = write(s, &frame, sizeof(frame)); 
    if(nbytes != sizeof(frame)) {
        return -1; // trasmission error
    }
    return 1;
}

int KoalaCommander::enableWheel(uint8_t node_id){
    if(write_SDO(node_id ,0x6040 , 0, U16, 0x000E) < 0) return -1;
    if(write_SDO(node_id ,0x6040 , 0, U16, 0x000F) < 0) return -1;
    return 1;             
}

int KoalaCommander::disableWheel(uint8_t node_id){
    return write_SDO(node_id, 0x6040 , 0, U16, 0x000D);
}

int KoalaCommander::setWheelOperatingMode(uint8_t node_id, int8_t opmode){
   return  write_SDO(node_id, 0x6060 , 0, U8, *((uint32_t*)&opmode)); 
}

void KoalaCommander::setLeds(uint8_t led_L,uint8_t led_C,uint8_t led_R){
    write_SDO(2 ,0x6002 , 1, U8, led_L); 
    write_SDO(4 ,0x6002 , 1, U8, led_C); 
    write_SDO(6 ,0x6002 , 1, U8, led_R); 
    Feedback.left.led = led_L;
    Feedback.center.led = led_C;
    Feedback.right.led = led_R;
}

void KoalaCommander::offsetForceSensor(void){  
    write_SDO(2 ,0x6003 , 1, U8, 1); 
    write_SDO(4 ,0x6003 , 1, U8, 1); 
    write_SDO(6 ,0x6003 , 1, U8, 1); 
}

void KoalaCommander::readControlParam(void){

    uint32_t dummy = read_SDO(2,0x6202,1); //force deadzone NUCLEO[L]
    Feedback.left.deadzone = *((float*)&dummy);
    dummy = read_SDO(2,0x6202,2); //force kp NUCLEO[L]
    Feedback.left.force_kp = *((float*)&dummy);
    dummy = read_SDO(6,0x6202,1); //force deadzone NUCLEO[R]
    Feedback.right.deadzone = *((float*)&dummy);
    dummy = read_SDO(6,0x6202,2); //force kp NUCLEO[R]
    Feedback.right.force_kp = *((float*)&dummy);

}
void KoalaCommander::setDeadzoneLeft(float d){   
    uint32_t data = 0;
    data = *((uint32_t*)&d);
    write_SDO(2 ,0x6202 , 1, U32, data);  //force deadzone NUCLEO[L]
    Feedback.left.deadzone =d;
}
void KoalaCommander::setGainLeft(float kp){
    uint32_t data = 0;
    data = *((uint32_t*)&kp);
    write_SDO(2 ,0x6202 , 2, U32, data);  //force kp NUCLEO[L]
    Feedback.left.force_kp = kp;
}

void KoalaCommander::setDeadzoneRight(float d){   
    uint32_t data = 0;
    data = *((uint32_t*)&d);
    write_SDO(6 ,0x6202 , 1, U32, data);  //force deadzone NUCLEO[L]
    Feedback.right.deadzone = d;
}
void KoalaCommander::setGainRight(float kp){
    uint32_t data = 0;
    data = *((uint32_t*)&kp);
    write_SDO(6 ,0x6202 , 2, U32, data);  //force kp NUCLEO[L]
    Feedback.right.force_kp =kp;
}

// PAYLOAD MODULE 
void KoalaCommander::setSensorProbeHeight(float h){
    uint32_t data = 0;
    data = *((uint32_t*)&h);
    write_SDO(9 ,0x6202 , 1, U32, data);  //probe hgt NUCLEO[P] CAN_ID 7
    SensorSetpoint.probe_height = h;
}
void KoalaCommander::setSensorBrushHeight(float h){
    uint32_t data = 0;
    data = *((uint32_t*)&h);
    write_SDO(9 ,0x6202 , 2, U32, data);  //brush hgt NUCLEO[P] CAN_ID 7
    SensorSetpoint.brush_height = h;
}
void KoalaCommander::setSensorBrushSpeed(float h){
    uint32_t data = 0;
    data = *((uint32_t*)&h);
    write_SDO(9 ,0x6202 , 3, U32, data);  //brush spd NUCLEO[P] CAN_ID 7
    SensorSetpoint.brush_speed = h;
}
void KoalaCommander::setSensorLight(uint8_t h){
    write_SDO(9 ,0x6207 , 1, U8, h);  //Light NUCLEO[P] CAN_ID 7
    SensorSetpoint.light = h;
}
void KoalaCommander::setSensorLaser(uint8_t h){
     write_SDO(9 ,0x6207 , 2, U8, h);  //Light NUCLEO[P] CAN_ID 7
    SensorSetpoint.laser = h;
}
void KoalaCommander::setSensorFan(uint8_t h){
     write_SDO(9 ,0x6207 , 3, U8, h);  //Light NUCLEO[P] CAN_ID 7
    SensorSetpoint.fan = h;
}
void KoalaCommander::setSensorThermalStream(uint8_t on){
    if(on) on =1;
    else on =0;
    write_SDO(9 ,0x6207 , 4, U8, on);  //Light NUCLEO[P] CAN_ID 7
    SensorSetpoint.enable_thermal_stream = on;
}

int KoalaCommander::clamp_homing_procedure(void){

    uint8_t node_id_l = 7;
    uint8_t node_id_r = 8;
    // int8_t opmode = 6; // homing mode
    uint32_t status_r  = 0;
    uint32_t status_l  = 0;
    bool homing_complete = false;

   
    //set homing mode
     printf("set homing mode\n");
    if(write_SDO(node_id_r, 0x6060 , 0, U8, 6) < 0) return -1; //set operating mode homing(6)
    if(write_SDO(node_id_l, 0x6060 , 0, U8, 6) < 0) return -1; //set operating mode homing(6)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(write_SDO(node_id_r ,0x6098 , 0, U8, 2) < 0) return -1; //homing method =2
    if(write_SDO(node_id_l ,0x6098 , 0, U8, 2) < 0) return -1; //homing method =2
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    //enable driver stage and run homing
    printf("enable driver stage\n");
    if(write_SDO(node_id_r ,0x6040 , 0, U16, 14) < 0) return -1; //write controlworld
    if(write_SDO(node_id_l ,0x6040 , 0, U16, 14) < 0) return -1; //write controlworld
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(write_SDO(node_id_r ,0x6040 , 0, U16, 15) < 0) return -1;
    if(write_SDO(node_id_l ,0x6040 , 0, U16, 15) < 0) return -1;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if(write_SDO(node_id_r ,0x6040 , 0, U16, 31) < 0) return -1; // run homing
    if(write_SDO(node_id_l ,0x6040 , 0, U16, 31) < 0) return -1; // run homing
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    //wait for homing completion
    while(!homing_complete){
        status_r = read_SDO(node_id_r, 0x6041, 0);
        status_l = read_SDO(node_id_l, 0x6041, 0);
        homing_complete = (status_l == 0x1427) && (status_r == 0x1427);
        //homing_complete = (status_l == 0x1427);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        printf("waiting for clamp homing completion...\n");
        printf("statusworld R =  0x%x \n",status_r);
        printf("statusworld L =  0x%x \n",status_l);
    }

    printf("homing completed !\n");
    // ste operating mode position CSP
    if(write_SDO(node_id_r, 0x6060 , 0, U8, 8) < 0) return -1; //set operating mode CSP(8)
    if(write_SDO(node_id_l, 0x6060 , 0, U8, 8) < 0) return -1; //set operating mode CSP(8)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //set node operational
    setNodeOperational(node_id_r); 
    setNodeOperational(node_id_l); 
}

void KoalaCommander::all_hardware_initialization(void){
    // initialize NUCLEO modules
    setNodeOperational(2); //start NUCLEO[L]
    setNodeOperational(4); //start NUCLEO[C]
    setNodeOperational(6); //start NUCLEO[R]

    //initialize wheels driver MC3001
    setWheelOperatingMode(1, 9); 
    setWheelOperatingMode(3, 9);
    setWheelOperatingMode(5, 9);
    enableWheel(1);
    enableWheel(3);
    enableWheel(5);
    setNodeOperational(1); //start MC3001_wheel[L]
    setNodeOperational(3); //start MC3001_wheel[C]
    setNodeOperational(5); //start MC3001_wheel[R]

    //initialize clamps driver MC3001
    //setNodeOperational(7); //start MC3001_wheel[R]
    //setNodeOperational(8); //start MC3001_wheel[L]
}

void KoalaCommander::all_hardware_deinit(void){
    setNodePreOperational(2); //stop NUCLEO[L]
    setNodePreOperational(4); //stop NUCLEO[C]
    setNodePreOperational(6); //stop NUCLEO[R]
    setNodePreOperational(7); //stop NUCLEO[P]

    disableWheel(1);
    disableWheel(3);
    disableWheel(5);  
    setNodePreOperational(1); //stop MC3001_wheel[L]
    setNodePreOperational(3); //stop MC3001_wheel[C]
    setNodePreOperational(5); //stop MC3001_wheel[R]

    disableWheel(7);
    disableWheel(8);  
    setNodePreOperational(7); //stop MC3001_wheel[L]
    setNodePreOperational(8); //stop MC3001_wheel[C]
}


void KoalaCommander::printFeedback(void){  
    printf("---------------------------------------- KOALA CRAWLER FEEDBACK ----------------------------------------------------\n");
    printf("NUCLEO[L] force= %6.05f             \t", Feedback.left.force); 
    printf("NUCLEO[C] force= %6.05f             \t", Feedback.center.force); 
    printf("NUCLEO[R] force= %6.05f             \n", Feedback.right.force);

    printf("NUCLEO[L] pivot= %6.02f             \t", Feedback.left.pivot); 
    printf("NUCLEO[C] pivot= %6.02f             \t", Feedback.center.pivot); 
    printf("NUCLEO[R] pivot= %6.02f             \n", Feedback.right.pivot);

    printf("NUCLEO[L] led= %3u                  \t", Feedback.left.led); 
    printf("NUCLEO[C] led= %3u                  \t", Feedback.center.led);
    printf("NUCLEO[R] led= %3u                  \n", Feedback.right.led);

    printf("NUCLEO[L] clampPos= %6f    \t", Feedback.left.clampPosition); 
    printf("NUCLEO[C]                                               \t");
    printf("NUCLEO[R] clampPos= %6f    \n", Feedback.right.clampPosition);

    printf("NUCLEO[L] range=[%3d, %3d, %3d, %3d]\t",Feedback.left.range[0] ,Feedback.left.range[1] ,Feedback.left.range[2] ,Feedback.left.range[3]);
    printf("NUCLEO[C] range=[%3d, %3d, %3d, %3d]\t",Feedback.center.range[0] ,Feedback.center.range[1] ,Feedback.center.range[2] ,Feedback.center.range[3]);
    printf("NUCLEO[R] range=[%3d, %3d, %3d, %3d]\n",Feedback.right.range[0] ,Feedback.right.range[1] ,Feedback.right.range[2] ,Feedback.right.range[3]);

    printf("MC3001[L]: pos=%6d vel=%6d\t",Feedback.left.wheelPosition,Feedback.left.wheelVelocity); 
    printf("MC3001[C]: pos=%6d vel=%6d\t",Feedback.center.wheelPosition,Feedback.center.wheelVelocity); 
    printf("MC3001[R]: pos=%6d vel=%6d   \n",Feedback.right.wheelPosition,Feedback.right.wheelVelocity); 
}

void KoalaCommander::printSetpoint(void){  
    printf("---------------------------------------- KOALA CRAWLER SETPOINT ----------------------------------------------------\n");
    printf("NUCLEO[L] clampPos= %6.02f             \t", Setpoint.left.clampPosition); 
    printf("NUCLEO[C]                           \t"); 
    printf("NUCLEO[R] clampPos= %6.02f \n", Setpoint.right.clampPosition);

    printf("NUCLEO[L] pivot= %6.02f             \t", Setpoint.left.pivot); 
    printf("NUCLEO[C] pivot= %6.02f             \t", Setpoint.center.pivot); 
    printf("NUCLEO[R] pivot= %6.02f \n", Setpoint.right.pivot);

    printf("MC3001[L] wheelVel= %6.02f             \t", Setpoint.left.wheelVelocity); 
    printf("MC3001[C] wheelVel= %6.02f             \t", Setpoint.center.wheelVelocity); 
    printf("MC3001[R] wheelVel= %6.02f \n", Setpoint.right.wheelVelocity);
}

void KoalaCommander::printSensorSetpoint(void){  
    printf("---------------------------------------- KOALA PAYLOAD SETPOINT ----------------------------------------------------\n");
    printf("NUCLEO[P] probe_hgt= %6.02f            \t", SensorSetpoint.probe_height); 
    printf("brush_hgt= %6.02f                      \t",SensorSetpoint.brush_height); 
    printf("brush_spd= %6.02f \n", SensorSetpoint.brush_speed);
    printf("NUCLEO[P] light= %u                    \t", SensorSetpoint.light); 
    printf("laser= %u                             \t", SensorSetpoint.laser); 
    printf("fan= %u   \n", SensorSetpoint.fan);
    printf("NUCLEO[P] enable_thermal_stream= %u             \n", SensorSetpoint.enable_thermal_stream); 
   
}