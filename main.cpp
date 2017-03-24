#include "mbed.h"
#include "rtos.h"
#include <stdio.h>
#include <RawSerial.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include <string>

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

#define testpin D1
//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
volatile int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

DigitalOut tt(testpin);

InterruptIn CA(CHA);
InterruptIn CB(CHB);
//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);
//Motor Drive outputs
PwmOut L1L(L1Lpin);
PwmOut L2L(L2Lpin);
PwmOut L3L(L3Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3H(L3Hpin);
//Set a given drive state

RawSerial pc(USBTX, USBRX); 
Timer test;
Thread thread;
Timer t;
Timer T;
volatile int revolution_counter=0;
volatile int CA_counter=0;
volatile int synchronise_counter=0;

volatile float time_CA=100.0f;
volatile float time_R=100.0f;
volatile float velocity=5.5;
volatile float duty_cycle=1.0;
volatile int revolution_target = 100;
volatile float velocity_target= 100;
volatile float totaltime=0;

volatile float m_period;

const float K1=0.25; //revolutions PD CONTROLLER parameter:K_proportional
const float K2=-0.013;//revolutions PD CONTROLLER parameter:K_differential
const float K3=0.3; //speed P CONTROLLER parameter:K_proportional


volatile int8_t intState = 0;
int8_t orState = 0;    //Rotot offset at motor state 0

//-----------------------------------------------------------------------------
typedef enum {
    state0 = 0,
    state1,
    state2,
    state3,
    state4,
} STATE;

char ch;
int mode=0; //0 stand for none, 1 stand for R only, 2 stand for V only, 3 stand for RV
double R,V;
int R_int,V_int;
double R_dec,V_dec;
double intR,decR,intV,decV;
int R_sign =1;
int V_sign =1;

//---------------------------------------------------------------------------//


void motorOut(int8_t driveState)
{
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    //Then turn on
    if (driveOut & 0x01)  L1L.write(duty_cycle);
    if (driveOut & 0x04)  L2L.write(duty_cycle);
    if (driveOut & 0x10)  L3L.write(duty_cycle);
    if (driveOut & 0x02)  L1H=0;
    if (driveOut & 0x08)  L2H=0;
    if (driveOut & 0x20)  L3H=0;
}
//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState()
{
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine
int8_t motorHome()
{
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.5);
    //Get the rotor state
    
    return readRotorState();
}


//update motor state
void update_motor_state()
{ 
    intState = readRotorState();
    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
   
}



void synchronise_cha()
{
    T.stop();
    if(revolution_counter==0) {
        synchronise_counter=CA_counter;
        time_R=100.0f;
    }
    if(revolution_counter>0)  {
        CA_counter=revolution_counter*117+synchronise_counter;    //profiling: I1 rising and next CHA rising;
        time_R=T.read();
    }
    revolution_counter++;
    //actual CHA_revolution=revolution_counter*117+synchronise_counter
    update_motor_state();
    T.reset();
    T.start();
}

void controller_update()  //from interrupt
{   
    t.stop();
    CA_counter++;
    time_CA=t.read(); //-------------------------------------------------change
    thread.signal_set(0x1);
    t.reset();
    t.start();
}

void speed_controller(float v)
{
    if(v==0.0f) {
        duty_cycle=0.0f;
        update_motor_state();
    }
    
    if( 0<v<=4.0f)  { //------------------------on/off control for slow speed;
        if(velocity<v) {
            duty_cycle=1.0f;
        } else {
            duty_cycle=0.0f;
        }
        update_motor_state();
    }


    if(v>4.0f) {
        if(CA_counter==revolution_counter*117+synchronise_counter+1 ) {
            duty_cycle=K3*(v-1.0f/time_R)+0.08f;
            update_motor_state();
        }
    }

}


void controller()
{   
    while(1) {
        Thread::signal_wait(0x1);
        I1.disable_irq();
        I2.disable_irq();
        I3.disable_irq();

        velocity=1.0f/117/time_CA;

        if(mode==1) {
            
            velocity_target=K1*((float)R-CA_counter/117.0f)+K2/(time_CA*117);
            if((float)R*117.0f-CA_counter<234.0f && (float)R*117.0f-CA_counter>0.0f){velocity_target=1.0f;}
            if((float)R<CA_counter/117.0f){velocity_target=0.0f;
            }
            if (velocity_target<0.0f){velocity_target=0.0f;}            
            speed_controller(velocity_target);
            
            }


        if(mode==2) {
            
            speed_controller((float)V);
            
        }

        if(mode==3) {
           velocity_target=K1*((float)R-CA_counter/117.0f)+K2/(time_CA*117);
            if((float)R*117.0f-CA_counter<234.0f && (float)R*117.0f-CA_counter>0.0f){velocity_target=1.0f;}
            if((float)R<CA_counter/117.0f){velocity_target=0.0f;}
            if (velocity_target<0.0f){velocity_target=0.0f;}            
            
            if(velocity_target>(float)V){velocity_target=(float)V;}
            speed_controller(velocity_target);
            
        }

        thread.signal_clr(0x1);
        I1.enable_irq();
        I2.enable_irq();
        I3.enable_irq();
    }

}

//input check-----------------------------------------------------------------//

//---------------------------------------------------------------------------//

string Notes[16];
int Seconds[16];
int function=1;//--------------------------------------------which function

int index=-1;
char ch2;

void GetSTR(){
    //pc.printf("please enter notes command\n\r");
    do{

        ch2=pc.getc();
        pc.putc(ch2);
        
        if(ch2>='A' && ch2<='G'){
            index++;
            Notes[index]=ch2;
        }

        if(ch2=='#' || ch2=='^'){
            Notes[index]=Notes[index]+ch2;
        }

        if(isdigit(ch2)){
            Seconds[index]=int(ch2-48);
        }


    }while(ch2!='\r' && index<=15);

    pc.printf("funtion: Melody\n\r");
    
}


int GetDigits(int n)
{
    int count=0;
    while(n!=0) {
        count++;
        n/=10;
    }
    return count;
}
//---------------------------------------------------------------------------//
void getInput()
{
    R=0;
    V=0;
    R_int=0;
    V_int=0;
    R_dec=0;
    V_dec=0;
    intR=0;
    decR=0;
    intV=0;
    decV=0;
    R_sign =1;
    V_sign =1;
    

    pc.printf("please enter command\n\r");
    STATE state = state0;
    do {
        ch=pc.getc();
        pc.putc(ch);
        //----------------------------------------------------------------------
        if(ch=='T'){
            function=2;
            GetSTR();
            break;
        }
        //----------------------------------------------------------------------
        //pc.printf("next char\n\r");
        switch(state) {
            case state0:        //zero state
                if(ch=='R') {
                    state=state1;
                }
                if(ch=='V') {
                    state=state3;
                }
                break;
            case state1:        //start with R, before "."
                if(ch=='V') {
                    state=state3;
                }
                if(ch=='.') {
                    state=state2;
                }
                if(ch=='-') {
                    state=state1;
                    R_sign=-1;
                }
                if(isdigit(ch)) {
                    state=state1;
                    intR++;
                    R_int=R_int + int(ch-48) * pow(10.0,(intR-1.0));    //define R integer part,with opposite direction
                }
                break;
            case state2:
                decR++;
                if(ch=='V') {
                    state=state3;
                }
                if(isdigit(ch)) {
                    state=state2;
                    R_dec=R_dec + (double)int(ch-48) * pow(0.1,decR);   //define R decimal part
                }
                break;

            case state3:
                if(ch=='.') {
                    state=state4;
                }
                if(ch=='-') {
                    state=state3;
                    V_sign=-1;
                }
                if(isdigit(ch)) {
                    state=state3;
                    intV++;
                    V_int=V_int + int(ch-48) * pow(10.0,(intV-1.0));    //define V integer part
                }
                break;
            case state4:
                decV++;
                if(isdigit(ch)) {
                    state=state4;
                    V_dec=V_dec + (double)int(ch-48) * pow(0.1,decV);//define V decimal part
                }
                break;

            default:
                state=state0;
                break;
        }
    }  while(ch != '\r');

   if(function==1){
        //recover real direction of R&V
        if(R_int>=100) {
            R_int=R_int % 10 * 100 + R_int % 100 / 10 * 10 + R_int / 100;
        } else {
            if(R_int>=10) {
                R_int=R_int % 10 * 10 + R_int / 10;
            }
        }

        if(intR!=GetDigits(R_int)) {
            R_int=R_int* pow(10.0,(intR-GetDigits(R_int)) );
        }

        R=R_sign*( (double)R_int+R_dec );

        if(V_int>=100) {
            V_int=V_int % 10 * 100 + V_int % 100 / 10 * 10 + V_int / 100;
        } else {
            if(V_int>=10) {
                V_int=V_int % 10 * 10 + V_int / 10;
            }
        }

        if(intV!=GetDigits(V_int)) {
            V_int=V_int* pow(10.0,(intV-GetDigits(V_int)) );
        }  

        V=V_sign*( (double)V_int+V_dec );

        if(R!=0 && V==0) {
            mode=1;
        }
        if(R==0 && V!=0) {
            mode=2;
        }
        if(R!=0 && V!=0) {
            mode=3;
        }
        
        pc.printf("funtion: RV Control\n\r");
        pc.printf("R: %f\n\r",R);
        pc.printf("V: %f\n\r",V);
        pc.printf("mode: %d\n\r",mode);
    }
}

void R_getInput()
{   
    getInput();
    revolution_target = (int)R;
}
int FindP(string Notes){
    volatile float m_fre;
    if(Notes=="A^"){m_fre=415.30;}
    if(Notes=="A"){m_fre=440.00;}
    if(Notes=="A#"||Notes=="B^"){m_fre=466.16;}
    if(Notes=="B"||Notes=="B#"){m_fre=493.88;}
    if(Notes=="C^"||Notes=="C"){m_fre=523.25;}
    if(Notes=="C#"||Notes=="D^"){m_fre=554.37;}
    if(Notes=="D"){m_fre=587.33;}
    if(Notes=="D#"||Notes=="E^"){m_fre=622.25;}
    if(Notes=="E"||Notes=="E#"){m_fre=659.25;}
    if(Notes=="F^"||Notes=="F"){m_fre=698.46;}
    if(Notes=="F#"||Notes=="G^"){m_fre=739.99;}
    if(Notes=="G"){m_fre=783.99;}
    if(Notes=="G#"){m_fre=830.61;}
    m_period=(1.0f/m_fre)*1000000.0f;
    return (int)m_period;}

void m_play(int S, int P){
    int m_counter=0;
    while(m_counter<=S){
        m_counter++;
        L1L.period_us(P);
        L2L.period_us(P);
        L3L.period_us(P);
        update_motor_state();
        wait(1);
    }
}
//---------------------------------------------------------------------------//


//Main

int main()
{   
    R_getInput();
    if(function==1){     //control RV
        //Serial pc(SERIAL_TX, SERIAL_RX);//Initialise the serial port
        pc.printf("%d\n\r",revolution_target);
        
        V=abs(V);
        R=abs(R);
        if(mode==2) {
            lead=V_sign*2;
        }
        if(mode==1) {
            lead=R_sign*2;
        }
        
        if(mode==3) {
            lead=R_sign*2;
        }
        


        pc.printf("lead: %d\n\r",lead);
        L1L.period_us(50);
        L2L.period_us(50);
        L3L.period_us(50);



        //Run the motor synchronisation
        orState = motorHome();
        pc.printf("Rotor origin: %x\n\r",orState); //orState is subtracted from future rotor state inputs to align rotor and motor states
        //----------------------------------------interrupt set up
        I1.rise(&synchronise_cha);
        I1.fall(&update_motor_state);
        I2.rise(&update_motor_state);
        I2.fall(&update_motor_state);
        I3.rise(&update_motor_state);
        I3.fall(&update_motor_state);
        CA.rise(&controller_update);
        // ----------------------------------------
        update_motor_state(); //start running

        t.start();
        T.start();
        thread.start(controller);

        osThreadSetPriority(osThreadGetId(), osPriorityIdle);  ///-----set main priority as the lowest.
        while (1) {
          //  tt=1;
          //  tt=0;
         
            pc.printf("revolutions: %f\n\r",CA_counter/117.0f);
            pc.printf("velocity_target %f\n\r", velocity_target);
            pc.printf("synchronise_counter %d\n\r", synchronise_counter);
            pc.printf("velocity_R %f\n\r", 1.0f/time_R);
            pc.printf("mode: %d\n\r",mode);
         
        }
    }
    
    if(function==2){        //play melody 
        duty_cycle=0.5;
        pc.printf("playing\n\r");
        while(1){
            for(int i=0;i<=index;i++){
                pc.printf("Note: %s\n\r",Notes[i]);
                pc.printf("Seconds: %d\n\r",Seconds[i]);
                m_period=FindP(Notes[i]);
                m_play(Seconds[i],m_period);
            }
        }
    }
}