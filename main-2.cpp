#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"
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
volatile int8_t orState = 0;
//Phase lead to make motor spin
volatile int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Threads
Thread CommOutT (osPriorityHigh,1024);
Thread SerialT (osPriorityHigh,1024);
Thread motorCtrlT (osPriorityNormal,1024);
Thread motorRotateT (osPriorityLow,1024);
RawSerial pc(SERIAL_TX, SERIAL_RX);


//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);
char instruction[18];
volatile uint64_t newKey = 0x0;
volatile float newVelocity = 0.0;
volatile float rotations = 0.0;
volatile int32_t speedController = 10;
Mutex newKey_mutex;
Queue<void, 8> inCharQ;

typedef struct{
    uint8_t code;
    uint32_t data;
    }message_t;

Mail<message_t,16> outMessages;

void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
}



void putMessage(uint8_t code, uint32_t data){
    message_t *pMessage = outMessages.alloc();
    pMessage->code = code;
    pMessage->data = data;
    outMessages.put(pMessage);
 }
void commOutFn(){
    while(1) {
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;
        if (pMessage->code == 4){
            pc.printf("Message %d with data 0x%016x\n\r",
            pMessage->code,pMessage->data);
            outMessages.free(pMessage);
        }else if (pMessage->code == 1){
            pc.printf("Position: %d\n\r",
            pMessage->data);
            outMessages.free(pMessage);
        }else if (pMessage->code == 2){
            pc.printf("Hash Rate: %d hz\n\r", pMessage->data);
            outMessages.free(pMessage);
        }
    }
}
//Set a given drive state
void motorOut(int8_t driveState, uint32_t torque){

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(torque);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(torque);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(torque);
    if (driveOut & 0x20) L3H = 0;
    }

    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }
void serialISR(){
    uint8_t newChar = pc.getc();
    pc.putc(newChar);
    inCharQ.put((void*)newChar);
}
//Basic synchronisation routine
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0, 1000);
    wait(2.0);

    //Get the rotor state
    return readRotorState();
}
int32_t motorPosition;
int32_t speed;

void rotate(){
    Timer t;
    int32_t target = rotations*6;
    int32_t previousError = target-motorPosition;
    t.start();
    t.stop();
    while (abs(motorPosition - target)>=6){
        speed = 10*(target-motorPosition)+10*(previousError - (abs(target-motorPosition)))/t.read();
        putMessage(1, motorPosition);
        t.start();
        previousError = (target-motorPosition);
        wait(0.05);
        t.stop();

    }
    //pc.printf("speed: %d target: %d motorPosition %d \n\r", speed, target, motorPosition);
    speed = 0;
}

void isr(){
    int32_t torqueGlobal = 0;
    static int8_t oldRotorState;
    int rotorState = readRotorState();
    int speedLocal;
    if ((speed>speedController && speedController != 0)){
        speedLocal = speedController;
    }else if (rotations == 0){
        speedLocal = speedController;
    }else{
        speedLocal = speed;
    }
    if (speedLocal < 0){
        lead = -2;
        if (speedLocal*(-1)>1000){
            torqueGlobal = 1000;
        }else{
            torqueGlobal = speedLocal*(-1);
        }
    }else{
        lead = 2;
        if (speedLocal>1000){
            torqueGlobal = 1000;
        }else{
            torqueGlobal = speedLocal;
        }
    }
    //pc.printf("Torque: %d \n\r", torqueGlobal);
    motorOut((readRotorState()-orState+lead+6)%6, torqueGlobal);
    if (rotorState - oldRotorState == 5) motorPosition--;
    else if (rotorState - oldRotorState == -5) motorPosition++;
    else motorPosition += (rotorState - oldRotorState);
    oldRotorState = rotorState;
}
void serialFn(){
    pc.attach(&serialISR);
    int i = 0;
    for (int k=0; k<18;k++){
        instruction[k]= ' ';
    }
    while(1){
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        if (i>=18){

        }else{
            if (newChar == '\r'){
                instruction[i]='\0';
                i=0;
                if (instruction[0]=='R'){
                    motorPosition = 0;
                    sscanf(instruction, "R%f", &rotations);
                    rotate();
                }else if(instruction[0]=='V'){

                    sscanf(instruction, "V%f", &newVelocity);

                }else if(instruction[0]=='K'){
                    newKey_mutex.lock();
                    sscanf(instruction, "K%llx", &newKey);
                    i=0;
                    newKey_mutex.unlock();
                }
            }else{
                instruction[i]=newChar;
                i++;
            }
        }
    }
}
void motorCrtlFn(){
    int32_t oldPosition;
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick, 100000);
    int8_t counter =0;
    int32_t velocity = 0;
    while(1){
        oldPosition = motorPosition;
        motorCtrlT.signal_wait(0x1);
        velocity = (motorPosition-oldPosition);
        if (counter%10 == 0){
            pc.printf("Velocity: %d \n\r", velocity);
        }
        if (newVelocity == 0){
            speedController = 0;
        }else{
            y =kp(actual - target)
            speedController = 75*(velocity-newVelocity);
        }
        counter++;
        }
}

//Main
int main() {
    Timer d;
    int counter=0;
    bool set = false;
    CommOutT.start(commOutFn);
    SerialT.start(serialFn);
    L1L.period_us(2000);
    L2L.period_us(2000);
    L3L.period_us(2000);
    SHA256 bitcoin;
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];
    //Initialise the serial port
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    I1.rise(&isr);
    I2.rise(&isr);
    I3.rise(&isr);
    I3.fall(&isr);
    I2.fall(&isr);
    I1.fall(&isr);
    orState = motorHome();
    motorCtrlT.start(motorCrtlFn);
    motorRotateT.start(rotate);
    //pc.printf("meh");
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    while (1) {
        *key = newKey;
        if (counter%5000==1){
            d.start();
        }
        SHA256::computeHash((uint8_t*) hash,(uint8_t*) key, sizeof(sequence));
        if (counter%5000 == 0){
            d.stop();
        }
        if (counter%5000 ==0){
            putMessage(2, 5000/d.read());
            d.reset();
        }
        counter++;
        //pc.printf("Hash:0x%016x\n\r", hash[1]);
        if ((hash[0] == 0) && (hash[1] == 0)){
            putMessage(4,*nonce);
        }
        *nonce +=(uint8_t) 1;
    }
}
