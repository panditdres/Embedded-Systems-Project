#include "arm_math.h"

#include "stm32f4xx.h"
#include "cmsis_os.h"

#include <stdio.h>
#include "smartrf_cc2500.h"
#include "cc2500.h"
#include "wireless.h"

#include "filter.h"
#include "lis3dsh.h"
#include "atan_LUT.h"
#include "accelerometer.h"
#include "keyboard.h"
#include "motor.h"
#include "led.h"

/*Moving Average Filter Sizes*/
#define PITCH_FILTER_SIZE   16
#define ROLL_FILTER_SIZE    16
#define DIST_FILTER_SIZE    32

/*Define if this Board is a Transmitter or Receiver*/
#define IS_TRANSMITTER      0

/*Delays used for each thread*/
#define TRANSMIT_DELAY      10
#define ACCR_DELAY          10
#define MOTOR_DELAY         10


/*!
 @brief Thread to perform menial tasks such as switching LEDs
 @param argument Unused
 */
void transmit_thread(void const * argument);
void receive_thread(void const *argument);
void accelerometer_thread(void const *argument);
void keyboard_thread(void const *argument);
void motor_thread(void const *argument);

//! Thread structure for above thread
osThreadDef(transmit_thread, osPriorityNormal, 1, 0);
osThreadDef(receive_thread, osPriorityNormal, 1, 0);
osThreadDef(accelerometer_thread, osPriorityNormal, 1, 0);
osThreadDef(keyboard_thread, osPriorityNormal, 1, 0);
osThreadDef(motor_thread, osPriorityNormal, 1, 0);

float global_pitch = .5f;
float global_roll= .5f;

//flag for stopping transmitting
int transmit_locked; 

//Global Struct Containing Display Values
struct Values{
    float pitch;
    float roll;
    uint8_t follow;
    float rollIncrement;
    float pitchIncrement;
    float time;
    char line1[16];
    char line2[16];
};

struct Values measurements = {0};

//Mutex to Protect Display Values
osMutexId measureUpdate;
osMutexDef(measure_update);

//OS Timer used for Motor Management
void Motor_Timer_Callback(void const *arg);
osTimerDef(MotorTimer, Motor_Timer_Callback);
uint32_t exec;
osTimerId timerid_motor;

//Global Thread ID
osThreadId tid_keyboard;  
osThreadId tid_motor;

/*!
 @brief Program entry point
 */
int main (void) {
	
  // ID for threads
  osThreadId tid_transmit;
  osThreadId tid_receive;
  osThreadId tid_accelerometer;    
  
  
  // Start wireless 
  uint8_t status;
  status = init_wireless();
  printf("CC2500 Ready (%x) \n",status);
  osDelay(1000);
  
	//Create Update Mutex
  measureUpdate = osMutexCreate(osMutex(measure_update));
  
	//Start either the Transmitter or Receiver
  if (IS_TRANSMITTER) {    
    init_accelerometer();
    transmit_locked = 0;
    enableKeyboard();
    enableKeyboardINT();
    
    //printf("Transmitter starting threads \n");
    tid_transmit = osThreadCreate(osThread(transmit_thread), NULL);
    tid_accelerometer = osThreadCreate(osThread(accelerometer_thread), NULL);
    tid_keyboard = osThreadCreate(osThread(keyboard_thread), &measurements);
  }
  
  else {
		motorEnable();
		init_accelerometer();
		enableLEDS(); 
    measurements.follow = 1; //Set Motors to follow received values
    //printf("Receiver starting threads \n");
    
    timerid_motor = osTimerCreate(osTimer(MotorTimer),osTimerPeriodic,&exec);
		tid_accelerometer = osThreadCreate(osThread(accelerometer_thread), NULL);
    tid_motor = osThreadCreate(osThread(motor_thread), &measurements);
    tid_receive = osThreadCreate(osThread(receive_thread), NULL);
  }
  
	// The main thread does nothing
	while(1){
		osDelay(osWaitForever);
	}
}

/*
	Thread is used to transmit the boards measured pitch and roll angle to the receiving board.
	Note we index the packets 0-255 to check on the receiver if all packets are received, but 
	if we miss packets this is not handled. Simply used for debugging.
*/
void transmit_thread(void const *argument) {
  uint8_t pkt_index = 0; //index for packets
	
	//Continously Transmit Pitch and Roll Data
  while (1) {
		
    if (!transmit_locked) {
      transmit_pitchroll(measurements.pitch, measurements.roll, pkt_index);
      pkt_index++;
      if (pkt_index >= 256) {
        pkt_index = 0;
      }  
    }
    else {
      pkt_index = 0;
    }  
    osDelay(TRANSMIT_DELAY);
  }
}

/*
	Main Receive Thread. It reads from the wireless  and checks the control part of the packet to determine
	what kind of data we are receiving. There are three types PACKET_CTRL1_BEGIN, PACKET_CTRL1_PR, PACKET_CTRL1_RECORD_BEGIN.
	PACKET_CTRL1_BEGIN = Receiving a desired Roll and Pitch Angle that the board should travel to over some time
	PACKET_CTRL1_PR = simply update the current pitch and roll angle for the board to follow. i.e. real time angle following
	PACKET_CTRL1_RECORD_BEGIN = indicate that we are receiving a sequence of angles that the board will follow.
	
*/
void receive_thread(void const *argument) {
  uint8_t status = CC2500_CommandProbe(CC2500_READBIT, CC2500_SRX);
  printf("Moved to RX (%x) \n",status);
  //osDelay(500);
  
  ring_buffer_t dist_filter; 
  int size = DIST_FILTER_SIZE; 
  int dist_buffer[size];
  init_buffer(&dist_filter,dist_buffer,size);  
    
  float f1, f2, value; 
  uint16_t control;
  uint8_t ctrl = 0;
  uint8_t ctrl2 = 0;
  while (1) { 
    control = receive_pitchroll(&f1, &f2, &ctrl2);
		
		//Check to see what kind of Packet we have received
    if (control == PACKET_CTRL1_BEGIN) {
      while (ctrl != PACKET_CTRL1_END) {
        receive_keypad(&ctrl, &value);
				//update our global values from the recieved data
        switch(ctrl) {
            case PACKET_CTRL1_PITCH: 
              measurements.pitchIncrement = value; 
              break;
            case PACKET_CTRL1_ROLL: 
              measurements.rollIncrement = value; 
              break;
            case PACKET_CTRL1_TIME: 
              measurements.time = value; 
              break;
        } 
      } 
      //printf("EX MOVE pitchIncrement = %f, rollIncrement = %f, time = %f \n",measurements.pitchIncrement,measurements.rollIncrement,measurements.time);
      //Turn off Motor Follow Mode
			measurements.follow = 0;
			//Start OS Timer for Motor 
      osTimerStart(timerid_motor,100);
			//Wait till follow flag has been reset
      while (!measurements.follow) {
        osDelay(500);
      }  
      ctrl = 0; // reset control
    }
		//Update Pitch and Roll angles for motor to follow
    else if (control == PACKET_CTRL1_PR) {

      measurements.pitch = f1;
      measurements.roll = f2;
      //Return the signal strength of the wireless receiver. 
      float f = filter_point(get_signal_strength(), &dist_filter);
      showSignalStrength(f); //updates the LEDs  
      //printf("READ: Pitch = %f (%f) Roll = %f (%f)  Control1 = %x  Pkt_index = %i Power = %f \n",f1,global_pitch,f2,global_roll,control,ctrl2,f);
    }
	//Begin playing Recorded Sequence	
	else if (control == PACKET_CTRL1_RECORD_BEGIN) {
	  float pitchBuffer[256];
	  float rollBuffer[256];
    float time_interval = f1;
		//start receiving the sequence and store in the buffers	
	  receive_record_sequence(pitchBuffer,rollBuffer,&time_interval);
	  //printf("RECV: Record Sequence: \n");
      int i;

      measurements.follow = 3; // Stop Motors Following
      
      //printf("Motors moving sequence, timedelay = %f \n",time_interval); 
      i = 0;
      uint8_t j;
      float f1, f2; 
      float pitchInc, rollInc; 
			//Playing back the Angle Sequence
      while (i < 256) {
        j = 0;
        pitchInc = (pitchBuffer[i+1] - pitchBuffer[i]) / 10;
        rollInc = (rollBuffer[i+1] - rollBuffer[i]) / 10;
        f1 = pitchBuffer[i];
        f2 = rollBuffer[i];
				//Linear Interpolation of the Data as we received only 1/10th the actual sequence
				//so we fill in the blanks using the average of two consequetive angles in the sequence.
        while (j < 10) {
            f1 += pitchInc; 
            f2 += rollInc;
            motorControl(f1, f2, global_pitch, global_roll);
            printf("Motors moving to angle: index = %i  Pitch = %f  Roll = %f \n",i,pitchBuffer[i],rollBuffer[i]); 
            j++;
						osDelay(10);//Delay of 100 so the Loop runs at 100Hz, the rate of which the accelerometer reads.
        }
        //osDelay((int)time_interval);
        i++;
      }
      
      osDelay(1000);
      measurements.follow = 1; // Stop following
		}	
			
  }
}
/*
	Accelerometer Thread. 
	Polls the acclerometer for axis data every 10ms.
	Updates the global pitch and roll variables in the measurements struct
*/
void accelerometer_thread(void const *argument) {
	float raw_pitch, raw_roll;
	float a_x, a_y, a_z;
	int32_t aggregateResult;
	uint8_t buffer[6];
	uint8_t x_flag, y_flag; 
	
	ring_buffer_t pitch_filter; 
	int size = PITCH_FILTER_SIZE; 
	int pitch_buffer[size];
	init_buffer(&pitch_filter,pitch_buffer,size);
	
	ring_buffer_t roll_filter; 
	size = ROLL_FILTER_SIZE; 
	int roll_buffer[size];
	init_buffer(&roll_filter,roll_buffer,size);
	
	/*
		Poll the accelerometer and filter the data.
	*/
	while(1) {		
			get_pitch_roll(&raw_pitch, &raw_roll);
      global_pitch = -filter_point((int) raw_pitch, &pitch_filter); //FIX
      global_roll = filter_point((int) raw_roll, &roll_filter);
      if (IS_TRANSMITTER) {
				measurements.pitch = global_pitch;
				measurements.roll = global_roll;
      }
			//printf("ACCR: Pitch = %f  Roll = %f \n",pitch,roll);
      osDelay(ACCR_DELAY);
	}
}
/*
	Motor Thread
	Handles the roll and pitch motors. It has three states, Follow, Goto Angle and Stop.
	Follow = simple updates the motor's angles to the values in the global struct.
	Goto = moves the motors to the specified angle over a specified time received from transmitter
	Stop =  simply stops the motors
*/
void motor_thread(void const *argument) {
  struct Values *values = (struct Values*)argument;
  char string[5]= {0};
  uint32_t mscount=0;

	while(1){
		//Follow Mode
		if(values->follow == 1){
			osMutexWait(measureUpdate, osWaitForever);
			motorControl(values->pitch, values->roll, global_pitch, global_roll);
			osMutexRelease(measureUpdate);    
		}
		//Goto Angle Mode
		if(values->follow == 0){
            osSignalWait(0x02, osWaitForever);
            osMutexWait(measureUpdate, osWaitForever);
            measurements.roll += values->rollIncrement;
            measurements.pitch += values->pitchIncrement;
            motorControl(measurements.pitch,measurements.roll, global_pitch, global_roll);
            ++mscount;
            //Check to see if we reached the desired time
            if (mscount >= values->time) {
                osTimerStop(timerid_motor); //stop timer
                mscount=0; //reset count
                // Delay 
                osDelay(1000);
                // Move back to follow mode
                measurements.follow = 1;
        }
        osMutexRelease(measureUpdate);
        }
    //Stop Mode
    if(values->follow == 3){
      osDelay(1000);
    }
		osDelay(MOTOR_DELAY);
	}
}
/*
	Keyboard Thread
	Checks the keys on the keyboard. Waits for an OSsignal from the Keyboard pins interrupts
	before checking what key has been pressed.
	It has three main states Idle, Record Sequence, or PRT
	Idle =  simply waits for a valid key press
	Record Sequence = begins recording the boards movements and saves to a buffer
	PRT = Pitch, Roll, Time waits for User input to send the desired angles to receiver.
*/
void keyboard_thread(void const *argument) {

	struct Values *values = (struct Values*)argument;
	
  uint8_t keyboard_mode=0;
  float currentRoll=0;
  float currentPitch=0;
  float previousRoll=0;
  float previousPitch=0;
  
  float mstime=0;
  
	uint8_t keyCurrent = 0xFF;
	

	while(1){
      
      // Wait for keypad Interrupt
      osSignalWait(0x08, osWaitForever);
			// Read the current key 
      keyCurrent = readKeyboard(); 
      
      transmit_locked = 1; // Lock transmitter
      
      //printf("Keyboard Value 0x%x\n", keyCurrent);
      
			//Record Angle Sequence State
      if((keyCurrent == D) && (keyboard_mode == KEYBOARD_STATE_IDLE)){
        //printf("Recording sequence!!! \n");
        
        float pitchBuffer[256];
        float rollBuffer[256];
        uint32_t time_ms = 100;
                
        // Record sequence
        int index = 0;
				//Record 256 samples or 25 seconds of movemement
        while (index < 256) {
          pitchBuffer[index] = measurements.pitch;
          rollBuffer[index] = measurements.roll;
          //printf("Index = %i  Pitch = %f  Roll = %f \n",index,pitchBuffer[index],rollBuffer[index]);
          index++;
          osDelay(time_ms); //Delay of 100ms
        }  
        
        //printf("Sending sequence!!! \n");
				//Send the sequence
        transmit_record_sequence(256, pitchBuffer, rollBuffer, (float) time_ms);
        
        //printf("Done transmitting sequence!!! \n");
        osDelay(10000);
        transmit_locked = 0; //Enable Broadcast Mode
      }
			
			//PRT STATE, sends Desired Pitch,Roll and Time
      else if((keyCurrent==A) || (keyboard_mode== KEYBOARD_STATE_PRT)){
        keyboard_mode = KEYBOARD_STATE_PRT;
        if(keyCurrent == A){
					//Reset Variables
          osMutexWait(measureUpdate, osWaitForever);
          values->follow=0;
          previousRoll= values->roll;
          previousPitch = values->pitch;
          measurements.rollIncrement = 0;
          measurements.pitchIncrement = 0;
          measurements.time = 0;
          osMutexRelease(measureUpdate);
          currentRoll=0;
          //Begin Keypad Transmission Command
          for (int i = 0; i < 1; i++)
            transmit_keypad_begin();
          //read Roll
          readValue(&currentRoll, &tid_keyboard);
        }
				//Read Pitch on B press
        if(keyCurrent == B){
          currentPitch=0;
          readValue(&currentPitch, &tid_keyboard);
				}
				//Read Time on C press
        if(keyCurrent == C){
            mstime=0;
            readValue(&mstime, &tid_keyboard);
						
						//Convert time to count value and pitch and roll to increment values
            osMutexWait(measureUpdate, osWaitForever);
            values->time= (float)(mstime/100);
            values->rollIncrement = (currentRoll-previousRoll)/(mstime/100);
            values->pitchIncrement = (currentPitch-previousPitch)/(mstime/100);
            //printf("Roll %f Pitch %f Time %f ms\n", currentRoll, currentPitch, mstime);
              
            // Transmit Data
            //printf("SENDING pitchIncrement = %f \n",values->pitchIncrement);
            transmit_keypad_pitch(values->pitchIncrement);
            osDelay(100);
            //printf("SENDING rollIncrement = %f \n",values->rollIncrement);
            transmit_keypad_roll(values->rollIncrement);
            osDelay(100);
            //printf("SENDING time = %f \n",values->time);
            transmit_keypad_time(values->time);
            osDelay(100);
            //printf("SENDING END \n");
            transmit_keypad_end();
            osDelay(100);
            osMutexRelease(measureUpdate);
            //osTimerStart(timerid_motor,100);
            //printf("Time %f\n", values->time);           
              
            // Start transmitting again after delay
            osDelay(6000);
            transmit_locked = 0; // unlock transmit
            keyboard_mode = KEYBOARD_STATE_IDLE;
        }
			}
      else {
        transmit_locked = 0;
      } 
		//Add delay before allowing next keyboard read to avoid debouncing.
    osDelay(200);
    osSignalClear(tid_keyboard, 0x08);
    }

	}

//KeyBoard Interrupt Handlers
void EXTI1_IRQHandler(){
	if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
    EXTI_ClearITPendingBit(EXTI_Line1); 
    osSignalSet(tid_keyboard, 0x08);
  }
}
void EXTI2_IRQHandler(){
	if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
    EXTI_ClearITPendingBit(EXTI_Line2);     
    osSignalSet(tid_keyboard, 0x08);
  }
}
void EXTI3_IRQHandler(){
	if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
    EXTI_ClearITPendingBit(EXTI_Line3);     
		osSignalSet(tid_keyboard, 0x08);    
    }
}
void EXTI9_5_IRQHandler(){
	if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
    EXTI_ClearITPendingBit(EXTI_Line6);     
		osSignalSet(tid_keyboard, 0x08);     
    }
}
//OS Timer Used for the Motor
void Motor_Timer_Callback(void const *arg){
  osSignalSet(tid_motor,0x02);
}

