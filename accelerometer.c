#include "accelerometer.h"

/*
	Initialize the Accelerometer for Use
*/
void init_accelerometer(void) {	
	LIS3DSH_InitTypeDef lis_init_s;
	lis_init_s.Power_Mode_Output_DataRate = LIS3DSH_DATARATE_100; 			/* OUT data rate 100 Hz / 400 Hz */
	//lis_init_s.Axes_Enable = LIS3DSH_Y_ENABLE;                    		/* Axes enable */
	lis_init_s.Continous_Update = LIS3DSH_ContinousUpdate_Enabled;	
	//lis_init_s.AA_Filter_BW = LIS3DSH_AA_BW_50;
	lis_init_s.Full_Scale = LIS3DSH_FULLSCALE_2;                  			/* Full scale */
	lis_init_s.Self_Test = LIS3DSH_SELFTEST_NORMAL;  // NORMAL,P,M      /* Self test */
	LIS3DSH_Init(&lis_init_s); //this also configures the low-level interface	
	
	/*The Following Fixes Bugs in the Accelerometer Drivers*/
	uint8_t ctrl = 0x67; 
	LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG4, 1);
	
	ctrl = 0xC8; // INT1
  //ctrl = 0xD0; // INT2	
	LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG3, 1);
	
	//ctrl = 0x20; // LIS3DSH_SENSITIVITY_16G
	ctrl = 0xC0; // 50Hz AA filter,  2G sense
	//ctrl = 0xC8; // 50Hz AA filter,  4G sense
	//ctrl = 0xC4; // 50Hz AA filter,  6G sense
	LIS3DSH_Write(&ctrl, LIS3DSH_CTRL_REG5, 1);
	
	//ctrl = 0x10; // Set ADD_INC bit: Register address automatically incremented during a multiple byte access with a serial interface 
	//LIS3DSH_Write(&ctrl, 0x25, 1); // LIS3DSH_CTRL_REG6
	
	ctrl = OFFSET_X;
	LIS3DSH_Write(&ctrl, LIS3DSH_OFF_X, 1);
	ctrl = OFFSET_Y;
	LIS3DSH_Write(&ctrl, LIS3DSH_OFF_Y, 1);
	ctrl = OFFSET_Z;
	LIS3DSH_Write(&ctrl, LIS3DSH_OFF_Z, 1);
}

/*
	Read Axis Data from the accerlerometer and updates the pitch and roll angles
*/
void get_pitch_roll(float *pitch_pnt, float *roll_pnt) { 
  uint8_t buffer[6];
 
  LIS3DSH_Read(&buffer[0], LIS3DSH_OUT_X_L, 1);
  LIS3DSH_Read(&buffer[1], LIS3DSH_OUT_X_H, 1);
  LIS3DSH_Read(&buffer[2], LIS3DSH_OUT_Y_L, 1);
  LIS3DSH_Read(&buffer[3], LIS3DSH_OUT_Y_H, 1);
  LIS3DSH_Read(&buffer[4], LIS3DSH_OUT_Z_L, 1);
  LIS3DSH_Read(&buffer[5], LIS3DSH_OUT_Z_H, 1);

  int32_t aggregateResult = (int32_t)(buffer[0] | buffer[1] << 8);
  float a_x =(float)(LIS3DSH_SENSITIVITY_2G * (float)aggregateResult);

  aggregateResult = (int32_t)(buffer[2] | buffer[3] << 8);
  float a_y =(float)(LIS3DSH_SENSITIVITY_2G * (float)aggregateResult);

  aggregateResult = (int32_t)(buffer[4] | buffer[5] << 8);
  float a_z =(float)(LIS3DSH_SENSITIVITY_2G * (float)aggregateResult);

  uint8_t x_flag = 0;
  uint8_t y_flag = 0;
  
  if (a_x > 2000) {
    a_x = 4000 - a_x;
    x_flag = 1;
  }
  if (a_y > 2000) {
    a_y = 4000 - a_y;
    y_flag = 1;
  }
  
  float raw_pitch, raw_roll; 
  if (x_flag)
    raw_pitch = (float) -atan_table(a_x / sqrt(a_y * a_y + a_z * a_z));
  else 
    raw_pitch = (float) atan_table(a_x / sqrt(a_y * a_y + a_z * a_z));
  
  if (y_flag)
    raw_roll = (float) -atan_table(a_y / sqrt(a_x * a_x + a_z * a_z));
  else
    raw_roll = (float) atan_table(a_y / sqrt(a_x * a_x + a_z * a_z));
  
  *pitch_pnt = raw_pitch;
  *roll_pnt = raw_roll;
}