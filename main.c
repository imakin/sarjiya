/* BISMILLAH																																											*/
///		Allah Maha Pemurah
///		Allah Maha Penyayang
																																														/*
 * made from scratch 
 * 30 OKTOBER 2011
 * Makin
 * 
 * ATMEGA 32:
 * 	EEPROM 1024 Bytes
 * 	SRAM	  2048 Bytes
 * 
 * ADC sensor dpn 34567 ~67 adc, 345 sel
 */

//~ #define F_CPU 12000000
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include "lcd_lib.h"
#include <avr/eeprom.h>
#include "makin.h"

#define elif else if


///~ #define EepromWrite(_BLOCK,_VALUE)	eeprom_write_byte((uint8_t*)(_BLOCK),_VALUE) 
///Use update instead of write
#define EepromWrite(_BLOCK,_VALUE)	eeprom_update_byte((uint8_t*)(_BLOCK),_VALUE)
#define EepromRead(_BLOCK)	eeprom_read_byte((uint8_t*)(_BLOCK))


#define A_CUSTOM_COUNT 1


///Sensor case
#define ADDRESS_BLOCK_CASE_N		2
#define CASE_SIZE					40
#define ADDRESS_BLOCK_CASE_H		ADDRESS_BLOCK_CASE_N+1
#define ADDRESS_BLOCK_CASE_L		ADDRESS_BLOCK_CASE_H+CASE_SIZE+1
#define ADDRESS_BLOCK_CASE_V		ADDRESS_BLOCK_CASE_L+CASE_SIZE+1

#define ADDRESS_BLOCK_MAP_WAYPOINTS_NUM		ADDRESS_BLOCK_CASE_V+CASE_SIZE+1
#define MAP_MAXIMUM_WAYPOINT				100
#define ADDRESS_BLOCK_MAP_ACT_LOW			ADDRESS_BLOCK_MAP_WAYPOINTS_NUM+1
#define ADDRESS_BLOCK_MAP_ACT_HIGH			ADDRESS_BLOCK_MAP_ACT_LOW+MAP_MAXIMUM_WAYPOINT+1
#define ADDRESS_BLOCK_MAP_UNTIL_CASE_L		ADDRESS_BLOCK_MAP_ACT_HIGH+MAP_MAXIMUM_WAYPOINT+1
#define ADDRESS_BLOCK_MAP_UNTIL_CASE_H		ADDRESS_BLOCK_MAP_UNTIL_CASE_L+MAP_MAXIMUM_WAYPOINT+1

///Deprecated
//~ 
//~ #define WAYPOINTS					50
//~ #define MAP_SIZE					WAYPOINTS
//~ #define ADDRESS_BLOCK_MAP_MAZE_TOTAL	ADDRESS_BLOCK_CASE_V+CASE_SIZE+1
//~ #define ADDRESS_BLOCK_MAP_CASE_H	ADDRESS_BLOCK_MAP_MAZE_TOTAL+1							/**																												**/
//~ #define ADDRESS_BLOCK_MAP_CASE_L	ADDRESS_BLOCK_MAP_CASE_H+MAP_SIZE+1						/**																												**/
//~ #define ADDRESS_BLOCK_MAP_ACT_SPEED ADDRESS_BLOCK_MAP_CASE_L+MAP_SIZE+1						/**																												**/
//~ #define ADDRESS_BLOCK_MAP_ACT_KP	ADDRESS_BLOCK_MAP_ACT_SPEED+MAP_SIZE+1					/**																												**/
//~ #define ADDRESS_BLOCK_MAP_ACT_KD	ADDRESS_BLOCK_MAP_ACT_KP+MAP_SIZE+1						/**																												**/
//~ #define ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A	ADDRESS_BLOCK_MAP_ACT_KD+MAP_SIZE+1 		//BIT CONFIG  [CASE_A][CASE_A][CASE_A]	[UNTIL_A][UNTIL_A][UNTIL_A]	[TURN][TURN]
//~ #define ADDRESS_BLOCK_MAP_UNTIL_H	ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+MAP_SIZE+1		/**	   bit7 [caseA s.kiri] [caseA s.tengah] [caseA s.kanan]	bit5										**/
//~ #define ADDRESS_BLOCK_MAP_UNTIL_L	ADDRESS_BLOCK_MAP_UNTIL_H+MAP_SIZE+1						/**				bit4 [untilA s.kiri] [untilA s.tengah] [untilA s.kanan]	bit2																									**/

//~ #define ADDRESS_BLOCK_ACTION			ADDRESS_BLOCK_CASE_V


///Default: KP 6, KD 9, speed 50
#define ADDRESS_BLOCK_SENSOR_REF	ADDRESS_BLOCK_MAP_UNTIL_CASE_H+MAP_MAXIMUM_WAYPOINT+1
#define ADDRESS_BLOCK_DRIVE_SPEED	ADDRESS_BLOCK_SENSOR_REF+20
#define ADDRESS_BLOCK_DRIVE_KP		ADDRESS_BLOCK_DRIVE_SPEED+1
#define ADDRESS_BLOCK_DRIVE_KD		ADDRESS_BLOCK_DRIVE_KP+1

																							/**					bit1 [TURN][TURN] bit0																						**/
// 2 bit driver command: 10 forward, 00 right, 01 backward, 11 left
#define MAP_ACT_TURN_FORWARD		2 //00																	#define SENS_A_LEFT	2
#define MAP_ACT_TURN_BACKWARD		1 //01																	#define SENS_A_MID	1
#define MAP_ACT_TURN_LEFT			3 //10																	#define SENS_A_RIGHT 0
#define MAP_ACT_TURN_RIGHT			0 //11
//ALIASES
#define TURN_FORWARD 				2
#define TURN_BACKWARD 				1
#define TURN_LEFT 					3
#define TURN_RIGHT 					0
#define DIR_FORWARD 				2
#define DIR_BACKWARD 				1
#define DIR_LEFT 					3
#define DIR_RIGHT 					0
#define DRIVE_FORWARD 				2
#define DRIVE_BACKWARD 				1
#define DRIVE_LEFT					3
#define DRIVE_RIGHT					0

#define DELAY_50_MS					0
#define DELAY_100_MS				1
#define DELAY_200_MS				2
#define DELAY_220_MS				22
#define DELAY_250_MS				25
#define DELAY_300_MS				3
#define DELAY_310_MS				31
#define DELAY_350_MS				35
#define DELAY_400_MS				4
#define DELAY_500_MS				5
#define DELAY_600_MS				6
#define DELAY_700_MS				7




#define MAP_GET_ACT_DIRECTION			14
#define MAP_GET_ACT_PWM_RIGHT			11
#define MAP_GET_ACT_PWM_LEFT			8
#define MAP_GET_ACT_TIME_MS				5
#define MAP_GET_ACT_NEXT_RUN_SAMPLES	2
#define MAP_GET_ACT_NEXT_SENSOR_CONFIG	1
#define MAP_GET_ACT_NEXT_RUN_UNTIL		0

uint8_t MapGetAct(uint8_t act, uint8_t act_low, uint8_t act_high);
uint8_t MapGetActTranslate(uint8_t act, uint8_t act_low, uint8_t act_high);
/** //deprecated, while the idea kept for a while
void MapWriteTurn(uint8_t queue_num, uint8_t turn);
void MapWriteCaseA(uint8_t queue_num, uint8_t casea);
void MapWriteUntilA(uint8_t queue_num, uint8_t untila);
void MapWriteTCAUA(uint8_t queue_num, uint8_t tcaua);
uint8_t MapReadTCAUA(uint8_t queue_num);
uint8_t MapReadCaseA(uint8_t queue_num);
uint8_t MapReadUntilA(uint8_t queue_num);
uint8_t MapReadTurn(uint8_t queue_num);

uint8_t gMazeMapNum=0; ///POSISI ROBOT SUDAH DIMANA
uint8_t gMazeMapTot=10;///TOTAL MAPPING
**/
uint8_t gInverted = 0;

#define OCR_LEFT	OCR1A
#define OCR_RIGHT	OCR1B

uint8_t Button0(void);
uint8_t Button1(void);
uint8_t Button2(void);
uint8_t Button3(void);
uint8_t ButtonEnter(void);
uint8_t ButtonBack(void);
uint8_t ButtonNext(void);
uint8_t ButtonPrev(void);
void ButtonWait(void);
uint8_t ButtonIsPressed(void);
uint8_t ButtonIsNotPressed(void);
uint8_t ButtonRead(void);
void Init(void);
void SaveCase(void);
void LoadCase(void);
void SetComplex(void);
void SetMinim(void);
void SetMinimInverse(void);
void PIDCalculateExecute(void);
void PIDCalculateExecuteCustom(uint8_t speed, uint8_t kp, uint8_t kd);
uint8_t PIDGetError(void);
void DirForward(void);
void DirBackward(void);
void DirTurnLeft(void);
void DirTurnRight(void);
void DriveMove(uint8_t pwm_ki, uint8_t pwm_ka);
void DriveTurn(uint8_t dir, uint8_t pwm_left, uint8_t pwm_right, uint8_t time_ms, uint8_t inverted_turn);
void SensorReadDigital(void);
void SensorReadAnalog(void);
void Calibrate(void);
void SensorCalibrate(void);
void SensorTest(void);
void SensorRemap(void);
void ReStrainScroll(void);
void PrintScroll(uint8_t sNum, uint8_t sMax);
void Menu(void);
void SettingMenu(void);
void SettingDebugMenu(void);
void SettingDebugSensor(void);
void SettingDebugPrintlog(char* nama,int8_t data[]);
void SettingDebugPrintlog2(char* nama,int8_t data[][2]); //Seems avr-gcc should define unique functions to get unique arguments
void SettingSetPIDSpeedSpeed(void);
void SettingSetPIDSpeedKP(void);
void SettingSetPIDSpeedKD(void);
void SettingErrorMenu(void);
void SettingSetPIDSpeedMenu(void);
void Run(void);
void RunInverse(void);
void RunOnce(uint8_t speed, uint8_t kp, uint8_t kd);
void RunMapping(uint8_t mulai);
void RunLearn(void);
void SetInverse(void);
void RunStart01(void);
void RunEb00ke01(void);
void RunEb01ke02(void);
void RunEb02ke03(void);
void RunEb03ke04(void);
void RunFin01ke02(void);
void RunFin02ke03(void);
void RunFin03ke04(void);
void RunFin04ke05(void);
void RunFin05ke06(void);
void RunFin00ke01(void);
void RunProgram(void);
void RunWhileNormal(uint8_t sampling_limit);
void RunWhileNormalDebug(uint8_t sampling_limit);
void RunTestCounter(void);
void ProgMapping(void);

uint8_t gDriveDir,gDriveSpeed,gDriveLimit;
int gDriveSpeedLeft, gDriveSpeedRight;
int8_t gDriveError, gDriveLastError;
int gDriveKP, gDriveKD;

uint8_t gCaseH[50], gCaseL[50],  gCaseN;//Case High, Case Low, Case Value, Case Num (banyak)
int8_t gCaseV[50];
uint16_t gSensAnalog[19];
uint16_t gSensRef[19];//Bates LOW-HIGH Analog
uint8_t gSensH, gSensL, gSensA; //gSensA = sensor kiri,kanan,tengah

uint32_t counter=0,countermax=30;

#define SENS_A_LEFT_BIT		0 //gSensA = sensor kiri,kanan,tengah
#define SENS_A_MID_BIT		2
#define SENS_A_RIGHT_BIT	1

#define ANALOG_SENSOR_LOW_CHANNEL	7
#define ANALOG_SENSOR_HIGH_CHANNEL	6
#define ANALOG_SENSOR_LEFT_CHANNEL	SENS_A_LEFT_BIT
#define ANALOG_SENSOR_RIGHT_CHANNEL	SENS_A_RIGHT_BIT
#define ANALOG_SENSOR_MID_CHANNEL	SENS_A_MID_BIT



uint16_t gState;
uint8_t gScrollNum;
uint8_t gScrollMax;
///MENU2 pnya MAX SCROLL,, ADA BERAPA PILIHAN DIDALAMNYA
#define MAX_SCROLL_MENU							16
#define MAX_SCROLL_SETTING_MENU					6
#define MAX_SCROLL_SETTING_DEBUG_MENU			3
#define MAX_SCROLL_SET_PID_SPEED_MENU			5
#define MAX_SCROLL_SAVE_MENU					2
#define MAX_SCROLL_SENSOR_MENU					1


/**																**/
///			ATURAN ==> LAST DIGIT = NOMER CURSOR, Previous significant bit : parent's number
/**																**/
uint8_t GetCursor(uint16_t numstate);
uint8_t GetParent(uint16_t numstate);
uint8_t GetChild(uint8_t parent, uint8_t childid);
#define GetParentCursor(_ns)	GetCursor(GetParent(_ns))
#define STATE_MENU								0 ///THIS ONE GOES 0, as 1~9 div 10 will be 0
#define STATE_SET_INVERSE						1
#define STATE_RUN_MAPPING_C1					2
#define STATE_RUN_MAPPING_C2					3
#define STATE_RUN_MAPPING_C3					4
#define STATE_RUN_MAPPING_C4					5
#define STATE_RUN								7	///GUA BERI!
#define STATE_RUN_INVERSE						8
#define STATE_CALIBRATE							9 ///DIBIKIN DI MENU UTAMA, KRN SERING DILAKUKAN
#define STATE_RUN_LEARN							10
#define STATE_RUN_MAPPING_F0					16
#define STATE_RUN_MAPPING_F1					15
#define STATE_RUN_MAPPING_F2					14
#define STATE_RUN_MAPPING_F3					13
#define STATE_RUN_MAPPING_F4					12
#define STATE_RUN_MAPPING_F5					11

#define STATE_RUN_MAPPING						100
#define STATE_RUN_MAPPING_EBOTEC				100
#define STATE_RUN_TEST_COUNTER					100

#define STATE_SETTING_MENU						6
#define STATE_SETTING_DEBUG_MENU				61
#define STATE_SETTING_DEBUG_SENSOR				 611
#define STATE_SETTING_DEBUG_ADC					 612
#define STATE_SETTING_DEBUG_DRIVER				 613
#define STATE_SETTING_DEBUG_PRINTLOG			 614
#define STATE_SETTING_SET_PID_SPEED_MENU		62 ///PID & SPEED
#define STATE_SETTING_SET_PID_SPEED_KP			 621
#define STATE_SETTING_SET_PID_SPEED_KD			 622
#define STATE_SETTING_SET_PID_SPEED_SPEED		 623
#define STATE_SETTING_SET_PID_SPEED_MINSPEED	 624
#define STATE_SETTING_SET_PID_SPEED_MAXSPEED	 625
#define STATE_SETTING_SAVE_MENU					63
#define STATE_SETTING_SAVE_SAVE					 631
#define STATE_SETTING_SAVE_LOAD					 632
#define STATE_SETTING_SENSOR_MENU				64
#define STATE_SETTING_SENSOR_MODE				 641
#define STATE_SETTING_SAMPLING					65
#define STATE_SETTING_ERROR						66 //BOBOT



#define BUTTON_0_DOWN	0
#define BUTTON_1_DOWN	1
#define BUTTON_2_DOWN	2
#define BUTTON_3_DOWN	3
#define BUTTON_ENTER_DOWN	1
#define BUTTON_BACK_DOWN	2
#define BUTTON_PREV_DOWN	0
#define BUTTON_NEXT_DOWN	3


int main(void)
{
	Init();	
	gState = STATE_MENU;
	while(1)
	{
		if (gState == STATE_CALIBRATE)						Calibrate();
		if (gState == STATE_MENU)							Menu();
		if (gState == STATE_RUN)							Run();
		if (gState == STATE_RUN_INVERSE)					RunInverse();
		if (gState == STATE_RUN_MAPPING)					RunMapping(0);
		if (gState == STATE_RUN_MAPPING_C1)					RunMapping(1);
		if (gState == STATE_RUN_MAPPING_C2)					RunMapping(2);
		if (gState == STATE_RUN_MAPPING_C3)					RunMapping(3);
		if (gState == STATE_RUN_MAPPING_C4)					RunMapping(4);
		if (gState == STATE_RUN_MAPPING_F0)					RunFin00ke01();
		if (gState == STATE_RUN_MAPPING_F1)					RunFin01ke02();
		if (gState == STATE_RUN_MAPPING_F2)					RunFin02ke03();
		if (gState == STATE_RUN_MAPPING_F3)					RunFin03ke04();
		if (gState == STATE_RUN_MAPPING_F4)					RunFin04ke05();
		if (gState == STATE_RUN_MAPPING_F5)					RunFin05ke06();
		
		if (gState == STATE_RUN_MAPPING_EBOTEC)				RunMapping(4);
		if (gState == STATE_RUN_LEARN)						RunLearn();
		if (gState == STATE_RUN_TEST_COUNTER)				RunTestCounter();
		if (gState == STATE_SETTING_MENU)					SettingMenu();
		if (gState == STATE_SETTING_SET_PID_SPEED_MENU)		SettingSetPIDSpeedMenu();
		if (gState == STATE_SETTING_SET_PID_SPEED_KD)		SettingSetPIDSpeedKD();
		if (gState == STATE_SETTING_SET_PID_SPEED_KP)		SettingSetPIDSpeedKP();
		if (gState == STATE_SETTING_SET_PID_SPEED_SPEED)	SettingSetPIDSpeedSpeed();
		if (gState == STATE_SETTING_DEBUG_MENU)				SettingDebugMenu();
		if (gState == STATE_SETTING_DEBUG_SENSOR)			SettingDebugSensor();
		if (gState == STATE_SETTING_ERROR)					SettingErrorMenu();
		if (gState == STATE_SET_INVERSE)					SetInverse();
	}
}

void Init()
{
	counter=0;
	SetMinim();
	LCDinit();
	LCDclr();
	LCDGotoXY(0,0);
	LCDclr();
	LCDGotoXY(0,0);
	initpwm();
	DDRD |= 0b00111100;
	DDRC=0b11100000;
	PORTC=0b00001111;
	DDRA=0b00111000;
	initadc();
	gScrollNum = 1;
	gDriveLastError=0;
	
	//~ gDriveLimit = 120;
	//~ gDriveSpeed = 60;
	//~ gDriveKP = 6;
	//~ gDriveKD = 9;
	
	gDriveLimit = 120;
	gDriveSpeed = EepromRead(ADDRESS_BLOCK_DRIVE_SPEED);
	gDriveKP = EepromRead(ADDRESS_BLOCK_DRIVE_KP);
	gDriveKD = EepromRead(ADDRESS_BLOCK_DRIVE_KD);
	
	for (int i = 0; i < 19; i++)
	{
		gSensRef[i] = 4*EepromRead(ADDRESS_BLOCK_SENSOR_REF+i);
	}
}


void RunMapping(uint8_t mulai)
{
	///start 0
	_delay_ms(250);
	if (mulai==0)
	{
		Run();
	}
	else if (mulai==1)
	{
		RunEb00ke01();
	}
	else if (mulai==2)
	{
		RunEb01ke02();
	}
	else if (mulai==3)
	{
		RunEb02ke03();
		
	}
	else if (mulai==4)
	{
		RunEb03ke04();
	}
	//~ Run();
	counter=0;
	gState=STATE_MENU;
	OCR1A=0;
	OCR1B=0;
	//~ while(1);
}

void RunLearn()
{
	uint8_t wp_current=0;
	uint8_t wp_number = EepromRead(ADDRESS_BLOCK_MAP_WAYPOINTS_NUM);
	uint8_t mode=0;//0 continue, 1 edit, 2 clearout
	uint8_t act=100;//no button have pressed
	uint8_t mapping_act_set_l=0;
	uint8_t mapping_act_set_h=0;
	
	///should clear out or just edit what previously have been saved
	gScrollNum = 1;
	gScrollMax = 3;
	
	while (act!=BUTTON_ENTER_DOWN && act!=BUTTON_BACK_DOWN)
	{
		ReStrainScroll();
		PrintScroll(gScrollNum,gScrollMax);
		
		if (gScrollNum == 1)
		{		
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("Continue   [EXIT"),16);
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
				mode = 0;
			else if (act == BUTTON_NEXT_DOWN)
				gScrollNum++;
			else if (act == BUTTON_PREV_DOWN)
				gScrollNum--;
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_MENU;
				return;
			}
		}
		else if (gScrollNum == 2)
		{		
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("Edit Data  [EXIT"),16);
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
				mode = 1;
			else if (act == BUTTON_NEXT_DOWN)
				gScrollNum++;
			else if (act == BUTTON_PREV_DOWN)
				gScrollNum--;
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_MENU;
				return;
			}
		}
		else if (gScrollNum == 3)
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("Start New  [EXIT"),16);
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
				mode = 2;
			else if (act == BUTTON_NEXT_DOWN)
				gScrollNum++;
			else if (act == BUTTON_PREV_DOWN)
				gScrollNum--;
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_MENU;
				return;
			}
		}
	}
	act=100;///No button have set
	if (mode == 0)
	{
		wp_number = EepromRead(ADDRESS_BLOCK_MAP_WAYPOINTS_NUM);
		wp_current = wp_number;
		if (wp_number == 0)
		{
			///No saved data found
			mode = 2;
		}
	}
	if (mode == 2)
	{
		///Start out mode
		wp_current = 0;
		wp_number = 0;
	}
	if (mode == 1)
	{
		wp_current = 0;
		wp_number = EepromRead(ADDRESS_BLOCK_MAP_WAYPOINTS_NUM);
	}
	
	
	if (mode==0)
	{
		///Continue mode
		///At first run as how it should go if it was from previous waypoint
		RunWhileNormal( MapGetActTranslate(MAP_GET_ACT_NEXT_RUN_SAMPLES, EepromRead(ADDRESS_BLOCK_MAP_ACT_LOW+wp_number), EepromRead(ADDRESS_BLOCK_MAP_ACT_HIGH+wp_number) ) );
	}
	
	// Show mapping menu
	DriveMove(0,0);
	
	while (gState=STATE_RUN_LEARN)
	{
		if (mode==0 || mode==2)
		{
			wp_number++;
		}
		else
		{
			///Mode edit, menu asks which waypoint to edit at first				
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("Waypoint to edit"),16);
			gScrollNum = wp_current;
			gScrollMax = wp_number;
			while (act!=BUTTON_ENTER_DOWN && act!=BUTTON_BACK_DOWN)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     ###    >>"),16);
				cetak_bil(gScrollNum,7,1,3);
				act = ButtonRead();
				//generalize other act
				if (act==BUTTON_ENTER_DOWN)
				{
					wp_current = gScrollNum;
				}
				else if (act==BUTTON_NEXT_DOWN)
				{
					if (gScrollNum>gScrollMax)
						gScrollNum = 0;
					else
						gScrollNum++;
				}
				else if (act==BUTTON_PREV_DOWN)
				{
					if (gScrollNum == 0)
						gScrollNum = gScrollMax;
					else
						gScrollNum--;
				}
				else if (act==BUTTON_BACK_DOWN)
				{
					LCDGotoXY(0,0);
					LCDstring((uint8_t*)("MAPPING DONE... "),16);
					gState=STATE_MENU; gScrollNum = GetCursor(STATE_RUN);
					_delay_ms(2000);
					return;
				}
				
			}
		}
		_delay_ms(200); act=100;
		///Pre step: set act
		mapping_act_set_l = 0;
		mapping_act_set_h = 0;
		
		/// 1st step : Set ForceMove Direction
		LCDGotoXY(0,0);
		LCDstring((uint8_t*)("Set ForceMove [B"),16);
		gScrollNum = 1;
		gScrollMax = 4;
			
		while (act!=BUTTON_ENTER_DOWN && act!=BUTTON_BACK_DOWN)
		{
			ReStrainScroll();
			if (gScrollNum == 1)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<  FORWARD   >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
				{
					mapping_act_set_h |= DIR_FORWARD<<6;
				}
			}
			else if (gScrollNum == 2)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<< TURN LEFT  >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
				{
					mapping_act_set_h |= DIR_LEFT<<6;
				}
			}
			else if (gScrollNum == 3)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<< TURN RIGHT >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
				{
					mapping_act_set_h |= DIR_RIGHT<<6;
				}
			}
			else if (gScrollNum == 4)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<  BACKWARD  >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
				{
					mapping_act_set_h |= DIR_BACKWARD<<6;
				}
			}
			//generalize other act
			if (act==BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act==BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act==BUTTON_BACK_DOWN)
			{
				LCDGotoXY(0,0);
				LCDstring((uint8_t*)("MAPPING DONE... "),16);
				gState=STATE_MENU; gScrollNum = GetCursor(STATE_RUN);
				_delay_ms(2000);
				return;
			}
			
		}/// Done 1st step : Set ForceMove Direction
		_delay_ms(200); act=100;
		///2nd step : Set PWM right
		LCDGotoXY(0,0);
		LCDstring((uint8_t*)("Set RightPWM  [B"),16);
		
		gScrollNum = 1;
		gScrollMax = 8;
		while (act!=BUTTON_ENTER_DOWN && act!=BUTTON_BACK_DOWN)
		{
			ReStrainScroll();
			if (gScrollNum == 1)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("       0      >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 0;
			}
			else if (gScrollNum == 2)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     5      >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 1<<3;
			}
			else if (gScrollNum == 3)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     10     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 2<<3;
			}
			else if (gScrollNum == 4)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     20     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 3<<3;
			}
			else if (gScrollNum == 5)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     30     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 4<<3;
			}
			else if (gScrollNum == 6)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     40     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 5<<3;
			}
			else if (gScrollNum == 7)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     60     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 6<<3;
			}
			else if (gScrollNum == 8)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     80       "),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 7<<3;
			}
			//generalize other act
			if (act==BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act==BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act==BUTTON_BACK_DOWN)
			{
				LCDGotoXY(0,0);
				LCDstring((uint8_t*)("MAPPING DONE... "),16);
				gState=STATE_MENU; gScrollNum = GetCursor(STATE_RUN);
				_delay_ms(2000);
				return;
			}
			
		}///Done 2nd step : Set PWM right
		_delay_ms(200); act=100;
		
		///3nd step : Set PWM Left
		LCDGotoXY(0,0);
		LCDstring((uint8_t*)("Set LeftPWM   [B"),16);
		
		gScrollNum = 1;
		gScrollMax = 8;
		while (act!=BUTTON_ENTER_DOWN && act!=BUTTON_BACK_DOWN)
		{
			ReStrainScroll();
			if (gScrollNum == 1)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     0      >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 0;
			}
			else if (gScrollNum == 2)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     5      >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 1<<0;
			}
			else if (gScrollNum == 3)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     10     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 2<<0;
			}
			else if (gScrollNum == 4)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     20     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 3<<0;
			}
			else if (gScrollNum == 5)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     30     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 4<<0;
			}
			else if (gScrollNum == 6)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     40     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 5<<0;
			}
			else if (gScrollNum == 7)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     60     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 6<<0;
			}
			else if (gScrollNum == 8)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     80     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_h |= 7<<0;
			}
			//generalize other act
			if (act==BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act==BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act==BUTTON_BACK_DOWN)
			{
				LCDGotoXY(0,0);
				LCDstring((uint8_t*)("MAPPING DONE... "),16);
				gState=STATE_MENU; gScrollNum = GetCursor(STATE_RUN);
				_delay_ms(2000);
				return;
			}
			
		}///Done 3rd step : Set PWM left
		_delay_ms(200); act=100;
		
		///4th step : Set Time ms
		LCDGotoXY(0,0);
		LCDstring((uint8_t*)("Set Duration  [B"),16);
		
		gScrollNum = 1;
		gScrollMax = 8;
		while (act!=BUTTON_ENTER_DOWN && act!=BUTTON_BACK_DOWN)
		{
			ReStrainScroll();
			if (gScrollNum == 1)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    50 ms   >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 0;
			}
			else if (gScrollNum == 2)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    100 ms  >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 1<<5;
			}
			else if (gScrollNum == 3)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    200 ms  >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 2<<5;
			}
			else if (gScrollNum == 4)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    300 ms  >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 3<<5;
			}
			else if (gScrollNum == 5)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    400 ms  >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 4<<5;
			}
			else if (gScrollNum == 6)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    500 ms  >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 5<<5;
			}
			else if (gScrollNum == 7)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    600 ms  >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 6<<5;
			}
			else if (gScrollNum == 8)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    700 ms  >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 7<<5;
			}
			//generalize other act
			if (act==BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act==BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act==BUTTON_BACK_DOWN)
			{
				LCDGotoXY(0,0);
				LCDstring((uint8_t*)("MAPPING DONE... "),16);
				gState=STATE_MENU; gScrollNum = GetCursor(STATE_RUN);
				_delay_ms(2000);
				return;
			}
			
		}///Done 4th step : Set Time ms
		_delay_ms(200); act=100;
		///5th step : Set Next run sampling
		LCDGotoXY(0,0);
		LCDstring((uint8_t*)("Set Nextrun S [B"),16);
		
		gScrollNum = 1;
		gScrollMax = 8;
		while (act!=BUTTON_ENTER_DOWN && act!=BUTTON_BACK_DOWN)
		{
			ReStrainScroll();
			if (gScrollNum == 1)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    0 none  >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 0;
			}
			else if (gScrollNum == 2)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    5       >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 1<<2;
			}
			else if (gScrollNum == 3)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    10      >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 2<<2;
			}
			else if (gScrollNum == 4)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    15      >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 3<<2;
			}
			else if (gScrollNum == 5)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    20      >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 4<<2;
			}
			else if (gScrollNum == 6)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    30      >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 5<<2;
			}
			else if (gScrollNum == 7)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    40      >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 6<<2;
			}
			else if (gScrollNum == 8)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<    60      >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 7<<2;
			}
			//generalize other act
			if (act==BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act==BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act==BUTTON_BACK_DOWN)
			{
				LCDGotoXY(0,0);
				LCDstring((uint8_t*)("MAPPING DONE... "),16);
				gState=STATE_MENU; gScrollNum = GetCursor(STATE_RUN);
				_delay_ms(2000);
				return;
			}
		}///Done 5th step : Set Next run sampling
		_delay_ms(200); act=100;
		
		///6th step : Set Next run Sensor Config
		LCDGotoXY(0,0);
		LCDstring((uint8_t*)("SensorConfig  [B"),16);
		
		gScrollNum = 1;
		gScrollMax = 2;
		while (act!=BUTTON_ENTER_DOWN && act!=BUTTON_BACK_DOWN)
		{
			ReStrainScroll();
			if (gScrollNum == 1)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<  SetMinim  >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 0;
			}
			else if (gScrollNum == 2)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<< SetComplex >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 1<<1;
			}
			//generalize other act
			if (act==BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act==BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act==BUTTON_BACK_DOWN)
			{
				LCDGotoXY(0,0);
				LCDstring((uint8_t*)("MAPPING DONE... "),16);
				gState=STATE_MENU; gScrollNum = GetCursor(STATE_RUN);
				_delay_ms(2000);
				return;
			}
		}///Done 6th step : Set Next run Sensor Config
		_delay_ms(200); act=100;
		///7th step : Set Next run use until condition occur
		LCDGotoXY(0,0);
		LCDstring((uint8_t*)("UntilCondtion [B"),16);
		
		gScrollNum = 1;
		gScrollMax = 2;
		while (act!=BUTTON_ENTER_DOWN && act!=BUTTON_BACK_DOWN)
		{
			ReStrainScroll();
			if (gScrollNum == 1)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     No     >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
					mapping_act_set_l |= 0;
			}
			else if (gScrollNum == 2)
			{
				LCDGotoXY(0,1);
				LCDstring((uint8_t*)("<<     Yes    >>"),16);
				act = ButtonRead();
				if (act==BUTTON_ENTER_DOWN)
				{
					mapping_act_set_l |= 1;
					///Get condition
					LCDGotoXY(0,0);
					LCDstring((uint8_t*)("OK] Cap. Sensor "),16);
					_delay_ms(200); act=100;
					uint8_t l_Num;
					while (!ButtonEnter())
					{
						SensorReadDigital();
						for (l_Num=0;l_Num<=7;l_Num++)
						{
							if (isset(gSensH,l_Num))	cetak_bil(1,15-l_Num-8,1,1);
							else 						cetak_bil(0,15-l_Num-8,1,1);
							if (isset(gSensL,l_Num))	cetak_bil(1,15-l_Num,1,1);
							else 						cetak_bil(0,15-l_Num,1,1);
						}
					}
					EepromWrite(ADDRESS_BLOCK_MAP_UNTIL_CASE_L+wp_current, gSensL);
					EepromWrite(ADDRESS_BLOCK_MAP_UNTIL_CASE_H+wp_current, gSensH);
					LCDGotoXY(0,0);
					LCDstring((uint8_t*)("Condition set   "),16);
					_delay_ms(500);
					///SAVE it right away to eeprom
				}
			}
			//generalize other act
			if (act==BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act==BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act==BUTTON_BACK_DOWN)
			{
				LCDGotoXY(0,0);
				LCDstring((uint8_t*)("MAPPING DONE... "),16);
				gState=STATE_MENU; gScrollNum = GetCursor(STATE_RUN);
				_delay_ms(2000);
				return;
			}
			
		}///Done 7th step : Set Next run use until condition occur
		_delay_ms(200); act=100;
		///SAVE it right away to eeprom
		EepromWrite(ADDRESS_BLOCK_MAP_ACT_LOW+wp_current, mapping_act_set_l);
		EepromWrite(ADDRESS_BLOCK_MAP_ACT_HIGH+wp_current, mapping_act_set_h);
		EepromWrite(ADDRESS_BLOCK_MAP_WAYPOINTS_NUM, wp_number);
		
		LCDGotoXY(0,0);
		LCDstring((uint8_t*)("  Waypoint ###  "),16);
		LCDstring((uint8_t*)("   mapping set  "),16);
		cetak_bil(wp_current, 8, 0, 3);
		//~ _delay_ms(500);
		while(ButtonIsNotPressed());
		
		//Run it single role
		DriveTurn(	MapGetActTranslate(MAP_GET_ACT_DIRECTION, mapping_act_set_l, mapping_act_set_h), 
					MapGetActTranslate(MAP_GET_ACT_PWM_LEFT, mapping_act_set_l, mapping_act_set_h), 
					MapGetActTranslate(MAP_GET_ACT_PWM_RIGHT, mapping_act_set_l, mapping_act_set_h), 
					MapGetActTranslate(MAP_GET_ACT_TIME_MS,  mapping_act_set_l, mapping_act_set_h), 
					gInverted);
		///Force drive done
		if (MapGetActTranslate(MAP_GET_ACT_NEXT_SENSOR_CONFIG, mapping_act_set_l, mapping_act_set_h)==1)
			SetComplex();
		else
			SetMinim();
		
		if (MapGetActTranslate(MAP_GET_ACT_NEXT_RUN_UNTIL,mapping_act_set_l, mapping_act_set_h)==0)
		{
			RunWhileNormal(	MapGetActTranslate(MAP_GET_ACT_NEXT_RUN_SAMPLES, mapping_act_set_l, mapping_act_set_h)	);
		}
		else
		{
			///Run until certain sensor condition detected
			counter=0;
			SensorReadDigital();
			PIDGetError();
			while (counter<MapGetActTranslate(MAP_GET_ACT_NEXT_RUN_SAMPLES, mapping_act_set_l, mapping_act_set_h)) //sampling limit
			{
				RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
				if ((!PIDGetError()))
				{
					counter++;
				}
				else
				{
					counter=0;
				}
				if (
						(gSensL == EepromRead(ADDRESS_BLOCK_MAP_UNTIL_CASE_L) ) &&
						(gSensH == EepromRead(ADDRESS_BLOCK_MAP_UNTIL_CASE_H) )
					)
				{
					counter = counter+100;
				}
				
			}
		}
		//Run done
		///back to default
		SetMinim();
		
		if (mode!=1)
			wp_current++;
	}///end while (gState=STATE_RUN_LEARN)
	return;
}

void RunProgram()
{
	return;
}
void RunWhileNormalDebug(uint8_t sampling_limit)
{
	counter=0;
	SensorReadDigital();
	PIDGetError();
	int8_t log[sampling_limit*2+2][2];
	uint8_t i=0;
	while (counter<sampling_limit)
	{
		
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
		
		if ((!PIDGetError()))
		{
			counter++;
			log[i][0] = gSensL;///Tampil di LCD
			log[i][1] = gSensH;///Tampil di LCD
			i++;
		}
		else
		{
			counter=0;
		}
	}
	DriveMove(0,0);
	counter=0;
	
	gState = STATE_SETTING_DEBUG_PRINTLOG;
	SettingDebugPrintlog2("LOG RUNWHILE ERR",log);
	return;
}
void RunWhileNormal(uint8_t sampling_limit)
{
	counter=0;
	SensorReadDigital();
	PIDGetError();
	while (counter<sampling_limit)
	{
		
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
		
		if ((!PIDGetError()))
		{
			counter++;
		}
		else
		{
			counter=0;
		}
	}
	DriveMove(0,0);
	counter=0;
	
	return;
}
///EBOTEC

void SettingDebugPrintlog(char* nama,int8_t data[])
{
	/// shows
	uint8_t act;
	uint8_t i=0;
	while (gState == STATE_SETTING_DEBUG_PRINTLOG)
	{
		LCDGotoXY(0,0);
		LCDstring(nama,16);
		LCDGotoXY(0,1);
		LCDstring("<<-   ####   +>>",16);
		cetak_bil(data[i],6,1,4);
		act=ButtonRead();
		if (act == BUTTON_NEXT_DOWN)		i++;
		else if (act == BUTTON_PREV_DOWN)	i--;
		else if (act == BUTTON_BACK_DOWN)	gState = GetParent(STATE_RUN_LEARN);
		else if (act == BUTTON_ENTER_DOWN) gState = GetParent(STATE_RUN_LEARN);
	}	
}
void SettingDebugPrintlog2(char* nama,int8_t data[][2])
{
	/// shows
	uint8_t act;
	uint8_t i=0;
	while (gState == STATE_SETTING_DEBUG_PRINTLOG)
	{
		LCDGotoXY(0,0);
		LCDstring(nama,16);
		LCDGotoXY(0,1);
		LCDstring("<<-####||####+>>",16);
		cetak_bil(data[i][0],3,1,4);
		cetak_bil(data[i][1],9,1,4);
		act=ButtonRead();
		if (act == BUTTON_NEXT_DOWN)		i++;
		else if (act == BUTTON_PREV_DOWN)	i--;
		else if (act == BUTTON_BACK_DOWN)	gState = GetParent(STATE_RUN_LEARN);
		else if (act == BUTTON_ENTER_DOWN) gState = GetParent(STATE_RUN_LEARN);
	}
	return;
	
}

void RunEb00ke01()
{
	DriveTurn(DIR_FORWARD,30,30,DELAY_300_MS,gInverted);
	
	RunWhileNormal(3);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	DriveTurn(DIR_RIGHT,30,30,DELAY_250_MS,gInverted);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	RunWhileNormal(3);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_250_MS,gInverted);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	RunWhileNormal(3);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	DriveTurn(DIR_RIGHT,30,30,DELAY_250_MS,gInverted);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	RunWhileNormal(3);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_250_MS,gInverted);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	RunWhileNormal(3);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_250_MS,gInverted);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	RunWhileNormal(3);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	DriveTurn(DIR_RIGHT,30,30,DELAY_250_MS,gInverted);
	
	DriveMove(0,0);
	_delay_ms(5);
	
	RunWhileNormal(3);
	DriveTurn(DIR_FORWARD,30,30,DELAY_300_MS,gInverted);
	
	RunWhileNormal(3);
	DriveTurn(DIR_RIGHT,30,30,DELAY_300_MS,gInverted);
	
	DriveMove(0,0);
	_delay_ms(10);
	
	RunWhileNormal(3);
	DriveTurn(DIR_LEFT,40,40,DELAY_300_MS,gInverted);
	
	DriveMove(0,0);
	_delay_ms(50);
	
	RunWhileNormal(3);
	DriveTurn(DIR_LEFT,40,40,DELAY_300_MS,gInverted);
	
	RunWhileNormal(5);
	DriveTurn(DIR_RIGHT,30,30,DELAY_300_MS,gInverted);
	
	RunWhileNormal(5);
	DriveTurn(DIR_RIGHT,30,30,DELAY_300_MS,gInverted);
	
	RunWhileNormal(5);
	
	RunEb01ke02();
	
	while (ButtonIsNotPressed());
	
	
}
void RunEb01ke02()
{
	DriveTurn(DIR_FORWARD,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(20);
	
	DriveTurn(DIR_FORWARD,60,10,DELAY_100_MS,gInverted);
	
	SetComplex();
	counter=0;
	SensorReadDigital();
	PIDGetError();
	while (counter<60) //sampling limit
	{
		
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
		
		if ((!PIDGetError()))
		{
			counter++;
		}
		else
		{
			counter=0;
		}
		if (
			(gSensH == 0b10000000 || gSensH == 0b11000000 || gSensH == 0b01000000 || gSensH == 0b01100000 )&& 
			(gSensL == 0b00000001 || gSensL == 0b00000011 || gSensL == 0b00000010 || gSensL == 0b00000110 )
			)
		{
			counter = counter+21;
		}
		
	}
	DriveMove(0,0);
	counter=0;
	
	//~ RunEb02ke03()
	///special case that this checkpoint start different from RunEb03ke03 function
	SetMinim();
	
	DriveTurn(DIR_LEFT,30,30,DELAY_200_MS,gInverted);
	RunWhileNormal(5);
	
	//~ DirTurnLeft();
	//~ DriveMove(30,30);
	//~ _delay_ms(350);
	DriveTurn(DIR_LEFT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_FORWARD,30,30,DELAY_200_MS,gInverted);
	RunWhileNormal(5);
	
	///BUNDERAN ITEM
	/**DirForward();
	DriveMove(7,95);
	_delay_ms(350);**/
	DriveTurn(DIR_FORWARD,7,95,DELAY_350_MS,gInverted);
	///DONE
	
	RunWhileNormal(5);
	
	DriveTurn(DIR_FORWARD,90,20,DELAY_200_MS,gInverted);
	RunWhileNormal(10);
	
	///BLACKWHITE
	SetMinimInverse();
	
	RunWhileNormal(5);
	if (gInverted==1)
		DriveTurn(DIR_RIGHT,120,50,DELAY_300_MS,gInverted);
	else
		DriveTurn(DIR_RIGHT,120,40,DELAY_300_MS,gInverted);
	DriveTurn(DIR_FORWARD,60,60,DELAY_300_MS,gInverted);
	///DONE
	SetMinim();
	
	RunEb03ke04();
}
void RunEb02ke03()
{
	SetMinim();
	RunWhileNormal(5);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_200_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_FORWARD,30,30,DELAY_200_MS,gInverted);
	RunWhileNormal(5);
	
	///BUNDERAN ITEM
	DriveTurn(DIR_FORWARD,7,95,DELAY_350_MS,gInverted);
	///DONE
	
	RunWhileNormal(5);
	
	DriveTurn(DIR_FORWARD,90,20,DELAY_200_MS,gInverted);
	RunWhileNormal(10);
	
	///BLACKWHITE
	SetMinimInverse();
	
	RunWhileNormal(5);
	if (gInverted==1)
		DriveTurn(DIR_RIGHT,110,50,DELAY_300_MS,gInverted);
	else
		DriveTurn(DIR_RIGHT,120,40,DELAY_300_MS,gInverted);
	DriveTurn(DIR_FORWARD,60,60,DELAY_300_MS,gInverted);
	///DONE
	SetMinim();
	
	RunEb03ke04();
	
}
void RunEb03ke04()
{
	
	RunWhileNormal(5);
	//~ RunWhileNormal(5);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_310_MS,gInverted);
	
	DriveTurn(DIR_FORWARD,30,30,DELAY_200_MS,gInverted);
	
	RunWhileNormal(5);
	
	DriveTurn(DIR_RIGHT,50,50,DELAY_300_MS,gInverted);
	RunWhileNormal(30);
	
}



void RunFin00ke01()
{
	DriveTurn(DIR_FORWARD,30,30,DELAY_50_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_RIGHT,40,40,DELAY_300_MS,gInverted);
	DriveMove(0,0);
	_delay_ms(100);
	RunWhileNormal(5);
	
	DriveTurn(DIR_RIGHT,40,40,DELAY_300_MS,gInverted);
	RunWhileNormal(3);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_200_MS,gInverted);
	{
		uint8_t sdef = gDriveSpeed;
		gDriveSpeed = 70;
		RunWhileNormal(3);
		gDriveSpeed = sdef;
	}
	DriveTurn(DIR_FORWARD,30,30,DELAY_200_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_FORWARD,30,30,DELAY_200_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_RIGHT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(5);
	
	RunFin01ke02();
	while (ButtonIsNotPressed());
	
}
void RunFin01ke02()
{
	RunWhileNormal(5);
	DriveTurn(DIR_LEFT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(10);
	
	DriveTurn(DIR_LEFT,40,40,DELAY_300_MS,gInverted);
	RunWhileNormal(3);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(8);
	
	//~ DriveTurn(DIR_LEFT,30,30,DELAY_200_MS,gInverted);
	//~ RunWhileNormal(3);
	
	DriveTurn(DIR_RIGHT,30,30,DELAY_300_MS,gInverted);
	//~ RunWhileNormal(5);
	//~ 
	//~ DriveTurn(DIR_RIGHT,30,30,DELAY_200_MS,gInverted);
	DriveTurn(DIR_FORWARD,90,30,DELAY_200_MS,gInverted);
	
	RunWhileNormal(5);
	
	DriveTurn(DIR_RIGHT,30,30,DELAY_300_MS,gInverted);
	DriveMove(0,0);
	_delay_ms(50);
	
	RunWhileNormal(2);
	
	DriveTurn(DIR_FORWARD,30,30,DELAY_200_MS,gInverted);
	RunWhileNormal(5);
	
	RunFin02ke03();
	while(ButtonIsNotPressed());
}
void RunFin02ke03()
{
	DriveTurn(DIR_FORWARD,30,30,DELAY_200_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_RIGHT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(7);
	
	DriveTurn(DIR_LEFT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_RIGHT,30,30,DELAY_300_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_FORWARD,60,40,DELAY_100_MS,gInverted);
	RunWhileNormal(5);
	
	//~ DriveTurn(DIR_FORWARD,100,40,DELAY_200_MS,gInverted);
	//~ DriveMove(0,0);
	//~ _delay_ms(2000);
	RunFin03ke04();
	while(ButtonIsNotPressed());
}
void RunFin03ke04()
{
	RunWhileNormal(2);
	DriveTurn(DIR_RIGHT,30,30,DELAY_200_MS,gInverted);
	DriveTurn(DIR_FORWARD,30,30,DELAY_50_MS,gInverted);
	{
		uint8_t default_speed = gDriveSpeed;
		gDriveSpeed = 45;
		RunWhileNormal(10);
		gDriveSpeed = default_speed;
	}
	
	//~ DriveTurn(DIR_LEFT,60,60,DELAY_250_MS,gInverted);
	DriveMove(0,0);
	_delay_ms(50);
	if (gInverted==0)
		DriveTurn(DIR_LEFT,60,60,DELAY_100_MS,gInverted);
	else
		DriveTurn(DIR_LEFT,60,60,DELAY_200_MS,gInverted);
	SensorReadDigital();
	while (gSensH==0 && gSensL==0)
	{
		SensorReadDigital();
		if (gInverted==0)
			DirTurnLeft();
		else
			DirTurnRight();
		DriveMove(50,50);
	}
	
	RunWhileNormal(10);
	if (gInverted==0)
		DriveTurn(DIR_FORWARD,0,60,DELAY_100_MS,gInverted);
	else
		DriveTurn(DIR_FORWARD,0,60,DELAY_100_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_FORWARD,30,30,DELAY_100_MS,gInverted);
	RunWhileNormal(5);
	
	if (gInverted==0)
		DriveTurn(DIR_FORWARD,70,90,DELAY_300_MS,gInverted);
	else
		DriveTurn(DIR_FORWARD,90,70,DELAY_300_MS,gInverted);
	RunFin04ke05();
	while(ButtonIsNotPressed());
}
void RunFin04ke05()
{
	RunWhileNormal(10);
	DriveTurn(DIR_LEFT	,30,90,DELAY_100_MS,gInverted);
	DriveTurn(DIR_RIGHT	,90,30,DELAY_100_MS,gInverted);
	RunWhileNormal(5);
	
	DriveTurn(DIR_FORWARD,30,40,DELAY_50_MS,gInverted);
	RunWhileNormal(3);
	DriveTurn(DIR_RIGHT,90,20,DELAY_200_MS,gInverted);
	RunWhileNormal(5);
	DriveTurn(DIR_FORWARD,90,90,DELAY_50_MS,gInverted);
	RunWhileNormal(5);
	
	if (gInverted==0)
		DriveTurn(DIR_LEFT,90,20,DELAY_200_MS,gInverted);
	else
		DriveTurn(DIR_LEFT,90,60,DELAY_200_MS,gInverted);
	DriveMove(0,0);
	_delay_ms(100);
	RunFin05ke06();
	while(ButtonIsNotPressed());
}
void RunFin05ke06()
{
	RunWhileNormal(10);
	DriveTurn(DIR_LEFT,40,40,DELAY_300_MS,gInverted);
	
	RunWhileNormal(10);
	DriveTurn(DIR_RIGHT,40,40,DELAY_300_MS,gInverted);
	
	RunWhileNormal(5);
	DriveTurn(DIR_RIGHT,90,20,DELAY_200_MS,gInverted);
	
	RunWhileNormal(5);
	//~ DriveTurn(DIR_RIGHT	,30,90,DELAY_100_MS,gInverted);
	//~ DriveTurn(DIR_LEFT	,90,30,DELAY_100_MS,gInverted);
	DriveTurn(DIR_FORWARD, 90,90,DELAY_50_MS,gInverted);
	RunWhileNormal(5);
	DriveTurn(DIR_RIGHT,70,30,DELAY_200_MS,gInverted);
	DriveTurn(DIR_FORWARD,50,150,DELAY_100_MS,gInverted);
	
	DriveTurn(DIR_FORWARD,30,30,DELAY_100_MS,gInverted);
	
	DriveMove(0,0);
	while(ButtonIsNotPressed());
}


void RunStart01(void)
{
	
	DriveTurn(DIR_FORWARD,30,30,DELAY_300_MS,gInverted);
	
	RunWhileNormal(5);
	
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirForward();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	RunEb01ke02();
	
	while (ButtonIsNotPressed());
	
}

void RunTestCounter()
{
	_delay_ms(250);
	while (!(ButtonEnter()))
	{
		cetak_bil(countermax,0,0,4);
		if (ButtonNext())	countermax+=10;
		if (ButtonPrev())	countermax-=10;
	}
	_delay_ms(250);
	SensorReadDigital();
	while (ButtonIsNotPressed() && (counter<countermax))
	{
		counter++;
		if ((gSensH==0)&&(gSensL==0))
		{
			if (isset(gSensA,SENS_A_LEFT_BIT) && isclear(gSensA,SENS_A_RIGHT_BIT))
			{
				while ((gSensL==0)&&(gSensH==0))
				{
					SensorReadDigital();
					DirTurnLeft();
					DriveMove(30,30);
				}
			}
			else if (isset(gSensA,SENS_A_RIGHT_BIT) && isclear(gSensA,SENS_A_LEFT_BIT))
			{
				while ((gSensL==0)&&(gSensH==0))
				{
					SensorReadDigital();
					DirTurnRight();
					DriveMove(30,30);
				}
			}
		}
		if (PIDGetError())	PIDCalculateExecute();
		else if ((gSensH==0) && (gSensL==0))
		{
			DirForward();
			OCR1A=40;
			OCR1B=40;
		}
		
	}
	OCR1A=0;
	OCR1B=0;
	gState = STATE_MENU;
}
void RunInverse(void)
{
	RunFin00ke01();
	gInverted = 1;
	gState = STATE_RUN;
	return;
}
void Run()
{
	gInverted = 0;
	
	uint8_t wp_current=0;
	uint8_t wp_number =0;
	wp_number = EepromRead(ADDRESS_BLOCK_MAP_WAYPOINTS_NUM);
	while (wp_current<wp_number)
	{
		uint8_t act_l = EepromRead(ADDRESS_BLOCK_MAP_ACT_LOW	+	wp_current);
		uint8_t act_h = EepromRead(ADDRESS_BLOCK_MAP_ACT_HIGH	+	wp_current);
				
		///at this waypoint, waypoint asks robot to force drive
		DriveTurn(	MapGetActTranslate(MAP_GET_ACT_DIRECTION, act_l, act_h), 
					MapGetActTranslate(MAP_GET_ACT_PWM_LEFT, act_l, act_h), 
					MapGetActTranslate(MAP_GET_ACT_PWM_RIGHT, act_l, act_h), 
					MapGetActTranslate(MAP_GET_ACT_TIME_MS,  act_l, act_h), 
					gInverted);
		///Force drive done
		if (MapGetActTranslate(MAP_GET_ACT_NEXT_SENSOR_CONFIG, act_l, act_h)==1)
			SetComplex();
		else
			SetMinim();
		
		if (MapGetActTranslate(MAP_GET_ACT_NEXT_RUN_UNTIL,act_l, act_h)==0)
		{
			RunWhileNormal(	MapGetActTranslate(MAP_GET_ACT_NEXT_RUN_SAMPLES, act_l, act_h)	);
		}
		else
		{
			///Run until certain sensor condition detected
			counter=0;
			SensorReadDigital();
			PIDGetError();
			while (counter<MapGetActTranslate(MAP_GET_ACT_NEXT_RUN_SAMPLES, act_l, act_h)) //sampling limit
			{
				RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
				if ((!PIDGetError()))
				{
					counter++;
				}
				else
				{
					counter=0;
				}
				if (
						(gSensL == EepromRead(ADDRESS_BLOCK_MAP_UNTIL_CASE_L) ) &&
						(gSensH == EepromRead(ADDRESS_BLOCK_MAP_UNTIL_CASE_H) )
					)
				{
					counter = counter+100;
				}
				
			}
		}
		
		///back to default
		SetMinim();
		wp_current++;
	}
	
	DriveMove(0,0);
	while (ButtonIsNotPressed());
	gState = GetParent(STATE_RUN);
	gScrollNum = GetCursor(STATE_RUN);
	return;
}

void RunOnce(uint8_t speed, uint8_t kp, uint8_t kd)
{
	counter++;
	if ((gSensH==0)&&(gSensL==0))
	{
		if (isset(gSensA,SENS_A_LEFT_BIT) && isclear(gSensA,SENS_A_RIGHT_BIT))
		{
			while ((gSensL==0)&&(gSensH==0))
			{
				SensorReadDigital();
				DirTurnLeft();
				DriveMove(30,30);
			}
		}
		else if (isset(gSensA,SENS_A_RIGHT_BIT) && isclear(gSensA,SENS_A_LEFT_BIT))
		{
			while ((gSensL==0)&&(gSensH==0))
			{
				SensorReadDigital();
				DirTurnRight();
				DriveMove(30,30);
			}
		}
	}
	if (PIDGetError())	PIDCalculateExecuteCustom(speed,kp,kd);
	else //if ((gSensH==0) && (gSensL==0))
	{
		DirForward();
		OCR1A=40;
		OCR1B=40;
	}
}
uint8_t Button0()
{
	return isclear(PINC,0);
}
uint8_t Button1()
{
	return isclear(PINC,1);
}
uint8_t Button2()
{
	return isclear(PINC,2);
}
uint8_t Button3()
{
	return isclear(PINC,3);
}
uint8_t ButtonEnter()
{
	return isclear(PINC,BUTTON_ENTER_DOWN);
}
uint8_t ButtonBack()
{
	return isclear(PINC,BUTTON_BACK_DOWN);
}
uint8_t ButtonNext()
{
	return isclear(PINC,BUTTON_NEXT_DOWN);
}
uint8_t ButtonPrev()
{
	return isclear(PINC,BUTTON_PREV_DOWN);
}



void ButtonWait()
{
	while (isset(PINC,0) && isset(PINC,1) && isset(PINC,2) && isset(PINC,3));
}
uint8_t ButtonIsPressed()
{
	return (isclear(PINC,0) || isclear(PINC,1) || isclear(PINC,2) || isclear(PINC,3));
}
uint8_t ButtonIsNotPressed()
{
	return (isset(PINC,0) && isset(PINC,1) && isset(PINC,2) && isset(PINC,3));
}
uint8_t ButtonRead()
{
	uint8_t output;
	ButtonWait();
	if (isclear(PINC,0))
		output = BUTTON_0_DOWN;
	else if (isclear(PINC,1))
		output = BUTTON_1_DOWN;
	else if (isclear(PINC,2))
		output = BUTTON_2_DOWN;
	else 
		output = BUTTON_3_DOWN;
	_delay_ms(100);
	return (output);
}

void SensorReadAnalog()
{
	///Knp map sensor gak pake data yg disimpan di eeprom? untuk meningkatkan performa
	/**
	 * 
		Tampak dr depan robot
		SensL (kiri)						SensH(kanan)
		*aslinya:
		0	1	2	3	4	5	6	7		8	9	10	11	12	13	14	15
		
		*harusnya:
16 bit:	5	7	3	1	2	4	0	6		10	12	8	14	13	15	11	9
8 bit :	5	7	3	1	2	4	0	6		2	4	0	6	5	7	3	1
	*/
	/// 0	1	2	3	4	5	6	7		8	9	10	11	12	13	14	15
	/// 		VVVVVVVVVVVV
	/// 7	6	5	4	3	2	1	0		15	14	13	12	11	10	9	8
	
	gSensAnalog[16+SENS_A_LEFT_BIT] = adc_baca(ANALOG_SENSOR_LEFT_CHANNEL);
	gSensAnalog[16+SENS_A_RIGHT_BIT] = adc_baca(ANALOG_SENSOR_RIGHT_CHANNEL);
	gSensAnalog[16+SENS_A_MID_BIT] = adc_baca(ANALOG_SENSOR_MID_CHANNEL);
	
	
	///MUX 0, untuk sensor di map 0 low, 13 high
	PORTA = (PORTA & 0b11000111) | (0<<3);
	gSensAnalog[6] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	gSensAnalog[10] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	
	///MUX 1, untuk sensor di map 4 low, 8 high
	PORTA = (PORTA & 0b11000111) | (1<<3);
	gSensAnalog[3] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	gSensAnalog[15] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	
	///MUX 2, untuk sensor di map 3 low, 15 high
	PORTA = (PORTA & 0b11000111) | (2<<3);
	gSensAnalog[4] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	gSensAnalog[8] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	
	///MUX 3, untuk sensor di map 5 low, 9 high
	PORTA = (PORTA & 0b11000111) | (3<<3);
	gSensAnalog[2] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	gSensAnalog[14] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	
	///MUX 4, untuk sensor di map 2 low, 14 high
	PORTA = (PORTA & 0b11000111) | (4<<3);
	gSensAnalog[5] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	gSensAnalog[9] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	
	///MUX 5, untuk sensor di map 7 low, 11 high
	PORTA = (PORTA & 0b11000111) | (5<<3);
	gSensAnalog[0] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	gSensAnalog[12] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	
	///MUX 6, untuk sensor di map 0 low, 12 high
	PORTA = (PORTA & 0b11000111) | (6<<3);
	gSensAnalog[7] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	gSensAnalog[11] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	
	///MUX 7, untuk sensor di map 6 low, 10 high
	PORTA = (PORTA & 0b11000111) | (7<<3);
	gSensAnalog[1] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	gSensAnalog[13] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	//~ ///MUX 0, untuk sensor di map 0 low, 13 high
	//~ PORTA = (PORTA & 0b11000111) | (0<<3);
	//~ gSensAnalog[1] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	//~ gSensAnalog[13] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	//~ 
	//~ ///MUX 1, untuk sensor di map 4 low, 8 high
	//~ PORTA = (PORTA & 0b11000111) | (1<<3);
	//~ gSensAnalog[4] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	//~ gSensAnalog[8] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	//~ 
	//~ ///MUX 2, untuk sensor di map 3 low, 15 high
	//~ PORTA = (PORTA & 0b11000111) | (2<<3);
	//~ gSensAnalog[3] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	//~ gSensAnalog[15] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	//~ 
	//~ ///MUX 3, untuk sensor di map 5 low, 9 high
	//~ PORTA = (PORTA & 0b11000111) | (3<<3);
	//~ gSensAnalog[5] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	//~ gSensAnalog[9] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	//~ 
	//~ ///MUX 4, untuk sensor di map 2 low, 14 high
	//~ PORTA = (PORTA & 0b11000111) | (4<<3);
	//~ gSensAnalog[2] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	//~ gSensAnalog[14] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	//~ 
	//~ ///MUX 5, untuk sensor di map 7 low, 11 high
	//~ PORTA = (PORTA & 0b11000111) | (5<<3);
	//~ gSensAnalog[7] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	//~ gSensAnalog[11] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	//~ 
	//~ ///MUX 6, untuk sensor di map 0 low, 12 high
	//~ PORTA = (PORTA & 0b11000111) | (6<<3);
	//~ gSensAnalog[0] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	//~ gSensAnalog[12] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	//~ 
	//~ ///MUX 7, untuk sensor di map 6 low, 10 high
	//~ PORTA = (PORTA & 0b11000111) | (7<<3);
	//~ gSensAnalog[6] 	= adc_baca(ANALOG_SENSOR_LOW_CHANNEL);
	//~ gSensAnalog[10] = adc_baca(ANALOG_SENSOR_HIGH_CHANNEL);
	
	
}
void SensorReadDigital()
{
	gSensH = 0;
	gSensL = 0;
	gSensA = 0;
	uint8_t l_Num;
	SensorReadAnalog();
	
	if (gSensAnalog[16+SENS_A_LEFT_BIT]>gSensRef[16+SENS_A_LEFT_BIT])	_set(gSensA,SENS_A_LEFT_BIT);
	if (gSensAnalog[16+SENS_A_RIGHT_BIT]>gSensRef[16+SENS_A_RIGHT_BIT])	_set(gSensA,SENS_A_RIGHT_BIT);
	if (gSensAnalog[16+SENS_A_MID_BIT]>gSensRef[16+SENS_A_MID_BIT])		_set(gSensA,SENS_A_MID_BIT);
	
	for (l_Num=0;l_Num<=7;l_Num++)
	{
		if (gSensAnalog[l_Num]>gSensRef[l_Num])
		{
			_set(gSensL,l_Num);
		}
		if (gSensAnalog[l_Num+8]>gSensRef[l_Num+8])
		{
			_set(gSensH,l_Num);
		}
	}
}
void Calibrate()
{
	SensorCalibrate();
}
void SensorCalibrate()
{
	_delay_ms(250);
	uint16_t refbellow[19];
	uint16_t refupper[19];
	uint8_t num;
	LCDclr();
	for (num=0;num<=18;num++)
	{
		refbellow[num] = 1023;
		refupper[num] = 0;
	}
	while (ButtonIsNotPressed())
	{
		SensorReadAnalog();
		for (num=0;num<=15;num++)
		{
			if (gSensAnalog[num]<refbellow[num])		refbellow[num]	= gSensAnalog[num];
			if (gSensAnalog[num]>refupper[num])		refupper[num]	= gSensAnalog[num];
			gSensRef[num] = (refbellow[num]+refupper[num])/2;
		}
		if (gSensAnalog[16+SENS_A_LEFT_BIT]<refbellow[16+SENS_A_LEFT_BIT])		refbellow[16+SENS_A_LEFT_BIT]=gSensAnalog[16+SENS_A_LEFT_BIT];
		if (gSensAnalog[16+SENS_A_LEFT_BIT]>refupper[16+SENS_A_LEFT_BIT])		refupper[16+SENS_A_LEFT_BIT]=gSensAnalog[16+SENS_A_LEFT_BIT];
		gSensRef[16+SENS_A_LEFT_BIT] = (refbellow[16+SENS_A_LEFT_BIT]+refupper[16+SENS_A_LEFT_BIT])/2;
		
		if (gSensAnalog[16+SENS_A_RIGHT_BIT]<refbellow[16+SENS_A_RIGHT_BIT])	refbellow[16+SENS_A_RIGHT_BIT]=gSensAnalog[16+SENS_A_RIGHT_BIT];
		if (gSensAnalog[16+SENS_A_RIGHT_BIT]>refupper[16+SENS_A_RIGHT_BIT])		refupper[16+SENS_A_RIGHT_BIT]=gSensAnalog[16+SENS_A_RIGHT_BIT];
		gSensRef[16+SENS_A_RIGHT_BIT] = (refbellow[16+SENS_A_RIGHT_BIT]+refupper[16+SENS_A_RIGHT_BIT])/2;
		
		if (gSensAnalog[16+SENS_A_MID_BIT]<refbellow[16+SENS_A_MID_BIT])		refbellow[16+SENS_A_MID_BIT]=gSensAnalog[16+SENS_A_MID_BIT];
		if (gSensAnalog[16+SENS_A_MID_BIT]>refupper[16+SENS_A_MID_BIT])			refupper[16+SENS_A_MID_BIT]=gSensAnalog[16+SENS_A_MID_BIT];
		gSensRef[16+SENS_A_MID_BIT] = (refbellow[16+SENS_A_MID_BIT]+refupper[16+SENS_A_MID_BIT])/2;
		
		SensorReadDigital();
		for (num=0;num<=7;num++)
		{
			if (isset(gSensH,num))	cetak_bil(1,15-num-8,1,1);
			else 						cetak_bil(0,15-num-8,1,1);
			if (isset(gSensL,num))	cetak_bil(1,15-num,1,1);
			else 						cetak_bil(0,15-num,1,1);
		}
		cetak_bil(isset(gSensA,SENS_A_LEFT_BIT),7,0,1);
		cetak_bil(isset(gSensA,SENS_A_MID_BIT),8,0,1);
		cetak_bil(isset(gSensA,SENS_A_RIGHT_BIT),9,0,1);
	}
	for (int i = 0; i < 19; i++)
	{
		EepromWrite(ADDRESS_BLOCK_SENSOR_REF+i,(gSensRef[i]/4));
	}
	
	gState = STATE_MENU;
}

void SensorRemap()
{
	//~ 
	//~ 
	//~ uint8_t num,i;
	//~ uint8_t map[16];
	//~ /**	
	 //~ * Untuk map[index] = addr berisi nilai di bit mana (addr) nilai sensor dari bit (index) tersebut berada
	 //~ * misal map[4] = 2, berarti bit ke 4 gSensL seharusnya di urutan bit ke 2
	 //~ * gSensL = 000[1]0000
	 //~ * msensL = 00000[1]00
	 //~ * 
	 //~ */
	//~ uint8_t msensH=0, msensL=0;
	//~ uint8_t cursor1=0,cursor2=0;
	//~ for (i=0;i<=15;i++)
	//~ {
		//~ map[i]=i;
	//~ }
	//~ _delay_ms(250);
	//~ while (!(ButtonBack()))
	//~ {
		//~ while (!(ButtonEnter()))
		//~ {
			//~ if (ButtonNext())	cursor1++;
			//~ if (ButtonPrev())	cursor1--;
			//~ if (ButtonBack())	break;
			//~ if (cursor1>250) cursor1 = 15; //negativ overflow
			//~ else if (cursor1>15) cursor1 = 0;
			//~ SensorReadDigital();
			//~ msensH=0;msensL=0;
			//~ for (i=0;i<=15;i++)
			//~ {
				//~ if (i<=7)
				//~ {
					//~ if (isset(gSensL,i))
					//~ {
						//~ if (map[i]<=7)	_set(msensL,map[i]);
						//~ else 			_set(msensH,map[i]-8);
					//~ }
				//~ }
				//~ else
				//~ {
					//~ if (isset(gSensH,i-8))
					//~ {
						//~ if (map[i]<=7)	_set(msensL,map[i]);
						//~ else 			_set(msensH,map[i]-8);
						//~ 
					//~ }
				//~ }
			//~ }
			//~ LCDclr();
			//~ LCDGotoXY(0,1);
			//~ for (i=0;i<=7;i++)
			//~ {
				//~ if (isset(msensL,i))	cetak_bil_lgsg(1,0);	else cetak_bil_lgsg(0,0);
			//~ }
			//~ for (i=8;i<=15;i++)
			//~ {
				//~ if (isset(msensH,i-8))	cetak_bil_lgsg(1,0);	else cetak_bil_lgsg(0,0);
			//~ }
			//~ cetak_bil(1,cursor1,0,0);
			//~ if (cursor2!=cursor1)	cetak_bil(2,cursor2,0,0);
		//~ } /*End while (!(ButtonEnter())) */
		//~ _delay_ms(500);
		//~ while (!(ButtonEnter()))
		//~ {
			//~ if (ButtonNext())	cursor2++;
			//~ if (ButtonPrev())	cursor2--;
			//~ if (ButtonBack())	break;
			//~ if (cursor2>250)	cursor2 = 15; //negativ overflow
			//~ else if (cursor2>15) cursor2 = 0;
			//~ SensorReadDigital();
			//~ msensH=0;msensL=0;
			//~ for (i=0;i<=15;i++)
			//~ {
				//~ if (i<=7)
				//~ {
					//~ if (isset(gSensL,i))
					//~ {
						//~ if (map[i]<=7)	_set(msensL,map[i]);
						//~ else 			_set(msensH,map[i]-8);
					//~ }
				//~ }
				//~ else
				//~ {
					//~ if (isset(gSensH,i-8))
					//~ {
						//~ if (map[i]<=7)	_set(msensL,map[i]);
						//~ else 			_set(msensH,map[i]-8);
						//~ 
					//~ }
				//~ }
			//~ }
			//~ LCDclr();
			//~ LCDGotoXY(0,1);
			//~ for (i=0;i<=7;i++)
			//~ {
				//~ if (isset(msensL,i))	cetak_bil_lgsg(1,0);	else cetak_bil_lgsg(0,0);
			//~ }
			//~ for (i=8;i<=15;i++)
			//~ {
				//~ if (isset(msensH,i-8))	cetak_bil_lgsg(1,0);	else cetak_bil_lgsg(0,0);
			//~ }
			//~ cetak_bil(1,cursor1,0,0);
			//~ if (cursor2!=cursor1)	cetak_bil(2,cursor2,0,0);
		//~ } /**End while (!(ButtonEnter())) */
		//~ uint8_t temp;
		//~ temp = map[cursor1];
		//~ map[cursor1] = map[cursor2];
		//~ map[cursor2] = temp;
		//~ cursor1++;
		//~ if (ButtonBack())	break;
		//~ _delay_ms(500);
	//~ }/**End while(ButtonIsNotPressed())*/
	//~ 
	//~ LCDclr();
	//~ uint8_t cursor=0,dip=0;
	//~ _delay_ms(500);
	//~ while (!(ButtonBack()))
	//~ {
		//~ if (ButtonNext())	cursor++;
		//~ if (ButtonPrev())	cursor--;
		//~ if (cursor>250) cursor = 15; //negativ overflow
		//~ else if (cursor>15) cursor = 0;
//~ 
		//~ LCDGotoXY(0,1);
		//~ for (i=0;i<=7;i++)
		//~ {
			//~ if ((cursor!=i) || (dip>3))
			//~ {
				//~ if (isset(msensL,i))	cetak_bil_lgsg(1,0);	else cetak_bil_lgsg(0,0);
			//~ }
			//~ else if ((map[cursor]!=i) || (dip>3))
			//~ {
				//~ if (isset(msensL,i))	cetak_bil_lgsg(1,0);	else cetak_bil_lgsg(0,0);
			//~ }
			//~ else
			//~ {
				//~ LCDstring(" ",1);
			//~ }
		//~ }
		//~ for (i=8;i<=15;i++)
		//~ {
			//~ if ((cursor!=i) || (dip<3))
			//~ {
				//~ if (isset(msensH,i-8))	cetak_bil_lgsg(1,0);	else cetak_bil_lgsg(0,0);
			//~ }
			//~ else if ((map[cursor]!=i) || (dip>3))
			//~ {
				//~ if (isset(msensH,i-8))	cetak_bil_lgsg(1,0);	else cetak_bil_lgsg(0,0);
			//~ }
			//~ else
			//~ {
				//~ LCDstring(" ",1);
			//~ }
			//~ 
		//~ }
		//~ if (dip>3)	dip = 0;
		//~ else 		dip++;
		//~ 
		//~ cetak_bil(cursor,0,0,2);
		//~ LCDstring(" <==> ",6);
		//~ cetak_bil(map[cursor],9,0,2);
	//~ }
	 //~ /* 0 - 13
	 //~ * 1 - 15
	 //~ * 2 - 11
	 //~ * 3 - 9
	 //~ * 4 - 10
	 //~ * 
	 //~ */
}









void SaveCase()
{
	eeprom_write_byte((uint8_t*)(ADDRESS_BLOCK_CASE_N),gCaseN);
	for (uint8_t n = 1; n<=gCaseN; n++)
	{
		eeprom_write_byte((uint8_t*)(ADDRESS_BLOCK_CASE_H+n),gCaseH[n]);
		eeprom_write_byte((uint8_t*)(ADDRESS_BLOCK_CASE_L+n),gCaseL[n]);
		eeprom_write_byte((uint8_t*)(ADDRESS_BLOCK_CASE_V+n),gCaseV[n]);
	}
}
void LoadCase()
{
	gCaseN = eeprom_read_byte((uint8_t*)(ADDRESS_BLOCK_CASE_N));
	for (uint8_t n = 1; n<=gCaseN; n++)
	{
		gCaseH[n] = eeprom_read_byte((uint8_t*)(ADDRESS_BLOCK_CASE_H+n));
		gCaseL[n] = eeprom_read_byte((uint8_t*)(ADDRESS_BLOCK_CASE_L+n));
		gCaseV[n] = eeprom_read_byte((uint8_t*)(ADDRESS_BLOCK_CASE_V+n));
	}
}
void SetComplex()
{
	
	gCaseN = 32;
	gCaseH[ 0] = 0b10000000; gCaseL[ 0] = 0b00000000; gCaseV[ 0] = 15;
	gCaseH[ 1] = 0b11000000; gCaseL[ 1] = 0b00000000; gCaseV[ 1] = 14;
	gCaseH[ 2] = 0b11100000; gCaseL[ 2] = 0b00000000; gCaseV[ 2] = 13;
	gCaseH[ 3] = 0b01100000; gCaseL[ 3] = 0b00000000; gCaseV[ 3] = 12;
	gCaseH[ 4] = 0b01110000; gCaseL[ 4] = 0b00000000; gCaseV[ 4] = 11;
	gCaseH[ 5] = 0b00110000; gCaseL[ 5] = 0b00000000; gCaseV[ 5] = 10;
	gCaseH[ 6] = 0b00111000; gCaseL[ 6] = 0b00000000; gCaseV[ 6] = 9;
	gCaseH[ 7] = 0b00011000; gCaseL[ 7] = 0b00000000; gCaseV[ 7] = 8;
	gCaseH[ 8] = 0b00011100; gCaseL[ 8] = 0b00000000; gCaseV[ 8] = 7;
	gCaseH[ 9] = 0b00001100; gCaseL[ 9] = 0b00000000; gCaseV[ 9] = 6;
	gCaseH[10] = 0b00001110; gCaseL[10] = 0b00000000; gCaseV[10] = 5;
	gCaseH[11] = 0b00000110; gCaseL[11] = 0b00000000; gCaseV[11] = 4;
	gCaseH[12] = 0b00000111; gCaseL[12] = 0b00000000; gCaseV[12] = 5;
	gCaseH[13] = 0b00000011; gCaseL[13] = 0b00000000; gCaseV[13] = 2;
	gCaseH[14] = 0b00000011; gCaseL[14] = 0b10000000; gCaseV[14] = 0;
	gCaseH[15] = 0b00000001; gCaseL[15] = 0b10000000; gCaseV[15] = 0;
	gCaseH[16] = 0b00000001; gCaseL[16] = 0b11000000; gCaseV[16] = 0;
	gCaseH[17] = 0b00000000; gCaseL[17] = 0b11000000; gCaseV[17] = -2;
	gCaseH[18] = 0b00000000; gCaseL[18] = 0b11100000; gCaseV[18] = -5;
	gCaseH[19] = 0b00000000; gCaseL[19] = 0b01100000; gCaseV[19] = -4;
	gCaseH[20] = 0b00000000; gCaseL[20] = 0b01110000; gCaseV[20] = -5;
	gCaseH[21] = 0b00000000; gCaseL[21] = 0b00110000; gCaseV[21] = -6;
	gCaseH[22] = 0b00000000; gCaseL[22] = 0b00111000; gCaseV[22] = -7;
	gCaseH[23] = 0b00000000; gCaseL[23] = 0b00011000; gCaseV[23] = -8;
	gCaseH[24] = 0b00000000; gCaseL[24] = 0b00011100; gCaseV[24] = -9;
	gCaseH[25] = 0b00000000; gCaseL[25] = 0b00001100; gCaseV[25] = -10;
	gCaseH[26] = 0b00000000; gCaseL[26] = 0b00001110; gCaseV[26] = -11;
	gCaseH[27] = 0b00000000; gCaseL[27] = 0b00000110; gCaseV[27] = -12;
	gCaseH[28] = 0b00000000; gCaseL[28] = 0b00000111; gCaseV[28] = -13;
	gCaseH[29] = 0b00000000; gCaseL[29] = 0b00000011; gCaseV[29] = -14;
	gCaseH[30] = 0b00000000; gCaseL[30] = 0b00000001; gCaseV[30] = -15;
	gCaseH[31] = 0b01111000; gCaseL[31] = 0b00000000; gCaseV[31] = 13;
	gCaseH[32] = 0b00000000; gCaseL[32] = 0b00011110; gCaseV[32] = -13;
	
	
	
	//~ gCaseH[33] = 0b00000111; gCaseL[33] = 0b10000000; gCaseV[33] = 12;
	//~ gCaseH[34] = 0b00000001; gCaseL[34] = 0b11100000; gCaseV[34] = -12;
	//~ gCaseH[35] = 0b00001111; gCaseL[35] = 0b00000000; gCaseV[35] = 12;
	//~ gCaseH[36] = 0b00000000; gCaseL[36] = 0b11110000; gCaseV[36] = -12;
	//~ gCaseH[37] = 0b00001111; gCaseL[37] = 0b10000000; gCaseV[37] = 12;
	//~ gCaseH[38] = 0b00000001; gCaseL[38] = 0b11110000; gCaseV[38] = -12;
	//~ gCaseH[39] = 0b00011111; gCaseL[39] = 0b10000000; gCaseV[39] = 14;
	//~ gCaseH[40] = 0b00000001; gCaseL[40] = 0b11111000; gCaseV[40] = -14;
	
	//~ gCaseH[ 0] = 0b00000000; gCaseL[ 0] = 0b00000000; gCaseV[ 0] = 8;
	//~ gCaseH[ 0] = 0b00000000; gCaseL[ 0] = 0b00000000; gCaseV[ 1] = 7;
	//~ gCaseH[ 2] = 0b00000000; gCaseL[ 2] = 0b00000000; gCaseV[ 2] = 6;
	//~ gCaseH[ 3] = 0b00000000; gCaseL[ 3] = 0b00000000; gCaseV[ 3] = 5;
	//~ gCaseH[ 4] = 0b00000000; gCaseL[ 4] = 0b00000000; gCaseV[ 4] = 4;
	//~ gCaseH[ 5] = 0b00000000; gCaseL[ 5] = 0b00000000; gCaseV[ 5] = 3;
	//~ gCaseH[ 6] = 0b00000000; gCaseL[ 6] = 0b00000000; gCaseV[ 6] = 2;
	//~ gCaseH[ 7] = 0b00000000; gCaseL[ 7] = 0b00000000; gCaseV[ 7] = 1;
	//~ gCaseH[ 8] = 0b00000000; gCaseL[ 8] = 0b00000000; gCaseV[ 8] = 0;
	//~ gCaseH[ 9] = 0b00000000; gCaseL[ 9] = 0b00000000; gCaseV[ 9] = -1;
	//~ gCaseH[10] = 0b00000000; gCaseL[10] = 0b00000000; gCaseV[10] = -2;
	//~ gCaseH[11] = 0b00000000; gCaseL[11] = 0b00000000; gCaseV[11] = -3;
	//~ gCaseH[12] = 0b00000000; gCaseL[12] = 0b00000000; gCaseV[12] = -4;
	//~ gCaseH[13] = 0b00000000; gCaseL[13] = 0b00000000; gCaseV[13] = -5;
	//~ gCaseH[14] = 0b00000000; gCaseL[14] = 0b00000000; gCaseV[14] = -6;
	//~ gCaseH[15] = 0b00000000; gCaseL[15] = 0b00000000; gCaseV[15] = -7;
	//~ gCaseH[16] = 0b00000000; gCaseL[16] = 0b00000001; gCaseV[16] = -8;
	//~ gCaseH[17] = 0b00000000; gCaseL[17] = 0b00000000; gCaseV[17] = 00;
	//~ gCaseH[18] = 0b00000000; gCaseL[18] = 0b00000000; gCaseV[18] = 00;
	//~ gCaseH[19] = 0b00000000; gCaseL[19] = 0b00000000; gCaseV[19] = 00;
	//~ gCaseH[20] = 0b00000000; gCaseL[20] = 0b00000000; gCaseV[20] = 00;
	//~ gCaseH[21] = 0b00000000; gCaseL[21] = 0b00000000; gCaseV[21] = 00;
}
void SetMinim()
{
	gCaseN = 44;
	gCaseH[ 0] = 0b10000000; gCaseL[ 0] = 0b00000000; gCaseV[ 0] = 15;
	gCaseH[ 1] = 0b11000000; gCaseL[ 1] = 0b00000000; gCaseV[ 1] = 14;
	gCaseH[ 2] = 0b11100000; gCaseL[ 2] = 0b00000000; gCaseV[ 2] = 14;
	gCaseH[ 3] = 0b01000000; gCaseL[ 3] = 0b00000000; gCaseV[ 3] = 14;
	gCaseH[ 4] = 0b01100000; gCaseL[ 4] = 0b00000000; gCaseV[ 4] = 12;
	gCaseH[ 5] = 0b01110000; gCaseL[ 5] = 0b00000000; gCaseV[ 5] = 12;
	gCaseH[ 6] = 0b00100000; gCaseL[ 6] = 0b00000000; gCaseV[ 6] = 12;
	gCaseH[ 7] = 0b00110000; gCaseL[ 7] = 0b00000000; gCaseV[ 7] = 10;
	gCaseH[ 8] = 0b00111000; gCaseL[ 8] = 0b00000000; gCaseV[ 8] = 10;
	gCaseH[ 9] = 0b00010000; gCaseL[ 9] = 0b00000000; gCaseV[ 9] = 10;
	gCaseH[10] = 0b00011000; gCaseL[10] = 0b00000000; gCaseV[10] = 8;
	gCaseH[11] = 0b00011100; gCaseL[11] = 0b00000000; gCaseV[11] = 8;
	gCaseH[12] = 0b00001000; gCaseL[12] = 0b00000000; gCaseV[12] = 8;
	gCaseH[13] = 0b00001100; gCaseL[13] = 0b00000000; gCaseV[13] = 6;
	gCaseH[14] = 0b00001110; gCaseL[14] = 0b00000000; gCaseV[14] = 0;
	gCaseH[14] = 1; gCaseL[14] = 0; gCaseV[14] = 0;
	gCaseH[15] = 0b00000100; gCaseL[15] = 0b00000000; gCaseV[15] = 6;
	gCaseH[16] = 0b00000110; gCaseL[16] = 0b00000000; gCaseV[16] = 4;
	gCaseH[17] = 0b00000111; gCaseL[17] = 0b00000000; gCaseV[17] = 0;
	gCaseH[17] = 1; gCaseL[17] = 0; gCaseV[17] = 0;
	gCaseH[18] = 0b00000010; gCaseL[18] = 0b00000000; gCaseV[18] = 2;
	gCaseH[19] = 0b00000011; gCaseL[19] = 0b00000000; gCaseV[19] = 0;
	gCaseH[20] = 0b00000011; gCaseL[20] = 0b10000000; gCaseV[20] = 0;
	gCaseH[20] = 1; gCaseL[20] = 0; gCaseV[20] = 0;
	gCaseH[21] = 0b00000001; gCaseL[21] = 0b00000000; gCaseV[21] = 0;
	gCaseH[22] = 0b00000001; gCaseL[22] = 0b10000000; gCaseV[22] = 0;
	gCaseH[23] = 0b00000001; gCaseL[23] = 0b11000000; gCaseV[23] = 0;
	gCaseH[23] = 1; gCaseL[23] = 0; gCaseV[23] = 0;
	gCaseH[24] = 0b00000000; gCaseL[24] = 0b10000000; gCaseV[24] = 0;
	gCaseH[25] = 0b00000000; gCaseL[25] = 0b11000000; gCaseV[25] = 0;
	gCaseH[26] = 0b00000000; gCaseL[26] = 0b11100000; gCaseV[26] = 0;
	gCaseH[26] = 1; gCaseL[26] = 0; gCaseV[26] = 0;
	gCaseH[27] = 0b00000000; gCaseL[27] = 0b01000000; gCaseV[27] = -2;
	gCaseH[28] = 0b00000000; gCaseL[28] = 0b01100000; gCaseV[28] = -4;
	gCaseH[29] = 0b00000000; gCaseL[29] = 0b01110000; gCaseV[29] = 0;
	gCaseH[29] = 1; gCaseL[29] = 0; gCaseV[29] = 0;
	gCaseH[30] = 0b00000000; gCaseL[30] = 0b00100000; gCaseV[30] = -6;
	gCaseH[31] = 0b00000000; gCaseL[31] = 0b00110000; gCaseV[31] = -6;
	gCaseH[32] = 0b00000000; gCaseL[32] = 0b00111000; gCaseV[32] = -8;
	gCaseH[33] = 0b00000000; gCaseL[33] = 0b00010000; gCaseV[33] = -8;
	gCaseH[34] = 0b00000000; gCaseL[34] = 0b00011000; gCaseV[34] = -8;
	gCaseH[35] = 0b00000000; gCaseL[35] = 0b00011100; gCaseV[35] = -10;
	gCaseH[36] = 0b00000000; gCaseL[36] = 0b00001000; gCaseV[36] = -10;
	gCaseH[37] = 0b00000000; gCaseL[37] = 0b00001100; gCaseV[37] = -10;
	gCaseH[38] = 0b00000000; gCaseL[38] = 0b00001110; gCaseV[38] = -12;
	gCaseH[39] = 0b00000000; gCaseL[39] = 0b00000100; gCaseV[39] = -12;
	gCaseH[40] = 0b00000000; gCaseL[40] = 0b00000110; gCaseV[40] = -12;
	gCaseH[41] = 0b00000000; gCaseL[41] = 0b00000111; gCaseV[41] = -14;
	gCaseH[42] = 0b00000000; gCaseL[42] = 0b00000010; gCaseV[42] = -14;
	gCaseH[43] = 0b00000000; gCaseL[43] = 0b00000011; gCaseV[43] = -14;
	gCaseH[44] = 0b00000000; gCaseL[44] = 0b00000001; gCaseV[44] = -15;
/**
	gCaseN = 30;
	gCaseH[ 0] = 0b10000000; gCaseL[ 0] = 0b00000000; gCaseV[ 0] = 15;
	gCaseH[ 1] = 0b11000000; gCaseL[ 1] = 0b00000000; gCaseV[ 1] = 14;
	gCaseH[ 2] = 0b01000000; gCaseL[ 2] = 0b00000000; gCaseV[ 2] = 14;
	gCaseH[ 3] = 0b01100000; gCaseL[ 3] = 0b00000000; gCaseV[ 3] = 12;
	gCaseH[ 4] = 0b00100000; gCaseL[ 4] = 0b00000000; gCaseV[ 4] = 12;
	gCaseH[ 5] = 0b00110000; gCaseL[ 5] = 0b00000000; gCaseV[ 5] = 10;
	gCaseH[ 6] = 0b00010000; gCaseL[ 6] = 0b00000000; gCaseV[ 6] = 10;
	gCaseH[ 7] = 0b00011000; gCaseL[ 7] = 0b00000000; gCaseV[ 7] = 8;
	gCaseH[ 8] = 0b00001000; gCaseL[ 8] = 0b00000000; gCaseV[ 8] = 8;
	gCaseH[ 9] = 0b00001100; gCaseL[ 9] = 0b00000000; gCaseV[ 9] = 6;
	gCaseH[10] = 0b00000100; gCaseL[10] = 0b00000000; gCaseV[10] = 6;
	gCaseH[11] = 0b00000110; gCaseL[11] = 0b00000000; gCaseV[11] = 4;
	gCaseH[12] = 0b00000010; gCaseL[12] = 0b00000000; gCaseV[12] = 4;
	gCaseH[13] = 0b00000011; gCaseL[13] = 0b00000000; gCaseV[13] = 2;
	gCaseH[14] = 0b00000001; gCaseL[14] = 0b00000000; gCaseV[14] = 0;
	gCaseH[15] = 0b00000001; gCaseL[15] = 0b10000000; gCaseV[15] = 0;
	gCaseH[16] = 0b00000000; gCaseL[16] = 0b10000000; gCaseV[16] = 0;
	gCaseH[17] = 0b00000000; gCaseL[17] = 0b11000000; gCaseV[17] = -2;
	gCaseH[18] = 0b00000000; gCaseL[18] = 0b01000000; gCaseV[18] = -4;
	gCaseH[19] = 0b00000000; gCaseL[19] = 0b01100000; gCaseV[19] = -4;
	gCaseH[20] = 0b00000000; gCaseL[20] = 0b00100000; gCaseV[20] = -6;
	gCaseH[21] = 0b00000000; gCaseL[21] = 0b00110000; gCaseV[21] = -6;
	gCaseH[22] = 0b00000000; gCaseL[22] = 0b00010000; gCaseV[22] = -8;
	gCaseH[23] = 0b00000000; gCaseL[23] = 0b00011000; gCaseV[23] = -8;
	gCaseH[24] = 0b00000000; gCaseL[24] = 0b00001000; gCaseV[24] = -10;
	gCaseH[25] = 0b00000000; gCaseL[25] = 0b00001100; gCaseV[25] = -10;
	gCaseH[26] = 0b00000000; gCaseL[26] = 0b00000100; gCaseV[26] = -12;
	gCaseH[27] = 0b00000000; gCaseL[27] = 0b00000110; gCaseV[27] = -12;
	gCaseH[28] = 0b00000000; gCaseL[28] = 0b00000010; gCaseV[28] = -14;
	gCaseH[29] = 0b00000000; gCaseL[29] = 0b00000011; gCaseV[29] = -14;
	gCaseH[30] = 0b00000000; gCaseL[30] = 0b00000001; gCaseV[30] = -15;
**/
	
}
void SetMinimInverse()
{
	
	gCaseN = 30;
	gCaseH[ 0] = 0b01111111; gCaseL[ 0] = 0b11111111; gCaseV[ 0] = 15;
	gCaseH[ 1] = 0b00111111; gCaseL[ 1] = 0b11111111; gCaseV[ 1] = 14;
	gCaseH[ 2] = 0b10111111; gCaseL[ 2] = 0b11111111; gCaseV[ 2] = 14;
	gCaseH[ 3] = 0b10011111; gCaseL[ 3] = 0b11111111; gCaseV[ 3] = 12;
	gCaseH[ 4] = 0b11011111; gCaseL[ 4] = 0b11111111; gCaseV[ 4] = 12;
	gCaseH[ 5] = 0b11001111; gCaseL[ 5] = 0b11111111; gCaseV[ 5] = 10;
	gCaseH[ 6] = 0b11101111; gCaseL[ 6] = 0b11111111; gCaseV[ 6] = 10;
	gCaseH[ 7] = 0b11100111; gCaseL[ 7] = 0b11111111; gCaseV[ 7] = 8;
	gCaseH[ 8] = 0b11110111; gCaseL[ 8] = 0b11111111; gCaseV[ 8] = 8;
	gCaseH[ 9] = 0b11110011; gCaseL[ 9] = 0b11111111; gCaseV[ 9] = 6;
	gCaseH[10] = 0b11111011; gCaseL[10] = 0b11111111; gCaseV[10] = 6;
	gCaseH[11] = 0b11111001; gCaseL[11] = 0b11111111; gCaseV[11] = 4;
	gCaseH[12] = 0b11111101; gCaseL[12] = 0b11111111; gCaseV[12] = 4;
	gCaseH[13] = 0b11111100; gCaseL[13] = 0b11111111; gCaseV[13] = 2;
	gCaseH[14] = 0b11111110; gCaseL[14] = 0b11111111; gCaseV[14] = 2;
	gCaseH[15] = 0b11111110; gCaseL[15] = 0b01111111; gCaseV[15] = 0;
	gCaseH[16] = 0b11111111; gCaseL[16] = 0b01111111; gCaseV[16] = 0;
	gCaseH[17] = 0b11111111; gCaseL[17] = 0b00111111; gCaseV[17] = -2;
	gCaseH[18] = 0b11111111; gCaseL[18] = 0b10111111; gCaseV[18] = -2;
	gCaseH[19] = 0b11111111; gCaseL[19] = 0b10011111; gCaseV[19] = -4;
	gCaseH[20] = 0b11111111; gCaseL[20] = 0b11011111; gCaseV[20] = -4;
	gCaseH[21] = 0b11111111; gCaseL[21] = 0b11001111; gCaseV[21] = -6;
	gCaseH[22] = 0b11111111; gCaseL[22] = 0b11101111; gCaseV[22] = -6;
	gCaseH[23] = 0b11111111; gCaseL[23] = 0b11100111; gCaseV[23] = -8;
	gCaseH[24] = 0b11111111; gCaseL[24] = 0b11110111; gCaseV[24] = -8;
	gCaseH[25] = 0b11111111; gCaseL[25] = 0b11110011; gCaseV[25] = -10;
	gCaseH[26] = 0b11111111; gCaseL[26] = 0b11111011; gCaseV[26] = -10;
	gCaseH[27] = 0b11111111; gCaseL[27] = 0b11111001; gCaseV[27] = -12;
	gCaseH[28] = 0b11111111; gCaseL[28] = 0b11111101; gCaseV[28] = -12;
	gCaseH[29] = 0b11111111; gCaseL[29] = 0b11111100; gCaseV[29] = -14;
	gCaseH[30] = 0b11111111; gCaseL[30] = 0b11111110; gCaseV[30] = -15;
}



uint8_t PIDGetError()
{
	///GET gError
	SensorReadDigital();
	for (uint8_t n = 0; n<=gCaseN; n++)
	{
		if (gSensH==gCaseH[n] && gSensL==gCaseL[n])
		{
			gDriveError = gCaseV[n];
			return 1;
		}
	}
	return 0;
}

void PIDCalculateExecute()
{
	int p,d;
	uint8_t varka,varki,pwm_ka,pwm_ki;
	p = gDriveKP * gDriveError;
	d = gDriveKD * (gDriveError - gDriveLastError);
	gDriveSpeedLeft = gDriveSpeed - p - d;
	gDriveSpeedRight = gDriveSpeed + p + d;
	
	if (gDriveSpeedRight>gDriveLimit)
	{
		varka = gDriveSpeedRight - gDriveLimit;
		varka = gDriveLimit;
	}
	else
	{
		varka = 0;
	}
	
	if (gDriveSpeedLeft>gDriveLimit)
	{
		varki = gDriveSpeedLeft - gDriveLimit;
		varki = gDriveLimit;
	}
	else
	{
		varki = 0;
	}
	
   pwm_ka = gDriveSpeedRight - varki;
   pwm_ki = gDriveSpeedLeft  - varka;                           
   //~ pwm_ka = gDriveSpeedRight;// - varki;
   //~ pwm_ki = gDriveSpeedLeft ;// - varka;                           

   gDriveLastError = gDriveError; 
   /// FOLLOWING LINE VARIABLES IS TO BE RECONSIDERED 
   /*if ((gDriveSpeedLeft>=0)&&(gDriveSpeedRight>=0))	
   {
	   DirForward();
   }
   if ((gDriveSpeedLeft>=0)&&(gDriveSpeedRight<0 ))	
   {
	   DirTurnRight();
	   //~ gDriveSpeedRight = abs(gDriveSpeedRight);//? courtesy joe
   }
   if ((gDriveSpeedLeft<0)&&(gDriveSpeedRight>=0 ))	
   {
	   DirTurnLeft();
	   //~ gDriveSpeedLeft = abs(gDriveSpeedLeft); 	//? courtesy joe
   }*/
   if ((gDriveSpeedLeft>=0)&&(gDriveSpeedRight>=0))	
   {
	   DirForward();
   }
   if ((gDriveSpeedLeft>=0)&&(gDriveSpeedRight<0 ))	
   {
	   DirTurnRight();
	   //~ pwm_ka = varki - gDriveSpeedRight;
   }
   if ((gDriveSpeedLeft<0)&&(gDriveSpeedRight>=0 ))	
   {
	   DirTurnLeft();
	   //~ pwm_ki = varka - gDriveSpeedLeft;
   }
   if (pwm_ka>gDriveLimit)
   {
	   pwm_ka = gDriveLimit;
   }
   if (pwm_ki>gDriveLimit)
   {
	   pwm_ki = gDriveLimit;
   }
   OCR_RIGHT = pwm_ka;
   OCR_LEFT = pwm_ki;

}
void PIDCalculateExecuteCustom(uint8_t speed, uint8_t kp, uint8_t kd)
{
	int p,d;
	uint8_t varka,varki,pwm_ka,pwm_ki;
	p = kp * gDriveError;
	d = kd * (gDriveError - gDriveLastError);
	gDriveSpeedLeft = speed - p - d;
	gDriveSpeedRight = speed + p + d;
	
	if (gDriveSpeedRight>gDriveLimit)
	{
		varka = gDriveSpeedRight - gDriveLimit;
		varka = gDriveLimit;
	}
	else
	{
		varka = 0;
	}
	
	if (gDriveSpeedLeft>gDriveLimit)
	{
		varki = gDriveSpeedLeft - gDriveLimit;
		varki = gDriveLimit;
	}
	else
	{
		varki = 0;
	}
	
   pwm_ka = gDriveSpeedRight - varki;
   pwm_ki = gDriveSpeedLeft  - varka;                           
   //~ pwm_ka = gDriveSpeedRight;// - varki;
   //~ pwm_ki = gDriveSpeedLeft ;// - varka;                           

   gDriveLastError = gDriveError; 
   /// FOLLOWING LINE VARIABLES IS TO BE RECONSIDERED 
   if ((gDriveSpeedLeft>=0)&&(gDriveSpeedRight>=0))	
   {
	   DirForward();
   }
   if ((gDriveSpeedLeft>=0)&&(gDriveSpeedRight<0 ))	
   {
	   DirTurnRight();
	   //~ pwm_ka = varki - gDriveSpeedRight;
   }
   if ((gDriveSpeedLeft<0)&&(gDriveSpeedRight>=0 ))	
   {
	   DirTurnLeft();
	   //~ pwm_ki = varka - gDriveSpeedLeft;
   }
   if (pwm_ka>gDriveLimit)
   {
	   pwm_ka = gDriveLimit;
   }
   if (pwm_ki>gDriveLimit)
   {
	   pwm_ki = gDriveLimit;
   }
   OCR_RIGHT = pwm_ka;
   OCR_LEFT = pwm_ki;

}

 // 2 bit driver command: 10 forward, 00 right, 01 backward, 11 left
void DirForward()
{
	PORTD = (PORTD & 0b11110011) | 0b00000100;
}
void DirBackward()
{
	PORTD = (PORTD & 0b11110011) | 0b00001000;
}
void DirTurnLeft()
{
	PORTD = (PORTD & 0b11110011) | 0b00001100;
}
void DirTurnRight()
{
	PORTD = (PORTD & 0b11110011) | 0b00000000;
}
void DriveMove(uint8_t pwm_ki, uint8_t pwm_ka)
{
	OCR_RIGHT= pwm_ka;
	OCR_LEFT = pwm_ki;
}

void DriveTurn(uint8_t dir, uint8_t pwm_left, uint8_t pwm_right, uint8_t time_ms, uint8_t inverted_turn)
{
	if (inverted_turn!=0)
	{
		if (dir == DRIVE_LEFT)
			dir = DRIVE_RIGHT;
		else if (dir == DRIVE_RIGHT)
			dir = DRIVE_LEFT;
	}
	
	if (dir == DRIVE_LEFT)		DirTurnLeft();
	else if (dir == DRIVE_RIGHT)	DirTurnRight();
	else if (dir == DRIVE_FORWARD)	DirForward();
	else if (dir == DRIVE_BACKWARD)	DirBackward();
	
	if (inverted_turn!=0)
	{
		DriveMove(pwm_right,pwm_left);
	}
	else
	{
		DriveMove(pwm_left,pwm_right);
	}
	
	if (time_ms == DELAY_50_MS)
		_delay_ms(50);
	else if (time_ms == DELAY_100_MS)
		_delay_ms(100);
	else if (time_ms == DELAY_200_MS)
		_delay_ms(200);
	else if (time_ms == DELAY_220_MS)
		_delay_ms(220);
	else if (time_ms == DELAY_250_MS)
		_delay_ms(250);
	else if (time_ms == DELAY_300_MS)
		_delay_ms(300);
	else if (time_ms == DELAY_400_MS)
		_delay_ms(400);
	else if (time_ms == DELAY_500_MS)
		_delay_ms(500);
	else if (time_ms == DELAY_600_MS)
		_delay_ms(600);
	else if (time_ms == DELAY_700_MS)
		_delay_ms(700);
	else if (time_ms == DELAY_350_MS)
		_delay_ms(350);
	else if (time_ms == DELAY_310_MS)
		_delay_ms(310);
	
	return;
}


uint8_t MapGetAct(uint8_t act, uint8_t act_low, uint8_t act_high)
{
	///Direction is in bit number 14,15
	if (act == MAP_GET_ACT_DIRECTION)
		return ( (act_high & 0b11000000)>>6);
	
	if (act == MAP_GET_ACT_PWM_RIGHT)
		return ( (act_high & 0b00111000)>>3);
		
	if (act == MAP_GET_ACT_PWM_LEFT)
		return ( (act_high & 0b00000111)>>0);
	
	if (act == MAP_GET_ACT_TIME_MS)
		return ( (act_low  & 0b11100000)>>5);
		
	if (act == MAP_GET_ACT_NEXT_RUN_SAMPLES)
		return ( (act_low  & 0b00011100)>>2);
		
	if (act == MAP_GET_ACT_NEXT_SENSOR_CONFIG)
		return ( (act_low  & 0b00000010)>>1);
		
	if (act == MAP_GET_ACT_NEXT_RUN_UNTIL)
		return ( (act_low  & 0b00000001)>>0);
}
uint8_t MapGetActTranslate(uint8_t act, uint8_t act_low, uint8_t act_high)
{
	uint8_t value;
	if (act == MAP_GET_ACT_DIRECTION) 
		return ( (act_high & 0b11000000)>>6); ///Already same act constant
	
	if (act == MAP_GET_ACT_PWM_RIGHT)
	{
		value = ( (act_high & 0b00111000)>>3);
		if (value==0) 	return 0;
		if (value==1) 	return 5;
		if (value==2) 	return 10;
		if (value==3) 	return 20;
		if (value==4) 	return 30;
		if (value==5) 	return 40;
		if (value==6) 	return 60;
		if (value==7) 	return 80;
		return 0; ///dummy
	}	
	
	if (act == MAP_GET_ACT_PWM_LEFT)
	{
		value = ( (act_high & 0b00000111)>>0);
		if (value==0) 	return 0;
		if (value==1) 	return 5;
		if (value==2) 	return 10;
		if (value==3) 	return 20;
		if (value==4) 	return 30;
		if (value==5) 	return 40;
		if (value==6) 	return 60;
		if (value==7) 	return 80;
		return 0; ///dummy
	}
	
	if (act == MAP_GET_ACT_TIME_MS)
	{
		return ( (act_low  & 0b11100000)>>5);
		///Time ms translated into its number, not miliseconds
	}	
	
	if (act == MAP_GET_ACT_NEXT_RUN_SAMPLES)
	{
		value = ( (act_low  & 0b00011100)>>2);
		if (value==0) 	return 0;
		if (value==1) 	return 5;
		if (value==2) 	return 10;
		if (value==3) 	return 15;
		if (value==4) 	return 20;
		if (value==5) 	return 30;
		if (value==6) 	return 40;
		if (value==7) 	return 60;
		return 0; ///dummy
	}
	
	if (act == MAP_GET_ACT_NEXT_SENSOR_CONFIG)
	{
		return ( (act_low  & 0b00000010)>>1);
	}	
	
	if (act == MAP_GET_ACT_NEXT_RUN_UNTIL)
	{
		return ( (act_low  & 0b00000001)>>0);
	}
}


//~ void RunMapping(void)
//~ {
	//~ while (1)
	//~ {
		//~ if (PIDGetError())
		//~ { //NORMAL CASE
			//~ PIDCalculateExecute();
		//~ }
		//~ else
		//~ {
			//~ //NOTE EEPROM drive to 1024, needs 16bit
			//~ uint16_t actturn;
			//~ uint16_t blockcaseH=ADDRESS_BLOCK_MAP_CASE_H+gMazeMapNum;
			//~ uint16_t blockcaseL=ADDRESS_BLOCK_MAP_CASE_L+gMazeMapNum;
			//~ while ((EepromRead(blockcaseH)!=gSensH)&&(EepromRead(blockcaseL)!=gSensL)&&(MapReadCaseA(gMazeMapNum)==gSensA))
			//~ {
				//~ SensorReadDigital();
			//~ }
			//~ actturn = MapReadTurn(gMazeMapNum);
			//~ if (actturn==MAP_ACT_TURN_FORWARD)	DirForward();
			//~ else if (actturn==MAP_ACT_TURN_BACKWARD) DirBackward();
			//~ else if (actturn==MAP_ACT_TURN_LEFT)	DirTurnLeft();
			//~ else if (actturn==MAP_ACT_TURN_RIGHT)	DirTurnRight();
			//~ uint8_t kp;
			//~ uint8_t kd;
			//~ uint8_t speed;
			//~ 
			//~ kp = EepromRead(ADDRESS_BLOCK_MAP_ACT_KP+gMazeMapNum);
			//~ kd = EepromRead(ADDRESS_BLOCK_MAP_ACT_KD+gMazeMapNum);
			//~ speed = EepromRead(ADDRESS_BLOCK_MAP_ACT_SPEED+gMazeMapNum);
			//~ if (kp==255)	kp=gDriveKP;///255 means no change/default
			//~ if (kd==255)	kd=gDriveKD;
			//~ if (speed==255)	speed=gDriveSpeed;
			//~ 
			//~ blockcaseH=ADDRESS_BLOCK_MAP_UNTIL_H+gMazeMapNum;
			//~ blockcaseL=ADDRESS_BLOCK_MAP_UNTIL_L+gMazeMapNum;
			//~ while ((EepromRead(blockcaseH)!=gSensH)&&(EepromRead(blockcaseL)!=gSensL)&&(MapReadCaseA(gMazeMapNum)==gSensA))
			//~ {
				//~ PIDGetError();
				//~ PIDCalculateExecuteCustom(speed,kp,kd);
				//~ SensorReadDigital();
			//~ }
			//~ ///RAMPUNG
			//~ gMazeMapNum++;
		//~ }
	//~ }
//~ }

void ProgMapping(void)
{
	///
}

/** Deprecated
void MapWriteTurn(uint8_t queue_num, uint8_t turn)
{
	uint8_t tcaua;
	tcaua = EepromRead(ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+queue_num);
	tcaua = tcaua & 0b11111100;
	tcaua = tcaua | turn;
	EepromWrite(ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+queue_num,tcaua);
}
uint8_t MapReadTurn(uint8_t queue_num)
{
	uint8_t tcaua;
	tcaua = EepromRead(ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+queue_num);
	tcaua = tcaua & 0b00000011;
	return tcaua;
}
void MapWriteCaseA(uint8_t queue_num, uint8_t casea)
{
	uint8_t tcaua;
	tcaua = EepromRead(ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+queue_num);
	tcaua = tcaua & 0b00011111;
	tcaua = tcaua | (casea<<5);
	EepromWrite(ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+queue_num,tcaua);
}
void MapWriteUntilA(uint8_t queue_num, uint8_t untila)
{
	uint8_t tcaua;
	tcaua = EepromRead(ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+queue_num);
	tcaua = tcaua & 0b11100011;
	tcaua = tcaua | (untila<<2);
	EepromWrite(ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+queue_num,tcaua);
}
uint8_t MapReadCaseA(uint8_t queue_num)
{
	uint8_t tcaua;
	tcaua = EepromRead(ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+queue_num);
	tcaua = tcaua & 0b11100000;
	tcaua = (tcaua>>5);
	return tcaua;
}
uint8_t MapReadUntilA(uint8_t queue_num)
{
	uint8_t tcaua;
	tcaua = EepromRead(ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+queue_num);
	tcaua = tcaua & 0b00011100;
	tcaua = (tcaua>>2);
	return tcaua;
}
void MapWriteTCAUA(uint8_t queue_num, uint8_t tcaua)
{
	EepromWrite(ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+queue_num,tcaua);
}
uint8_t MapReadTCAUA(uint8_t queue_num)
{
	return EepromRead(ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+queue_num);
}
**/
uint8_t GetCursor(uint16_t numstate)
{
	if (numstate>=20)
		return (numstate%10);
	return numstate;
}
uint8_t GetParent(uint16_t numstate)
{
	if (numstate<20)
		return 0;///its parent is menu
	return (numstate/10);
}
uint8_t GetChild(uint8_t parent, uint8_t childid)
{
	return (parent*10)+childid;
}

void ReStrainScroll()
{
	if (gScrollNum<1)
	{
		gScrollNum = gScrollMax;
	}
	else if (gScrollNum>gScrollMax)
	{
		gScrollNum = 1;
	}
}

void PrintScroll(uint8_t sNum, uint8_t sMax)
{
	uint8_t mulai = 0;
	uint8_t nScroll = 1;
	lcdhapus(0,15,1);
	mulai = 8-(sMax/2);
	LCDGotoXY(mulai,1);
	for (nScroll = 1; nScroll<=sMax; nScroll++)
	{
		if (nScroll == sNum)
		{
			LCDstring("#",1);
		}
		else
		{
			LCDstring("-",1);
		}
	}
}

void Menu()
{
	gScrollMax = MAX_SCROLL_MENU;
	ReStrainScroll();
	while (gState == STATE_MENU)
	{		
		ReStrainScroll();
		PrintScroll(gScrollNum,gScrollMax);
		if (gScrollNum == GetCursor(STATE_RUN))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("RUN           0B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_INVERSE))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("RUN INVERSE   0B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_INVERSE;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("RUN MAPPING   0B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING_C1))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MAPPING C0    B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING_C1;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING_C2))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MAPPING C1    B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING_C2;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING_C3))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MAPPING C2    B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING_C3;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING_C4))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MAPPING C3    B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING_C4;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING_F0))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MAPPING F0    B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING_F0;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING_F1))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MAPPING F1    B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING_F1;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING_F2))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MAPPING F2    B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING_F2;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING_F3))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MAPPING F3    B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING_F3;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING_F4))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MAPPING F4    B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING_F4;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING_F5))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MAPPING F5    B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING_F5;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		
		else if (gScrollNum == GetCursor(STATE_RUN_TEST_COUNTER))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("RUN TEST CTR   B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_TEST_COUNTER;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		
		else if (gScrollNum == GetCursor(STATE_CALIBRATE))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("CALIBRATE     0B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_CALIBRATE;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_SETTING_MENU))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("SETTING       0B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_MENU;
				gScrollNum = 1;
			}
			
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_MAPPING_EBOTEC))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("RUN M. EBOTEC 0B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_MAPPING_EBOTEC;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_RUN_LEARN))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("RUN LEARN      B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_RUN_LEARN;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		else if (gScrollNum == GetCursor(STATE_SET_INVERSE))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("INVERSE TOGGLE B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SET_INVERSE;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
		}
		
	}
}
void SetInverse()
{
	gScrollMax = 2;
	gScrollNum = 1;
	while (gState==STATE_SET_INVERSE)
	{
		ReStrainScroll();
		if (gScrollNum == 1)
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("Inverse ON    1B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gInverted = 1;
				gState = GetParent(gState);
				gScrollNum = GetCursor(STATE_SET_INVERSE);
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = GetParent(gState);
				gScrollNum = GetCursor(STATE_SET_INVERSE);
			}
		}
		else if (gScrollNum == 2)
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("Inverse OFF   1B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gInverted = 0;
				gState = GetParent(gState);
				gScrollNum = GetCursor(STATE_SET_INVERSE);
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = GetParent(gState);
				gScrollNum = GetCursor(STATE_SET_INVERSE);
			}
		}
	}
}

void SettingMenu()
{
	gScrollMax = MAX_SCROLL_SETTING_MENU;
	while (gState == STATE_SETTING_MENU)
	{
		ReStrainScroll();
		PrintScroll(gScrollNum,gScrollMax);
		if (gScrollNum == GetCursor(STATE_SETTING_DEBUG_MENU))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("DEBUG         1B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_DEBUG_MENU;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_MENU;
				gScrollNum = GetCursor(STATE_SETTING_MENU);
			}
		}
		else if (gScrollNum == GetCursor(STATE_SETTING_SET_PID_SPEED_MENU))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("PID&SPEED     1B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_SET_PID_SPEED_MENU;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_MENU;
				gScrollNum = GetCursor(STATE_SETTING_MENU);
			}
		}
		
		else if (gScrollNum == GetCursor(STATE_SETTING_SAVE_MENU))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("SAVES         1B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_SAVE_MENU;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_MENU;
				gScrollNum = GetCursor(STATE_SETTING_MENU);
			}
		}
		else if (gScrollNum == GetCursor(STATE_SETTING_ERROR))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("ERROR/BOBOT   1B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_ERROR;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_MENU;
				gScrollNum = GetCursor(STATE_SETTING_MENU);
			}
		}
		else if (gScrollNum == GetCursor(STATE_SETTING_SAMPLING))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("SAMPLING      1B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_SAMPLING;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_MENU;
				gScrollNum = GetCursor(STATE_SETTING_MENU);
			}
		}
		
		else if (gScrollNum == GetCursor(STATE_SETTING_SENSOR_MENU))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("SENSOR-SET    1B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_SENSOR_MENU;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_MENU;
				gScrollNum = GetCursor(STATE_SETTING_MENU);
			}
		}
	}
}

void SettingErrorMenu()
{
	
}
/**STATE_SETTING_SET_PID_SPEED_MENU**/
void SettingSetPIDSpeedMenu()
{
	gScrollMax = MAX_SCROLL_SET_PID_SPEED_MENU;
	while (gState == STATE_SETTING_SET_PID_SPEED_MENU)
	{
		ReStrainScroll();
		PrintScroll(gScrollNum,gScrollMax);
		if (gScrollNum == GetCursor(STATE_SETTING_SET_PID_SPEED_SPEED))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("SPEED         [B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_SET_PID_SPEED_SPEED;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_SETTING_MENU;
				gScrollNum = GetCursor(STATE_SETTING_SET_PID_SPEED_MENU);
			}
		}
		else if (gScrollNum == GetCursor(STATE_SETTING_SET_PID_SPEED_KP))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("KP            [B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_SET_PID_SPEED_KP;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_SETTING_MENU;
				gScrollNum = GetCursor(STATE_SETTING_SET_PID_SPEED_MENU);
			}
		}
		else if (gScrollNum == GetCursor(STATE_SETTING_SET_PID_SPEED_KD))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("KD            [B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_SET_PID_SPEED_KD;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_SETTING_MENU;
				gScrollNum = GetCursor(STATE_SETTING_SET_PID_SPEED_MENU);
			}
		}
		else if (gScrollNum == GetCursor(STATE_SETTING_SET_PID_SPEED_MAXSPEED))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MAXSPEED      [B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_SET_PID_SPEED_MAXSPEED;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_SETTING_MENU;
				gScrollNum = GetCursor(STATE_SETTING_SET_PID_SPEED_MENU);
			}
		}
		else if (gScrollNum == GetCursor(STATE_SETTING_SET_PID_SPEED_MINSPEED))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("MINSPEED      [B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_SET_PID_SPEED_MINSPEED;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_SETTING_MENU;
				gScrollNum = GetCursor(STATE_SETTING_SET_PID_SPEED_MENU);
			}
		}
	}
}
/**STATE_SETTING_SET_PID_SPEED_SPEED**/
void SettingSetPIDSpeedSpeed()
{
	uint8_t speed,act;
	speed = gDriveSpeed;
	while (gState == STATE_SETTING_SET_PID_SPEED_SPEED)
	{
		LCDGotoXY(0,0);
		LCDstring("OK   {SPEED}  [B",16);
		LCDGotoXY(0,1);
		LCDstring("<<-    ###   +>>",16);
		cetak_bil(speed,7,1,3);
		act=ButtonRead();
		if (act == BUTTON_NEXT_DOWN)		speed++;
		else if (act == BUTTON_PREV_DOWN)	speed--;
		else if (act == BUTTON_BACK_DOWN)	gState = GetParent(STATE_SETTING_SET_PID_SPEED_SPEED);
		else if (act == BUTTON_ENTER_DOWN)
		{
			gDriveSpeed = speed;
			EepromWrite(ADDRESS_BLOCK_DRIVE_SPEED,speed);
			gState = GetParent(STATE_SETTING_SET_PID_SPEED_SPEED);
		}
	}
}
void SettingSetPIDSpeedKP()
{
	uint8_t kp,act;
	kp = gDriveKP;
	while (gState == STATE_SETTING_SET_PID_SPEED_KP)
	{
		LCDGotoXY(0,0);
		LCDstring("OK   { KP }   [B",16);
		LCDGotoXY(0,1);
		LCDstring("<<-    ##    +>>",16);
		cetak_bil(kp,7,1,2);
		act=ButtonRead();
		if (act == BUTTON_NEXT_DOWN)		kp++;
		else if (act == BUTTON_PREV_DOWN)	kp--;
		else if (act == BUTTON_BACK_DOWN)	gState = GetParent(STATE_SETTING_SET_PID_SPEED_KP);
		else if (act == BUTTON_ENTER_DOWN)
		{
			gDriveKP = kp;
			EepromWrite(ADDRESS_BLOCK_DRIVE_KP,kp);
			gState = GetParent(STATE_SETTING_SET_PID_SPEED_KP);
		}
	}
}

void SettingSetPIDSpeedKD()
{
	uint8_t kd,act;
	kd = gDriveKD;
	while (gState == STATE_SETTING_SET_PID_SPEED_KD)
	{
		LCDGotoXY(0,0);
		LCDstring("OK   { KD }   [B",16);
		LCDGotoXY(0,1);
		LCDstring("<<-    ##    +>>",16);
		cetak_bil(kd,7,1,2);
		act=ButtonRead();
		if (act == BUTTON_NEXT_DOWN)		kd++;
		else if (act == BUTTON_PREV_DOWN)	kd--;
		else if (act == BUTTON_BACK_DOWN)	gState = GetParent(STATE_SETTING_SET_PID_SPEED_KD);
		else if (act == BUTTON_ENTER_DOWN)
		{
			gDriveKD = kd;
			EepromWrite(ADDRESS_BLOCK_DRIVE_KD,kd);
			gState = GetParent(STATE_SETTING_SET_PID_SPEED_KD);
		}
	}
}
void SettingDebugMenu()
{
	gScrollMax = MAX_SCROLL_SETTING_DEBUG_MENU;
	while (gState == STATE_SETTING_DEBUG_MENU)
	{
		ReStrainScroll();
		PrintScroll(gScrollNum,gScrollMax);
		if (gScrollNum == GetCursor(STATE_SETTING_DEBUG_SENSOR))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("DEBUG-SENSOR  2B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_DEBUG_SENSOR;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_SETTING_MENU;
				gScrollNum = GetCursor(STATE_SETTING_DEBUG_MENU);
			}
		}
		else if (gScrollNum == GetCursor(STATE_SETTING_DEBUG_ADC))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("DEBUG-ADC     2B"),16);
			uint8_t act;
			act = ButtonRead();
			
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_DEBUG_ADC;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_SETTING_MENU;
				gScrollNum = GetCursor(STATE_SETTING_DEBUG_MENU);
			}
		}
		
		else if (gScrollNum == GetCursor(STATE_SETTING_DEBUG_DRIVER))
		{
			LCDGotoXY(0,0);
			LCDstring((uint8_t*)("DEBUG-DRIVER  2B"),16);
			uint8_t act;
			act = ButtonRead();
			
			if (act == BUTTON_ENTER_DOWN)
			{
				gState = STATE_SETTING_DEBUG_DRIVER;
				gScrollNum = 1;
			}
			else if (act == BUTTON_NEXT_DOWN)
			{
				gScrollNum++;
			}
			else if (act == BUTTON_PREV_DOWN)
			{
				gScrollNum--;
			}
			else if (act == BUTTON_BACK_DOWN)
			{
				gState = STATE_SETTING_MENU;
				gScrollNum = GetCursor(STATE_SETTING_DEBUG_MENU);
			}
		}
	}
}

void SettingDebugSensor()
{
	LCDclr();
	uint8_t l_Num=0;
	while (!(ButtonBack()))
	{ 
		SensorReadDigital();
		for (l_Num=0;l_Num<=7;l_Num++)
		{
			if (isset(gSensH,l_Num))	cetak_bil(1,15-l_Num-8,1,1);
			else 						cetak_bil(0,15-l_Num-8,1,1);
			if (isset(gSensL,l_Num))	cetak_bil(1,15-l_Num,1,1);
			else 						cetak_bil(0,15-l_Num,1,1);
		}
		cetak_bil(isset(gSensA,SENS_A_LEFT_BIT),7,0,1);
		cetak_bil(isset(gSensA,SENS_A_MID_BIT),8,0,1);
		cetak_bil(isset(gSensA,SENS_A_RIGHT_BIT),9,0,1);
	}
	gScrollNum = GetCursor(STATE_SETTING_DEBUG_SENSOR);
	gState = STATE_SETTING_DEBUG_MENU;
}







//~ void SettingSetPIDSpeedMenu(uint8_t m)
//~ {
	//~ gScrollMax = m;
	/**	gScrollMax = GetMaxScroll(m);**/
	//~ while (gState == m);
	//~ {
		//~ ReStrainScroll();
		//~ PrintScroll(gScrollNum,gScrollMax);
		//~ for (i = 1; i <= gScrollMax; i++)
		//~ {
			//~ if (gScrollNum = GetCursor(GetChild(m,i)))
			//~ {
				//~ 
			//~ }
		//~ }
		//~ 
	//~ }
