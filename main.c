/* BISMILLAH																																											*/
///		Allah Maha Pemurah
///		Allah Maha Penyayang
																																														/*
 * made from scratch 
 * 30 OKTOBER 2011
 * Makin
 * 
<<<<<<< HEAD
 * ATMEGA 32:
 * 	EEPROM 1024 Bytes
 * 	SRAM	  2048 Bytes
=======
 * EEPROM 1024 Bytes
 * SRAM	  2048 Bytes
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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


//~ #define EepromWrite(_BLOCK,_VALUE)	eeprom_write_byte((uint8_t*)(_BLOCK),_VALUE)
#define EepromWrite(_BLOCK,_VALUE)	eeprom_update_byte((uint8_t*)(_BLOCK),_VALUE)
#define EepromRead(_BLOCK)	eeprom_read_byte((uint8_t*)(_BLOCK))


#define A_CUSTOM_COUNT 1
#define ADDRESS_BLOCK_CASE_N		2
#define ADDRESS_BLOCK_CASE_H		ADDRESS_BLOCK_CASE_N+1
#define CASE_SIZE					40
#define ADDRESS_BLOCK_CASE_L		ADDRESS_BLOCK_CASE_H+CASE_SIZE+1
#define ADDRESS_BLOCK_CASE_V		ADDRESS_BLOCK_CASE_L+CASE_SIZE+1


#define MAP_SIZE					50
#define ADDRESS_BLOCK_MAP_MAZE_TOTAL	ADDRESS_BLOCK_CASE_V+CASE_SIZE+1
#define ADDRESS_BLOCK_MAP_CASE_H	ADDRESS_BLOCK_MAP_MAZE_TOTAL+1							/**																												**/
#define ADDRESS_BLOCK_MAP_CASE_L	ADDRESS_BLOCK_MAP_CASE_H+MAP_SIZE+1						/**																												**/
#define ADDRESS_BLOCK_MAP_ACT_SPEED ADDRESS_BLOCK_MAP_CASE_L+MAP_SIZE+1						/**																												**/
#define ADDRESS_BLOCK_MAP_ACT_KP	ADDRESS_BLOCK_MAP_ACT_SPEED+MAP_SIZE+1					/**																												**/
#define ADDRESS_BLOCK_MAP_ACT_KD	ADDRESS_BLOCK_MAP_ACT_KP+MAP_SIZE+1						/**																												**/
#define ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A	ADDRESS_BLOCK_MAP_ACT_KD+MAP_SIZE+1 		//BIT CONFIG  [CASE_A][CASE_A][CASE_A]	[UNTIL_A][UNTIL_A][UNTIL_A]	[TURN][TURN]
#define ADDRESS_BLOCK_MAP_UNTIL_H	ADDRESS_BLOCK_MAP_ACT_TURN_CASE_A_UNTIL_A+MAP_SIZE+1		/**	   bit7 [caseA s.kiri] [caseA s.tengah] [caseA s.kanan]	bit5										**/
#define ADDRESS_BLOCK_MAP_UNTIL_L	ADDRESS_BLOCK_MAP_UNTIL_H+MAP_SIZE+1						/**				bit4 [untilA s.kiri] [untilA s.tengah] [untilA s.kanan]	bit2																									**/

#define ADDRESS_BLOCK_SENSOR_REF	ADDRESS_BLOCK_MAP_UNTIL_L+MAP_SIZE+1
#define ADDRESS_BLOCK_DRIVE_SPEED	ADDRESS_BLOCK_SENSOR_REF+20
#define ADDRESS_BLOCK_DRIVE_KP		ADDRESS_BLOCK_DRIVE_SPEED+1
#define ADDRESS_BLOCK_DRIVE_KD		ADDRESS_BLOCK_DRIVE_KP+1
																							/**					bit1 [TURN][TURN] bit0																						**/
#define MAP_ACT_TURN_FORWARD		0 //00																	#define SENS_A_LEFT	2
#define MAP_ACT_TURN_BACKWARD		1 //01																	#define SENS_A_MID	1
#define MAP_ACT_TURN_LEFT			2 //10																	#define SENS_A_RIGHT 0
#define MAP_ACT_TURN_RIGHT			3 //11

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
void SetDefault(void);
void SetMinim(void);
void PIDCalculateExecute(void);
void PIDCalculateExecuteCustom(uint8_t speed, uint8_t kp, uint8_t kd);
uint8_t PIDGetError(void);
void DirForward(void);
void DirBackward(void);
void DirTurnLeft(void);
void DirTurnRight(void);
void DriveMove(uint8_t pwm_ki, uint8_t pwm_ka);
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
<<<<<<< HEAD
void SettingDebugPrintlog(char* nama,int8_t data[]);
void SettingDebugPrintlog2(char* nama,int8_t data[][2]); //Seems avr-gcc should define unique functions to get unique arguments
=======
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
void SettingSetPIDSpeedSpeed(void);
void SettingSetPIDSpeedKP(void);
void SettingSetPIDSpeedKD(void);
void SettingErrorMenu(void);
void SettingSetPIDSpeedMenu(void);
void Run(void);
void RunOnce(uint8_t speed, uint8_t kp, uint8_t kd);
void RunMapping(uint8_t mulai);
<<<<<<< HEAD
void RunLearn(void);
=======
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
void RunC00ke01(void);
void RunC01ke02(void);
void RunC02ke03(void);
void RunC03ke04(void);
<<<<<<< HEAD
void RunEb00ke01(void);
void RunEb01ke02(void);
void RunEb02ke03(void);
void RunProgram(void);
void RunWhileNormal(uint8_t sampling_limit);
=======
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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
<<<<<<< HEAD
uint8_t gSensH, gSensL, gSensA; //gSensA = sensor kiri,kanan,tengah

uint32_t counter=0,countermax=30;

#define SENS_A_LEFT_BIT		0 //gSensA = sensor kiri,kanan,tengah
=======
uint8_t gSensH, gSensL, gSensA;

uint32_t counter=0,countermax=30;

#define SENS_A_LEFT_BIT		0
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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
<<<<<<< HEAD
///MENU2 pnya MAX SCROLL,, ADA BERAPA PILIHAN DIDALAMNYA
#define MAX_SCROLL_MENU							9 ///GUA GIBENG JUGA NIH
=======
///MAX SCROLL,, ADA BERAPA PILIHAN DIDALAMNYA
#define MAX_SCROLL_MENU							8 ///GUA GIBENG JUGA NIH
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
#define MAX_SCROLL_SETTING_MENU					6
#define MAX_SCROLL_SETTING_DEBUG_MENU			3
#define MAX_SCROLL_SET_PID_SPEED_MENU			5
#define MAX_SCROLL_SAVE_MENU					2
#define MAX_SCROLL_SENSOR_MENU					1


/**																**/
///			ATURAN == LAST DIGIT = NOMER CURSOR
/**																**/
uint8_t GetCursor(uint16_t numstate);
uint8_t GetParent(uint16_t numstate);
uint8_t GetChild(uint8_t parent, uint8_t childid);
#define GetParentCursor(_ns)	GetCursor(GetParent(_ns))
#define STATE_MENU								0 ///KAMPRETOS
#define STATE_RUN								7	///GUA BERI!
#define STATE_CALIBRATE							5 ///DIBIKIN DI MENU UTAMA, KRN SERING DILAKUKAN
<<<<<<< HEAD
#define STATE_RUN_MAPPING						9
#define STATE_RUN_MAPPING_C1					10
#define STATE_RUN_LEARN							1
#define STATE_RUN_MAPPING_C2					2
#define STATE_RUN_MAPPING_C3					3
#define STATE_RUN_MAPPING_EBOTEC				4
=======
#define STATE_RUN_MAPPING						1
#define STATE_RUN_MAPPING_C1					2
#define STATE_RUN_MAPPING_C2					3
#define STATE_RUN_MAPPING_C3					4
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd

#define STATE_RUN_TEST_COUNTER					8

#define STATE_SETTING_MENU						6
#define STATE_SETTING_DEBUG_MENU				61
<<<<<<< HEAD
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
=======
#define STATE_SETTING_DEBUG_SENSOR				 311
#define STATE_SETTING_DEBUG_ADC					 312
#define STATE_SETTING_DEBUG_DRIVER				 313
#define STATE_SETTING_SET_PID_SPEED_MENU		62 ///PID & SPEED
#define STATE_SETTING_SET_PID_SPEED_KP			 321
#define STATE_SETTING_SET_PID_SPEED_KD			 322
#define STATE_SETTING_SET_PID_SPEED_SPEED		 323
#define STATE_SETTING_SET_PID_SPEED_MINSPEED	 324
#define STATE_SETTING_SET_PID_SPEED_MAXSPEED	 325
#define STATE_SETTING_SAVE_MENU					63
#define STATE_SETTING_SAVE_SAVE					 331
#define STATE_SETTING_SAVE_LOAD					 332
#define STATE_SETTING_SENSOR_MENU				64
#define STATE_SETTING_SENSOR_MODE				 341
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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


	/* 0 = menu
	 * 1 = jalan
	 * 2 = kalibrasi
	 * 3 = setting
	 * 30 = setting_debugmenu
	 * 31 = debug sensor
	 * 32 = debug adc
	 * 33 = debug driver
	 * 40 = atur PID
	 * 41 = atur KP
	 * 42 = atur speed menu
	 * 43 = atur KD
	 * 46 = atur speed 
	 * 47 = atur maxspeed
	 * 48 = atur minspeed
	 * 5 = SIMPAN
	 * 51= Reload
	 * 56= Save
	 * 6 = atur mode sensor
	 * 7 = atursampling menu
	 * 100 = atur bobot
	 */

int main(void)
{
	Init();
	
	gState = STATE_MENU;
	//~ Calibrate();
	//~ ProgMapping();
	while(1)
	{
		if (gState == STATE_CALIBRATE)						Calibrate();
		if (gState == STATE_MENU)							Menu();
		if (gState == STATE_RUN)							Run();
		if (gState == STATE_RUN_MAPPING)					RunMapping(0);
		if (gState == STATE_RUN_MAPPING_C1)					RunMapping(1);
		if (gState == STATE_RUN_MAPPING_C2)					RunMapping(2);
		if (gState == STATE_RUN_MAPPING_C3)					RunMapping(3);
<<<<<<< HEAD
		if (gState == STATE_RUN_MAPPING_EBOTEC)				RunMapping(4);
		if (gState == STATE_RUN_LEARN)						RunLearn();
		
=======
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
		if (gState == STATE_RUN_TEST_COUNTER)				RunTestCounter();
		if (gState == STATE_SETTING_MENU)					SettingMenu();
		if (gState == STATE_SETTING_SET_PID_SPEED_MENU)		SettingSetPIDSpeedMenu();
		if (gState == STATE_SETTING_SET_PID_SPEED_KD)		SettingSetPIDSpeedKD();
		if (gState == STATE_SETTING_SET_PID_SPEED_KP)		SettingSetPIDSpeedKP();
		if (gState == STATE_SETTING_SET_PID_SPEED_SPEED)	SettingSetPIDSpeedSpeed();
		if (gState == STATE_SETTING_DEBUG_MENU)				SettingDebugMenu();
		if (gState == STATE_SETTING_DEBUG_SENSOR)			SettingDebugSensor();
		if (gState == STATE_SETTING_ERROR)					SettingErrorMenu();
		
		
		
		//~ if (Button0())	Calibrate();
		//~ cetak_bil(PIDGetError(),8,0,2);
		//~ PIDCalculateExecute();
		//~ cetak_bil(OCR1A,0,1,5);
		//~ cetak_bil(OCR1B,5,1,5);
		//~ cetak_bil(gDriveError,8,1,5);
		//~ LCDGotoXY(0,0);
		//~ cetak_bil_lgsg(isset(PIND,2),1); //10 maju, 00 kanan, 01 mundur, 11 kiri
		//~ cetak_bil_lgsg(isset(PIND,3),1);
		//~ if (Button0())	_togle(PORTD,2);
		//~ if (Button1())	_togle(PORTD,3);
		//~ _delay_ms(100);
	}
}

void Init()
{
	counter=0;
	SetDefault();
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
<<<<<<< HEAD
		RunProgram();
	}
	else if (mulai==1)
	{
		RunEb00ke01();
		RunEb01ke02();
		//~ RunC00ke01();
		//~ RunEb02ke03();
	}
	else if (mulai==2)
	{
		RunWhileNormal(10);
		RunEb01ke02();
		//~ RunEb02ke03();
	}
	else if (mulai==3)
	{
		RunWhileNormal(10);
		//~ RunEb02ke03();
		
	}
	else if (mulai==4)
	{
		RunEb00ke01();
	}
	//~ Run();
=======
		RunC00ke01();
		RunC01ke02();
		RunC02ke03();
		RunC03ke04();
	}
	else if (mulai==1)
	{
		RunC01ke02();
		RunC02ke03();
		RunC03ke04();
	}
	else if (mulai==2)
	{
		RunC02ke03();
		RunC03ke04();
	}
	else if (mulai==3)
	{
		RunC03ke04();
	}
	
	Run();
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
	counter=0;
	gState=STATE_MENU;
	OCR1A=0;
	OCR1B=0;
	//~ while(1);
}

<<<<<<< HEAD
void RunLearn()
{
	RunWhileNormal(5);
}

void RunC00ke01()
{
	DirForward();
	DriveMove(30,30);
	_delay_ms(300);
	
	counter=0;
	SensorReadDigital();
	while (counter<=50)
	{
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
	}
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(250);
	
	counter=0;
	SensorReadDigital();
	while (counter<=50)
	{
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
	}
	
	DirForward();
	DriveMove(30,30);
	_delay_ms(250);
	
	counter = 0;
	while (counter<=50)
	{
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
	}
	
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(250);
	
	counter = 0;
	while (counter<=400 && isclear(gSensA,SENS_A_LEFT_BIT))
	{
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
	}
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(250);
	
	DirForward();
	DriveMove(30,30);
	_delay_ms(100);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(250);
	
	
	DriveMove(0,0);while(ButtonIsNotPressed());
	
=======
void RunC00ke01()
{
	_delay_ms(250);
	counter=0;
	SensorReadDigital();
	while (counter<=1750)
	{
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
	}
	counter=0;
	//~ gDriveSpeed = 100;
	gDriveLimit=140;
	while (counter<=150)
	{
		RunOnce(100,10,15);
	}
	gDriveLimit=120;
	counter=0;
	while(counter<=800)
	{
		RunOnce(30,gDriveKP,gDriveKD);
	}
	counter=0;
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
}
void RunC01ke02()
{
	uint8_t cnum;
	cnum = gCaseN;
	gCaseN = 36;
	counter=0;
	SensorReadDigital();
	while(counter<=3200)
	{
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
	}
	gCaseN = cnum;
	counter=0;
}
void RunC02ke03()
{
	uint8_t cnum;
	cnum = gCaseN;
	gCaseN = 30;
	counter=0;
	SensorReadDigital();
	while(counter<=800)
	{
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
	}
	gCaseN = cnum;
	counter=0;
}
void RunC03ke04()
{
	uint8_t cnum;
	while (!((gSensH==0b00011111)&&(gSensL==0b11111000))	&&	!(	isset(gSensA,SENS_A_LEFT_BIT)&&isset(gSensA,SENS_A_RIGHT_BIT)	)		)
	{
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
	}
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(300);
	counter=0;
	SetMinim();
	while (counter<=700)
	{
		RunOnce(60,6,gDriveKD);
	}
	//~ DriveMove(0,0);
	//~ _delay_ms(500);
	while (!(isset(gSensA,SENS_A_LEFT_BIT)&&isset(gSensA,SENS_A_RIGHT_BIT)))
	{
		RunOnce(20,3,gDriveKD);
	}
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(300);
	DirForward();
	counter=0;
	while (isclear(gSensA,SENS_A_RIGHT_BIT))
	{
		RunOnce(50,5,gDriveKD);
	}
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(300);
	DirForward();
	counter=0;
	SetDefault();
	while (isclear(gSensA,SENS_A_RIGHT_BIT))
	{
		RunOnce(30,3,gDriveKD);
	}
}
<<<<<<< HEAD
void RunProgram()
{
	counter=0;
	SensorReadDigital();
	while (PIDGetError() && (counter<50))
	{
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
		if (!(PIDGetError()))
			counter++;
	}
	DriveMove(0,0);
	while(ButtonIsNotPressed());
}
void RunWhileNormal(uint8_t sampling_limit)
{
	counter=0;
	SensorReadDigital();
	
	int8_t log[sampling_limit*2+2][2];
	uint8_t i=0;
	while (counter<sampling_limit)
	{
		RunOnce(gDriveSpeed,gDriveKP,gDriveKD);
		
		if ((!PIDGetError()) && gSensH!=0 && gSensL!=0)
		{
			counter++;
			log[i][0] = gSensL;///Tampil di LCD
			log[i][1] = gSensH;///Tampil di LCD
		}
		else
		{
			counter=0;
		}
	}
	DriveMove(0,0);
	counter=0;
	
	SettingDebugPrintlog("LOG RUNWHILE ERR",log);
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
		else if (act == BUTTON_BACK_DOWN)	gState = GetParent(STATE_SETTING_DEBUG_PRINTLOG);
		else if (act == BUTTON_ENTER_DOWN)
		{
			gState = GetParent(STATE_SETTING_DEBUG_PRINTLOG);
		}
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
		else if (act == BUTTON_BACK_DOWN)	gState = GetParent(STATE_SETTING_DEBUG_PRINTLOG);
		else if (act == BUTTON_ENTER_DOWN)
		{
			gState = GetParent(STATE_SETTING_DEBUG_PRINTLOG);
		}
	}
	
}

void RunEb00ke01()
{
	DirForward();
	DriveMove(30,30);
	_delay_ms(300);
	
	RunWhileNormal(5);
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(200);
	
	
	RunWhileNormal(5);
	DirForward();
	DriveMove(40,40);
	_delay_ms(300);
	
	RunWhileNormal(5);
	
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(250);
	
	
	RunWhileNormal(10);
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(250);
	DirForward();
	DriveMove(30,30);
	_delay_ms(150);
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(200);
	
	
	RunWhileNormal(10);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(250);
	
	RunWhileNormal(10);
	
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(250);
	
	RunWhileNormal(5);
	
	DirForward();
	DriveMove(30,30);
	_delay_ms(200);
	
	RunWhileNormal(5);
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(250);
	DirForward();
	DriveMove(30,30);
	_delay_ms(300);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(250);
	RunWhileNormal(5);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(250);
	RunWhileNormal(5);
	
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(250);
	
	RunWhileNormal(10);
	
}
void RunEb01ke02()
{
	///CP 2
	
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(250);
	RunWhileNormal(10);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(250);
	RunWhileNormal(10);
	
	DirForward();
	DriveMove(30,30);
	_delay_ms(200);
	
	RunWhileNormal(5);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(250);
	
	RunWhileNormal(10);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(250);
	
	RunWhileNormal(10);
	
	DirForward();
	DriveMove(30,30);
	_delay_ms(200);
	
	RunWhileNormal(5);
	
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(250);
	
	RunWhileNormal(15);
	
	DriveMove(0,0);while(ButtonIsNotPressed());
	
	while(ButtonIsNotPressed());
}
void RunEb02ke03();
/*	counter=0;
	while (counter<=3)
	{
		DirForward();
		DriveMove(30,30);
		counter++;
		_delay_ms(100);
	}
	counter = 0;
	while (counter<=500)
	{
		RunOnce(60,gDriveKP,gDriveKD);
	}
	while(isclear(gSensA,SENS_A_LEFT_BIT))
	{
		RunOnce(40,gDriveKP,gDriveKD);
	}
	DriveMove(0,0);
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(300);
	DirForward();
	_delay_ms(100);
	counter = 0;
	//~ while (counter<=400)
	//~ {
		//~ RunOnce(60,gDriveKP,gDriveKD);
	//~ }
	//~ DriveMove(0,0);
	//~ while(ButtonIsNotPressed());
	while (isclear(gSensA,SENS_A_RIGHT_BIT))
	{
		RunOnce(40,gDriveKP,gDriveKD);
	}
	DriveMove(0,0);
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(300);
	DirForward();
	_delay_ms(100);
	while (isclear(gSensA,SENS_A_RIGHT_BIT))
	{
		RunOnce(40,gDriveKP,gDriveKD);
	}
	DriveMove(0,0);
	DirTurnRight();
	DriveMove(30,30);
	_delay_ms(300);
	DirForward();
	_delay_ms(100);
	while (isclear(gSensA,SENS_A_LEFT_BIT))
	{
		RunOnce(40,gDriveKP,gDriveKD);
	}
	DriveMove(0,0);
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(300);
	DirForward();
	_delay_ms(100);
	while (isclear(gSensA,SENS_A_LEFT_BIT))
	{
		RunOnce(40,gDriveKP,gDriveKD);
	}
	DriveMove(0,0);
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(300);
	DirForward();
	_delay_ms(100);
	while (isclear(gSensA,SENS_A_RIGHT_BIT))
	{
		RunOnce(40,gDriveKP,gDriveKD);
	}
	DriveMove(0,0);
	DirTurnLeft();
	DriveMove(30,30);
	_delay_ms(300);
	DirForward();
	_delay_ms(100);
	
	counter = 0;
	while (ButtonIsNotPressed());
	
}*/

=======
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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

void Run()
{
	_delay_ms(250);
	SensorReadDigital();
	while (ButtonIsNotPressed())
	{
		//~ cetak_bil(gDriveError,0,0,4);
		//~ if ((gSensH==0b11110000))
		//~ {
			//~ LCDclr();
			//~ LCDGotoXY(0,1);
			//~ LCDstring("KOTAK",5);
		//~ }
		if ((gSensH==0)&&(gSensL==0))
		{
			if (isset(gSensA,SENS_A_LEFT_BIT) && isclear(gSensA,SENS_A_RIGHT_BIT))
			{
				//~ SensorReadDigital();
				//~ DirTurnLeft();
				//~ DriveMove(30,30);
				//~ _delay_ms(250);
				while ((gSensL==0)&&(gSensH==0))
				{
					SensorReadDigital();
					DirTurnLeft();
					DriveMove(30,30);
				}
			}
			else if (isset(gSensA,SENS_A_RIGHT_BIT) && isclear(gSensA,SENS_A_LEFT_BIT))
			{
				//~ SensorReadDigital();
				//~ DirTurnRight();
				//~ DriveMove(30,30);
				//~ _delay_ms(250);
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
		//~ else if (!((gSensH==0) && (gSensL==0)))
		//~ {
			//~ OCR1A=0;
			//~ OCR1B=0;
		//~ }
		
		
	}
	OCR1A=0;
	OCR1B=0;
	gState = STATE_MENU;
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
<<<<<<< HEAD
	else //if ((gSensH==0) && (gSensL==0))
=======
	else if ((gSensH==0) && (gSensL==0))
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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
void SetDefault()
{
	
<<<<<<< HEAD
	gCaseN = 32;
=======
	gCaseN = 40;
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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
<<<<<<< HEAD
	gCaseH[14] = 0b00000011; gCaseL[14] = 0b10000000; gCaseV[14] = 0;
	gCaseH[15] = 0b00000001; gCaseL[15] = 0b10000000; gCaseV[15] = 0;
	gCaseH[16] = 0b00000001; gCaseL[16] = 0b11000000; gCaseV[16] = 0;
=======
	gCaseH[14] = 0b00000011; gCaseL[14] = 0b10000000; gCaseV[14] = 1;
	gCaseH[15] = 0b00000001; gCaseL[15] = 0b10000000; gCaseV[15] = 0;
	gCaseH[16] = 0b00000001; gCaseL[16] = 0b11000000; gCaseV[16] = -1;
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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
<<<<<<< HEAD
	//~ gCaseH[33] = 0b00000111; gCaseL[33] = 0b10000000; gCaseV[33] = 12;
	//~ gCaseH[34] = 0b00000001; gCaseL[34] = 0b11100000; gCaseV[34] = -12;
	//~ gCaseH[35] = 0b00001111; gCaseL[35] = 0b00000000; gCaseV[35] = 12;
	//~ gCaseH[36] = 0b00000000; gCaseL[36] = 0b11110000; gCaseV[36] = -12;
	//~ gCaseH[37] = 0b00001111; gCaseL[37] = 0b10000000; gCaseV[37] = 12;
	//~ gCaseH[38] = 0b00000001; gCaseL[38] = 0b11110000; gCaseV[38] = -12;
	//~ gCaseH[39] = 0b00011111; gCaseL[39] = 0b10000000; gCaseV[39] = 14;
	//~ gCaseH[40] = 0b00000001; gCaseL[40] = 0b11111000; gCaseV[40] = -14;
=======
	gCaseH[33] = 0b00000111; gCaseL[33] = 0b10000000; gCaseV[33] = 12;
	gCaseH[34] = 0b00000001; gCaseL[34] = 0b11100000; gCaseV[34] = -12;
	gCaseH[35] = 0b00001111; gCaseL[35] = 0b00000000; gCaseV[35] = 12;
	gCaseH[36] = 0b00000000; gCaseL[36] = 0b11110000; gCaseV[36] = -12;
	gCaseH[37] = 0b00001111; gCaseL[37] = 0b10000000; gCaseV[37] = 12;
	gCaseH[38] = 0b00000001; gCaseL[38] = 0b11110000; gCaseV[38] = -12;
	gCaseH[39] = 0b00011111; gCaseL[39] = 0b10000000; gCaseV[39] = 14;
	gCaseH[40] = 0b00000001; gCaseL[40] = 0b11111000; gCaseV[40] = -14;
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
	
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
	
	gCaseN = 16;
	gCaseH[ 0] = 0b10000000; gCaseL[ 0] = 0b00000000; gCaseV[ 0] = 15;
	gCaseH[ 1] = 0b11000000; gCaseL[ 1] = 0b00000000; gCaseV[ 1] = 14;
	gCaseH[ 2] = 0b01100000; gCaseL[ 2] = 0b00000000; gCaseV[ 2] = 12;
	gCaseH[ 3] = 0b00110000; gCaseL[ 3] = 0b00000000; gCaseV[ 3] = 10;
	gCaseH[ 4] = 0b00011000; gCaseL[ 4] = 0b00000000; gCaseV[ 4] = 8;
	gCaseH[ 5] = 0b00001100; gCaseL[ 5] = 0b00000000; gCaseV[ 5] = 6;
	gCaseH[ 6] = 0b00000110; gCaseL[ 6] = 0b00000000; gCaseV[ 6] = 4;
	gCaseH[ 7] = 0b00000011; gCaseL[ 7] = 0b00000000; gCaseV[ 7] = 2;
	gCaseH[ 8] = 0b00000001; gCaseL[ 8] = 0b10000000; gCaseV[ 8] = 0;
	gCaseH[ 9] = 0b00000000; gCaseL[ 9] = 0b11000000; gCaseV[ 9] = -2;
	gCaseH[10] = 0b00000000; gCaseL[10] = 0b01100000; gCaseV[10] = -4;
	gCaseH[11] = 0b00000000; gCaseL[11] = 0b00110000; gCaseV[11] = -6;
	gCaseH[12] = 0b00000000; gCaseL[12] = 0b00011000; gCaseV[12] = -8;
	gCaseH[13] = 0b00000000; gCaseL[13] = 0b00001100; gCaseV[13] = -10;
	gCaseH[14] = 0b00000000; gCaseL[14] = 0b00000110; gCaseV[14] = -12;
	gCaseH[15] = 0b00000000; gCaseL[15] = 0b00000011; gCaseV[15] = -14;
	gCaseH[16] = 0b00000000; gCaseL[16] = 0b00000001; gCaseV[16] = -15;
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

 //10 maju, 00 kanan, 01 mundur, 11 kiri
void DirForward()
{
	PORTD = (PORTD & 0b11110011) | 0b00000100;
	//~ LCDGotoXY(0,0);
	//~ LCDstring("maju",4);
}
void DirBackward()
{
	PORTD = (PORTD & 0b11110011) | 0b00001000;
	//~ LCDGotoXY(0,0);
	//~ LCDstring("mundur",6);
}
void DirTurnLeft()
{
	PORTD = (PORTD & 0b11110011) | 0b00001100;
	//~ LCDGotoXY(0,0);
	//~ LCDstring("kiri",4);
}
void DirTurnRight()
{
	PORTD = (PORTD & 0b11110011) | 0b00000000;
	//~ LCDGotoXY(0,0);
	//~ LCDstring("kanan",5);
}

void DriveMove(uint8_t pwm_ki, uint8_t pwm_ka)
{
	OCR_RIGHT= pwm_ka;
	OCR_LEFT = pwm_ki;
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
	uint8_t bA, bB;
	gMazeMapNum=0;
	gMazeMapTot=0;
	uint8_t sempling=0,maxsempling=25;
	LCDclr();
	while(!ButtonEnter())
	{
		if (ButtonNext())	maxsempling++;
		if (ButtonPrev())	maxsempling--;
		cetak_bil(maxsempling,0,0,3);
		if (ButtonBack())
		{
			gState = STATE_RUN;
			Run();
		}
	}
	while(1)
	{
		//~ if ((gSensH==0)&&(gSensL==0))
		//~ {
			//~ if (isset(gSensA,SENS_A_LEFT_BIT))
			//~ {
				//~ while ((gSensL==0)&&(gSensH==0))
				//~ {
					//~ SensorReadDigital();
					//~ DirTurnLeft();
					//~ DriveMove(30,30);
				//~ }
			//~ }
			//~ else if (isset(gSensA,SENS_A_RIGHT_BIT))
			//~ {
				//~ while ((gSensL==0)&&(gSensH==0))
				//~ {
					//~ SensorReadDigital();
					//~ DirTurnRight();
					//~ DriveMove(30,30);
				//~ }
			//~ }
		//~ }
		//~ if (PIDGetError())	PIDCalculateExecute();
		//~ else if ((gSensH==0) && (gSensL==0))
		//~ {
			//~ DirForward();
			//~ OCR1A=30;
			//~ OCR1B=30;
		//~ }
		
		while (PIDGetError() && !((gSensH==0) && (gSensL==0)) && (isclear(gSensA,SENS_A_LEFT_BIT)) && (isclear(gSensA,SENS_A_RIGHT_BIT)))
		{
			PIDCalculateExecute();
			sempling=0;
		}
		if ((gSensH==0)&&(gSensL==0)&&(		(isset(gSensA,SENS_A_LEFT_BIT))||(isset(gSensA,SENS_A_RIGHT_BIT))		))
		{
			if (isset(gSensA,SENS_A_LEFT_BIT))
			{
				while ((gSensL==0)&&(gSensH==0))
				{
					SensorReadDigital();
					DirTurnLeft();
					DriveMove(30,30);
				}
			}
			else if (isset(gSensA,SENS_A_RIGHT_BIT))
			{
				while ((gSensL==0)&&(gSensH==0))
				{
					SensorReadDigital();
					DirTurnRight();
					DriveMove(30,30);
				}
			}
			sempling = 0;
		}
		else
		{
			sempling++;
			if (sempling>maxsempling)
			{
				cetak_bil(gMazeMapNum,0,0,3);
				cetak_bil(gSensH,0,1,3);
				cetak_bil(gSensL,4,1,3);
				cetak_bil(gSensA,8,1,3);
				gMazeMapNum++;
				gMazeMapTot++;
				bA=OCR1A;
				bB=OCR1B;
				OCR1A=0;
				OCR1B=0;
				ButtonWait();
				OCR1A = bA;
				OCR1B = bB;
				sempling=0;
				gDriveError = 0;
				gDriveLastError = 0;
			}
		}
	}
}

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

uint8_t GetCursor(uint16_t numstate)
{
	return (numstate%10);
}
uint8_t GetParent(uint16_t numstate)
{
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
			LCDstring((uint8_t*)("MAPPING C1    B"),16);
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
			LCDstring((uint8_t*)("MAPPING C2    B"),16);
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
			LCDstring((uint8_t*)("MAPPING C3    B"),16);
			uint8_t act;
			act = ButtonRead();
			if (act == BUTTON_ENTER_DOWN)
			{
<<<<<<< HEAD
				gState = STATE_RUN_MAPPING_C3;
=======
				gState = STATE_RUN_MAPPING_C1;
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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
<<<<<<< HEAD
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
		
=======
		}	
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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
<<<<<<< HEAD
		else if (act == BUTTON_BACK_DOWN)	gState = GetParent(STATE_SETTING_SET_PID_SPEED_SPEED);
=======
		else if (act == BUTTON_BACK_DOWN)	gState == GetParent(STATE_SETTING_SET_PID_SPEED_SPEED);
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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
<<<<<<< HEAD
		else if (act == BUTTON_BACK_DOWN)	gState = GetParent(STATE_SETTING_SET_PID_SPEED_KP);
=======
		else if (act == BUTTON_BACK_DOWN)	gState == GetParent(STATE_SETTING_SET_PID_SPEED_KP);
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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
<<<<<<< HEAD
		else if (act == BUTTON_BACK_DOWN)	gState = GetParent(STATE_SETTING_SET_PID_SPEED_KD);
=======
		else if (act == BUTTON_BACK_DOWN)	gState == GetParent(STATE_SETTING_SET_PID_SPEED_KD);
>>>>>>> 8073276461a254b68bf9110ac059fa859b7e8ffd
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
	//~ uint8_t dip=0;///BIKIN YG DIPICK LEBIH CETHO
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
		//~ for (uint8_t num=0;num<=7;num++)
		//~ {
			//~ if ((dip!=2) || (pick!=num))
			//~ {
				//~ if (isset(gSensL,num))
					//~ cetak_bil(1,num,1,1);
				//~ else
					//~ cetak_bil(0,num,1,1);
			//~ }
			//~ else
			//~ {
				//~ LCDGotoXY(num,1);
				//~ LCDstring(" ",1);
			//~ }
			//~ if ((dip!=2) || (pick != num+8))
			//~ {
				//~ if (isset(gSensH,num))
					//~ cetak_bil(1,num+8,1,1);
				//~ else
					//~ cetak_bil(0,num+8,1,1);
			//~ }
			//~ else
			//~ {
				//~ LCDGotoXY(num+8,1);
				//~ LCDstring(" ",1);
			//~ }
		//~ }
		//~ if ((ButtonNext()) && (pick<15))
		//~ {
			//~ pick++;
			//~ _delay_ms(50);
		//~ }
		//~ else if ((ButtonPrev()) && (pick>1))
		//~ {
			//~ pick--;
			//~ _delay_ms(50);
		//~ }
		//~ if (dip>1) dip=0;
		//~ else dip++;
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
