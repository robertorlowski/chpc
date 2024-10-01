/*
	Cheap Heat Pump Controller (CHPC) firmware.
	Copyright (C) 2018-2019 Gonzho (gonzho@web.de)

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	
	See https://github.com/gonzho000/chpc/ for more details
*/



//-----------------------USER OPTIONS-----------------------
#define BOARD_TYPE_G  //Type "G", PCB from github.com/gonzho000/chpc/
//#define BOARD_TYPE_F				  //Type "F"
//#define BOARD_TYPE_G9 			  //Type "G9" or "G-MAX", current testing

//#define DISPLAY_096 		1		  //1st tests, support WILL BE DROPPED OUT SOON! small OLEDs support
#define DISPLAY_1602 2  //if only 1st character appears: patch 1602 library "inline size_t LiquidCrystal_I2C::write(uint8_t value)"  "return 1" instead of "return 0"
//#define DISPLAY_NONE		-1

#define INPUTS_AS_BUTTONS 1  //pulldown resistors required

//#define RS485_PYTHON		1
#define RS485_HUMAN 2
//#define RS485_NONE		3

#define EEV_SUPPORT
//#define	EEV_ONLY				      //NO target, no relays. Oly EEV, Tae, Tbe, current sensor and may be additional T sensors

#define HUMAN_AUTOINFO 5000  //10000			//print stats to console

#define WATCHDOG  //only if u know what to do

//-----------------------TEMPERATURES-----------------------
#define T_SETPOINT_MAX 45;           //defines max temperature that ordinary user can set
#define T_DELTA_MAX 10.0;              //defines max delta temperature
#define T_HOTCIRCLE_DELTA_MIN 10.0;    //useful for "water heater vith intermediate heat exchanger" scheme, Target == sensor in water, hot side CP will be switched on if "target - hot_out > T_HOTCIRCLE_DELTA_MIN"
#define T_SUMP_MIN 8;                  //9.0;	//HP will not start if T lower
#define T_SUMP_MAX 90.0;               //116 //HP will stop if T higher
#define T_SUMP_HEAT_THRESHOLD 12.0     //16.0;	//sump heater will be powered on if T lower
#define T_BEFORE_CONDENSER_MAX 108.0;  //discharge MAX, system stops if discharge higher
#define T_EVAPORATOR_WAR -2.0;   // CLOSE
#define T_AFTER_EVAPORATOR_MIN -5.0;   //-7.0;	//suction MIN, HP stops if lower, anti-freeze and anti-liquid at suction protection
#define T_COLD_MIN -3.0;               //-8.0; //cold loop anti-freeze: stop if inlet or outlet temperature lower
#define T_HOTOUT_MAX 60.0;             //hot loop: stop if outlet temperature higher than this
#define T_WORKINGOK_SUMP_MIN 5.0;     //compressor MIN temperature, HP stops if it lower after 5 minutes of pumping, need to be not very high to normal start after deep freeze

//-----------------------TUNING OPTIONS -----------------------
#define MAX_WATTS 3400.0  //user for power protection

#define DEFFERED_STOP_HOTCIRCLE  60000  //3000 000
#define DEFFERED_STOP_COLDCIRCLE 30000  //3000 000
#define POWERON_PAUSE 50000             //50s
#define MINCYCLE_POWEROFF 1200000       //10 mins
#define MINCYCLE_POWERON   300000       //5 min  	//60 mins
#define POWERON_HIGHTIME 4000           //1 sec, defines time after start when power consumption can be 2 times greater than normal
#define COLDOFF_HIGHTIME 15000          //15 sec
#define MINCYKLE_CHECK 60000            //60 sec
//#define MINCYKLE_START_TO_WORK 30000  //60 sec
//#define MINCYKLE_OVERLOAD 3600000     //60 min
//EEV
#define EEV_MAXPULSES  480
#define EEV_MAXPULSES_OPEN  60

#define EEV_PULSE_FCLOSE_MILLIS 20    //fast close, set waiting pos., close on danger
#define EEV_PULSE_CLOSE_MILLIS 30000	//precise close
#define EEV_PULSE_WOPEN_MILLIS 20     //waiting pos. set
#define EEV_PULSE_FOPEN_MILLIS 1300   //fast open, fast search
#define EEV_PULSE_OPEN_MILLIS 80000   //60000   //precise open

//#define EEV_STOP_HOLD		500		    //0.1..1sec for Sanhua
#define EEV_CLOSE_ADD_PULSES 8  //read below, close algo
#define EEV_OPEN_AFTER_CLOSE 50
//0 - close to zero position, than close on EEV_CLOSE_ADD_PULSES (close insurance, read EEV manuals for this value)
//N - close to zero position, than close on EEV_CLOSE_ADD_PULSES, than open on EEV_OPEN_AFTER_CLOSE pulses
//i.e. it is "waiting position" while HP not working
#define EEV_MINWORKPOS 54  //52
// position will be not less during normal work, set after compressor start
#define EEV_PRECISE_START 12 //8.6
//T difference, threshold: make slower pulses if (real_diff-target_diff) less than this value. Used for fine auto-tuning.
#define EEV_EMERG_DIFF 2.5
//if dangerous condition:  real_diff =< (target_diff - EEV_EMERG_DIFF)
//occured then EEV will be closed to min. work position
//Ex: EEV_EMERG_DIFF = 2.0, target diff 5.0, if real_diff =< (5.0 - 2.0) than EEV will be closed
#define EEV_HYSTERESIS 0.6
//must be less than EEV_PRECISE_START,
//ex: target difference = 4.0, hysteresis = 0.1, when difference in range 4.0..4.1 no EEV pulses will be done;
#define EEV_CLOSEEVERY 86400000
//86400000: EEV will be closed (calibrated) every 24 hours, done while HP is NOT working
#define EEV_TARGET_TEMP_DIFF 4.0
//target difference between Before Evaporator and After Evaporator, the head of whole algo
//#define EEV_DEBUG			    1	//debug, usefull during system fine tuning, "RS485_HUMAN" only

#define MAGIC 0x49  //change if u want to reinit T sensors
#define eeprom_addr_dT 0x90
//-----------------------USER OPTIONS END -----------------------

//#define INPUTS_AS_INPUTS	2  		//
//#define RS485_MACHINE 	3 		//?? or part of Python?

//-----------------------changelog-----------------------
/*
v1.0:
- Displays support
- define TYPE F/G and E:
 ports
- multi-DS18b20 support on lane
- skip non-important DS18B20 during init
- rewrite Main Cycle to unification: some sensors can be absent, ex: T_hot_out can be absent because i'ts used as target
- 2 on-board buttons support: +/- aim
- DISPLAY: indication: real and aim
- RS485_HUMAN: remote commands +,-,G,0x20/?/Enter
- buttons: < > increase_decrease t
- simpliest thermostat scheme: only T target
- rename all procs
- RS485_PYTHON: print to console inspite of mode diring init proc
- faster wattage overload processing
- write aim value to EE if needed, period: 15 mins (eq. 1041 days)
- deferred stop of hot side circle
- 80 microseconds at 9600

v1.1, 15 Apr 2019:
- HUMAN_AUTOINFO time
- EEV_ONLY mode
- EEV_Support
- EEV auto poweron/poweroff every 10 sec
- EEV_recalibration_time to stop HP and recalibrate EEV from zero level ex: every 24 hours

v1.2, 16 Apr 2019:
- "Type F" support

v1.3, 30 Apr 2019:
- EEV changed "overheating" to "delta T"
- EEV algo v1.1

v1.4, 02 Jun 2019:
- minor fixes
- EEV more asyncy
- T options to header

v1.5, 01 Jul 2019:
- prototyping 9

v1.6, 30 Apr 2021:
- sensors init issue fix

//TODO:
- 0.0 to -127 fix: only 2 attempts than pass 0.0
- poss. DoS: infinite read to nowhere, fix it, set finite counter (ex: 200)
- Dev and Host ID to header
- add speaker and err code for ""ERR: no Tae or Tbe for EEV!""
- min_user_t/max_user_t to header
- rs485_modbus
- full relays halification
? wclose and fclose to EEV
- liquid ref. protection: start cold circle and sump heater if tsump =< tco/tci+1, add option to header
- periodical start of hot side circle
- valve_4way
- inputs support
- ? emergency jumper support
- ? rewite re-init proc from MAGIC to emergency jumper removal at board start
- ? EEV target to EEPROM
- ? list T and other things on screen with buttons
- ? EEV define maximum working position
- ? few devices at same lane for RS485_HUMAN
*/
//-----------------------changelog END-----------------------

// DS18B20 pins: GND DATA VDD

//Connections:
//DS18B20 Pinout (Left to Right, pins down, flat side toward you)
//- Left   = Ground
//- Center = Signal (Pin N of arduino):  (with 3.3K to 4.7K resistor to +5 or 3.3 )
//- Right  = +5 or +3.3 V
//


//
// high volume scheme:        +---- +5V (12V not tested)
//                            |
//                       +----+
//                    1MOhm   piezo
//                       +----+
//                            |(C)
// pin -> 1.6 kOhms -> (B) 2n2222        < front here
//                            |(E)
//                            +--- GND
//

/*
scheme SCT-013-000:

2 pins used: tip and sleeve, center (ring) not used http://cms.35g.tw/coding/wp-content/uploads/2014/09/SCT-013-000_UNO-1.jpg
pins are interchangeable due to AC

32 Ohms (22+10) between sensor pins  (35 == ideal)

Pin1: 
- via elect. cap. to GND
- via ~10K..470K resistor to GND
- via ~10K..470K resistor to +5 (same as prev.)
if 10K+10K used: current is 25mA
use 100K+100K for 3 phases

Pin2:
- to analog pin
- via 32..35 Ohms resistor to Pin1

+5 -------------------------+
                            |                  
                            |                                            
                            # R1 10K+                        
                            |                                
                            |                                
                            |~2.5 at this point              
            +---------------+--------------------------------------+----+
            |               |                                      |    |
            #_ elect. cap.  # R2 10K+ (same as R1)     SCT-013-000 $    # R3 = 35 Ohms (ideal case), 32 used  
            |               |                                      |    |
GND --------+---------------+                                      +----+--------> to Analog pin


WARNING: calibrate 3 sensors together, from different sellers, due to case of incorrectly worked 1 of 3 sensor

P(watts)=220*220/R(Ohms)
*/

//
//MAX 485 voltage - 5V
//
// use resistor at RS-485 GND
// 1st test: 10k result lot of issues
// 2nd test: 1k, issues
// 3rd test: 100, see discussions


//16-ch Multiplexer EN pin: active LOW, connect to GND

//used pins:
//!!! ACTUALISE
//2: Z
//3: S3
//4: S2
//5: S1
//6: S0
//7: relay 2
//8: relay 3
//9: speaker
//10: relay 4
//11-13: rs485
//A0: relay 1
//A1: power monitor

/*
relay 1: heat pump
relay 2: hot side pump
relay 3: cold side pump
relay 4: (future) heatpump sump heater

t0: room
t1: heatpump sump
t2: cold in
t3: cold out
t4: hot in
t5: hot out
t6: before condenser
t7: condenser-evaporator
t8: after evaporator
t9: outer
tA: warm floor

wattage1

*/

String fw_version = "1.6";

#ifdef DISPLAY_096
#define DISPLAY DISPLAY_096
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;
#endif

#ifdef DISPLAY_1602
#define DISPLAY DISPLAY_1602
#include <Wire.h>
#include "LiquidCrystal_I2C.h"
//LiquidCrystal_I2C lcd(0x3f,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
//LiquidCrystal_I2C lcd(0x27, 20, 4);
LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

#ifdef DISPLAY_NONE
#define DISPLAY DISPLAY_NONE
#endif

#ifndef DISPLAY
#define DISPLAY -1
#endif

//

#ifdef INPUTS_AS_BUTTONS
#define INPUTS INPUTS_AS_BUTTONS
#endif

#ifdef INPUTS_AS_INPUTS
#define INPUTS INPUTS_AS_INPUTS
#endif

//

#ifdef RS485_PYTHON
#define RS485 RS485_PYTHON
char ishuman = 0;
#endif

#ifdef RS485_HUMAN
#define RS485 RS485_HUMAN
char ishuman = 1;
#endif

#ifdef RS485_NONE
char ishuman = 0;
#endif

//hardware resources
#define OW_BUS_ALLTSENSORS 12
#define SerialTxControl 13  //RS485 Direction control DE and RE to this pin
#define speakerOut 6
#define em_pin1 A6
#define emergency_pin A7

#ifdef BOARD_TYPE_G
String hw_version = "Type G v1.x";
//+Awaria
// #define RELAY_HEATPUMP 8
// #define RELAY_HOTSIDE_CIRCLE 9
// #define RELAY_COLDSIDE_CIRCLE 7
// #define RELAY_SUMP_HEATER 10
// #define RELAY_4WAY_VALVE 11

#define RELAY_HEATPUMP 7
#define RELAY_HOTSIDE_CIRCLE 9
#define RELAY_COLDSIDE_CIRCLE 11
#define RELAY_SUMP_HEATER 10
#define RELAY_4WAY_VALVE 8
//-Awaria

#ifdef INPUTS_AS_BUTTONS
#define BUT_RIGHT A3
#define BUT_LEFT A2
#define BUT_3 A1
#endif
#ifdef EEV_SUPPORT
#define EEV_1 2
#define EEV_2 4
#define EEV_3 3
#define EEV_4 5
#endif
#endif
#ifdef BOARD_TYPE_F
String hw_version = "Type F v1.x";
#define RELAY_HEATPUMP 7
#define RELAY_COLDSIDE_CIRCLE 8
#define LATCH_595 10
#define CLK_595 11
#define DATA_595 9
//595.0: relay 3 RELAY_HOTSIDE_CIRCLE, 595.1: relay 4 RELAY_SUMP_HEATER, 595.2: relay 5 RELAY_4WAY_VALVE, 595.3: uln 6, 595.4: uln 7, 595.5: uln 8, 595.6: uln 9, 595.7: uln 10
#ifdef EEV_SUPPORT
#define EEV_1 5
#define EEV_2 3
#define EEV_3 4
#define EEV_4 2
#endif
#ifdef INPUTS_AS_BUTTONS  //not sure
#define BUT_RIGHT A3
#define BUT_LEFT A2
#endif

#endif
#ifdef BOARD_TYPE_G9
String hw_version = "Type G9 v1.x";
#define RELAY_4WAY_VALVE 8
#define RELAY_SUMP_HEATER 7

#define LATCH_595 10
#define CLK_595 9
#define DATA_595 11
#define OE_595 A1
/*
	595.0: relay 10(not used)
	595.1: relay 8
	595.2: relay 9
	595.3: relay 5 		RELAY_HEATPUMP
	595.4: relay 4 		RELAY_COLDSIDE_CIRCLE
	595.5: relay 3 		RELAY_HOTSIDE_CIRCLE
	595.6: relay 6
	595.7: relay 7
	*/
#ifdef EEV_SUPPORT
#define EEV_1 2
#define EEV_2 4
#define EEV_3 3
#define EEV_4 5
#endif
#endif
//---------------------------memory debug
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char *sbrk(int incr);
#else   // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char *>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else   // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
//---------------------------memory debug END


#include <avr/wdt.h>
#include <EEPROM.h>
//#include <FastCRC.h>
/*FastCRC16 CRC16;
union _crc {
	unsigned int   integer;
	char           bytes[2];
} crc;
*/

#include <SoftwareSerial.h>

#define SerialRX 0  //RX connected to RO - Receiver Output
#define SerialTX 1  //TX connected to DI - Driver Output Pin
#define RS485Transmit HIGH
#define RS485Receive LOW

const char devID = 0x41;
const char hostID = 0x30;

SoftwareSerial RS485Serial(SerialRX, SerialTX);  // RX, TX

#include <OneWire.h>
#include <DallasTemperature.h>
//library's DEVICE_DISCONNECTED_C -127.0

OneWire ow_ALLTSENSORS(OW_BUS_ALLTSENSORS);
DallasTemperature s_allTsensors(&ow_ALLTSENSORS);

typedef struct {
  DeviceAddress addr;
  bool e;  //enabled
  double T;
} st_tsens;

DeviceAddress dev_addr;  //temp

st_tsens Tae;
st_tsens Tbe;
st_tsens Ttarget;
st_tsens Tsump;
st_tsens Tci;
st_tsens Tco;
st_tsens Thi;
st_tsens Tho;
st_tsens Tbc;
st_tsens Tac;
st_tsens Touter;
st_tsens Ts1;
st_tsens Ts2;

#define BIT_Tae 0
#define BIT_Tbe 1
#define BIT_Ttarget 2
#define BIT_Tsump 3
#define BIT_Tci 4
#define BIT_Tco 5
#define BIT_Thi 6
#define BIT_Tho 7
#define BIT_Tbc 8
#define BIT_Tac 9
#define BIT_Touter 10
#define BIT_Ts1 11
#define BIT_Ts2 12

unsigned int used_sensors = 0;  //bit array

double T_delta = 0.0;
double T_setpoint = 30.0;
double T_setpoint_lastsaved = T_setpoint;
double T_EEV_setpoint = EEV_TARGET_TEMP_DIFF;
double T_EEV_dt = 0.0;       //real, used during run
double T_EEV_Be = 0.0;  
double T_EEV_Ae = 0.0;  
//bool T_EEV_dt_change = 0;

const double cT_delta_max = T_DELTA_MAX;
const double cT_setpoint_max = T_SETPOINT_MAX;
const double cT_hotcircle_delta_min = T_HOTCIRCLE_DELTA_MIN;
const double cT_sump_min = T_SUMP_MIN;
const double cT_sump_max = T_SUMP_MAX;
const double cT_sump_heat_threshold = T_SUMP_HEAT_THRESHOLD;
//const double cT_sump_outerT_threshold	= 18.0;    	//?? seems to be not useful
const double cT_before_condenser_max = T_BEFORE_CONDENSER_MAX;
const double cT_after_evaporator_min = T_AFTER_EVAPORATOR_MIN;  // working evaporation presure ~= -10, it is constant due to large evaporator volume     // waterhouse v1: -12 is too high
const double cT_evaporator_war = T_EVAPORATOR_WAR;
const double cT_cold_min = T_COLD_MIN;
const double cT_hotout_max = T_HOTOUT_MAX;
//const double cT_workingOK_cold_delta_min = 0.5; 	// 0.7 - 1st try, 2nd try 0.5
//const double cT_workingOK_hot_delta_min	= 0.5;
const double cT_workingOK_sump_min = T_WORKINGOK_SUMP_MIN;  //need to be not very high to normal start after deep freeze
const double c_wattage_max = MAX_WATTS;                     //FUNAI: 1000W seems to be normal working wattage INCLUDING 1(one) CR25/4 at 3rd speed
                                                            //PH165X1CY : 920 Watts, 4.2 A
//const double c_workingOK_wattage_min 	= c_wattage_max/2.5;     //
const double c_workingOK_wattage_min = c_wattage_max / 4;

bool heatpump_state = 0;
bool hotside_circle_state = 0;
bool coldside_circle_state = 0;
bool sump_heater_state = 0;
bool valve4w_state = 0;

bool relay6_state = 0;
bool relay7_state = 0;
bool relay8_state = 0;
bool relay9_state = 0;

const long poweron_pause = POWERON_PAUSE;          //default 5 mins
const long mincycle_poweroff = MINCYCLE_POWEROFF;  //default 5 mins
const long mincycle_poweron = MINCYCLE_POWERON;    //default 60 mins
bool _1st_start_sleeped = 0;
//??? TODO: periodical start ?
//const long floor_circle_maxhalted = 6000000;  //circle NOT works max 100 minutes
const long deffered_stop_hotcircle = DEFFERED_STOP_HOTCIRCLE;
const long deffered_stop_coldcircle = DEFFERED_STOP_COLDCIRCLE;

int EEV_cur_pos = 0;
int EEV_apulses = 0;  //for async
bool EEV_adonotcare = 0;
const unsigned char EEV_steps[4] = { 0b1010, 0b0110, 0b0101, 0b1001 };
char EEV_cur_step = 0;
bool EEV_fast = 0;

//main cycle vars
unsigned long millis_prev = 0;
unsigned long millis_now = 0;
unsigned long millis_cycle = 1000;

unsigned long millis_last_heatpump_on = 0;
unsigned long millis_last_heatpump_off = 0;

unsigned long millis_notification = 0;
unsigned long millis_notification_interval = 33000;

unsigned long millis_displ_update = 0;
unsigned long millis_displ_update_interval = 10000;
unsigned long displ_inc = 0;

unsigned long millis_escinput = 0;
unsigned long millis_charinput = 0;

unsigned long millis_lasteesave = 0;

unsigned long millis_last_printstats = 0;

unsigned long millis_eev_last_close = 0;
unsigned long millis_eev_last_on = 0;
unsigned long millis_eev_last_step = 0;

unsigned long overload_count = 0;

int skipchars = 0;

#define ERR_HZ 2500

char inData[50];   // Allocate some space for the string, do not change that size!
char inChar = -1;  // space to store the character read
byte index = 0;    // Index into array; where to store the character

//-------------temporary variables
char temp[10];
int i = 0;
int z = 0;
int x = 0;
int y = 0;
int d = 0;
double tempdouble = 0.0;
int tempint = 0;

String outString;
//-------------EEPROM
int eeprom_magic_read = 0x00;
int eeprom_addr = 0x00;
//initial values, saved to EEPROM and can be modified later
//CHANGE eeprom_magic after correction!
const int eeprom_magic = MAGIC;

//-------------ERROR states
#define ERR_OK 0
#define ERR_T_SENSOR 1
#define ERR_HOT_PUMP 2
#define ERR_COLD_PUMP 3
#define ERR_HEATPUMP 4
#define ERR_WATTAGE 5

int errorcode = 0;


//--------------------------- for wattage
#define ADC_BITS 10  //10 fo regular arduino
#define ADC_COUNTS (1 << ADC_BITS)
float em_calibration = 62.5;
int em_samplesnum = 2960;  // Calculate Irms only 1480 == full 14 periods for 50Hz
//double Irms       	= 0;      	//for tests with original procedure
int supply_voltage = 0;
int em_i = 0;
//phase 1
int sampleI_1 = 0;
double filteredI_1 = 0;
double offsetI_1 = ADC_COUNTS >> 1;  //Low-pass filter output
double sqI_1, sumI_1 = 0;            //sq = squared, sum = Sum, inst = instantaneous
double async_Irms_1 = 0;
double async_wattage = 0;
int emergency = 0;
//--------------------------- for wattage END

//--------------------------- functions
long ReadVcc() {
// Read 1.1V reference against AVcc
// set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2);             // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);  // Start conversion
  while (bit_is_set(ADCSRA, ADSC))
    ;  // measuring

  uint8_t low = ADCL;   // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH;  // unlocks both

  long result = (high << 8) | low;
  //constant NOT same as in battery controller!
  result = 1126400L / result;  // Calculate Vcc (in mV); (me: !!) 1125300  (!!) = 1.1*1023*1000
  return result;               // Vcc in millivolts
}

char CheckAddrExists(void) {

  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Tae.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Tbe.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Ttarget.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Tsump.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Tci.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Tco.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Thi.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Tho.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Tbc.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Tac.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Touter.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Ts1.addr[i]) break;
  }
  if (i == 8) return 1;
  for (i = 0; i < 8; i++) {
    if (dev_addr[i] != Ts2.addr[i]) break;
  }
  if (i == 8) return 1;
  return 0;

  /*
	//incorrect way: 0.06 % chance for 13 sensors to false positive, calculated for true random.
	for (i = 0; i < 8; i++) {	
		if (	(dev_addr[i] != Tae.addr[i]) 	&&
			(dev_addr[i] != Tbe.addr[i]) 	&&
			(dev_addr[i] != Ttarget.addr[i]) &&
			(dev_addr[i] != Tsump.addr[i]) 	&&
			(dev_addr[i] != Tci.addr[i]) 	&&
			(dev_addr[i] != Tco.addr[i]) 	&&
			(dev_addr[i] != Thi.addr[i]) 	&&
			(dev_addr[i] != Tho.addr[i]) 	&&
			(dev_addr[i] != Tbc.addr[i]) 	&&
			(dev_addr[i] != Tac.addr[i]) 	&&
			(dev_addr[i] != Touter.addr[i]) &&
			(dev_addr[i] != Ts1.addr[i]) 	&&
			(dev_addr[i] != Ts2.addr[i]) 
		)
			break;	
	}
	if (i == 8) return 1;
	return 0;	
	*/
}

void InitS_and_D(void) {
#ifdef DISPLAY_096
  Wire.begin();
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
#endif
#ifdef DISPLAY_1602
  lcd.init();       // initialize the lcd
  lcd.begin(16,2);
  lcd.backlight();  // not really needed
#endif
  RS485Serial.begin(9600);
}

void PrintS(String str) {
#ifdef RS485_HUMAN
  char *outChar = &str[0];
  digitalWrite(SerialTxControl, RS485Transmit);
  delay(1);
  RS485Serial.print(outChar);
  RS485Serial.println();
  RS485Serial.flush();
  digitalWrite(SerialTxControl, RS485Receive);
#endif
}

void PrintS_and_D(String str, int printSerial = 1) {
  char *outChar = &str[0];

  if (str == "") {
    return;
  }

  //#ifdef RS485_HUMAN
  if (ishuman != 0) {
    if (printSerial == 1) {
      digitalWrite(SerialTxControl, RS485Transmit);
      delay(1);
      RS485Serial.print(outChar);
      RS485Serial.println();
      RS485Serial.flush();
      digitalWrite(SerialTxControl, RS485Receive);
    }
  }
  //#endif

#ifdef DISPLAY_096
  oled.clear();
  oled.println(str);
#endif
#ifdef DISPLAY_1602
  lcd.backlight();
  lcd.clear();
  lcd.print(str);
#endif
}

void Print_D2(String outString, int line = 0) {
#ifdef DISPLAY_1602
  lcd.setCursor(0, line);
  lcd.print(outString);
#endif
}

// void _PrintHelp(void) {
//   PrintS("CHPC, https://github.com/gonzho000/chpc/ fw: " + fw_version + " board: " + hw_version);
//   PrintS(F("Commands: \n (?) help\n (+) increase aim T\n (-) decrease aim T\n \n"));
// #ifdef EEV_SUPPORT
//   PrintS(F("(<) decrease EEV T diff \n(>) increase EEV T diff"));
// #endif
//   PrintS(F("(G) get stats"));
// }

void PrintS_and_D_double(double double_to_print) {
  dtostrf(double_to_print, 1, 2, temp);
  PrintS_and_D(temp);
}

void Inc_Tdelta(void) {
  T_delta += 1;
  if (T_delta > cT_delta_max) {
    T_delta = 0;
  }
}

int Inc_T(void) {
  if (T_setpoint + 0.5 > cT_setpoint_max) {
    PrintS_and_D(F("Max!"));
    delay(200);
    return 0;
  }
  T_setpoint += 0.5;
  PrintS_and_D_double(T_setpoint);
  return 1;
}

int Dec_T(void) {
  if (T_setpoint - 0.5 < 1.0) {
    PrintS_and_D(F("Min!"));
    delay(200);
    return 0;
  }
  T_setpoint -= 0.5;
  PrintS_and_D_double(T_setpoint);
  return 1;
}

int Inc_E(void) {  ///!!!!!! unprotected
  T_EEV_setpoint += 0.25;
  PrintS_and_D_double(T_EEV_setpoint);
  return 1;
}

int Dec_E(void) {  ///!!!!!! unprotected
  T_EEV_setpoint -= 0.25;
  PrintS_and_D_double(T_EEV_setpoint);
  return 1;
}

void print_Serial_SaD(String str, double double_to_print = -999) {
  char *outChar = &str[0];
  delay(1);
  RS485Serial.print(outChar);
  if (double_to_print = -999) {
    RS485Serial.println();
  } else {
    dtostrf(double_to_print, 1, 2, temp);
    RS485Serial.println(temp);
  }
}

void PrintStats_Serial(void) {
#ifdef RS485_HUMAN
  digitalWrite(SerialTxControl, RS485Transmit);
  delay(1);
  //print_Serial_SaD("--- Stats serial ----");
  if (Tae.e == 1) {
    outString = "Tae: " + String(Tae.T, 1);
    print_Serial_SaD(outString);
  }
  if (Tbe.e == 1) {
    outString = "Tbe: " + String(Tbe.T, 1);
    print_Serial_SaD(outString);
  }

  outString = String(T_EEV_dt, 1);
  print_Serial_SaD("EEV_dt: " + outString);
  //outString = String(T_EEV_dt_change, 1);
  //print_Serial_SaD("EEV_dt change: " + outString);

  if (Ttarget.e == 1) {
    outString = "Ttarget: " + String(Ttarget.T, 1);
    print_Serial_SaD(outString);
  }
  if (Tsump.e == 1) {
    outString = "Tsump: " + String(Tsump.T, 1);
    print_Serial_SaD(outString);
  }
  if (Tci.e == 1) {
    outString = "Tci: " + String(Tci.T, 1);
    print_Serial_SaD(outString);
  }
  if (Tco.e == 1) {
    outString = "Tco: " + String(Tco.T, 1);
    print_Serial_SaD(outString);
  }
  if (Thi.e == 1) {
    outString = "Thi: " + String(Thi.T, 1);
    print_Serial_SaD(outString);
  }
  if (Tho.e == 1) {
    outString = "Tho: " + String(Tho.T, 1);
    print_Serial_SaD(outString);
  }
  if (Tbc.e == 1) {
    outString = "Tbc: " + String(Tbc.T, 1);
    print_Serial_SaD(outString);
  }
  if (Tac.e == 1) {
    outString = "Tac: " + String(Tac.T, 1);
    print_Serial_SaD(outString);
  }
  if (Touter.e == 1) {
    outString = "Touter: " + String(Touter.T, 1);
    print_Serial_SaD(outString);
  }
  if (Ts1.e == 1) {
    outString = "Ts1: " + String(Ts1.T, 1);
    print_Serial_SaD(outString);
  }
  if (Ts2.e == 1) {
    outString = "Ts2: " + String(Ts2.T, 1);
    print_Serial_SaD(outString);
  }

  outString = "T set min: " + String((T_setpoint - T_delta), 1);
  print_Serial_SaD(outString);
  outString = "T set max: " + String(T_setpoint, 1);
  print_Serial_SaD(outString);
  outString = String(async_wattage, 1);
  print_Serial_SaD("Watts: " + outString);
  outString = "Heatpump state: " + String((heatpump_state == 1) ? "ON" : "OFF");
  print_Serial_SaD(outString);
  outString = "Err: " + String(errorcode);
  print_Serial_SaD(outString);
  outString = "Cold wather: " + String(emergency);
  print_Serial_SaD(outString);

  if (heatpump_state == 1) {
    outString = String((unsigned long)(millis() - millis_last_heatpump_on) / 1000);
    print_Serial_SaD("Cykle HP power on: " + outString);
    outString = String((unsigned long)(mincycle_poweron) / 1000);
    print_Serial_SaD("Min. cykle power on: " + outString);

  } else if (heatpump_state == 0 && millis_last_heatpump_off != 0) {
    outString = String((unsigned long)(millis() - millis_last_heatpump_off) / 1000);
    print_Serial_SaD("Cykle HP power off: " + outString);
    outString = String((unsigned long)(mincycle_poweroff) / 1000);
    print_Serial_SaD("Min. cykle power off: " + outString);
  }

  outString = "Hot circle state: " + String((hotside_circle_state == 1) ? "ON" : "OFF");
  print_Serial_SaD(outString);
  outString = "Cold circle state: " + String((coldside_circle_state == 1) ? "ON" : "OFF");
  print_Serial_SaD(outString);

  // if ( hotside_circle_state  == 1 && heatpump_state == 0) {
  //     if (  ( Tho.e == 1 && Tho.T > (Ttarget.T + cT_hotcircle_delta_min)  ) ||
  //           ( Thi.e == 1 && Thi.T > (Ttarget.T + cT_hotcircle_delta_min)  )) {
  //         outString = String((unsigned long)(Ttarget.T) + cT_hotcircle_delta_min,1) + " > Tho or Thi ";
  //         print_Serial_SaD( "Target + String > Tho or Thi);
  //     }
  //   }

#ifdef EEV_SUPPORT
  print_Serial_SaD("EEV: " + String(T_EEV_setpoint, 1));
  outString = "EEV_pos: " + String(EEV_cur_pos);
  print_Serial_SaD(outString);
  outString = "EEV_apulses: " + String(EEV_apulses);
  print_Serial_SaD(outString);
#endif

  RS485Serial.println("/");
  RS485Serial.flush();
  digitalWrite(SerialTxControl, RS485Receive);
  delay(1);
#endif
}

void ReadEECheckAddr(unsigned char *to_addr) {
  for (i = 0; i < 8; i++) {
    to_addr[i] = EEPROM.read(eeprom_addr);
    eeprom_addr++;
  }
  i = 0;
  CheckIsInvalidCRCAddr(to_addr);
  if (i != 0) {
    while (1) {
      //PrintAddr(to_addr);
      PrintS_and_D(F("Err:EEPROM, reinit!"));
      delay(5000);
    }
  }
}

void CheckIsInvalidCRCAddr(unsigned char *addr) {
  if (OneWire::crc8(addr, 7) != addr[7]) {
    i += 1;
  }
}

void CopyAddrStoreEE(unsigned char *addr_to, int bit_offset) {  //get result from dev_addr, autoincrement eeprom_addr
  //dev_addr and z from globals used
  for (i = 0; i < 8; i++) {  //no matter
    if (z == 0) {
      dev_addr[i] = 0x00;
    }
    addr_to[i] = dev_addr[i];
    EEPROM.write(eeprom_addr, dev_addr[i]);
    eeprom_addr++;
  }
  bitWrite(used_sensors, bit_offset, z);
}

void WriteFloatEEPROM(int addr, float val) {
  byte *x = (byte *)&val;
  for (byte u = 0; u < 4; u++) EEPROM.write(u + addr, x[u]);
}

float ReadFloatEEPROM(int addr) {
  byte x[4];
  for (byte u = 0; u < 4; u++) x[u] = EEPROM.read(u + addr);
  float *y = (float *)&x;
  return y[0];
}

void SaveSetpointEE(int pforce = 0) {
  if ((T_setpoint_lastsaved != T_setpoint) && (pforce == 1 || ((unsigned long)(millis_now - millis_lasteesave) > 15 * 60 * 1000) || (millis_lasteesave == 0))) {
    eeprom_addr = 1;
    WriteFloatEEPROM(eeprom_addr, T_setpoint);
    millis_lasteesave = millis_now;
    T_setpoint_lastsaved = T_setpoint;
  }
}

void PrintAddr(unsigned char *str) {
  outString = "";
  for (i = 0; i < 8; i++) {
    if (str[i] < 0x10) outString += "0";
    outString += String(str[i], HEX);
  }
  PrintS_and_D(outString);
}

unsigned char FindAddr(String what, int required = 0) {
  i = 1;
  while (RS485Serial.available() > 0) {
    inChar = RS485Serial.read();
    delay(1);
  }
  inChar = 0x00;
  while (1) {
    while (!s_allTsensors.getAddress(dev_addr, 0)) {
      if (required == 0) {
        PrintS_and_D(F("Press > to skip"));
        delay(500);
        while (RS485Serial.available() > 0) {
          inChar = RS485Serial.read();
          if (inChar == 0x3E) {
            PrintS_and_D("Skipped: " + what);
            return 0;
          }
        }
#ifdef INPUTS_AS_BUTTONS
        i = digitalRead(BUT_RIGHT);
        if (i == 1) {
          PrintS_and_D("Skipped: " + what);
          delay(4000);
          return 0;
        }
#endif
      }
      PrintS_and_D("Insert " + what);
      delay(1000);
    }
    if (OneWire::crc8(dev_addr, 7) != dev_addr[7]) {
      PrintS_and_D(F("Invalid CRC! Remove and insert same sensor!\n"));
      delay(200);
      continue;
    } else if (CheckAddrExists() == 1) {
      PrintS_and_D(F("USED! Remove!"));
      delay(1000);
      continue;
    } else {
      break;
    }
  }
  while (1) {
    PrintAddr(dev_addr);
    delay(1000);
    if (s_allTsensors.getAddress(dev_addr, 0)) {
      PrintS_and_D("OK! Remove " + what);
      delay(1000);
    } else {
      delay(100);
      break;
    }
  }
  return 1;
}

double GetT(unsigned char *str) {
  tempdouble = -127.0;
  for (i = 0; i < 8; i++) {
#ifdef WATCHDOG
    wdt_reset();
#endif
#ifdef EEV_SUPPORT
    eevise();
#endif
    tempdouble = s_allTsensors.getTempC(str);
    if ((tempdouble == 85.0) || (tempdouble == -127.0)) {
      if (tempdouble == 85.0) {  //initial value in dallas register after poweron
        delay(375);              //375 actual for 11 bits resolution, 2-3 retries OK for 12-bits resolution
      } else {
        delay(37);
      }
    } else {
      break;
    }
  }
  return tempdouble;
}

void Get_Temperatures(void) {
  if (Tae.e) Tae.T = GetT(Tae.addr);
  if (Tbe.e) Tbe.T = GetT(Tbe.addr);
  if (Ttarget.e) Ttarget.T = GetT(Ttarget.addr);
  if (Tsump.e) Tsump.T = GetT(Tsump.addr);
  if (Tci.e) Tci.T = GetT(Tci.addr);
  if (Tco.e) Tco.T = GetT(Tco.addr);
  if (Thi.e) Thi.T = GetT(Thi.addr);
  if (Tho.e) Tho.T = GetT(Tho.addr);
  if (Tbc.e) Tbc.T = GetT(Tbc.addr);
  if (Tac.e) Tac.T = GetT(Tac.addr);
  if (Touter.e) Touter.T = GetT(Touter.addr);
  if (Ts1.e) Ts1.T = GetT(Ts1.addr);
  if (Ts2.e) Ts2.T = GetT(Ts2.addr);
  s_allTsensors.requestTemperatures();  //global request
                                        //---------DEBUG and self-test !!!--------
                                        /*PrintS_and_D("");
	PrintS_and_D_double(Tae.T);
	PrintS_and_D_double(Tbe.T);
	PrintS_and_D_double(Ttarget.T);
	PrintS_and_D_double(Tsump.T);
	PrintS_and_D("");*/
                                        /*
	PrintS_and_D("Sensor 1 ");
	PrintS_and_D_double(tr_sens_1);
	PrintS_and_D(",\tci ");
	PrintS_and_D_double(tr_cold_in);
	PrintS_and_D(",\tcout ");
	PrintS_and_D_double(tr_cold_out);
	PrintS_and_D(",\thin ");
	PrintS_and_D_double(tr_hot_in);    
	PrintS_and_D(",\tho ");
	PrintS_and_D_double(tr_hot_out);
	PrintS_and_D(",\tbcond ");
	PrintS_and_D_double(tr_before_condenser);    
	PrintS_and_D(",\t outer ");
	PrintS_and_D_double(tr_outer);
	PrintS_and_D(",\t Sensor 2 ");
	PrintS_and_D_double(tr_sens_2);
	*/

  //---------DEBUG END--------
}

#ifdef EEV_SUPPORT
void on_EEV() {  //1 = do not take care of position
  x = EEV_steps[EEV_cur_step];
  digitalWrite(EEV_1, bitRead(x, 0));
  digitalWrite(EEV_2, bitRead(x, 1));
  digitalWrite(EEV_3, bitRead(x, 2));
  digitalWrite(EEV_4, bitRead(x, 3));
}

void off_EEV() {  //1 = do not take care of position
  digitalWrite(EEV_1, 0);
  digitalWrite(EEV_2, 0);
  digitalWrite(EEV_3, 0);
  digitalWrite(EEV_4, 0);
  //PrintS_and_D("off_EEV");
}

#endif

void halifise(void) {
#ifdef BOARD_TYPE_F
  /*#define LATCH_595 = 10;
		#define CLK_595 = 11;
		#DEFINE DATA_595 = 9;
		//595.0: relay 3 RELAY_HOTSIDE_CIRCLE, 595.1: relay 4 RELAY_SUMP_HEATER, 595.2: relay 5 RELAY_4WAY_VALVE, 595.3: uln 6, 595.4: uln 7, 595.5: uln 8, 595.6: uln 9, 595.7: uln 10 
		*/
  digitalWrite(LATCH_595, 0);
  //7
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, 0);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //6
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, 0);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //5
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, 0);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //4
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, 0);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //3
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, 0);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //2
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, 0);  //4way valve here
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //1
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, sump_heater_state);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //0
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, hotside_circle_state);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(CLK_595, 0);
  //
  digitalWrite(LATCH_595, 1);

  digitalWrite(RELAY_HEATPUMP, heatpump_state);
  digitalWrite(RELAY_COLDSIDE_CIRCLE, coldside_circle_state);
#endif
#ifdef BOARD_TYPE_G
  digitalWrite(RELAY_SUMP_HEATER, sump_heater_state);
  digitalWrite(RELAY_HOTSIDE_CIRCLE, hotside_circle_state);
  digitalWrite(RELAY_HEATPUMP, heatpump_state);
  digitalWrite(RELAY_COLDSIDE_CIRCLE, coldside_circle_state);
#endif
#ifdef BOARD_TYPE_G9
  //#define RELAY_4WAY_VALVE      8
  //#define RELAY_SUMP_HEATER 	7
  /*
		595.0: relay 10(not used)
		595.1: relay 8		//use for 1st test of DAC
		595.2: relay 9		//use for 1st test of DAC
		595.3: relay 5 		RELAY_HEATPUMP
		595.4: relay 4 		RELAY_COLDSIDE_CIRCLE
		595.5: relay 3 		RELAY_HOTSIDE_CIRCLE
		595.6: relay 6
		595.7: relay 7		
		*/

  digitalWrite(LATCH_595, 0);
  //7
  digitalWrite(CLK_595, 0);
  digitalWrite(DATA_595, relay7_state);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //6
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, relay6_state);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //5
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, hotside_circle_state);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //4
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, coldside_circle_state);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //3
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, heatpump_state);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //2
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, relay9_state);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //1
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, relay8_state);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  //0
  digitalWrite(CLK_595, 0);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(DATA_595, 0);
  digitalWrite(CLK_595, 1);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(CLK_595, 0);
  //
  digitalWrite(LATCH_595, 1);
  __asm__ __volatile__("nop\n\t");
  digitalWrite(LATCH_595, 0);
  digitalWrite(RELAY_SUMP_HEATER, sump_heater_state);
  digitalWrite(RELAY_4WAY_VALVE, valve4w_state);
#endif
}

void eevise(void) {
  int eee = EEV_MAXPULSES_OPEN; 
  if( Ttarget.T > 41 ) {
    eee = EEV_MAXPULSES_OPEN - 1;
  } if( Ttarget.T > 42 ) {
    eee = EEV_MAXPULSES_OPEN - 2;
  } if( Ttarget.T > 43 ) {
    eee = EEV_MAXPULSES_OPEN - 3;
  } if( Ttarget.T > 44 ) {
    eee = EEV_MAXPULSES_OPEN - 4;
  } if( Ttarget.T > 45 ) {
    eee = EEV_MAXPULSES_OPEN - 5;
  }

  if (EEV_cur_pos > eee) {
    EEV_apulses = -1;
  }

  if ((((EEV_apulses < 0) && (EEV_fast == 1)) && ((unsigned long)(millis_now - millis_eev_last_step) > (EEV_PULSE_FCLOSE_MILLIS))) || (((EEV_apulses < 0) && (EEV_fast == 0)) && ((unsigned long)(millis_now - millis_eev_last_step) > (EEV_PULSE_CLOSE_MILLIS))) || (((EEV_apulses > 0) && (EEV_cur_pos < EEV_MINWORKPOS)) && ((unsigned long)(millis_now - millis_eev_last_step) > (EEV_PULSE_WOPEN_MILLIS))) || (((EEV_apulses > 0) && (EEV_fast == 1) && (EEV_cur_pos >= EEV_MINWORKPOS)) && ((unsigned long)(millis_now - millis_eev_last_step) > (EEV_PULSE_FOPEN_MILLIS))) || (((EEV_apulses > 0) && (EEV_fast == 0) && (EEV_cur_pos >= EEV_MINWORKPOS)) && ((unsigned long)(millis_now - millis_eev_last_step) > (EEV_PULSE_OPEN_MILLIS))) || (millis_eev_last_step == 0)) {

    if (EEV_apulses != 0) {
      if (EEV_apulses > 0) {
        if (EEV_cur_pos + 1 <= eee) {
          EEV_cur_pos += 1;
          EEV_cur_step += 1;
          EEV_apulses -= 1;
        } else {
          EEV_apulses = 0;
#ifdef EEV_DEBUG
          PrintS_and_D("EEmax!");
#endif
        }
      }

      if (EEV_apulses < 0) {
        if ((EEV_cur_pos - 1 >= EEV_MINWORKPOS) || (EEV_adonotcare == 1)) {
          EEV_cur_pos -= 1;
          EEV_cur_step -= 1;
          EEV_apulses += 1;
        } else {
          EEV_apulses = 0;
#ifdef EEV_DEBUG
          PrintS_and_D("EEmin!");
#endif
        }
      }

      if (EEV_cur_step > 3) EEV_cur_step = 0;
      if (EEV_cur_step < 0) EEV_cur_step = 3;

      x = EEV_steps[EEV_cur_step];
      digitalWrite(EEV_1, bitRead(x, 0));
      digitalWrite(EEV_2, bitRead(x, 1));
      digitalWrite(EEV_3, bitRead(x, 2));
      digitalWrite(EEV_4, bitRead(x, 3));
    }

    if (EEV_cur_pos < 0) {
      EEV_cur_pos = 0;
    }

    millis_eev_last_step = millis_now;
#ifdef RS485_HUMAN
//	print_Serial_SaD("New EEV pos: " + String (EEV_cur_pos));
#endif
  }
}

//--------------------------- functions END

void setup(void) {

#ifdef BOARD_TYPE_G
  pinMode(RELAY_HEATPUMP, OUTPUT);
  pinMode(RELAY_COLDSIDE_CIRCLE, OUTPUT);
  digitalWrite(RELAY_HEATPUMP, LOW);
  digitalWrite(RELAY_COLDSIDE_CIRCLE, LOW);
  //
  pinMode(RELAY_SUMP_HEATER, OUTPUT);
  pinMode(RELAY_HOTSIDE_CIRCLE, OUTPUT);
  digitalWrite(RELAY_SUMP_HEATER, LOW);
  digitalWrite(RELAY_HOTSIDE_CIRCLE, LOW);
  halifise();
#endif
#ifdef BOARD_TYPE_F
  pinMode(RELAY_HEATPUMP, OUTPUT);
  pinMode(RELAY_COLDSIDE_CIRCLE, OUTPUT);
  digitalWrite(RELAY_HEATPUMP, LOW);
  digitalWrite(RELAY_COLDSIDE_CIRCLE, LOW);
  //
  pinMode(LATCH_595, OUTPUT);
  pinMode(CLK_595, OUTPUT);
  pinMode(DATA_595, OUTPUT);
  digitalWrite(LATCH_595, LOW);
  digitalWrite(CLK_595, LOW);
  digitalWrite(DATA_595, LOW);
  halifise();
#endif
#ifdef BOARD_TYPE_G9
  pinMode(LATCH_595, OUTPUT);
  pinMode(CLK_595, OUTPUT);
  pinMode(DATA_595, OUTPUT);
  pinMode(RELAY_SUMP_HEATER, OUTPUT);
  pinMode(RELAY_4WAY_VALVE, OUTPUT);
  pinMode(OE_595, OUTPUT);
  digitalWrite(LATCH_595, LOW);
  digitalWrite(CLK_595, LOW);
  digitalWrite(DATA_595, LOW);
  digitalWrite(RELAY_SUMP_HEATER, LOW);
  digitalWrite(RELAY_4WAY_VALVE, LOW);
  halifise();
  digitalWrite(OE_595, LOW);
#endif

#ifdef WATCHDOG
  wdt_disable();
  delay(2000);
#endif
  InitS_and_D();
  pinMode(SerialTxControl, OUTPUT);
  digitalWrite(SerialTxControl, RS485Receive);
  //digitalWrite(SerialTxControl, RS485Transmit);
  //RS485Serial.println("starting...");	//!!!debug
  delay(100);
  PrintS_and_D("ID: 0x" + String(devID, HEX));
  //Print_Lomem(C_ID);
  delay(200);
#ifdef EEV_SUPPORT
  pinMode(EEV_1, OUTPUT);
  pinMode(EEV_2, OUTPUT);
  pinMode(EEV_3, OUTPUT);
  pinMode(EEV_4, OUTPUT);
  off_EEV();
#endif

  pinMode(em_pin1, INPUT);
  pinMode(emergency_pin, INPUT);

  //PrintS_and_D("setpoint (C):");
  //PrintS_and_D(setpoint);
  //PrintS_and_D(String(freeMemory()));  //!!! debug
  s_allTsensors.begin();
  s_allTsensors.setWaitForConversion(false);  //ASYNC mode, request before get, see Dallas library for details

  //----------------------------- self-tests !!!----------------------------- ----------------------------- -----------------------------
  /*
	digitalWrite(RELAY_HEATPUMP,HIGH);
	delay(300);
	digitalWrite(RELAY_HOTSIDE_CIRCLE,HIGH);
	delay(300);
	digitalWrite(RELAY_COLDSIDE_CIRCLE,HIGH);
	delay(300);
	digitalWrite(RELAY_SUMP_HEATER,HIGH);
	delay(2000);
	digitalWrite(RELAY_HEATPUMP,LOW);
	delay(300);
	digitalWrite(RELAY_HOTSIDE_CIRCLE,LOW);
	delay(300);
	digitalWrite(RELAY_COLDSIDE_CIRCLE,LOW);
	delay(300);
	digitalWrite(RELAY_SUMP_HEATER,LOW);
	*/
  /*
	tone(speakerOut, 2250);
	delay (500); // like ups power on
	noTone(speakerOut);
	*/
  /*	
	while ( 1 == 1) {
		
		heatpump_state    	= 1;	halifise();	delay(1000);
		coldside_circle_state 	= 1;	halifise();	delay(1000);
		hotside_circle_state  	= 1;	halifise();	delay(1000);
		sump_heater_state    	= 1;	halifise();	delay(1000);
		valve4w_state    	= 1;	halifise();	delay(1000);
		#ifdef BOARD_TYPE_G9
			relay6_state		= 1;	halifise();	delay(1000);
			relay7_state		= 1;	halifise();	delay(1000);
			relay8_state		= 1;	halifise();	delay(1000);
			relay9_state		= 1;	halifise();	delay(1000);
		#endif
		
		delay(3000);
		heatpump_state    	= 0;	halifise();	delay(1000);
		coldside_circle_state 	= 0;	halifise();	delay(1000);
		hotside_circle_state  	= 0;	halifise();	delay(1000);
		sump_heater_state    	= 0;	halifise();	delay(1000);
		valve4w_state    	= 0;	halifise();	delay(1000);
		#ifdef BOARD_TYPE_G9
			relay6_state		= 0;	halifise();	delay(1000);
			relay7_state		= 0;	halifise();	delay(1000);
			relay8_state		= 0;	halifise();	delay(1000);
			relay9_state		= 0;	halifise();	delay(1000);
		#endif
		delay(3000);
		break;
	}
	*/
  /*
	//EEV self-test
	while ( 1 == 1 ) {
		EEV_apulses =  -(EEV_MAXPULSES + EEV_CLOSE_ADD_PULSES);
		EEV_adonotcare = 1;
		EEV_fast = 1;
		while (EEV_apulses < 0){
			millis_now = millis();
			eevise();
		}
		delay(1000);
		EEV_apulses =  EEV_MAXPULSES;
		EEV_fast = 1;
		while (EEV_apulses > 0){
			millis_now = millis();
			eevise();
		}
		delay(1000);
	}	
	*/
  //----------------------------- self-test END----------------------------- ----------------------------- -----------------------------


  eeprom_magic_read = EEPROM.read(eeprom_addr);
#ifdef INPUTS_AS_BUTTONS
  pinMode(BUT_RIGHT, INPUT);
  //digitalWrite	(BUT_RIGHT, LOW);
  pinMode(BUT_LEFT, INPUT);
  //digitalWrite	(BUT_LEFT, LOW);
#endif
  //EEPROM content:
  //0x00 - magic,
  //0x01  .. 0x04 Target value,
  //0x05 and 0x06 if sensor enabled or not, used_sensors HI and LO
  //0x07  .. 0x0e 1st addr, etc..

  // tr_after_evaporator(0);       tr_before_evaporator(1);     tr_target(2);             tr_sump(3);
  // tr_cold_in(4);                tr_cold_out(5);              tr_hot_in(6);             tr_hot_out(7);
  // tr_before_condenser(8);       tr_after_condenser(9);       tr_outer(10);              tr_sens_1(11);
  // tr_sens_2(12);

  eeprom_addr = 0x00;
  if (eeprom_magic_read == eeprom_magic) {
    T_delta = ReadFloatEEPROM(eeprom_addr_dT);
    if (isnan(T_delta)) {
      T_delta = 0.0;
    }
    eeprom_addr += 1;
    T_setpoint = ReadFloatEEPROM(eeprom_addr);
    eeprom_addr += 4;
    //PrintS_and_D("EEPROM->T " + String(T_setpoint));

    z = EEPROM.read(eeprom_addr);  //high
    eeprom_addr += 1;
    i = EEPROM.read(eeprom_addr);  //lo
    eeprom_addr += 1;
    used_sensors = word(z, i);

    Tae.e = bitRead(used_sensors, BIT_Tae);
    Tbe.e = bitRead(used_sensors, BIT_Tbe);
    Ttarget.e = bitRead(used_sensors, BIT_Ttarget);
    Tsump.e = bitRead(used_sensors, BIT_Tsump);
    Tci.e = bitRead(used_sensors, BIT_Tci);
    Tco.e = bitRead(used_sensors, BIT_Tco);
    Thi.e = bitRead(used_sensors, BIT_Thi);
    Tho.e = bitRead(used_sensors, BIT_Tho);
    Tbc.e = bitRead(used_sensors, BIT_Tbc);
    Tac.e = bitRead(used_sensors, BIT_Tac);
    Touter.e = bitRead(used_sensors, BIT_Touter);
    Ts1.e = bitRead(used_sensors, BIT_Ts1);
    Ts2.e = bitRead(used_sensors, BIT_Ts2);
#ifdef EEV_SUPPORT
    if (Tae.e != 1 || Tbe.e != 1) {
      while (1) {
        PrintS_and_D("ERR: no Tae or Tbe for EEV!");
        delay(1000);
      }
    }
#endif
    ReadEECheckAddr(Tae.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Tae");
    ReadEECheckAddr(Tbe.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Tbe");
    ReadEECheckAddr(Ttarget.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Ttarget");
    ReadEECheckAddr(Tsump.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Tsump");
    ReadEECheckAddr(Tci.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Tci");
    ReadEECheckAddr(Tco.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Tco");
    ReadEECheckAddr(Thi.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Thi");
    ReadEECheckAddr(Tho.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Tho");
    ReadEECheckAddr(Tbc.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Tbc");
    ReadEECheckAddr(Tac.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Tac");
    ReadEECheckAddr(Touter.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Touter");
    ReadEECheckAddr(Ts1.addr);  //eeprom_addr incremeneted here
    //PrintS_and_D("k:Ts1");
    ReadEECheckAddr(Ts2.addr);  //eeprom_addr incremeneted here
                                //PrintS_and_D("k:Ts2");

    /*
		//?code duplicated, see ReadEECheckAddr
		i = 0;
		if (Tae.e == 1) 	CheckIsInvalidCRCAddr(Tae.addr	);
		if (Tbe.e == 1) 	CheckIsInvalidCRCAddr(Tbe.addr	);
		if (Ttarget.e == 1) 	CheckIsInvalidCRCAddr(Ttarget.addr);
		if (Tsump.e == 1) 	CheckIsInvalidCRCAddr(Tsump.addr);
		if (Tci.e == 1) 	CheckIsInvalidCRCAddr(Tci.addr	);
		if (Tco.e == 1) 	CheckIsInvalidCRCAddr(Tco.addr	);
		if (Thi.e == 1) 	CheckIsInvalidCRCAddr(Thi.addr	);
		if (Tho.e == 1) 	CheckIsInvalidCRCAddr(Tho.addr	);
		if (Tbc.e == 1) 	CheckIsInvalidCRCAddr(Tbc.addr	);
		if (Tac.e == 1) 	CheckIsInvalidCRCAddr(Tac.addr	);
		if (Touter.e == 1) 	CheckIsInvalidCRCAddr(Touter.addr);
		if (Ts1.e == 1) 	CheckIsInvalidCRCAddr(Ts1.addr	);
		if (Ts2.e == 1) 	CheckIsInvalidCRCAddr(Ts2.addr	);
		if (i != 0) {
			while ( 1 ) { PrintS_and_D(F("EEPROM err1!")); delay (1000); }
		}
		*/
  } else {
    eeprom_addr += 1;
    ishuman += 1;
    WriteFloatEEPROM(eeprom_addr, T_setpoint);
    //PrintS_and_D(F("init EEPROM"));
    eeprom_addr += 4;
    eeprom_addr += 2;  //used sensors, skip
//Ttarget -needed, other - optional
#ifdef EEV_SUPPORT
    z = FindAddr("Tae", 1);  //holds result in dev_addr, returns "is used"
#else
    z = FindAddr("Tae");  //holds result in dev_addr, returns "is used"
#endif
    Tae.e = z;
    CopyAddrStoreEE(Tae.addr, BIT_Tae);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

#ifdef EEV_SUPPORT
    z = FindAddr("Tbe", 1);
#else
    z = FindAddr("Tbe");
#endif
    Tbe.e = z;
    CopyAddrStoreEE(Tbe.addr, BIT_Tbe);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

#ifdef EEV_ONLY
      //z = FindAddr("Ttarget");
    z = 0;
#else
    z = FindAddr("Ttarget", 1);
#endif
    Ttarget.e = z;
    CopyAddrStoreEE(Ttarget.addr, BIT_Ttarget);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

    z = FindAddr("Tsump");
    Tsump.e = z;
    CopyAddrStoreEE(Tsump.addr, BIT_Tsump);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

    z = FindAddr("Tci");
    Tci.e = z;
    CopyAddrStoreEE(Tci.addr, BIT_Tci);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

    z = FindAddr("Tco");
    Tco.e = z;
    CopyAddrStoreEE(Tco.addr, BIT_Tco);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

    z = FindAddr("Thi");
    Thi.e = z;
    CopyAddrStoreEE(Thi.addr, BIT_Thi);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

    z = FindAddr("Tho");
    Tho.e = z;
    CopyAddrStoreEE(Tho.addr, BIT_Tho);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

    z = FindAddr("Tbc");
    Tbc.e = z;
    CopyAddrStoreEE(Tbc.addr, BIT_Tbc);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

    z = FindAddr("Tac");
    Tac.e = z;
    CopyAddrStoreEE(Tac.addr, BIT_Tac);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

    z = FindAddr("Touter");
    Touter.e = z;
    CopyAddrStoreEE(Touter.addr, BIT_Touter);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

    z = FindAddr("Ts1");
    Ts1.e = z;
    CopyAddrStoreEE(Ts1.addr, BIT_Ts1);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

    z = FindAddr("Ts2");
    Ts2.e = z;
    CopyAddrStoreEE(Ts2.addr, BIT_Ts2);  //dev_addr and z used by proc, autoincrement eeprom_addr, store bit

    //
    //final, off-the-sequence
    EEPROM.write(0 + 1 + 4 + 0, highByte(used_sensors));
    EEPROM.write(0 + 1 + 4 + 1, lowByte(used_sensors));
    EEPROM.write(0x00, eeprom_magic);
    ishuman -= 1;
  }
  T_setpoint_lastsaved = T_setpoint;
  //s_allTsensors.setResolution(ad_Tae, 12);
  /*PrintAddr(Tae.addr);
	PrintAddr(Tbe.addr);
	PrintAddr(Ttarget.addr);
	PrintAddr(Tsump.addr);
	PrintAddr(Tci.addr);
	PrintAddr(Tco.addr);
	PrintAddr(Thi.addr);
	PrintAddr(Tho.addr);
	PrintAddr(Tbc.addr);
	PrintAddr(Tac.addr);
	PrintAddr(Touter.addr);
	PrintAddr(Ts1.addr);
	PrintAddr(Ts2.addr);*/

#ifdef WATCHDOG
  wdt_enable(WDTO_8S);
#endif

  Get_Temperatures();

  outString.reserve(200);
  //PrintS_and_D(String(freeMemory()));  //!!! debug
  //!!!
  //analogWrite(speakerOut, 10);
  //delay (1500);
  //analogWrite(speakerOut, 0);
  //delay (1500);
  //!!!
  tone(speakerOut, 2250);
  delay(1500);  // like ups power on
  noTone(speakerOut);
}


void loop(void) {
  digitalWrite(SerialTxControl, RS485Receive);
  millis_now = millis();

#ifdef RS485_HUMAN
  if (((unsigned long)(millis_now - millis_last_printstats) > HUMAN_AUTOINFO) || (millis_last_printstats == 0)) {
    PrintStats_Serial();
    millis_last_printstats = millis_now;
  }
#endif
  //--------------------async fuctions start
  if (em_i == 0) {
    supply_voltage = ReadVcc();
  }
  if (em_i < em_samplesnum) {
    sampleI_1 = analogRead(em_pin1);
    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset, then subtract this - signal is now centered on 0 counts.
    offsetI_1 = (offsetI_1 + (sampleI_1 - offsetI_1) / 1024);
    filteredI_1 = sampleI_1 - offsetI_1;

    // Root-mean-square method current
    // 1) square current values
    sqI_1 = filteredI_1 * filteredI_1;
    // 2) sum
    sumI_1 += sqI_1;

    em_i += 1;
  } else {
    em_i = 0;
    double I_RATIO = em_calibration * ((supply_voltage / 1000.0) / (ADC_COUNTS));
    async_Irms_1 = I_RATIO * sqrt(sumI_1 / em_samplesnum);
    async_wattage = async_Irms_1 * 220.0;

    //Reset accumulators
    sumI_1 = 0;

    //----------------------------- self-test !!!
    /*
		PrintS_and_D("Async impl. results 1:  ");
		PrintS_and_D(String(async_wattage));	           // Apparent power
		PrintS_and_D(" ");
		PrintS_and_D(String(async_Irms_1));	           // Irms
		PrintS_and_D(" voltage: ");
		PrintS_and_D(String(supply_voltage));
		*/
    //----------------------------- self-test END
  }
#ifdef EEV_SUPPORT
  eevise();
#endif
  //--------------------async fuctions END

  if (heatpump_state == 1 && async_wattage > c_wattage_max) {
    if (((unsigned long)(millis_now - millis_last_heatpump_on) > POWERON_HIGHTIME) || (async_wattage > c_wattage_max * 2.7)) {
#ifdef RS485_HUMAN
      lcd.begin(16,2);
      lcd.clear();
      PrintS_and_D(("Overload." + String(async_wattage)));
#endif
      overload_count += 1;
      millis_last_heatpump_off = millis_now;
      heatpump_state = 0;
      halifise();
    }
  }

  //0 - OK / 1- NotOK
  emergency = analogRead(emergency_pin);
  emergency = (emergency / offsetI_1);
  //emergency = 0;
  if ((heatpump_state == 1) && (emergency == 1) && ((unsigned long)(millis_now - millis_last_heatpump_on) > COLDOFF_HIGHTIME)) {
#ifdef RS485_HUMAN
    lcd.begin(16,2);
    lcd.clear();
    PrintS_and_D("Error: Cold Pomp");
#endif
    overload_count += 1;
    millis_last_heatpump_off = millis_now;
    heatpump_state = 0;
    hotside_circle_state = 0;
    coldside_circle_state = 0;
    sump_heater_state = 0;
    valve4w_state = 0;
    halifise();
  }

  if (overload_count >= 3) {
    heatpump_state = 0;
    hotside_circle_state = 0;
    coldside_circle_state = 0;
    sump_heater_state = 0;
    valve4w_state = 0;
    halifise();  
    return;
  }

//-------------------buttons processing
#ifdef INPUTS_AS_BUTTONS
  z = digitalRead(BUT_LEFT);
  i = digitalRead(BUT_RIGHT);
  d = digitalRead(BUT_3);

  if ((z == 1) && (i == 1)) {
    //
  } else if ((z == 1) || (i == 1) || (d == 1)) {
#ifndef EEV_ONLY
    if (z == 1) {
      x = Dec_T();
    }
    if (i == 1) {
      x = Inc_T();
    }
    if (x == 1) {
      PrintS_and_D("T(z): " + String(T_setpoint));
      delay(300);
      SaveSetpointEE(1);
    }

    if (d == 1) {
      Inc_Tdelta();
      PrintS_and_D("dT: " + String(T_delta));
      delay(300);
      WriteFloatEEPROM(eeprom_addr_dT, T_delta);
    }


#else
    if (z == 1) {
      T_EEV_setpoint -= 0.25;
    }
    if (i == 1) {
      T_EEV_setpoint += 0.25;
    }
    PrintS_and_D("New EEV Td: " + String(T_EEV_setpoint));
    delay(300);
#endif
  }
#endif
//-------------------buttons processing END

//-------------------display
#if (DISPLAY == 2) || (DISPLAY == 1)
  if ((_1st_start_sleeped == 1) && (((unsigned long)(millis_now - millis_displ_update) > millis_displ_update_interval) || (millis_displ_update == 0))) {
//!!!EEV_ONLY SUPPORT???
#ifndef EEV_ONLY
    lcd.begin(16,2);
    lcd.clear();
    delay(10);
    if (  (displ_inc & 0x01) == 0 ) {
      outString = "Z:" + String(T_setpoint - T_delta, 1) + " / " + String(T_setpoint, 1);
      Print_D2(outString, 0);
      if (Ttarget.e == 1) {
        outString = "A:" + String(Ttarget.T, 1) + " E:" + String(EEV_cur_pos);
        Print_D2(outString, 1);
      } else {
        outString += "ERR";
        Print_D2(outString, 0);  //do not print serial
      }
    } else {
      outString = "Be:" + String(Tbe.T, 1) + " Ae:" + String(Tae.T, 1);
      Print_D2(outString, 0);

      outString = "Hp:" + String(Tsump.T, 1) + " dT:" + String(T_EEV_dt, 1);
      Print_D2(outString, 1);
    }
    displ_inc++;
#else
    outString = "be:";
    if (Tbe.e == 1) {
      outString += String(Tbe.T, 1);
    }
    outString += " ae:";
    if (Tae.e == 1) {
      outString += String(Tae.T, 1);
    }
    Print_D2(outString, 1);  //do not print serial

#endif
    millis_displ_update = millis_now;
  }
#endif
  //-------------------display END

  //-------------------check cycle
  if (((unsigned long)(millis_now - millis_prev) > millis_cycle) || (millis_prev == 0)) {
    millis_prev = millis_now;
    Get_Temperatures();  //      wdt_reset here due to 85.0'C filtration
    SaveSetpointEE();

    //--------------------important logic
    //check T sensors
    if ((errorcode == ERR_OK) && ((Tae.e == 1 && Tae.T == -127) || (Tbe.e == 1 && Tbe.T == -127) || (Ttarget.e == 1 && Ttarget.T == -127) || (Tsump.e == 1 && Tsump.T == -127) || (Tci.e == 1 && Tci.T == -127) || (Tco.e == 1 && Tco.T == -127) || (Thi.e == 1 && Thi.T == -127) || (Tho.e == 1 && Tho.T == -127) || (Tbc.e == 1 && Tbc.T == -127) || (Tac.e == 1 && Tac.T == -127) || (Touter.e == 1 && Touter.T == -127) || (Ts1.e == 1 && Ts1.T == -127) || (Ts2.e == 1 && Ts2.T == -127))) {
      errorcode = ERR_T_SENSOR;
#ifdef RS485_HUMAN
      PrintS_and_D("ERR:T.sens." + String(errorcode));
#endif
    }
    //auto-clean sensor error on sensor appear
    // add 1xor enable here!
    if ((errorcode == ERR_T_SENSOR) && (((Tae.e == 1 && Tae.T != -127) || (Tae.e ^ 1)) && ((Tbe.e == 1 && Tbe.T != -127) || (Tbe.e ^ 1)) && ((Ttarget.e == 1 && Ttarget.T != -127) || (Ttarget.e ^ 1)) && ((Tsump.e == 1 && Tsump.T != -127) || (Tsump.e ^ 1)) && ((Tci.e == 1 && Tci.T != -127) || (Tci.e ^ 1)) && ((Tco.e == 1 && Tco.T != -127) || (Tco.e ^ 1)) && ((Thi.e == 1 && Thi.T != -127) || (Thi.e ^ 1)) && ((Tho.e == 1 && Tho.T != -127) || (Tho.e ^ 1)) && ((Tbc.e == 1 && Tbc.T != -127) || (Tbc.e ^ 1)) && ((Tac.e == 1 && Tac.T != -127) || (Tac.e ^ 1)) && ((Touter.e == 1 && Touter.T != -127) || (Touter.e ^ 1)) && ((Ts1.e == 1 && Ts1.T != -127) || (Ts1.e ^ 1)) && ((Ts2.e == 1 && Ts2.T != -127) || (Ts2.e ^ 1)))) {
      errorcode = ERR_OK;
    }

    //process errors
    //beep N times error
    if (errorcode != ERR_OK) {
      if (((unsigned long)(millis_now - millis_notification) > millis_notification_interval) || millis_notification == 0) {
        millis_notification = millis_now;
#ifdef RS485_HUMAN
        PrintS_and_D("Error:" + String(errorcode));
#endif
        for (i = 0; i < errorcode; i++) {
          tone(speakerOut, ERR_HZ);
          delay(500);
          noTone(speakerOut);
          delay(500);
        }
      }
    }

//-------------- EEV cycle
#ifdef EEV_SUPPORT
    /*
			//v1 algo
			if ( EEV_apulses == 0 )	{
				if ( 	((async_wattage < c_workingOK_wattage_min) && ((unsigned long)(millis_now - millis_eev_last_close) > EEV_CLOSEEVERY))		||  millis_eev_last_close == 0  ){
					PrintS_and_D("EEV: FULL closing");//!!!
					if ( millis_eev_last_close != 0 ) {
						EEV_apulses =  -(EEV_cur_pos + EEV_CLOSE_ADD_PULSES);
					} else {
						EEV_apulses =  -(EEV_MAXPULSES + EEV_CLOSE_ADD_PULSES);
					}
					EEV_adonotcare = 1;
					EEV_fast = 1;
					//delay(EEV_STOP_HOLD);
					millis_eev_last_close = millis_now;
				} else if (errorcode != 0 || async_wattage <= c_workingOK_wattage_min) {		//err or sleep
					PrintS_and_D("EEV: err or sleep");//!!!
					if (EEV_cur_pos <= 0 && EEV_OPEN_AFTER_CLOSE != 0) {				//set waiting pos
						EEV_apulses = +EEV_OPEN_AFTER_CLOSE;
						EEV_adonotcare = 0;
						EEV_fast = 1;
					}
					if (EEV_cur_pos > 0  && EEV_cur_pos != EEV_OPEN_AFTER_CLOSE && EEV_cur_pos <= EEV_MAXPULSES) {	//waiting pos. set
						PrintS_and_D("EEV: close");//!!!
						EEV_apulses =  -(EEV_cur_pos + EEV_CLOSE_ADD_PULSES);
						EEV_adonotcare = 1;
						EEV_fast = 1;
					}
				} else if (errorcode == 0 && async_wattage > c_workingOK_wattage_min) {
					T_EEV_dt = Tae.T - Tbe.T;
					PrintS_and_D("EEV: driving " + String(T_EEV_dt));//!!!
					if (EEV_cur_pos <= 0){
						PrintS_and_D("EEV: full close protection");	
						if (EEV_OPEN_AFTER_CLOSE != 0) {				//full close protection
							EEV_apulses = +EEV_OPEN_AFTER_CLOSE;
							EEV_adonotcare = 0;
							EEV_fast = 1;
						}
					} else if (EEV_cur_pos > 0) {
						if (T_EEV_dt  < (T_EEV_setpoint - EEV_EMERG_DIFF) ) {				//emerg!
							PrintS_and_D("EEV: emergency closing!");//!!!
							EEV_apulses = -EEV_EMERG_STEPS;
							EEV_adonotcare = 0;
							EEV_fast = 1;
						} else if (T_EEV_dt  < T_EEV_setpoint) {					//too
							PrintS_and_D("EEV: closing");//!!!
							//EEV_apulses = -EEV_NONPRECISE_STEPS;
							EEV_apulses = -1;
							EEV_adonotcare = 0;
							EEV_fast = 0;
						} else if (T_EEV_dt  > T_EEV_setpoint + EEV_HYSTERESIS + EEV_PRECISE_START) {	//very
							PrintS_and_D("EEV: fast opening");//!!!
							//EEV_apulses =  +EEV_NONPRECISE_STEPS;
							EEV_apulses =  +1;
							EEV_adonotcare = 0;
							EEV_fast = 1;
						} else if (T_EEV_dt > T_EEV_setpoint + EEV_HYSTERESIS) {			//too
							PrintS_and_D("EEV: opening");//!!!
							EEV_apulses =  +1;
							EEV_adonotcare = 0;
							EEV_fast = 0;
						} else if (T_EEV_dt  > T_EEV_setpoint) {					//ok
							PrintS_and_D("EEV: OK");//!!!
							//
						}
					}
					off_EEV();
				} 
				
			}
			*/
    //v1.1 algo
    if (errorcode == 0 && async_wattage > c_workingOK_wattage_min && EEV_cur_pos > 0) {   

      T_EEV_dt = Tae.T - Tbe.T;      
      if (Tbe.T > 3 && T_EEV_dt > EEV_HYSTERESIS ) {
        T_EEV_dt = T_EEV_setpoint + EEV_HYSTERESIS;
      } else  if (Tbe.T > 3 && T_EEV_dt > -1 ) {
        T_EEV_dt = T_EEV_setpoint;
      } 

      //zamykamy zawór jak tempataruea Tbe <= 0 lub T sprężarki bęedzie >= 85 
      if (Tbe.T <= 0 || Tae.T <= 0 || Tsump.T > (cT_sump_max - 10.0) || T_EEV_dt < 0) {
        T_EEV_dt = T_EEV_setpoint - EEV_EMERG_DIFF;
      }

      if (Tbe.T < cT_evaporator_war || Tae.T < cT_evaporator_war || Tsump.T > (cT_sump_max - 5.0) ) {
        T_EEV_dt = EEV_HYSTERESIS;
      }

      //zawor otwarty
      if (EEV_apulses >= 0 && EEV_cur_pos >= EEV_MINWORKPOS) {
        if (Tbe.T < cT_evaporator_war) {  //emerg!
#ifdef EEV_DEBUG
          PrintS(F("EEV: 1 emergency closing!"));
#endif
          EEV_apulses = -1;
          EEV_adonotcare = 0;
          EEV_fast = 1;
        }
        //jełsi temperatura przegrzania < 1.5 to zamykaj zawór NATYCHMIAST
        //else if (T_EEV_dt < (T_EEV_setpoint - EEV_EMERG_DIFF)) {  //emerg!
        else if (T_EEV_dt <= EEV_HYSTERESIS) {  //emerg!
#ifdef EEV_DEBUG
          PrintS(F("EEV: 1 emergency closing!"));
#endif
          EEV_apulses = -1;
          EEV_adonotcare = 0;
          EEV_fast = 1;
        }
        //jełsi temperatura przegrzania < 4.0 to zamykaj zawór, NORMALNIE
        else if (T_EEV_dt < T_EEV_setpoint) {  //too
#ifdef EEV_DEBUG
          PrintS(F("EEV: 2 closing"));
#endif
          //EEV_apulses = -EEV_NONPRECISE_STEPS;
          EEV_apulses = -1;
          EEV_adonotcare = 0;
          EEV_fast = 0;
        }
        // wysoka temperatura przegrzania, otwórz zwór NATYCHMIAST (dt>10)
        //faster open when needed, condition copypasted (see EEV_apulses <= 0)
        if (T_EEV_dt > T_EEV_setpoint + EEV_HYSTERESIS + EEV_PRECISE_START) {  //very
#ifdef EEV_DEBUG
          PrintS(F("EEV: 3 enforce faster opening"));
#endif
          //EEV_apulses =  +EEV_NONPRECISE_STEPS;
          //EEV_apulses =  +1;
          EEV_adonotcare = 0;
          EEV_fast = 1;
        }
      }

      if (EEV_apulses <= 0) {
        // wysoka temperatura przegrzania, otwórz zwór NATYCHMIAST (dt>10)
        if (T_EEV_dt > T_EEV_setpoint + EEV_HYSTERESIS + EEV_PRECISE_START) {  //very
#ifdef EEV_DEBUG
          PrintS(F("EEV: 4 fast opening"));
#endif
          //EEV_apulses =  +EEV_NONPRECISE_STEPS;
          EEV_apulses = +1;
          EEV_adonotcare = 0;
          EEV_fast = 1;

          // wysoka temperatura przegrzania, otwórz zwór NORMALNIE (dt>4.6)
        } else if (T_EEV_dt >= T_EEV_setpoint + EEV_HYSTERESIS) {			//too
        //} else if ((T_EEV_dt >= T_EEV_setpoint + EEV_HYSTERESIS) && (T_EEV_dt_change == 0 || EEV_cur_pos == EEV_MINWORKPOS )) {  // jak temperatura spada to już nie otwieramy
#ifdef EEV_DEBUG
          PrintS(F("EEV: 5 opening"));
#endif
          EEV_apulses = +1;
          EEV_adonotcare = 0;
          EEV_fast = (EEV_cur_pos == EEV_MINWORKPOS) ? 1: 0;
          //pozostaw w aktualnej pozycji
        } else if (T_EEV_dt > T_EEV_setpoint) {  //ok
#ifdef EEV_DEBUG
          PrintS(F("EEV: 6 OK"));
#endif
          //
        }

        if (Tbe.T <= cT_evaporator_war) {  //emerg!
#ifdef EEV_DEBUG
          PrintS(F("EEV: 1 emergency closing!"));
#endif
          EEV_adonotcare = 0;
          EEV_fast = 1;
        }
        //faster closing when needed, condition copypasted (see EEV_apulses >= 0)
        //jełsi temperatura przegezania (dt<2.5) to go zamknij NATYCHMIAST
        //else if (T_EEV_dt < (T_EEV_setpoint - EEV_EMERG_DIFF)) {  //emerg!
        else if (T_EEV_dt < EEV_HYSTERESIS) {  //emerg!
#ifdef EEV_DEBUG
          PrintS(F("EEV: 7 enforce faster closing!"));
#endif
          //EEV_apulses = -EEV_EMERG_STEPS;
          EEV_adonotcare = 0;
          EEV_fast = 1;
        }
      }
      off_EEV();
    }

    if (EEV_apulses == 0) {
      if (((async_wattage < c_workingOK_wattage_min) && ((unsigned long)(millis_now - millis_eev_last_close) > EEV_CLOSEEVERY)) || millis_eev_last_close == 0) {  //close every 24h by default
#ifdef EEV_DEBUG
        PrintS(F("EEV: 10 FULL closing"));
#endif
        if (millis_eev_last_close != 0) {
          EEV_apulses = -(EEV_cur_pos + EEV_CLOSE_ADD_PULSES);
        } else {
          EEV_apulses = -(EEV_MAXPULSES + EEV_CLOSE_ADD_PULSES);
        }
        EEV_adonotcare = 1;
        EEV_fast = 1;
        //delay(EEV_STOP_HOLD);
        millis_eev_last_close = millis_now;
      } else if (errorcode != 0 || async_wattage < c_workingOK_wattage_min) {  //err or sleep
        if (EEV_cur_pos > 0 && EEV_cur_pos > EEV_OPEN_AFTER_CLOSE) {           //waiting pos. set
#ifdef EEV_DEBUG
          PrintS(F("EEV: 11 close before open"));
#endif
          EEV_apulses = -(EEV_cur_pos + EEV_CLOSE_ADD_PULSES);
          EEV_adonotcare = 1;
          EEV_fast = 1;
        }
      }
      off_EEV();
    }

    if (EEV_apulses == 0 && async_wattage < c_workingOK_wattage_min && EEV_cur_pos < EEV_OPEN_AFTER_CLOSE) {
#ifdef EEV_DEBUG
      PrintS(F("EEV: 12 full close protection"));
#endif
      if (EEV_OPEN_AFTER_CLOSE != 0) {  //full close protection
        EEV_apulses = EEV_OPEN_AFTER_CLOSE - EEV_cur_pos;
        EEV_adonotcare = 0;
        EEV_fast = 1;
      }
      off_EEV();
    }
    if (EEV_apulses == 0 && async_wattage >= c_workingOK_wattage_min && EEV_cur_pos < EEV_MINWORKPOS) {
#ifdef EEV_DEBUG
      PrintS(F("EEV: 13 open to work"));
#endif
      if (EEV_MINWORKPOS != 0 && EEV_MINWORKPOS > EEV_cur_pos) {  //full close protection
        EEV_apulses = EEV_MINWORKPOS - EEV_cur_pos;
        EEV_adonotcare = 0;
        EEV_fast = 1;
      }
      off_EEV();
    }
    if (((unsigned long)(millis_now - millis_eev_last_on) > 10000) || millis_eev_last_on == 0) {
      //PrintS_and_D("EEV: ON/OFF");
      on_EEV();
      //delay(30);
      //off_EEV();	//off_EEV called everywhere takes care of it
      millis_eev_last_on = millis_now;

      // T_EEV_dt_falls = (T_EEV_dt < T_EEV_dt_last);
      // T_EEV_dt_last = T_EEV_dt;
      //T_EEV_dt_change = ((unsigned long)(Tbe.T *10)) != ((unsigned long)(T_EEV_Be*10));
      //T_EEV_dt_change = T_EEV_dt_change || ((unsigned long)(Tae.T)) != ((unsigned long)(T_EEV_Ae)) ;
      //T_EEV_Be = Tbe.T;
      //T_EEV_Ae = Tae.T;

    }
#endif
    //-------------- EEV cycle END

#ifndef EEV_ONLY
    //process heatpump sump heater
    if (Tsump.e == 1) {
      if (Tsump.T < cT_sump_heat_threshold && sump_heater_state == 0 && Tsump.T != -127) {
        sump_heater_state = 1;
      } else if (Tsump.T >= cT_sump_heat_threshold && sump_heater_state == 1) {
        sump_heater_state = 0;
      } else if (Tsump.T == -127) {
        sump_heater_state = 0;
      }
      halifise();
    }

    //main logic
    if (_1st_start_sleeped == 0) {
      //PrintS_and_D("!!!!sleep disabled!!!!");
      //_1st_start_sleeped = 1;
      if ((millis_now < poweron_pause) && (_1st_start_sleeped == 0)) {
        PrintS_and_D("Wait: " + String(((poweron_pause - millis_now)) / 1000) + " s.");
        return;
      } else {
        _1st_start_sleeped = 1;
      }
    }

    //process_heatpump:
    //start if
    //    (last_on > N or not_started_yet)
    //    and (no errors)
    //    and (t hot out < t target + heat_delta_min)
    //    and (sump t > min'C)
    //    and (sump t < max'C)
    //    and (t watertank < target)
    //    and (t after evaporator > after evaporator min)
    //    and (t cold in > cold min)
    //    and (t cold out > cold min)
    if (heatpump_state == 0 && (((unsigned long)(millis_now - millis_last_heatpump_off) > mincycle_poweroff) || (millis_last_heatpump_off == 0)) &&
        //( tr_hot_out < (tr_sens_1 + cT_hotcircle_delta_min)  )    &&
        errorcode == 0 && ((Tsump.e == 1 && Tsump.T > cT_sump_min) || (Tsump.e ^ 1)) && ((Tsump.e == 1 && Tsump.T < cT_sump_max) || (Tsump.e ^ 1)) &&
        //t1_sump > t2_cold_in   && ???
        //RO+
        //Ttarget.T < T_setpoint  						&&    //was room here, change to advanced algo with room temperature
        Ttarget.T + T_delta < T_setpoint &&
        //
        ((Tae.e == 1 && Tae.T > cT_after_evaporator_min) || (Tae.e ^ 1)) && ((Tbc.e == 1 && Tbc.T < cT_before_condenser_max) || (Tbc.e ^ 1)) && ((Tci.e == 1 && Tci.T > cT_cold_min) || (Tci.e ^ 1)) && ((Tco.e == 1 && Tco.T > cT_cold_min) || (Tco.e ^ 1))) {
        millis_last_heatpump_off = millis_now;
        millis_last_heatpump_on = millis_now;
        heatpump_state = 1;
    }
    //stop if
    //    ( (last_off > N) and (t watertank > target) )
    if (heatpump_state == 1 && (Ttarget.T > T_setpoint)) {
      if ((unsigned long)(millis_now - millis_last_heatpump_on) > mincycle_poweron) {
        millis_last_heatpump_off = millis_now;
        heatpump_state = 0;

        //reset overload
        overload_count = 0;
      }
    }

    if ((heatpump_state == 0) && (hotside_circle_state == 1)) {
      if ((deffered_stop_hotcircle != 0 && ((unsigned long)(millis_now - millis_last_heatpump_off) > deffered_stop_hotcircle))) {
        //if ((Tho.e == 1 && Tho.T > (Ttarget.T + cT_hotcircle_delta_min)) || (Thi.e == 1 && Thi.T > (Ttarget.T + cT_hotcircle_delta_min))) {
        if ((Tho.e == 1 && Tho.T > (Ttarget.T + cT_hotcircle_delta_min))) {
          // #ifdef RS485_HUMAN
          //   PrintS(String("Tho or Thi > ") + String((unsigned long)(Ttarget.T) + cT_hotcircle_delta_min) + String(" -> Hot WP ON"));
          // #endif
        } else {
          // #ifdef RS485_HUMAN
          //   PrintS(F("Hot WP OFF"));
          // #endif
          hotside_circle_state = 0;
        }
      }
    }
    //process_hot_side_pump:
    //start if (heatpump_enabled)
    //stop if (heatpump_disabled and (t hot out or in < t target + heat delta min) )
    //delayed start hot side
    else if ((heatpump_state == 1) && (hotside_circle_state == 0) && ((unsigned long)(millis_now - millis_last_heatpump_on) > POWERON_HIGHTIME)) {
      // #ifdef RS485_HUMAN
      // 	PrintS(F("Hot WP ON"));
      // #endif
      hotside_circle_state = 1;
    }

    //process_cold_side_pump:
    //start if (heatpump_enabled)
    //stop if (heatpump_disbled)
    //delayed start cold side
    if ((heatpump_state == 1) && (coldside_circle_state == 0) && ((unsigned long)(millis_now - millis_last_heatpump_on) > POWERON_HIGHTIME)) {
      // #ifdef RS485_HUMAN
      // 	PrintS(F("Cold WP ON"));
      // #endif
      coldside_circle_state = 1;
    }

    if ((heatpump_state == 0) && (coldside_circle_state == 1)) {
      if ((deffered_stop_coldcircle != 0 && ((unsigned long)(millis_now - millis_last_heatpump_off) > deffered_stop_coldcircle))) {
        // #ifdef RS485_HUMAN
        //   PrintS(F("Cold WP OFF"));
        // #endif
        coldside_circle_state = 0;
      }
    }


    //protective_cycle:
    //stop if
    //      (error)
    //      (t hot out > hot out max)
    //      (sump t > max'C)
    //      or (t after evaporator < after evaporator min)
    //      or (t cold in < cold min)
    //      or (t cold out < cold min)
    //
    if (heatpump_state == 1 && (errorcode != 0 || (Tho.e == 1 && Tho.T > cT_hotout_max) || (Tsump.e == 1 && Tsump.T > cT_sump_max) || (Tae.e == 1 && Tae.T < cT_after_evaporator_min) || (Tbc.e == 1 && Tbc.T > cT_before_condenser_max) || (Tci.e == 1 && Tci.T < cT_cold_min) || (Tco.e == 1 && Tco.T < cT_cold_min))) {
#ifdef RS485_HUMAN
      PrintS_and_D(String("Protective STOP") + " Err.:" + String(errorcode, HEX));
#endif
      millis_last_heatpump_off = millis_now;
      heatpump_state = 0;
      //digitalWrite(RELAY_HEATPUMP, heatpump_state);  // old, now halifised
    }

    //alive_check_cycle_after_5_mins:
    //error if
    //v1.3: not error, just poweroff all
    //      or (t cold in - t cold out < t workingok min)
    //      or (t hot out - t hot in < t workingok min)
    //      or (sump t < 25'C)
    //      or wattage too low

    if (heatpump_state == 1 && ((unsigned long)(millis_now - millis_last_heatpump_on) > MINCYKLE_CHECK)) {
      //cold side processing simetimes works incorrectly, after long period of inactivity, due to T inertia on cold tube sensor, commented out
      //if ( ( errorcode == ERR_OK )     &&   (  tr_cold_in - tr_cold_out < cT_workingOK_cold_delta_min ) ) {
      //    errorcode = ERR_COLD_PUMP;
      //}
      //if ( ( errorcode == ERR_OK )     &&   (  Tho.e == 1 && Thi.e == 1 && (Tho.T - Thi.T < cT_workingOK_hot_delta_min )) ) {
      //	errorcode = ERR_HOT_PUMP;
      //}
      if ((errorcode == ERR_OK) && (Tsump.e == 1 && Tsump.T < cT_workingOK_sump_min)) {
        //errorcode = ERR_HEATPUMP;
        millis_last_heatpump_off = millis_now;
        heatpump_state = 0;
#ifdef RS485_HUMAN
        PrintS_and_D("Error HEATPUMP: " + String(Tsump.T));
#endif
      }
      if ((errorcode == ERR_OK) && (async_wattage < c_workingOK_wattage_min)) {
        //errorcode = ERR_WATTAGE;
        millis_last_heatpump_off = millis_now;
        heatpump_state = 0;
#ifdef RS485_HUMAN
        PrintS_and_D("Error WATTAGE: " + String(async_wattage));
#endif
      }
      //digitalWrite(RELAY_HEATPUMP, heatpump_state);	// old, now halifised
    }

    //disable pump by error
    if (errorcode != ERR_OK) {
      millis_last_heatpump_off = millis_now;
      heatpump_state = 0;
//digitalWrite(RELAY_HEATPUMP, heatpump_state);	// old, now halifised
#ifdef RS485_HUMAN
      PrintS_and_D("Error stop: " + String(errorcode, HEX));
#endif
    }

//prevent error - zepsuty przekaźnik
if ( async_wattage > c_workingOK_wattage_min && heatpump_state == 0 && (millis_now - millis_last_heatpump_off)> 10000 
      && ( coldside_circle_state == 0 || hotside_circle_state == 0 )) {
  coldside_circle_state = 1;
  hotside_circle_state = 1;
  PrintS_and_D("Error RY");
}
    //!!! self-test
    ///heatpump_state = 1;

    //!!! write states to relays, old, now halifised
    //digitalWrite	(RELAY_HEATPUMP, 	heatpump_state);
    //digitalWrite	(RELAY_COLDSIDE_CIRCLE,	coldside_circle_state);
    halifise();
#endif
  }

  // if (RS485Serial.available() > 0) {
  // 	//RS485Serial.println("some on serial..");	//!!!debug
  // 	#ifdef RS485_HUMAN
  // 		if (RS485Serial.available()) {
  // 			inChar = RS485Serial.read();
  // 			//RS485Serial.print(inChar);	//!!!debug
  // 			if ( inChar == 0x1B ) {
  // 				skipchars += 3;
  // 				inChar = 0x00;
  // 				millis_escinput = millis();
  // 			}
  // 			if ( skipchars != 0 ) {
  // 				millis_charinput = millis();
  // 				//if (millis_escinput + 2 > millis_charinput)
  // 				if ((unsigned long)(millis_charinput - millis_escinput) < 16*2 ) {	//2 chars for 2400
  // 					if (inChar != 0x7e) {
  // 						skipchars -= 1;
  // 					}
  // 					if (inChar == 0x7e) {
  // 						skipchars = 0;
  // 					}
  // 					if (inChar >= 0x30 && inChar <= 0x35) {
  // 						skipchars += 1;
  // 					}
  // 					inChar = 0x00;
  // 				} else {
  // 					skipchars = 0;
  // 				}
  // 			}

  // 			//- RS485_HUMAN: remote commands +,-,G,0x20/?/Enter
  // 			switch (inChar) {
  // 				case 0x00:
  // 					break;
  // 				case 0x20:
  // 				case 0x3F:
  // 				case 0x0D:
  // 					_PrintHelp();
  // 					break;
  // 				case 0x2B:
  // 					Inc_T();
  // 					break;
  // 				case 0x2D:
  // 					Dec_T();
  // 					break;
  // 				case 0x3C:
  // 					Dec_E();
  // 					break;
  // 				case 0x3E:
  // 					Inc_E();
  // 					break;
  // 				case 0x47:
  // 				case 0x67:
  // 					PrintStats_Serial();
  // 					break;
  // 				}
  // 		}
  // 	#endif

  // 	#ifdef RS485_PYTHON
  // 		index = 0;
  // 		while (RS485Serial.available() > 0) { // Don't read unless you know there is data
  // 			if(index < 49) {   //  size of the array minus 1
  // 				inChar = RS485Serial.read(); 	// Read a character
  // 				inData[index] = inChar;      	// Store it
  // 				index++;                     	// Increment where to write next
  // 				inData[index] = '\0';        	// clear next symbol, null terminate the string
  // 				delayMicroseconds(80);       	//80 microseconds - the best choice at 9600, "no answer"disappeared
  // 								//40(20??) microseconds seems to be good, 9600, 49 symbols
  // 								//
  // 			} else {            //too long message! read it to nowhere
  // 				inChar = RS485Serial.read();
  // 				delayMicroseconds(80);
  // 				//break;    //do not break if symbols!!
  // 			}
  // 		}

  // 		//!!!debug, be carefull, can cause strange results
  // 		/*
  // 		if (inData[0] != 0x00) {
  // 		RS485Serial.println("-");
  // 		RS485Serial.println(inData);
  // 		RS485Serial.println("-");
  // 		}
  // 		*/
  // 		//or this debug
  // 		/*
  // 		digitalWrite(SerialTxControl, RS485Transmit);
  // 		delay(10);
  // 		RS485Serial.println(inData);
  // 		RS485Serial.flush();
  // 		RS485Serial.println(index);
  // 		*/

  // 		//ALL lines must be terminated with \n!
  // 		if ( (inData[0] == hostID) && (inData[1] == devID) ) {
  // 			//  COMMANDS:
  // 			// G (0x47): (G)et main data
  // 			// TNN.NN (0x54): set aim (T)emperature
  // 			digitalWrite(SerialTxControl, RS485Transmit);
  // 			delay(1);
  // 			//PrintS_and_D(freeMemory());
  // 			outString = "";
  // 			outString += devID;
  // 			outString += hostID;
  // 			outString +=  "A ";  //where A is Answer, space after header

  // 			if ( (inData[2] == 0x47 ) ) {
  // 				//PrintS_and_D("G");
  // 				//WARNING: this procedure can cause "NO answer" effect if no or few T sensors connected
  // 				outString += "{";
  // 				outString += "\"E1\":" + String(errorcode);
  // 				if (Ts1.e == 1) {
  // 					outString += ",\"TS1\":" + String(Ts1.T);
  // 				}
  // 				if (Tsump.e == 1) {
  // 					outString += ",\"TS\":" + String(Tsump.T);
  // 				}
  // 				if (Tho.e == 1) {
  // 					outString += ",\"THO\":" + String(Tho.T);
  // 				}
  // 				if (Tae.e == 1) {
  // 					outString += ",\"TAE\":" + String(Tae.T);
  // 				}
  // 				char *outChar=&outString[0];
  // 				RS485Serial.write(outChar);                    	//dirty hack to transfer long string
  // 				RS485Serial.flush();
  // 				delay (1);                                      //lot of errors without delay
  // 				outString = "";
  // 				if (Tbe.e == 1) {
  // 					outString += ",\"TBE\":" + String(Tbe.T);
  // 				}
  // 				if (Touter.e == 1) {
  // 					outString += ",\"TO\":" + String(Touter.T);
  // 				}
  // 				if (Tco.e == 1) {
  // 					outString += ",\"TCO\":" + String(Tco.T);
  // 				}
  // 				outString += ",\"W1\":" + String(async_wattage);
  // 				#ifndef EEV_ONLY
  // 					outString += ",\"A1\":" + String(T_setpoint);  //(A)im (target)
  // 					//!!!!! must be changed for G9 v1.4 - personal pin !!!!!!!
  // 					#ifndef BOARD_TYPE_G9
  // 						outString += ",\"RP\":" + String(heatpump_state*RELAY_HEATPUMP);
  // 					#endif
  // 					#ifdef BOARD_TYPE_G9
  // 						outString += ",\"RP\":" + String(heatpump_state*20);
  // 					#endif
  // 					//!!!!!
  // 				#endif
  // 				if (Tci.e == 1) {
  // 					outString += ",\"TCI\":" + String(Tci.T);
  // 				}
  // 				RS485Serial.write(outChar);                       //dirty hack to transfer long string
  // 				RS485Serial.flush();
  // 				delay (1);                                        //lot of errors without delay
  // 				outString = "";
  // 				if (Thi.e == 1) {
  // 					outString += ",\"THI\":" + String(Thi.T);
  // 				}
  // 				#ifndef EEV_ONLY
  // 					outString += ",\"RSH\":" + String(sump_heater_state*3);
  // 					outString += ",\"RH\":" + String(hotside_circle_state*2);
  // 					outString += ",\"RC\":" + String(coldside_circle_state*1);
  // 				#endif
  // 				if (Tbc.e == 1) {
  // 					outString += ",\"TBC\":" + String(Tbc.T);
  // 				}
  // 				RS485Serial.write(outChar);                        //dirty hack to transfer long string
  // 				RS485Serial.flush();
  // 				delay (1);                                        //lot of errors without delay
  // 				outString = "";
  // 				if (Ts2.e == 1) {
  // 					outString += ",\"TS2\":" + String(Ts2.T);
  // 				}
  // 				if (Tac.e == 1) {
  // 					outString += ",\"TAC\":" + String(Tac.T);
  // 				}
  // 				if (Ttarget.e == 1) {
  // 					outString += ",\"TT\":" + String(Ttarget.T);
  // 				}
  // 				#ifdef EEV_SUPPORT
  // 					outString += ",\"EEVP\":" + String(EEV_cur_pos);
  // 					outString += ",\"EEVA\":" + String(T_EEV_setpoint);
  // 				#endif
  // 				outString += "}";
  // 			} else if ( (inData[2] == 0x54 ) || (inData[2] == 0x45 )) {  //(T)arget or (E)EV target format NN.NN, text
  // 				if ( isDigit(inData[ 3 ]) && isDigit(inData[ 4 ]) && (inData[ 5 ] == 0x2e)  && isDigit(inData[ 6 ]) && isDigit(inData[ 7 ]) && ( ! isDigit(inData[ 8 ])) ) {

  // 					tone(speakerOut, 2250);
  // 					delay (100); // like ups power on
  // 					noTone(speakerOut);

  // 					char * carray = &inData[ 3 ];
  // 					tempdouble = atof(carray);
  // 					if (inData[2] == 0x54 ){
  // 						if (tempdouble > cT_setpoint_max) {
  // 							outString += "{\"err\":\"too hot!\"}";
  // 						} else if (tempdouble < 1.0) {
  // 							outString += "{\"err\":\"too cold!\"}";
  // 						} else {
  // 							T_setpoint = tempdouble;
  // 							outString += "{\"result\":\"ok, new value is: ";
  // 							outString += String(T_setpoint);
  // 							outString += "\"}";
  // 						}
  // 					}
  // 					if (inData[2] == 0x45 ) {
  // 						if (tempdouble > 10.0) {		//!!!!!!! hardcode !!!
  // 							outString += "{\"err\":\"too hot!\"}";
  // 						} else if (tempdouble < 0.1) {		//!!!!!!! hardcode !!!
  // 							outString += "{\"err\":\"too cold!\"}";
  // 						} else {
  // 							T_EEV_setpoint = tempdouble;
  // 							outString += "{\"result\":\"ok, new EEV value is: ";
  // 							outString += String(T_EEV_setpoint);
  // 							outString += "\"}";
  // 						}
  // 					}
  // 				} else {
  // 					outString += "{\"err\":\"NaN, format: NN.NN\"}";
  // 				}
  // 			} else {
  // 				//default, just for example
  // 				outString += "{\"err\":\"no_command\"}";
  // 			}
  // 			//crc.integer = CRC16.xmodem((uint8_t& *) outString, outString.length());
  // 			//outString += (crc, HEX);
  // 			outString += "\n";
  // 			char *outChar=&outString[0];
  // 			RS485Serial.write(outChar);
  // 		}

  // 		index = 0;
  // 		for (i=0;i<49;i++) {  //clear buffer
  // 			inData[i]=0;
  // 		}
  // 		RS485Serial.flush();
  // 		digitalWrite(SerialTxControl, RS485Receive);
  // 		delay(1);
  // 	#endif
  // }
}
