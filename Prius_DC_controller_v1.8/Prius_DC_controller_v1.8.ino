
// VERSION 1080 23/07/2021 ( implimented ouput voltage limiting)

//Teensy 3.2 based control for Prius GEN2 inverter for use as DC motor control,  
//for use with and appropriately modified GEN2 power stage used as high side switch with all 6 MG1&2 phases parralled 
//Experimental code. Only tested by me! Use at your own risk! has be known to kill an inverter, you have been warned.
// By using this code and any accosiated hardware designs you agree that any damage to equipement or person is your responsibilty.
// S.Pilcher EV8
// (tie inverter pins GSDN(14) & MSDN(25) to +12v to enable powerstages)

//INPUTS & OUTPUTS IN UPPERCASE
//varialbles in lowercase

#include <Filters.h>


#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))
#define CPU_REBOOT WRITE_RESTART(0x5FA0004)

const int s_version=1080;

//INPUTS
int THROTTLE = A0;// 0-5v throttle pedal
int CR1 = A2; // GIVA inverter pin 2
int CR2 = A3; // GIWA inverter pin 18
int CR3 = A4; // MIVA inverter pin 7
int CR4 = A5; // MIWA inverter pin 23
int CR5 = A1; // LEM HASS 600s 
int BMS = 10; // BMS ready status (switch gnd from SimpBMS)
int BRAKE = 11; //+12v Brake pedal signal
int REVERSE =12; //+12 Reverse switch signal
int T1 = A6; // MIVT inverter pin 26
int HV1 = A7; // HV inverter pin 12
int F1 = A8; // GFIV inverter pin 22
int F2 = A9; // MFIV inverter pin 27

//OUTPUTS
int PWM_3 = 3; //Main Contactor
int PWM_5 = 5; //precharge
int PWM_4 = 4; // MG1 & MG2
int PWM_6 = 6; // temp gauge 
int LED = 13;

//VARIABLES
int bms_state=LOW;
int bms_errorcount=0;
int brake_state=LOW;
int reverse_state=LOW;
int analog_throttle=0;
int pwm_throttle=0;
int filtered_throttle=0;
int amps_current1=0;
int analog_current1=0;
int amps_current2=0;
int analog_current2=0;
int amps_current3=0;
int analog_current3=0;
int amps_current4=0;
int analog_current4=0;
int mg1_current=0;
int mg2_current=0;
int mg_amps=0;
int filtered_current=0;
int lem_analog;
int lem_amps;
int pwm_duty;
int pwm_requested;
int temp_analog;
int temp_degrees;
int temp_pwm;
int filtered_temp;
int hv_analog;
int hv_volts;
int filtered_volts=0;
int fail_1=0;
int fail_2=0;
bool mg1_fail=0;
bool mg2_fail=0;
int per_duty;
int per_throttle;
int error=0;
int current_limit;
int output_volts;




// USER SET VARIABLES
int cl_low = 400; //lower current limit
int cl_high = 700; //upper current limit  
const int dt = 60; //Temperature threshold for inverter overtemp
int volt_high = 220; //inverter output voltage limit
int volt_low = 250; // battery low warning

//Throttle filter//
float filterFrequency = 0.5 ;
FilterOnePole lowpassFilter( LOWPASS, filterFrequency );

// Current filter (for display purposes only)//

FilterOnePole lowpassFilter2( LOWPASS, filterFrequency );

//Voltage filter (for display purposes only)//

FilterOnePole lowpassFilter3( LOWPASS, filterFrequency );

//Temp filter (for display purposes only)//

FilterOnePole lowpassFilter4( LOWPASS, filterFrequency );




//TIMERS
elapsedMillis timer1; // Display refresh timer
elapsedMillis timer2; // Dog kicking timer
elapsedMillis timer3; // serial update timer
elapsedMillis runtime;

void setup() {
  // put your setup code here, to run once:
 analogWriteFrequency(3, 6000); // Teensy pin 3&4 changes to 6 kHz
 analogWriteFrequency(5, 20000); // Teensy pin 5&6 changes to 20 kHz
 
  
  Serial.begin(9600); // intillalize serial port at baurd rate 9600
  Serial1.begin (115200);

 



pinMode (BRAKE, INPUT);
pinMode (REVERSE, INPUT);
pinMode (BMS, INPUT);
pinMode(LED, OUTPUT);  
digitalWrite (LED, HIGH);


 

Serial1.print("page 0");
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
  delay(1000);



//CONTACTOR WELD TEST

hv_analog = analogRead (HV1);
hv_volts = map (hv_analog, 70,1023, 0, 975);
   
if (hv_volts <= 30) {
analogWrite(PWM_5,254);
delay(1000);
analogWrite (PWM_5,160);
delay (2000);}

else {
  Serial.println ("!!!WELD TEST FAILURE!!!");
  Serial1.print("page 2");
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write("status2.txt=");
  Serial1.write(0x22);
  Serial1.print ("!!! WELD TEST FAILURE !!!"); 
    Serial1.write(0x22);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
        delay (5000);
        CPU_REBOOT ;}



 // Display reason the Teensy was last reset
  Serial.println();
  Serial.println("Reason for last Reset: ");

  if (RCM_SRS1 & RCM_SRS1_SACKERR)   Serial.println("Stop Mode Acknowledge Error Reset");
  if (RCM_SRS1 & RCM_SRS1_MDM_AP)    Serial.println("MDM-AP Reset");
  if (RCM_SRS1 & RCM_SRS1_SW)        Serial.println("Software Reset");                   // reboot with SCB_AIRCR = 0x05FA0004
  if (RCM_SRS1 & RCM_SRS1_LOCKUP)    Serial.println("Core Lockup Event Reset");
  if (RCM_SRS0 & RCM_SRS0_POR)       Serial.println("Power-on Reset");                   // removed / applied power
  if (RCM_SRS0 & RCM_SRS0_PIN)       Serial.println("External Pin Reset");               // Reboot with software download
  if (RCM_SRS0 & RCM_SRS0_WDOG)      Serial.println("Watchdog(COP) Reset");              // WDT timed out
  if (RCM_SRS0 & RCM_SRS0_LOC)       Serial.println("Loss of External Clock Reset");
  if (RCM_SRS0 & RCM_SRS0_LOL)       Serial.println("Loss of Lock in PLL Reset");
  if (RCM_SRS0 & RCM_SRS0_LVD)       Serial.println("Low-voltage Detect Reset");
  Serial.println();
  ///////////////////

  
// PRECHARGE TEST

hv_analog = analogRead (HV1);
hv_volts = map (hv_analog, 70,1023, 0, 975);

if (hv_volts >= 220) {analogWrite (PWM_3,255);}

else { Serial.println ("!!! PRECHARGE FAILURE !!!");
analogWrite(PWM_5,0);
  Serial1.print("page 2");
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write("status2.txt=");
  Serial1.write(0x22);
  Serial1.print ("!!! PRECHARGE FAILURE !!!"); 
  
  Serial1.write(0x22);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
        delay (10000);
        CPU_REBOOT ;}

delay (1000);
analogWrite (PWM_5,0);


  // enable WDT
  noInterrupts();                                         // don't allow interrupts while setting up WDOG
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;                         // unlock access to WDOG registers
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1);                                   // Need to wait a bit..

  WDOG_TOVALH = 0x1000;
  WDOG_TOVALL = 0x0000;
  WDOG_PRESC  = 0;
  WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
                  WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
                  WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
  interrupts();
  

    
 
}





void loop() 
{

error=0;
  
   //THROTTLE

   
   
analog_throttle = analogRead (THROTTLE);
pwm_throttle = map (analog_throttle, 15,350, 0,254);
pwm_throttle = constrain(pwm_throttle, 0, 254);

 lowpassFilter.input(pwm_throttle);
 filtered_throttle = lowpassFilter.output();


  per_duty = pwm_duty/2.54;
 per_throttle= pwm_throttle/2.54;
  
  
        


//CURRENT


current_limit=map(filtered_throttle,0,254,cl_low,cl_high);

analog_current1 = analogRead (CR1);
amps_current1 = map(analog_current1,9,1023,0,400);

analog_current2 = analogRead (CR2);
amps_current2 = map(analog_current2,9,1023,0,400);

analog_current3 = analogRead (CR3);
amps_current3 = map(analog_current3,9,1023,0,400);

analog_current4 = analogRead (CR4);
amps_current4 = map(analog_current4,9,1023,0,400);

mg_amps = ((amps_current1 + amps_current2) * 1.5) + ((amps_current3 + amps_current4) * 1.5);
if (mg_amps <0) mg_amps =0;

//lem_analog = analogRead (CR5);
//lem_amps = map (lem_analog, 288,736, -1100,1100);
//lem_amps = constrain (lem_amps,0,1100);

lowpassFilter2.input(mg_amps);
 filtered_current= lowpassFilter2.output();




   

// INPUTS
bms_state = digitalRead (BMS);
brake_state = digitalRead (BRAKE);
reverse_state = digitalRead (REVERSE) ;



// TEMP
temp_analog = analogRead (T1);
temp_degrees = map (temp_analog, 1023,0, 0,160);


lowpassFilter3.input(temp_degrees);
 filtered_temp= lowpassFilter3.output();

     
temp_pwm = map (filtered_temp, 0,160, 0, 254);
analogWrite (PWM_6, temp_pwm);

if (filtered_temp >= dt) {  current_limit = current_limit /4 ; // over temperature current reduction
                          
                          Serial.println (" !!! POWERSTAGE TEMP HIGH !!! "); 
                              
  Serial1.write("status.txt=");
  Serial1.write(0x22);
  Serial1.print ("!!! Temp High !!!"); 
  Serial1.write(0x22);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);}


// VOLTS
hv_analog = analogRead (HV1);
hv_volts = map (hv_analog, 80,1023, 0, 975);


lowpassFilter4.input(hv_volts);
 filtered_volts= lowpassFilter4.output();

 output_volts = ((filtered_volts * pwm_duty)/254); 

 

  

   
       
//STATES

fail_1 = analogRead (F1);
  fail_1 = fail_1 * 0.0098;
  if (fail_1 < 5 ) mg1_fail=1;
    else (mg1_fail=0);

    fail_2 = analogRead (F2);
  fail_2 = fail_2 * 0.0098;
  if (fail_2 < 5 ) mg2_fail=1;
    else (mg2_fail=0);

    




if (bms_state==LOW) 
          {bms_errorcount ++; }
              
else{bms_errorcount =0;}



if (reverse_state== LOW && mg1_fail==0 && mg2_fail==0) { Forward();}
else if (reverse_state== HIGH && mg1_fail==0 && mg2_fail==0) { Reverse();}
else {error=3; fail() ;}


if (filtered_volts <= volt_low) {error=1; fail();}

          

          
 


//ClOSING FUNCTIONS

 if (timer3 >= 100)
      {timer3 = timer3-100;
       serial_update();
       dash_update();}
   
 

 if (timer2 >=800) 
    {timer2 = timer2-800; 
    resetwdog();}


    




//END OF LOOP
}



void Forward() 
{   

  //Serial.println ("Forwards");
  Serial1.write("status.txt=");
  Serial1.write(0x22);
  Serial1.print ("Forwards");
   Serial1.write(0x22);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);

  
  if (filtered_throttle > pwm_duty && current_limit > mg_amps && volt_high > output_volts) { pwm_duty++; pwm_duty++;} 

  
  if (filtered_throttle<pwm_duty ) {  pwm_duty --; pwm_duty--; }
    
       
  
  if (pwm_duty<0)  pwm_duty=0; // keeping pwm within min of 0%  limits
  if (pwm_duty>242)  pwm_duty=245; // keeping pwm within max of 95%
    
  
  if (brake_state == HIGH) pwm_duty =0; // Brake switch kills pwm output
   if (bms_errorcount >= 2000)  
       { error = 2; fail();
          }
           

    
  analogWrite(PWM_4, pwm_duty);


  
  }

void Reverse() {  

  Serial.println ("Rerverse");
    Serial1.write("status.txt=");
  Serial1.write(0x22);
  Serial1.print ("Reverse");
   Serial1.write(0x22);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
 
      
 
      
  if (filtered_throttle/2>pwm_duty && mg_amps <= 100) pwm_duty++; 
  if (filtered_throttle/2<pwm_duty) pwm_duty--;
      

  
  if (pwm_duty<0)  pwm_duty=0; // keeping pwm within 0-254 limits
  if (pwm_duty>=40)  pwm_duty--;
    
      

  if (mg_amps >= 300) pwm_duty =0;
  if (brake_state == HIGH) pwm_duty =0; // Brake switch kills pwm output
 // if (bms_state == LOW) pwm_duty = 0;
    
  analogWrite(PWM_4, pwm_duty);
  
  }

  void fail()
  {
  
  
  if (error==3)

  {
  analogWrite(PWM_4,0) ;
  
  analogWrite (PWM_5,0);

  
  Serial.println("  !!! POWERSTAGE FAULT !!!  ");
    
  Serial1.print("page 2");
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write("status2.txt=");
  Serial1.write(0x22);
  Serial1.print ("!!! POWERSTAGE FAULT !!!");
  Serial1.write(0x22);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
   delay (8000);
        CPU_REBOOT ;
  }

  if(error==2)
  {
    analogWrite(PWM_4,0) ;
    Serial1.print("page 2");
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write("status2.txt=");
    Serial1.write(0x22);
    Serial1.print ("!!! BMS FAULT !!!");
    Serial1.write(0x22);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    delay (3000);
    Serial1.print("page 0");
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    loop();
  
      
  }

   if(error==1)
  {
    analogWrite(PWM_4,0) ;
    Serial1.print("page 2");
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write("status2.txt=");
    Serial1.write(0x22);
    Serial1.print ("!!! VOLTAGE LOW !!!");
    Serial1.write(0x22);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    delay (3000);
        CPU_REBOOT ;
    
   }

  }


void serial_update()
{

Serial.print ("Sofware ");
Serial.println (s_version);
Serial.println (""); 

Serial.print ("Runtime ");
Serial.print (runtime);
Serial.println ("");
Serial.print ("Throttle ADC");
Serial.println (analog_throttle);

Serial.print ("Throttle pwm requested");
Serial.print (" ");
Serial.println (pwm_throttle);
Serial.print ("Throttle pwm actual");
Serial.print (" ");
Serial.println (pwm_duty);
Serial.print("MG current");
Serial.print (" ");
Serial.println(filtered_current);
Serial.print ("LEM current");
Serial.print (" ");
Serial.println (lem_amps);
Serial.print ("Temperature");
Serial.print (" ");
Serial.println (filtered_temp);
Serial.print("Bus Voltage");
Serial.print(" ");
Serial.println(filtered_volts);
Serial.print ("Output Voltage ");
Serial.println (output_volts);

Serial.print ("fail1 volts");
Serial.println (fail_1);
Serial.print ("fail2 volts");
Serial.println (fail_2);

Serial.print ("AMPS MG1s1");
Serial.println (amps_current1);
Serial.print ("AMPS MG1s2");
Serial.println (amps_current2);
Serial.print ("AMPS MG2s1");
Serial.println (amps_current3);
Serial.print ("AMPS MG2s2");
Serial.println (amps_current4);  
Serial.println ("");
Serial.println ("");

Serial.print ("Reverse state");
Serial.println (reverse_state);
Serial.print ("Brake State");
Serial.println (brake_state);
Serial.print ("BMS state");
Serial.println (bms_state);

Serial.print ("VH adc");
Serial.println (hv_analog);


if (brake_state == HIGH) Serial.println ("BRAKE");

Serial.println ("");
Serial.println ("");
Serial.println ("");




  }

  void dash_update()
  {

 
  
  Serial1.print("amps1.val=");
  Serial1.print(filtered_current *10 );
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.print("thr.val=");
  Serial1.print(per_throttle);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.print("pwm.val=");
  Serial1.print(per_duty);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
   
 if (timer1 >= 500) { timer1 = timer1 - 500;    
    Serial1.print("hv1.val=");
    Serial1.print(filtered_volts *10);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("temp1.val=");
    Serial1.print(filtered_temp *10);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
    }
  }


void resetwdog()
{
  noInterrupts();                                     //   No - reset WDT
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}






  
  
