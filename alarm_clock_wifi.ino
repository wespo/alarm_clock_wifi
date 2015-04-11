#include "sRTCsec.h";
#include "legacymsp430.h"
#include "TimerSerial.h"
#include "ClickButton.h"
#include "TM1637.h"
#ifndef __CC3200R1M1RGC__
// Do not include SPI for CC3200 LaunchPad
#include <SPI.h>
#endif
#include <WiFi.h>

// your network name also called SSID
char ssid[] = "AndroidAP";
// your network password
char password[] = "launchpad";

unsigned int localPort = 2390;      // local port to listen for UDP packets

IPAddress timeServer(206,246,122,250); // time.nist.gov NTP server

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;



#define CLK 9//pins definitions for TM1637 and can be changed to other ports
#define DIO 10

#define BUZZER_PIN               39            /* sig pin of the buzzer */
TM1637 tm1637(CLK,DIO);

#define OFF 0
#define ON 1

int8_t TimeDisp[] = {
  0x00,0x00,0x00,0x00};
  
int8_t RealTimeDisp[] = {
  0x00,0x00,0x00,0x00};
unsigned char ClockPoint = 1;
unsigned char Update;

// Declaring the constant values that will not change 

const int  buttonPin1 = PUSH1;    // the pin that the pushbutton is attached to
ClickButton button1(buttonPin1, LOW, CLICKBTN_PULLUP);

const int  buttonPin2 = PUSH2;    // the pin that the pushbutton is attached to
ClickButton button2(buttonPin2, LOW, CLICKBTN_PULLUP);

const int ledPin1 = RED_LED;       // the pin that the LED is attached to
const int ledPin2 = GREEN_LED;      // the pin that the LED is attached to

TimerSerial mySerial;
RealTimeClockSec myRTC;

int ledState = LOW;  

// Declaring the variables that will change:

// Referring to PUSH 1
int buttonPushCounter1 = 0;   // counter for the number of button presses
int buttonState1 = 0;         // current state of the button
int lastButtonState1 = 0;     // previous state of the button

// Referring to PUSH 2
int buttonPushCounter2 = 0;   // counter for the number of button presses
int buttonState2 = 0;         // current state of the button
int lastButtonState2 = 0;     // previous state of the button

int hrsSet =  0;              // keeps track of the actual time hour set 
int minSet =  0;              // keeps track of the actual time minute set 
int alarmHrs = 0;             // keeps track of the alarm time hour set 
int alarmMin = 0;             // keeps track of the alarm time minute set 
int alarmSec = 0;             // keeps track of the alarm time second set 

// Various flags used  throughout the timeset and alarmset functions:
int hrflag = 0;               // keeps track of time set toggle for hours
int minflag = 0;              // keeps track of time set toggle for minutes
int firstTime = 1;
int clicksflag = -1;
int stateflag = 0;
int alarmflag  = 0; // keeps track of alarm on/off 0 = off



// ButtonClicks1 function calculates the number of times Push 1 has been pressed
void ButtonClicks1(void) {
  // read the pushbutton input pin:
  buttonState1 = !digitalRead(buttonPin1);

  // compare the buttonState to its previous state
  if (buttonState1 != lastButtonState1) {
    // if the state has changed, increment the counter
    if (buttonState1 == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      buttonPushCounter1++;
      Serial.println("on");
      Serial.print("number of button 1 pushes:  ");
      Serial.println(buttonPushCounter1);
    }
    else {
      // if the current state is LOW then the button
      // wend from on to off:
      Serial.println("off");
    }
  }
  // save the current state as the last state,
  //for next time through the loop
  lastButtonState1 = buttonState1;
}


// ButtonClicks2 function calculates the number of times Push 2 has been pressed
void ButtonClicks2(void) {
  // read the pushbutton input pin:
  buttonState2 = !digitalRead(buttonPin2);

  // compare the buttonState to its previous state
  if (buttonState2 != lastButtonState2) {
    // if the state has changed, increment the counter
    if (buttonState2 == HIGH) {
      // if the current state is HIGH then the button
      // wend from off to on:
      buttonPushCounter2++;
      Serial.println("on");
      Serial.print("number of button 2 pushes:  ");
      Serial.println(buttonPushCounter2);
    }
    else {
      // if the current state is LOW then the button
      // wend from on to off:
      Serial.println("off");
    }
  }
  // save the current state as the last state,
  //for next time through the loop
  lastButtonState2 = buttonState2;

}


// TimeUpdate allows the user to set the actual time and places the values set into RTC
// Time is displayed in 4-Digit display 

void TimeUpdate(void)
{
  if(ClockPoint)tm1637.point(POINT_ON);    // Turns 
  else tm1637.point(POINT_OFF);

   while(1){
   button1.Update();
   ButtonClicks1();
   ButtonClicks2();
  if ((buttonPushCounter1 % 2) == 0)      // Editing the hour side HH:MM = H1 H2 :M1 M2
  {
Serial.println("TIME UPDATE MODE we are changing hours");
    if(hrflag == 0){
      hrflag = 1;
      minflag = 0;

      if(firstTime){
        minSet = 0;
        firstTime = 0;
      }
      else
        minSet = buttonPushCounter2;

      buttonPushCounter2 = hrsSet;
    }

    hrsSet = (buttonPushCounter2 % 24);
  
    TimeDisp[0] = (hrsSet/10)%10;
    TimeDisp[1] = (hrsSet%10)%10;
    
  }
  else if ((buttonPushCounter1 % 2) == 1) 
  {
    Serial.println("TIME UPDATE MODE we are changing mins");
     if(minflag == 0){
      hrflag = 0;
      minflag = 1;
      hrsSet = buttonPushCounter2;
      buttonPushCounter2 = minSet;
    }

    minSet = (buttonPushCounter2 % 60);
    
    TimeDisp[2] = (minSet /10)%10;
    TimeDisp[3] = (minSet %10)%10;

  }
    
  button1.Update();
  tm1637.display(TimeDisp);
  

    if(button1.clicks ==2){    // Exit Time Update mode when double clicking
    
    myRTC.RTC_hr = hrsSet % 24;
    myRTC.RTC_min = minSet % 60;
    myRTC.RTC_sec = 0;
    stateflag = 0;
    return;
   }
   }
  Update = OFF;
};



// AlarmSet function allows user to set an alarm time and stores values 
// Alarm time is displayed in 4-Digit display 

void AlarmSet(void)
{
  
  if(ClockPoint)tm1637.point(POINT_ON);
  else tm1637.point(POINT_OFF);

   while(1){
   button1.Update();
   ButtonClicks1();
   ButtonClicks2();
  if ((buttonPushCounter1 % 2) == 0)  
  {
Serial.println("ALARM SET MODE we are changing hours");
    if(hrflag == 0){
      hrflag = 1;
      minflag = 0;

      if(firstTime){
        minSet = 0;
        firstTime = 0;
      }
      else
        minSet = buttonPushCounter2;

      buttonPushCounter2 = hrsSet;
    }

    hrsSet = (buttonPushCounter2 % 24);
  
    TimeDisp[0] = (hrsSet/10)%10;
    TimeDisp[1] = (hrsSet%10)%10;
    
  }
  else if ((buttonPushCounter1 % 2) == 1) 
  {
    Serial.println("ALARM SET MODE we are changing mins");
     if(minflag == 0){
      hrflag = 0;
      minflag = 1;
      hrsSet = buttonPushCounter2;
      buttonPushCounter2 = minSet;
    }

    minSet = (buttonPushCounter2 % 60);
    
    TimeDisp[2] = (minSet /10)%10;
    TimeDisp[3] = (minSet %10)%10;
  }
   
  
  button1.Update();
  tm1637.display(TimeDisp);
     
    alarmHrs = hrsSet % 24;
    alarmMin = minSet % 60;
    
    if(button1.clicks ==2){    // Exit Alarm Set mode when double clicking
    stateflag = 0;
    Serial.print("Alarm Time Set:");
    Serial.print(alarmHrs);
    Serial.print(":");
    Serial.print(alarmMin);
    return;
   }
   }
  Update = OFF;
};


// AlarmOn checks if all criteria is met to turn on the Alarm
void AlarmOn(void)
{
  // If the alarm is on and the alarm time set matches the current time
  if ((alarmflag == 1) && (alarmHrs == myRTC.RTC_hr) && (alarmMin == myRTC.RTC_min) && (alarmSec == myRTC.RTC_sec))
    {  
    digitalWrite(ledPin2, HIGH);       // turn the Green LED on while alarm goes off
    Serial.println("Alarm IS ON!!");   // Prints to Serial Monitor (Alarm is on)
    digitalWrite(BUZZER_PIN,HIGH);
   
    }
    else if (digitalRead(PUSH2) == LOW) {
    digitalWrite(ledPin2, LOW);   // turn the Green LED off while alarm goes off
    digitalWrite(BUZZER_PIN,LOW);
    return;
    }  
}


// Debug Function - Allows user to view the current time, 
void Debug (void)
{
    Serial.print(myRTC.RTC_hr, DEC);
    Serial.print(":");
    Serial.print(myRTC.RTC_min, DEC);
    Serial.print(":");
    Serial.print(myRTC.RTC_sec, DEC);
    Serial.print("   Alarm Flag: ");
    Serial.print(alarmflag);
    Serial.print("   Alarm Set Time ");
    Serial.print(alarmHrs);
    Serial.print(":");
    Serial.println(alarmMin);
    
}

// QuickBeep funtion makes the buzzer beep once for 200ms
void QuickBeep(void){
  digitalWrite(BUZZER_PIN,HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN,LOW);
}

// QuickBeeps funtion makes the buzzer beep twice for 200ms with 100ms delay in between
void QuickBeeps(void){
  digitalWrite(BUZZER_PIN,HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN,LOW);
  delay(100);
  digitalWrite(BUZZER_PIN,HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN,LOW);
}
void fin() {
  Serial.end();
};




// Setup function runs once at the start

void setup() {
  // initialize the button pin as a input:
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  // initialize the LED as an output:
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);    
  digitalWrite(BUZZER_PIN,LOW);
  pinMode(2, OUTPUT);
  
  // This is to close the serial output so you are able to upload a new sketch to the LaunchPad
  pinMode(5, INPUT_PULLUP),
  attachInterrupt(5,fin,LOW);

  // initialize serial communication:
  Serial.begin(9600);
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to Network named: ");
  // print the network name (SSID);
  Serial.println(ssid); 
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  WiFi.begin(ssid);//, password);
  while ( WiFi.status() != WL_CONNECTED) {
    // print dots while we wait to connect
    Serial.print(".");
    delay(300);
  }
  
  Serial.println("\nYou're connected to the network");
  Serial.println("Waiting for an ip address");
  
  while (WiFi.localIP() == INADDR_NONE) {
    // print dots while we wait for an ip addresss
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nIP Address obtained");
  
  // you're connected now, so print out the status  
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  Udp.begin(localPort);
  
  // Adjusting the timer preferences
  tm1637.init();
  tm1637.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;

  button1.debounceTime   = 20;   // Debounce timer in ms
  button1.multiclickTime = 250;  // Time limit for multi clicks
  button1.longClickTime  = 1000; // time until "held-down clicks" register
  

}

int NTPTimeUpdate()
{
   sendNTPpacket(timeServer); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(2000);
  if ( Udp.parsePacket() ) {
    Serial.println("packet received");
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    epoch = epoch - (7 * 3600); //time zome offset, PDT

  myRTC.RTC_hr = (epoch  % 86400L) / 3600;
  myRTC.RTC_min = (epoch  % 3600) / 60;
  myRTC.RTC_sec = epoch % 60;
  return 1;
  }
  return 0;
}
void loop() {
  
  while(!NTPTimeUpdate()){};
     

  button1.Update();

  ButtonClicks1();
  ButtonClicks2();

  
switch (stateflag){ //Toggle between the different modes
  
  case 2:    // TIME UPDATE MODE 
    
    Serial.println("ENTERED TIME UPDATE MODE");
    digitalWrite(ledPin2, HIGH);   // turn the Green LED while time is being set
    TimeUpdate();                  // Calls the TimeUpdate function
    tm1637.display(TimeDisp);      // Displays the time on LCD
    digitalWrite(ledPin2, LOW);    // turn the green LED off 
    Serial.println("EXIT TIME UPDATE MODE");
    QuickBeeps();
    
    break;
    
  case 3:    // ALARM SET MODE
    
    Serial.println("ENTERED ALARM SET MODE");
    digitalWrite(ledPin1, HIGH);   // turn the Red LED while alarm time is being set
    AlarmSet();                    // Calls the AlarmSet funtion
    tm1637.display(TimeDisp);      // Displays time on LCD
    digitalWrite(ledPin1, LOW);    // turn the red LED off
    Serial.println("EXIT ALARM SET MODE");
    QuickBeeps();
    break;
  
  case 0:    // Default mode where time is displayed once it has been set
    
    Serial.println("TIME DISPLAY MODE");
    
    while(1) {
    AlarmOn();
    RealTimeDisp[0] = (myRTC.RTC_hr / 10);
    RealTimeDisp[1] = (myRTC.RTC_hr % 10); 
    RealTimeDisp[2] = (myRTC.RTC_min / 10); 
    RealTimeDisp[3] = (myRTC.RTC_min % 10);
    tm1637.display(RealTimeDisp);   
  
    if ((myRTC.RTC_sec % 2)==0 && ledState== LOW) {
      ledState = HIGH;
     Debug();
    };
  
    if ((myRTC.RTC_sec % 2)==1 && ledState== HIGH) {
      ledState = LOW;
     Debug();
     
   };


  ButtonClicks2();
  
  // Turning the Alarm On or Off pressing Push 2 while in Default Mode
   if ((buttonPushCounter2 % 2) == 0) {
      digitalWrite(ledPin1, LOW);   // turn the Red LED to display ALARM is ON
      alarmflag = 0;
      
    }
    else if ((buttonPushCounter2 % 2) == 1) {
     digitalWrite(ledPin1, HIGH);   // turn the Red LED to display ALARM is ON    
     alarmflag = 1;
     
  }
    button1.Update();
      
    if (button1.clicks == 2){
      stateflag = 2;
     
      
      QuickBeeps();
      break;
    }
    if (button1.clicks == 3){
      stateflag = 3;
      QuickBeeps();
      break;
    }
    }
    break;
}

};



interrupt(TIMER1_A0_VECTOR) Tic_Tac(void) {
  myRTC.Inc_sec();              // Update seconds
};

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  //Serial.println("1");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  //Serial.println("2");
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  //Serial.println("3");

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  //Serial.println("4");
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  //Serial.println("5");
  Udp.endPacket();
  //Serial.println("6");
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}







