
//*************************************Bibs**********************************
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DS3231.h>
#include <Stepper.h>
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <Servo.h>
#include <IRremote.h>
#include <EEPROM.h>
//***********************DFPLAYER****************************
SoftwareSerial mySoftwareSerial(5, 6); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
bool musicFullHour    =false;
bool musicQuarterHour =false;
bool musicAlarm1      =false;
bool musicAlarm2      =false;
bool musicTimer       =false;
bool music            =false;
//************************************************************************
//***********************Stepper/Stundenzeiger****************************
#define STEPS 2048 // the number of steps in one revolution of your motor (28BYJ-48)
Stepper stepper(STEPS, 4, 8, 7, 12);    //Stepper anschluesse
byte stunden ;
byte prevStunden ;  //Speichern/auslesen im EEPROM
const byte hallPin=2;
//*************************************************************************
//***********************Ziffernblatt****************************
const int address = 0x20;
byte LEDminute;
byte LEDprevMin=61;
bool LEDentry=false;
unsigned long prevMillis=0;
bool toggle=true;
//*************************************************************************
//***********************LCD's/////RTC**************************************
LiquidCrystal_I2C lcd1(0x27, 16,2);
LiquidCrystal_I2C lcd2(0x26, 16,2);
DS3231  rtc(SDA, SCL);
byte lcd2AnzeigeNummer=0;
byte lcd2PrevAnzeigeNummer=0;
//***************************************************************************
//*********************************TIMER*************************************
bool timerState = false;
byte timerStunden;
byte  timerMinuten;
byte  timerSekunden;
const byte timerSong =3;  
//***************************************************************************
//*********************************Alarm*************************************
//*********************************Alarm1*************************************
bool alarm1State = true;
bool repeatAlarm1 =true;
byte alarm1Stunden ; //Eeprom
byte alarm1Minuten ;  //Eeprom
byte alarm1Song ;
const byte alarm1StundenAddress=1;
const byte alarm1MinutenAddress=2;
const byte alarm1SongAddress=3;
//*********************************Alarm12************************************
bool alarm2State =true;
bool repeatAlarm2 =true;
byte alarm2Stunden ; //Eeprom
byte alarm2Minuten ;  //Eeprom
byte alarm2Song ;
const byte alarm2StundenAddress=4;
const byte alarm2MinutenAddress=5;
const byte alarm2SongAddress=6;
//***************************************************************************
//***********************************Kuckuck****************************************
Servo myservo;
const byte servoPin=9;
bool kuckuckState=false;
bool forward=true;
byte kuckuckPOS=0;
unsigned long kuckuckMillis;
//***************************************************************************''''''
//************************************Hauptmenü************************************
enum menuConfig{idle,mainMenu,DateTime,alarmMenu,alarm1Menu, alarm2Menu, alarm1Time, alarm2Time, alarm1Tone, alarm2Tone, timerMenu, nightmodeMenu,dateMenu,timeMenu};

menuConfig menuState=idle;
menuConfig prevMenuState=idle;
//*********************************************************************************
//************************************GENERAL''''''''''''''''''''''''''''''''''''''
byte prevSek;
bool timerAktion;
bool alarm1Trigger =false;
bool alarm2Trigger =false;
const byte fullHourSong=1;
const byte quarterHourSong=2;
byte minuten;
byte sekunden;
byte prevMin;
bool entry=true;
//*********************************************************************************
//************************************HELLIGKEITLCD********************************
const byte pinLCDBrightness1 =10;
const byte pinLCDBrightness2 =3;
const int  LDReingang= A0;
byte brightness; 
//*********************************************************************************
//*************************************IR******************************************
const byte  RECV_PIN =11;
IRrecv irrecv(RECV_PIN);
decode_results results;
//*********************************************************************************
//*************************************NACHTMODUS******************************************
bool nightMode=false;

//*********************************************************************************
//************************************BUTTONS*************************************
unsigned long debounceTime=0;
enum buttonConfig{notPressed, Ok, Return, Left, Up, Down, Right}; 
buttonConfig buttonState = notPressed;
const int  buttonsEingang= A1;
//********************************************************************************
//****************************************HOCHZÄHLVARIABLEN FÜR DATUM UHRZEIT ALARM TIMER **********************************
byte einstellHour;
byte einstellMin;
byte einstellSec;
byte einstellDate;
byte einstellMonth;
int  einstellYear;
int einstellDOW;
bool einstellBool=true;
byte einstellTon;
byte cursorPos=1;
#define maxNumbersAlarm 3   //hh:mm:repeat
#define maxNumbersTime 3    //hh:mm:ss
#define maxNumbersDate 4    //DD:MM:YYYY:DOW
#define UP true
#define DOWN false
#define RIGHT true
#define LEFT false
const byte UParrow[]{
 B00000,
  B00100,
  B01110,
  B11111,
  B00100,
  B00100,
  B00100,
  B00000
};
const byte DOWNarrow[] = {
  B00000,
  B00100,
  B00100,
  B00100,
  B11111,
  B01110,
  B00100,
  B00000
};
const byte RIGHTarrow[] = {
  B00000,
  B00000,
  B00100,
  B00110,
  B11111,
  B00110,
  B00100,
  B00000
};
const byte LEFTarrow[] = {
  B00000,
  B00000,
  B00100,
  B01100,
  B11111,
  B01100,
  B00100,
  B00000
};
const byte repeatSign[] = {
  B00000,
  B00111,
  B00110,
  B10101,
  B10001,
  B01010,
  B00100,
  B00000
};
const byte noRepeatSign[] = {
  B00000,
  B00100,
  B01010,
  B10001,
  B10001,
  B01010,
  B00100,
  B00000
};
//*******************************************************************************************************
//*********************************************************************NODE RED**************************
int firstNumber;
int secondNumber;
int thirdNumber;
int fourthNumber;
int fifthNumber;
int sixthNumber;
int seventhNumber;
//************************************************************************************************************

void setup()
{ 
   Wire.begin();
   Serial.begin(115200);
   //Serial.println("Start");
//*****************************Read EEPROM************************
alarm1Stunden= EEPROM.read(alarm1StundenAddress);   
alarm1Minuten= EEPROM.read(alarm1MinutenAddress); 
alarm1Song= EEPROM.read(alarm1SongAddress); 
alarm2Stunden= EEPROM.read(alarm2StundenAddress);   
alarm2Minuten= EEPROM.read(alarm2MinutenAddress); 
alarm2Song= EEPROM.read(alarm2SongAddress); 
//**************LCD************************************************
  lcd1.begin();
  lcd1.clear();
  lcd1.backlight();
  lcd1.print("Hello");
  lcd2.begin();
  lcd2.clear();
  lcd2.backlight();
  lcd2.createChar(0, LEFTarrow);
  lcd2.createChar(1, UParrow);
  lcd2.createChar(2, DOWNarrow);
  lcd2.createChar(3, RIGHTarrow);
  lcd2.createChar(4, repeatSign);
  lcd2.createChar(5, noRepeatSign);
  lcd2.home();
  //*************RTC***************************
  rtc.begin();
  //rtc.setTime(23,59,30);
  //rtc.setDate(31,12,2020);
  //rtc.setDOW(4);
   //******DFPLAYER******************************************
  mySoftwareSerial.begin(9600);
  //Serial.println("Start");
    if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  //Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.volume(20);
  //*******************KUCKUCK***************************
   myservo.attach(servoPin);
   myservo.write(kuckuckPOS);
  //***********STUNDENZEIGER*****************************
  pinMode(hallPin,INPUT);
  stepper.setSpeed(15);
  stunden=rtc.getTime().hour;
  stundenzeigerSetup();
  stepper.step(HtoStep(stunden));
  prevStunden=stunden;
  // ******************Ziffernblatt********************************
  pf575_write(word(B11111111,B11111111));   //Alle Pins als Output
  minuten=rtc.getTime().min;
  ziffernblattSetup(minuten);
  //*****************************IR***********************************
  irrecv.enableIRIn();
  //****************************GENERAL*******************************
  prevSek=rtc.getTime().sec;
  prevMin=minuten;
  //***************************HelligkeitLCD*************************
  pinMode(pinLCDBrightness1,OUTPUT); //Oberen i2C pin an pwm pin 10
  pinMode(pinLCDBrightness2,OUTPUT); //Oberen i2C pin an pwm pin 10
  

}
void loop()
{
  SerialRead();
  SerialPrint();
  IRRemote();
  tastendruckRead(analogRead(buttonsEingang));
  ControlAll();
  stundenzeigerMOVE();
  if(music==true){
  mp3player(fullHourSong,quarterHourSong,alarm1Song,alarm2Song,timerSong);
  music=false;
  }
  if(nightMode==true&&kuckuckState==true){
    kuckuckState=false;
  }
  if(nightMode==false){
  ziffernblatt(minuten);
  kuckuck();
}
  brightnessControl(analogRead(LDReingang));
  menuSwitch(menuState,buttonState);
  anzeige1();
  anzeige2();
}
String Timer(bool zaehlSekundeRunter)
{
  String timerZeit;
if(timerStunden==0&&timerMinuten==0&&timerSekunden==0&&timerState==true){
      musicTimer=true;
      timerState=false;
      music=true;
    }
  else if(timerMinuten==0&&timerSekunden==0&&timerState==true){
      timerStunden--;
      timerMinuten=59;
    }
  else if(timerSekunden==0&&timerState==true){
      timerMinuten--;
  }

    timerZeit=String("T: "+intConverter(timerStunden)+":"+intConverter(timerMinuten)+":"+intConverter(timerSekunden));
    
    if(timerSekunden==0){
      timerSekunden=59;      
    }
    if(zaehlSekundeRunter==true && timerState==true){
      timerSekunden--;
    }
    if(timerState==false){
      timerSekunden=0;
    }
        timerAktion=false;
        return timerZeit;
  }
String AlarmOne(bool alarm1Ausloesen, bool alarm1Status)
{

  String alarm1Ausgabe;
  if(alarm1Ausloesen==true){
      alarm1Trigger=false;
  }
  if(alarm1Status==true){
    if(repeatAlarm1==true){
      alarm1Ausgabe = String("A1: "+intConverter(alarm1Stunden)+":"+intConverter(alarm1Minuten)+" ON R");  //Repeatzeichen einfügen bzw RepeatZeichen funktion aufrufen
      //lcd2.setCursor(14,0);
      //lcd2.write(4);
    }
    else{
    alarm1Ausgabe = String("A1: "+intConverter(alarm1Stunden)+":"+intConverter(alarm1Minuten)+" ON");
    }
  }
    if(alarm1Status==false){

    alarm1Ausgabe = String("A1: "+intConverter(alarm1Stunden)+":"+intConverter(alarm1Minuten)+" OFF");
    
  }
  return(alarm1Ausgabe);
}
String AlarmTwo(bool alarm2Ausloesen, bool alarm2Status)
{

  String alarm2Ausgabe;
  if(alarm2Ausloesen==true){
      alarm2Trigger=false;
  }
  if(alarm2Status==true){
    if(repeatAlarm2==true){
    alarm2Ausgabe = String("A2: "+intConverter(alarm2Stunden)+":"+intConverter(alarm2Minuten)+" ON R"); //Repeatzeichen einfügen
    //lcd2.setCursor(14,0);
    //lcd2.write(4);
    }
    else{
    alarm2Ausgabe = String("A2: "+intConverter(alarm2Stunden)+":"+intConverter(alarm2Minuten)+" ON");
    }
  }
    if(alarm2Status==false){

   alarm2Ausgabe = String("A2: "+intConverter(alarm2Stunden)+":"+intConverter(alarm2Minuten)+" OFF");
  }
  return(alarm2Ausgabe);
}  
void ziffernblattSetup(byte newMin) //Einmalig zu Beginn und nach jedem Uhrzeiteinstellen, nachtmodus wechsel ausführen
{    
 
  if( newMin <5){
  pf575_write(word(B00001111,B11111110));
  LEDminute=0;
  }
  if(newMin >=5 && newMin <10){
  pf575_write(word(B00001111,B11111100));
  LEDminute=5;
  }
  if(newMin>=10 && newMin <15){
    pf575_write(word(B00001111,B11111000));
    LEDminute=10;
  }
  if(newMin>=15 && newMin <20){
      pf575_write(word(B00011111,B11110000));
      LEDminute=15;
  }
  if(newMin>=20 && newMin <25){
        pf575_write(word(B00001111,B11100000));
        LEDminute=20;
  }
  if(newMin>=25 && newMin <30){
      pf575_write(word(B00001111,B11000000));
      LEDminute=25;
  }
  if(newMin>=30 && newMin <35){
 pf575_write(word(B00001111,B10000000));
 LEDminute=30;
  }
  if(newMin>=35 && newMin <40){
   pf575_write(word(B00001111,B00000000));
   LEDminute=35;
  }
  if(newMin>=40 && newMin <45){
     pf575_write(word(B00001110,B00000000));
     LEDminute=40;
  }
  if(newMin>=45 && newMin <50){
     pf575_write(word(B00001100,B00000000));
     LEDminute=45;
  }
  if(newMin>=50 && newMin <55){
     pf575_write(word(B00001000,B00000000));
     LEDminute=50;
  }
  if(newMin>=55){
     pf575_write(word(B00000000,B00000001));
     LEDminute=55;
  }
}
void ziffernblatt(int a)  //Steuerung der Ziffernblatt LEDS a=minuten
{
if(kuckuckState==true){
     if(millis()-prevMillis >500){     
         if(toggle==true){  
                    pf575_write(word(B00001111,B11111111));
                    toggle=false;
                    }
           else{
                pf575_write(word(B00000000,B00000000));
                toggle=true;
                }    
           prevMillis=millis();   
              }  
             } 
   else{          
  switch(a){
    case 0:
            if(LEDentry==true){
             pf575_write(word(B00001111,B11111110));
             LEDentry=false;
             LEDminute=0;
             }

     break;   
     case 5:
            if(LEDentry==true){
            pf575_write(word(B00001111,B11111100));
            LEDentry=false;
            LEDminute=5;
            }     
     break;
     case 10:
            if(LEDentry==true){
            pf575_write(word(B00001111,B11111000));
            LEDentry=false;
            LEDminute=10;
            }
     break;
     case 15:
            if(LEDentry==true){
            pf575_write(word(B00011111,B11110000));
            LEDentry=false;
            LEDminute=15;
            }
     break;
     case 20:
            if(LEDentry==true){
            pf575_write(word(B00001111,B11100000));
            LEDentry=false;
            LEDminute=20;
            }
     break;
     case 25:
            if(LEDentry==true){
            pf575_write(word(B00001111,B11000000));
            LEDentry=false;
            LEDminute =25;
            }
     break;
     case 30:
            if(LEDentry==true){
            pf575_write(word(B00001111,B10000000));
            LEDentry=false;
            LEDminute=30;
            }
     break;
     case 35:
            if(LEDentry==true){
            pf575_write(word(B00001111,B00000000));
                      LEDentry=false;
                      LEDminute=35;
            }
     break;
     case 40:
            if(LEDentry==true){
            pf575_write(word(B00001110,B00000000));
            LEDentry=false;
            LEDminute=40;
            }
     break;
     case 45:
            if(LEDentry==true){
            pf575_write(word(B00001100,B00000000));
            LEDentry=false;
            LEDminute=45;
            }
     break;
     case 50:
            if(LEDentry==true){
            pf575_write(word(B00001000,B00000000));
            LEDentry=false;
            LEDminute=50;
            }
     break;
     case 55:
            if(LEDentry==true){
            pf575_write(word(B00000000,B00000001));
            LEDentry=false;
            LEDminute=55;
            }
     break; 
  }
 }
}
void pf575_write(uint16_t data) // Function for writing two Bytes to the I2C expander device
{

  Wire.beginTransmission(address);
  Wire.write(lowByte(data));
  Wire.write(highByte(data));
  Wire.endTransmission();
}
void stundenzeigerMOVE()  //Bewegung des Stundenzeigers
{    
    if(stunden==0&&prevStunden==11){
      while(digitalRead(hallPin!=0)){
        stepper.step(1);
        }    
      prevStunden=stunden;
    }
    if(stunden==1&&prevStunden==0){
     stepper.step(171);
     prevStunden=stunden;
    }
   else{ 
     stepper.step(HtoStep(stunden)-HtoStep(prevStunden));
     prevStunden=stunden;
   }
}
void stundenzeigerSetup()
{
  while(digitalRead(hallPin)!=0){
        stepper.step(1);
        
        }
            
}
int HtoStep (int n) //Funktion der Stepperwerte
{ 
  int stundenStep;
    if(n>11){ //Stunden auf 12Format
    n=n-12;
  }
    switch(n){
    case 0:
      stundenStep=2048;
      break;
    case 1:
      stundenStep = 171;
      break;
    case 2:
      stundenStep=342;
      break;
    case 3:
      stundenStep=512;
      break;
    case 4:
      stundenStep = 683;
      break;
    case 5:
      stundenStep=854;
      break;
     case 6:
      stundenStep=1024;
      break;
    case 7:
      stundenStep = 1195;
      break;
    case 8:
      stundenStep=1366;
      break;
    case 9:
      stundenStep=1536;
      break;
    case 10:
      stundenStep = 1707;
      break;
    case 11:
      stundenStep=1878;
      break;                
                   
  }
  return stundenStep;
}
void mp3player(byte b, byte c, byte d,byte e, byte f)  //songnummer für stunden,viertelstunden,A1,A2,Timer
{
  int songLaenge;
  if(musicFullHour==true){
    myDFPlayer.playFolder(15, b);
   musicFullHour=false;
  }
  if(musicQuarterHour==true){
    myDFPlayer.playFolder(15, c);
    musicQuarterHour=false;
  }
    if(musicAlarm1==true){
    myDFPlayer.playFolder(16, d);
    musicAlarm1=false;
  }
    if(musicAlarm2==true){
    myDFPlayer.playFolder(16, e);
    musicAlarm2=false;
  }
    if(musicTimer==true){
    myDFPlayer.playFolder(15, f);
    musicTimer=false;
  } 
}
void anzeige1()  //Funktion LCD1
{
  lcd1.setCursor(0,0);
  lcd1.print(rtc.getTimeStr());
  lcd1.setCursor(0,1);
  lcd1.print(rtc.getDOWStr(FORMAT_SHORT));
  lcd1.setCursor(6,1);
  lcd1.print(rtc.getDateStr());
  lcd1.setCursor(10,0);
  lcd1.print(rtc.getTemp());
  lcd1.setCursor(14,0);
  lcd1.print((char)223);
  lcd1.setCursor(15,0);
  lcd1.print("C");
}
void anzeige2() //Funktion LCD2
{
if(menuState!=idle){   
      MenuAusgabe(menuState,prevMenuState); //Ausgabefunktion des Menüs
       lcd2AnzeigeNummer=0;
       lcd2PrevAnzeigeNummer=0;
  }
  else if(timerState==true){
    
      lcd2AnzeigeNummer=1;
      if(lcd2AnzeigeNummer!=lcd2PrevAnzeigeNummer){
      lcd2.clear();
      lcd2PrevAnzeigeNummer=lcd2AnzeigeNummer;
      }
      lcd2.setCursor(0,0);
      lcd2.print(Timer(timerAktion));
      lcd2.setCursor(0,1);
      lcd2.write(2);
      lcd2.setCursor(1,1);
      lcd2.print(" OFF");
    }
    //}
    else{
       lcd2AnzeigeNummer=2;
      if(lcd2AnzeigeNummer!=lcd2PrevAnzeigeNummer){
      lcd2.clear();
      lcd2PrevAnzeigeNummer=lcd2AnzeigeNummer;
      }
      lcd2.setCursor(0,0);
      lcd2.print(AlarmOne(alarm1Trigger,alarm1State));
      lcd2.setCursor(0,1);
      lcd2.print(AlarmTwo(alarm2Trigger,alarm2State));
    }
  }
void ControlAll() //Kontrolliert abläufe
{
       stunden=rtc.getTime().hour;
       minuten=rtc.getTime().min;
       sekunden=rtc.getTime().sec;
       if(sekunden!=prevSek && timerState==true){
         timerAktion=true;
       }
       if(stunden==alarm1Stunden && minuten==alarm1Minuten && sekunden==0 && alarm1State==true&&entry==true){
        alarm1Trigger=true;
        musicAlarm1 =true;
        if(nightMode==true){
          nightMode=false;
        }
        kuckuckState=true;
        if(repeatAlarm1==false){
          alarm1State==false;
        }
        music=true;
        entry=false;
       }
       if(stunden==alarm2Stunden && minuten==alarm2Minuten && sekunden==0 && alarm2State==true&&entry==true){
        alarm2Trigger=true;
        musicAlarm2 =true;
        if(nightMode==true){
          nightMode=false;
        }
        kuckuckState=true;
        if(repeatAlarm2==false){
          alarm2State==false;
        }
        music=true;
        entry=false;
       }
       if(sekunden==0&&minuten==0&&alarm1Trigger==false&&alarm2Trigger==false&&entry==true&&nightMode==false){
        musicFullHour=true;
        kuckuckState=true;
        LEDminute=0;
        music=true;
        entry=false;
       }
       if((sekunden == 0&&alarm1Trigger==false && alarm2Trigger==false&& entry==true&&nightMode==false) && (minuten==15 || minuten==30 || minuten==45)){
        musicQuarterHour=true;
        music=true;
        entry=false;
       }
       if (sekunden!=prevSek){
        prevSek=sekunden;  
        entry = true;  
       }  
       if(minuten !=prevMin){
        LEDentry=true;
        prevMin=minuten;
       }

}
void kuckuck()    //Kuckuck kommt raus
{
if(kuckuckState==true){
  kuckuckMOVE();   
}
}
void kuckuckMOVE()  //Kuckuckbewegung
{
    if(kuckuckPOS>=0 && forward==true){
      kuckuckPOS++;
      myservo.write(kuckuckPOS);             
      if(kuckuckPOS==180){
        forward=false;
        kuckuckMillis=millis();             
      }
  }
    if(kuckuckPOS<=180 && forward==false&&millis()-kuckuckMillis>2000){
      kuckuckPOS--;
      myservo.write(kuckuckPOS);             
      if(kuckuckPOS==0){
        forward=true;
        kuckuckState=false; 
        myDFPlayer.pause(); 
        ziffernblatt(LEDminute);           
      }
  }
}
String intConverter(byte toConvert) //Zahlen unter 10 zu 0X konvertieren
{
  String convertedInt;
  if(toConvert<10){  
  convertedInt=String("0"+String(toConvert));
  }
  else{
    convertedInt=String(toConvert);
  }
  return convertedInt;
}
void brightnessControl(int sensorWert)//LCD Helligkeit über LDR Sensor //sensorWert=analogRead(LDReingang);
{
if(nightMode==true && menuState==0){
    brightness=10;
    analogWrite(pinLCDBrightness1,brightness);
    analogWrite(pinLCDBrightness2,brightness);
    
}
else{
    if(sensorWert<100){
    brightness=10;
    analogWrite(pinLCDBrightness1,brightness);
    analogWrite(pinLCDBrightness2,brightness);
    }
  if(sensorWert>=20&&sensorWert<40){
    brightness=30;
    analogWrite(pinLCDBrightness1,brightness);
    analogWrite(pinLCDBrightness2,brightness);
    }
    if(sensorWert>=40&&sensorWert<80){
    brightness=80;
    analogWrite(pinLCDBrightness1,brightness);
    analogWrite(pinLCDBrightness2,brightness);
    }
    if(sensorWert>=80&&sensorWert<300){
    brightness=150;
    analogWrite(pinLCDBrightness1,brightness);
    analogWrite(pinLCDBrightness2,brightness);
    }
   if(sensorWert>=300){
    brightness=230;
    analogWrite(pinLCDBrightness1,brightness);
    analogWrite(pinLCDBrightness2,brightness);
    }
}
}
void MenuAusgabe(int menuNummer, int prevMenuNummer)  //AusgabeFuntkion des Menüs aud LCD2
{
if(menuNummer!=prevMenuNummer){
  lcd2.clear();
  prevMenuState=menuNummer;
}
  switch(menuState){
    case mainMenu:
      lcd2.setCursor(0,0);
      lcd2.write(0);
      lcd2.setCursor(1,0);
      lcd2.print("Da./Time");
      lcd2.setCursor(9,0);
      lcd2.write(1);
      lcd2.setCursor(10,0);
      lcd2.print("Alarm");
      lcd2.setCursor(0,1);
      lcd2.write(2);
      lcd2.setCursor(1,1);
      lcd2.print("Timer  ");
      lcd2.setCursor(8,1);
      lcd2.write(3);
      lcd2.setCursor(9,1);
      lcd2.print("N.mode");
    break;
    case DateTime:
      lcd2.setCursor(0,0);
      lcd2.write(0);
      lcd2.setCursor(2,0);
      lcd2.print("DATE");
      lcd2.setCursor(0,1);
      lcd2.write(3);
      lcd2.setCursor(2,1);
      lcd2.print("TIME");
    break;
    case alarmMenu:
      lcd2.setCursor(0,0);
      lcd2.write(0);
      lcd2.setCursor(2,0);
      lcd2.print("ALARM 1");
      lcd2.setCursor(0,1);
      lcd2.write(3);
      lcd2.setCursor(2,1);
      lcd2.print("ALARM 2");
    break;
    case timerMenu:
      lcd2.setCursor(0,0);
      lcd2.print("TIMER: hh:mm:ss");
      lcd2.setCursor(0,1);
      lcd2.print(String(intConverter(einstellHour)+":"+intConverter(einstellMin)+":"+intConverter(einstellSec))); 
    break;
    case nightmodeMenu:
      lcd2.setCursor(0,0);
      lcd2.print("Nightmode");
      lcd2.setCursor(0,1);
      lcd2.write(1);
      lcd2.setCursor(2,1);
      lcd2.print("OFF");
      lcd2.setCursor(7,1);
      lcd2.write(2);
      lcd2.setCursor(9,1);
      lcd2.print("ON");

    break;
    case dateMenu:
      lcd2.setCursor(0,0);
      lcd2.print("Date DD:MM:YYYY;DOW ");
      lcd2.setCursor(0,1);
      lcd2.print(String(intConverter(einstellDate)+":"+intConverter(einstellMonth)+":"+String(einstellYear)+";"+String(einstellDOW)));
    break;
    case timeMenu:
      lcd2.setCursor(0,0);
      lcd2.print("TIME: hh:mm:ss");
      lcd2.setCursor(0,1);
      lcd2.print(String(intConverter(einstellHour)+":"+intConverter(einstellMin)+":"+intConverter(einstellSec)));
    break;
    case alarm1Menu:
      lcd2.setCursor(0,0);
      lcd2.print("A1: ");
      lcd2.setCursor(4,0);
      lcd2.write(0); //LEFT arrow
      lcd2.setCursor(5,0);
      lcd2.print("Time");
      lcd2.setCursor(10,0);
      lcd2.write(1);    //UP arrow
      lcd2.setCursor(11,0);
      lcd2.print(" ON");
      lcd2.setCursor(4,1);
      lcd2.write(3); //RIGHT arrow
      lcd2.setCursor(5,1);
      lcd2.print("TONE");
      lcd2.setCursor(10,1);
      lcd2.write(2);    //DOWN arrow
      lcd2.setCursor(11,1);
      lcd2.print("OFF");
    break;
    case alarm2Menu:
      lcd2.setCursor(0,0);
      lcd2.print("A2: ");
      lcd2.setCursor(4,0);
      lcd2.write(0); //LEFT arrow
      lcd2.setCursor(5,0);
      lcd2.print("Time");
      lcd2.setCursor(10,0);
      lcd2.write(1);    //UP arrow
      lcd2.setCursor(11,0);
      lcd2.print(" ON");
      lcd2.setCursor(4,1);
      lcd2.write(3); //RIGHT arrow
      lcd2.setCursor(5,1);
      lcd2.print("TONE");
      lcd2.setCursor(10,1);
      lcd2.write(2);    //DOWN arrow
      lcd2.setCursor(11,1);
      lcd2.print("OFF");
    break;
    case alarm1Time:
      lcd2.setCursor(0,0);
      lcd2.print("A1: hh:mm:REPEAT");
      lcd2.setCursor(0,1);
      lcd2.print(String(intConverter(einstellHour)+":"+intConverter(einstellMin)));
      lcd2.setCursor(7,1);
      lcd2.write(repeatZeichen(einstellBool));
    break;
    case alarm2Time:
      lcd2.setCursor(0,0);
      lcd2.print("A2: hh:mm:REPEAT");
      lcd2.setCursor(0,1);
      lcd2.print(String(intConverter(einstellHour)+":"+intConverter(einstellMin)));
      lcd2.setCursor(7,1);
      lcd2.write(repeatZeichen(einstellBool));
    break;
    case alarm1Tone:
      lcd2.setCursor(0,0);
      lcd2.print("A1: TONE");
      lcd2.setCursor(0,1);
      lcd2.print(String("Songnummer: "+intConverter(einstellTon)));   
    case alarm2Tone:
      lcd2.setCursor(0,0);
      lcd2.print("A2: TONE");
      lcd2.setCursor(0,1);
      lcd2.print(String("Songnummer: "+intConverter(einstellTon)));
    break;
  }
}
void menuSwitch(int h,byte j)  // menuState,buttonState Funktion des Menuwechselns
{
  switch(h){//menuState
    case idle:                     //kein menu
          switch(j){//buttonState
            case Ok:             //OK
                  menuState=mainMenu;
            break;
            case Return:             //RETURN
                  menuState=idle;
                  
            break;
            case Left:             //LEFT
                  menuState=idle;
            break;
            case Up:             //UP
                  menuState=idle;
            break;
            case Down:             //DOWN
                  menuState=idle;
                  if(timerState==true){
                    timerState=false;
                    timerMinuten=0;
                    timerSekunden=0;
                    timerStunden=0;
                  }
            break;
            case Right:             //RIGHT
                  menuState=idle;
            break;
          }
    break;     
    case mainMenu:                 //Hauptmenü
          switch(j){//buttonState
            case Ok:              //OK
                  menuState=mainMenu;
            break;
            case Return:             //return        
                  menuState=idle; //kein menu
                  
            break;
            case Left:            //LEFT   
                  menuState=DateTime;  //DatumUhrzeit
            break;
            case Up:             //UP
                  menuState=alarmMenu;  //Wecker einstellen
            break;  
            case Down:            //DOWN            
                  menuState=timerMenu;  //Timer Einstellen
            break;
            case Right:           //RIGHT
                  menuState=nightmodeMenu;  //Nachtmodus Einstellen
            break;
          }
    break;       
    case DateTime:                 //Datum uhrzeit
          switch(j){//buttonState
            case Ok:               //OK
                  menuState=DateTime;
            break;
            case Return:               //RETURN
                  menuState=mainMenu;
            break;
            case Left:               //LEFT
                  menuState=dateMenu;   //Datum Einstellen
                  einstellDate=rtc.getTime().date;
                  einstellMonth=rtc.getTime().mon;
                  einstellYear=rtc.getTime().year;
                  einstellDOW=rtc.getTime().dow;
                  
            break;
            case Up:               //UP
                  menuState=DateTime;
            break;
            case Down:               //DOWN
                  menuState=DateTime;
            break;
            case Right:               //RIGHT
                  menuState=timeMenu;   //UhrzeitEinstellen
                  einstellHour=rtc.getTime().hour;
                  einstellMin=rtc.getTime().min;
                  einstellSec=rtc.getTime().sec;
            break;
          }
     break;
     case alarmMenu:                 //Wecker einstellen
        switch(j){//buttonState
           case Ok:               //OK
                  menuState=alarmMenu;
           break;
           case Return:               //RETURN
                  menuState=mainMenu;
           break;
           case Left:               //LEFT
                  menuState=alarm1Menu;   //Wecker 1
           break;
           case Up:               //UP
                  menuState=alarmMenu;
           break;
           case Down:               //DOWN
                  menuState=alarmMenu;
           break;
           case Right:               //RIGHT
                  menuState=alarm2Menu;   //Wecker 2
           break;
          }
     break;
     case timerMenu:                 //Timer Einstellen
        switch(j){//buttonState
           case Ok:               //OK WERTE ÜBERNEHMEN
                  menuState=idle;
                  timerStunden=einstellHour;
                  timerMinuten=einstellMin;
                  timerSekunden=einstellSec;
                  einstellHour=0;
                  einstellMin=0;
                  einstellSec=0;
                  timerState=true;
                  cursorPos=1;
                  
           break;
           case Return:               //RETURN WERTE VERWERFEN
                  menuState=mainMenu;
                  einstellHour=0;
                  einstellMin=0;
                  einstellSec=0;
                  cursorPos=1;

           break;
           case Left:               //LEFT VORHERIGE ZAHL
                  menuState=timerMenu;
                  cursorPos=changeNumbersToCount(LEFT, cursorPos,maxNumbersTime);    
           break;
           case Up:               //UP ZAHL ERHÖHEN
                  menuState=timerMenu;
                  menuNavigation(cursorPos, menuState,UP);
           break;
           case Down:               //DOWN ZAHL VERRINGERN
                  menuState=timerMenu;
                  menuNavigation(cursorPos, menuState,DOWN);
           break;
           case Right:               //RIGHT  Nächste ZAHL
                  menuState=timerMenu;
                  cursorPos=changeNumbersToCount(RIGHT, cursorPos,maxNumbersTime); //
           break;
          }
     break;
     case nightmodeMenu:                 //NACHTMODUS
        switch(j){//buttonState
           case Ok:               //OK
                  menuState=nightmodeMenu;
           break;
           case Return:               //RETURN WERTE VERWERFEN
                  menuState=mainMenu;
           break;
           case Left:               //LEFT 
                  menuState=nightmodeMenu;   
           break;
           case Up:               //UP AUS
                  menuState=idle;
                  nightMode=false;
                  ziffernblattSetup(minuten);

           break;
           case Down:               //DOWN EIN
                  menuState=idle;
                  nightMode=true;
                  
                  pf575_write(word(B00001111,B11111111)); //alle leds aus

           break;
           case Right:               //RIGHT  
                  menuState=nightmodeMenu;   
           break;
          }
     break;
          case dateMenu:                 //Datum Einstellen
        switch(j){//buttonState
           case Ok:               //OK 
                  menuState=idle;  //DATUM Übernehmen
                  rtc.setDate(einstellDate,einstellMonth,einstellYear);
                  rtc.setDOW(einstellDOW);
                  einstellDate=0;
                  einstellMonth=0;
                  einstellYear=0;
                  einstellDOW=0;
                  cursorPos=1;
           break;
           case Return:               //RETURN WERTE VERWERFEN
                  menuState=DateTime;
                  einstellDate=0;
                  einstellMonth=0;
                  einstellYear=0;
                  einstellDOW=0;
                  cursorPos=1;

           break;
           case Left:               //LEFT  VORHERIGE ZAHL
                  menuState=dateMenu;
                  cursorPos=changeNumbersToCount(LEFT, cursorPos,maxNumbersDate);   
           break;
           case Up:               //UP ZAHL ERHÖHEN
                  menuState=dateMenu;
                  menuNavigation(cursorPos, menuState,UP);
                 
           break;
           case Down:               //DOWN ZAHL Verringern
                  menuState=dateMenu;
                  menuNavigation(cursorPos, menuState,DOWN);
           break;
           case Right:               //RIGHT  NÄCHSTE ZAHL 
                  menuState=dateMenu;
                  cursorPos=changeNumbersToCount(RIGHT, cursorPos,maxNumbersDate);   
           break;
          }
     break;
     case timeMenu:                 //UHRZEIT Einstellen
        switch(j){//buttonState
           case Ok:               //OK URHEZIT Übernehmen
                  menuState=idle;
                  rtc.setTime(einstellHour,einstellMin,einstellSec);
                  ziffernblattSetup(einstellMin);
                  einstellHour=0;
                  einstellMin=0;
                  einstellSec=0;
                  cursorPos=1;
                    
           break;
           case Return:               //RETURN WERTE VERWERFEN
                  menuState=DateTime;
                  einstellHour=0;
                  einstellMin=0;
                  einstellSec=0;
                  cursorPos=1;

           break;
           case Left:               //LEFT  VORHERIGE ZAHL
                  menuState=timeMenu;
                  cursorPos=changeNumbersToCount(LEFT, cursorPos,maxNumbersTime); 
                     
           break;
           case Up:               //UP ZAHL ERHÖHEN
                  menuState=timeMenu;
                  menuNavigation(cursorPos, menuState,UP); 
           break;
           case Down:               //DOWN ZAHL Verringern
                  menuState=timeMenu;
                  menuNavigation(cursorPos, menuState,DOWN); 
           break;
           case Right:               //RIGHT  NÄCHSTE ZAHL 
                  menuState=timeMenu; 
                  cursorPos=changeNumbersToCount(RIGHT, cursorPos,maxNumbersTime);  
           break;
          }
     break;
     case alarm1Menu:                 //Wecker1 Einstellen
        switch(j){              //buttonState
           case Ok:               //OK 
                  menuState=alarm1Menu;
                    
           break;
           case Return:               //RETURN 
                  menuState=alarmMenu;
           break;
           case Left:               //LEFT  WECKZEIT1
                  menuState=alarm1Time; 
                  einstellHour=alarm1Stunden;
                  einstellMin=alarm1Minuten;  
           break;
           case Up:               //UP WECKZEIT1 EIN
                  menuState=idle;
                  alarm1State=true;
           break;
           case Down:               //DOWN WECKEZIT1 AUS
                  menuState=idle;
                  alarm1State=false;
           break;
           case Right:               //RIGHT  TON1
                  menuState=alarm1Tone; 
                  einstellTon=alarm1Song;  
           break;
          }
     break;
     case alarm2Menu:                 //Wecker2 Einstellen
        switch(j){            //buttonState
           case Ok:               //OK 
                  menuState=alarm2Menu;
                   
           break;
           case Return:               //RETURN 
                  menuState=alarmMenu;
                  
           break;
           case Left:               //LEFT  WECKZEIT2
                  menuState=alarm2Time;
                  einstellHour=alarm2Stunden;
                  einstellMin=alarm2Minuten;   
           break;
           case Up:               //UP WECKZEIT2 EIN
                  menuState=idle;
                  alarm2State=true;
           break;
           case Down:               //DOWN WECKEZIT2 AUS
                  menuState=idle;
                  alarm2State=false;
           break;
           case Right:               //RIGHT  TON2
                  menuState=alarm2Tone;
                  einstellTon=alarm2Song;   
           break;
          }
     break;
      case alarm1Time:                 //WECKZEIT 1
        switch(j){              //buttonState
           case Ok:               //OK WERTE ÜBERNEHMEN, WECKZEIT1==EIN
                  menuState=idle;
                  alarm1Stunden=einstellHour;
                  alarm1Minuten=einstellMin;
                  repeatAlarm1=einstellBool;
                  EEPROM.update(alarm1StundenAddress, einstellHour);
                  EEPROM.update(alarm1MinutenAddress, einstellMin);
                  alarm1State=true;
                  einstellHour=0;
                  einstellMin=0;
                  cursorPos=1;
                   
           break;
           case Return:               //RETURN 
                  menuState=alarm1Menu;
                  einstellHour=0;
                  einstellMin=0;
                  einstellSec=0;
                  einstellBool=true;
                  cursorPos=1;
                  
           break;
           case Left:               //LEFT  VORHERIGE ZAHL
                  menuState=alarm1Time;
                  cursorPos=changeNumbersToCount(LEFT, cursorPos,maxNumbersAlarm);    
           break;
           case Up:               //UP ZAHL ERHÖHEN
                  menuState=alarm1Time;
                  menuNavigation(cursorPos, menuState,UP);
           break;
           case Down:               //DOWN ZAHL VERRINGERN
                  menuState=alarm1Time;
                  menuNavigation(cursorPos, menuState,DOWN);
           break;
           case Right:               //RIGHT  NÄCHSTE ZAHL
                  menuState=alarm1Time;
                  cursorPos=changeNumbersToCount(RIGHT, cursorPos,maxNumbersAlarm);    
           break;
          }
     break;
     case alarm2Time:                 //WECKZEIT 2
        switch(j){              //buttonState
           case Ok:               //OK WERTE ÜBERNEHMEN, WECKZEIT2==EIN
                  menuState=idle;
                  alarm2Stunden=einstellHour;
                  alarm2Minuten=einstellMin;
                  repeatAlarm2=einstellBool;
                  EEPROM.update(alarm2StundenAddress, einstellHour);
                  EEPROM.update(alarm2MinutenAddress, einstellMin);
                  alarm2State=true;
                  einstellHour=0;
                  einstellMin=0;
                  einstellSec=0;
                  cursorPos=1;
                    
           break;
           case Return:               //RETURN 
                  menuState=alarm2Menu;
                  einstellHour=0;
                  einstellMin=0;
                  einstellSec=0;
                  einstellBool=true;
                  cursorPos=1;
                  
           break;
           case Left:               //LEFT  VORHERIGE ZAHL
                  menuState=alarm2Time;
                  cursorPos=changeNumbersToCount(LEFT, cursorPos,maxNumbersAlarm);   
           break;
           case Up:               //UP ZAHL ERHÖHEN
                  menuState=alarm2Time;
                  menuNavigation(cursorPos, menuState,UP);
           break;
           case Down:               //DOWN ZAHL VERRINGERN
                  menuState=alarm2Time;
                  menuNavigation(cursorPos, menuState,DOWN);
           break;
           case Right:               //RIGHT  NÄCHSTE ZAHL
                  menuState=alarm2Time;
                  cursorPos=changeNumbersToCount(RIGHT, cursorPos,maxNumbersAlarm);   
           break;
          }
     break;
     case alarm1Tone:                 //TON1
        switch(j){              //buttonState
           case Ok:               //OK WERTE ÜBERNEHMEN
                  menuState=idle;
                  alarm1Song=einstellTon;
                  EEPROM.update(alarm1SongAddress, einstellTon);
                  cursorPos=1; 
           break;
           case Return:               //RETURN 
                  menuState=alarm1Menu;
                  einstellTon=0;
                  cursorPos=1;
           break;
           case Left:               //LEFT  
                  menuState=alarm1Tone;   
           break;
           case Up:               //UP ZAHL ERHÖHEN
                  menuState=alarm1Tone;
                  menuNavigation(cursorPos, menuState,UP);
           break;
           case Down:               //DOWN ZAHL VERRINGERN
                  menuState=alarm1Tone;
                  menuNavigation(cursorPos, menuState,DOWN);
           break;
           case Right:               //RIGHT  
                  menuState=alarm1Tone;   
           break;
          }
     break;
     case alarm2Tone:                 //TON2
        switch(j){              //buttonState
           case Ok:               //OK WERTE ÜBERNEHMEN
                  menuState=idle;
                  alarm2Song=einstellTon;
                  EEPROM.update(alarm2SongAddress, einstellTon);
                  cursorPos=1;  
           break;
           case Return:               //RETURN 
                  menuState=alarm2Menu;
                  einstellTon=0;
                  cursorPos=1;
           break;
           case Left:               //LEFT  
                  menuState=alarm2Tone;   
           break;
           case Up:               //UP ZAHL ERHÖHEN
                  menuState=alarm2Tone;
                  menuNavigation(cursorPos, menuState,UP);

           break;
           case Down:               //DOWN ZAHL VERRINGERN
                  menuState = alarm2Tone;
                  menuNavigation(cursorPos, menuState,DOWN);

           break;
           case Right:               //RIGHT  
                 menuState = alarm2Tone;   
           break;
          }
     break;                                    
  }
  
  buttonState= notPressed;
}
byte counterForNumbers(bool y, byte l) //hoch==true runter==false  , zahl die erhöht/ verringert werden soll
{
  byte changedNumber;    
        if(y==true){    //wenn UP Zahl Erhöhen
          if(l==59){
            l=0;
          }
          else{
          l++;
          }
        }
       else{
              if(l==0){
                l=59;
              }
              else{
                l--;
              }
       }
       changedNumber=l;
       return changedNumber;
}
byte changeNumbersToCount(bool x, byte n,byte maxNumbers) //true ==right, left==false, ausgangsposition, maxWerte die einzustellen sind 
{
  byte newCursorPos;

          if(n==maxNumbers&&x==true){
            n=1;
          }
          else if(n==1&&x==false){
            n=maxNumbers;
          }
          else{
            if(x==true){
              n++;
            }
            else{
              n--;
            }
          }
       newCursorPos=n;   
       return newCursorPos;
  
}
void menuNavigation(byte cursorPosition, int whatToChange,bool state) //cursorPOS, MenuState, state==Hoch,Runter 
{
  
if(whatToChange==alarm2Time){  //Da zeit für alarme gleich ist
  whatToChange=alarm1Time;
}
if(whatToChange==timeMenu){   //Da zeit für timer und uhr gliech sind
  whatToChange=timerMenu;
}
if(whatToChange==alarm2Tone){    //Da Ton für alarme gleich ist
  whatToChange=alarm1Tone;
}
  switch(whatToChange){
    case dateMenu:    //Datum
         switch(cursorPosition){
          case 1:
                einstellDate=counterForDays(einstellDate,maxDaysOfMonth(einstellMonth,einstellYear),state);
          break;
          case 2:
                einstellMonth=counterForMonths(einstellMonth,state);
          break;
          case 3:
                einstellYear=counterForYears(einstellYear,state);
          break;
          case 4:
              einstellDOW=counterForDOW(einstellDOW, state);
          break;
         }
    break;
    case timerMenu:
        switch(cursorPosition){
                case 1:
                    einstellHour=counterForNumbers(state,einstellHour);
                    if(einstellHour>23){
                      einstellHour=0;
                      }
                break;
                case 2:      
                    einstellMin=counterForNumbers(state,einstellMin);

                break;      
                case 3:
                 einstellSec=counterForNumbers(state,einstellSec);

                break; 
            }
    break;     
    case alarm1Time:
          switch(cursorPosition){
                 case 1:
                    einstellHour=counterForNumbers(state,einstellHour);
                    if(einstellHour>23){
                    einstellHour=0;
                    }
                 break;
                 case 2:      
                    einstellMin=counterForNumbers(state,einstellMin);

                 break;
                 case 3:
                    einstellBool= !einstellBool;
                  break;             
                }
     break;      
    case alarm1Tone:  
               einstellTon=counterForSongs(einstellTon,state);
    break;      
               
  
}
}
void tastendruckRead(int value)  //funktion für tastenbedienung
{

  //Serial.println(value);
  if(millis()-debounceTime > 200){

  debounceTime=millis();
  if(value>740&&value<775){
    buttonState=Ok;
    //Serial.println("1");
  }
    if(value>320&&value<370){
    buttonState=Return;
    //Serial.println("2");
  }
    if(value>185&&value<240){
    buttonState=Right;
    //Serial.println("6");
  }
    if(value>130&&value<165){
    buttonState=Down;
    //Serial.println("5");
  }
    if(value>95&&value<120){
    buttonState=Up;
    //Serial.println("4");
  }
    if(value>45&&value<85){
    buttonState=Left;
    //Serial.println("3");
  }

  }
 

}
byte repeatZeichen(bool zeichenONOFF)  //ausgabe des Repeat/NoRepeatzeichens
{
  byte zeichen;
         if(zeichenONOFF==true){
          zeichen=4;     //Repeatzeichen
         }
         else{
          zeichen=5;      //NORepeatzeichen
         }
  return zeichen;
}
byte counterForDays(byte tag, byte maxDays, bool nextPrevDay)   //einstellDate, maxDays=maxDaysOfMonth(einstellMon,einstellYear), state
{
  byte changedDay;
  if(nextPrevDay==true){
    if(tag==maxDays){
      tag=1;
    }
    else{
      tag++;
    }
  }
  if(nextPrevDay==false){
    if(tag==1){
      tag=maxDays;
    }
    else{
      tag--;
    }
  }
  changedDay=tag;
  return changedDay;
}
byte counterForMonths(byte monat, bool nextPrevMonth)   //einstellMon, state
{
  byte changedMonth;
   if(nextPrevMonth==true){
    if(monat==12){
      monat=1;
    }
    else{
      monat++;
    }
  }
  if(nextPrevMonth==false){
    if(monat==1){
      monat=12;
    }
    else{
      monat--;
    }
  }
  changedMonth=monat;
  return changedMonth;
}
int counterForYears(int jahr,bool nextPrevYear)     //einstellYear, state
{
  int changedYear;
   if(nextPrevYear==true){
      jahr++;   
  }
  if(nextPrevYear==false){
    if(jahr==2001){
      jahr=2001;
    }
    else{
      jahr--;
    }
  }
  changedYear=jahr;
  return changedYear;
}
byte maxDaysOfMonth(byte thisMonth, int thisYear)//einstellMon,einstellYear
{  
  byte DaysOfMonth;
  switch (thisMonth){
    case 1:
          DaysOfMonth=31;
    break;
    case 2:
          if(schaltjahr(thisYear)==true){
            DaysOfMonth=29;
          }
          else{
            DaysOfMonth=28;
          }
    break;
    case 3:
          DaysOfMonth=31;
    break;
    case 4:
          DaysOfMonth=30;
    break;
    case 5:
          DaysOfMonth=31;
    break;
    case 6:
          DaysOfMonth=30;
    break;
    case 7:
          DaysOfMonth=31;
    break;
    case 8:
          DaysOfMonth=31;
    break;
    case 9:
          DaysOfMonth=30;
    break;
    case 10:
          DaysOfMonth=31;
    break;
    case 11:
          DaysOfMonth=30;
    break;
    case 12:
           DaysOfMonth=31;
    break;
        
  }
  return DaysOfMonth;
}
bool  schaltjahr(int yearToCheck)  //Gucken Ob Schaltjahr
{
    bool offset; 

  if(yearToCheck%4==0){
    offset=true;
  }
  else{
    offset=false;
  }
    return offset;
}
byte counterForDOW(byte wochenTag, bool nextPrevDOW)
{
    byte changedDOW;  
    if(nextPrevDOW==true){
        if(wochenTag==7){
            wochenTag=1;
        }
    else{
      wochenTag++;
    }
  }
  if(nextPrevDOW==false){
    if(wochenTag==1){
      wochenTag=7;
    }
    else{
      wochenTag--;
    }
  }
  changedDOW=wochenTag;
  return changedDOW;
  
}
byte counterForSongs(byte songToChoose,bool nextPrevSong)
{
  byte changedSong;
  if(nextPrevSong==true){
        if(songToChoose==myDFPlayer.readFileCountsInFolder(16)){
            songToChoose=1;
            }
        
    else{
      songToChoose++;
    }
  }
    if(nextPrevSong==false){
        if(songToChoose==1){
            songToChoose=myDFPlayer.readFileCountsInFolder(16);       
        }
    else{
      songToChoose--;
    }
    }
  changedSong=songToChoose;
  return changedSong;
}
void IRRemote()  //IR Remote funktion
{
  if(irrecv.decode(&results)){
  //Serial.println(results.value);

    switch (results.value){

        case 16724175:   //Ir übertragender Wert beim Druck auf die Taste 1
        //Serial.println("1=OK");
        buttonState=Ok;
        break;
        
        case 16718055:
        //Serial.println("2=UP");
        buttonState=Up;
        break;
        
        case 16743045:
        //Serial.println("3=return");
        buttonState=Return;
        break;
        
        case 16716015:
        //Serial.println("4=left");
        buttonState=Left;
        break;
        
        
        case 16734885:
        //Serial.println("6=right");
        buttonState=Right;
        break;
        
        case 16730805:
        //Serial.println("8=down");
        buttonState=Down;
        break;
        

    }
    irrecv.resume();
  }
}
void SerialRead(){ //einleseFunktion node Red
  while (Serial.available() > 0) {
      
    // look for the next valid integer in the incoming serial stream:
     firstNumber  = Serial.parseInt();
     secondNumber = Serial.parseInt();
     thirdNumber = Serial.parseInt();
     SerialBreak();
     fourthNumber  = Serial.parseInt();
     SerialBreak();
     fifthNumber = Serial.parseInt();
     sixthNumber  = Serial.parseInt();
     seventhNumber  = Serial.parseInt();
     SerialBreak();
  }
}   
void SerialBreak(){ //abschluss der Nodered eingaben
    if (Serial.read() == '\n') {
      nodeRed();
    }
}
void nodeRed(){   //bearbeitung nodered eingaben
      if(firstNumber==1){
      rtc.setTime(fifthNumber,sixthNumber,seventhNumber);
      rtc.setDate(secondNumber,thirdNumber,fourthNumber);
      stunden=rtc.getTime().hour;
      stundenzeigerSetup();
       stepper.step(HtoStep(stunden));
      prevStunden=stunden;
      ziffernblattSetup(sixthNumber);
     firstNumber  = 0;
     secondNumber = 0;
     thirdNumber = 0;
     fourthNumber  = 0;
     fifthNumber = 0;
     sixthNumber  = 0;
     seventhNumber  = 0;
      }
      if(firstNumber==2){
      alarm1Stunden=secondNumber;
      alarm1Minuten=thirdNumber;
      alarm1State=true;
     firstNumber  = 0;
     secondNumber = 0;
     thirdNumber = 0;
     fourthNumber  = 0;
     fifthNumber = 0;
     sixthNumber  = 0;
     seventhNumber  = 0;
    }
     if(firstNumber==3){
      alarm2Stunden=secondNumber;
      alarm2Minuten=thirdNumber;
      alarm2State=true;
     firstNumber  = 0;
     secondNumber = 0;
     thirdNumber = 0;
     fourthNumber  = 0;
     fifthNumber = 0;
     sixthNumber  = 0;
     
     seventhNumber  = 0;
    }
     if(firstNumber==4){
      timerStunden=secondNumber;
      timerMinuten=thirdNumber;
      timerSekunden=fourthNumber;
      timerState=true;
     firstNumber  = 0;
     secondNumber = 0;
     thirdNumber = 0;
     fourthNumber  = 0;
     fifthNumber = 0;
     sixthNumber  = 0;
     seventhNumber  = 0;
    }
  }
void SerialPrint()   //ausgabefunktion nodered
{
  Serial.print(rtc.getTimeStr());
  Serial.print(",");
  Serial.print(AlarmOne(alarm1Trigger,alarm1State));
  Serial.print(",");
  Serial.print(AlarmTwo(alarm2Trigger,alarm2State));
  Serial.print(",");
  Serial.print(rtc.getTemp());
  Serial.print(",");
  Serial.println(Timer(timerAktion));
  
}
