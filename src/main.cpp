// 6 Channel Transmitter | 6 Kanal Verici
// KendinYap Channel

#include "main.h"
#include <SPI.h>
#include <EEPROM.h>
//#include <Adafruit_LiquidCrystal.h>
#include <U8g2lib.h>
//#include <U8x8lib.h>
//#include <Wire.h>
#include "display.h"
//#include "expo.h"
#include "expo8.h"
#include <nRF24L01.h>
#include <RF24.h>
#include <Bounce2.h> // github.com/thomasfredericks/Bounce2

#include <elapsedMillis.h>
#include "defines.h"

const uint64_t pipeOut = 0xABCDABCD71LL;         // NOTE: The address in the Transmitter and Receiver code must be the same "0xABCDABCD71LL" | Verici ve Alıcı kodundaki adres aynı olmalıdır

extern "C" 

//U8G2_SSD1327_WS_128X128_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE);

// github.com/olikraus/u8g2/discussions/1865
//U8X8_SSD1327_WS_128X128_HW_I2C u8x8(U8X8_PIN_NONE);

// 0.96"
// >> code in display.h

//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
uint16_t loopcounter0 = 0;
uint16_t loopcounter1 = 0;
//uint8_t charh = 0;
//uint8_t balkenh = 50;
//uint8_t balkenb = 5;
//U8X8_SSD1327_WS_128X128_HW_I2C u8g2(A4,A5);
#define TEST 0

#define CE_PIN 9
#define CSN_PIN 10

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

#define LOOPLED 4

#define BUZZPIN 6

#define EEPROMTASTE  5

#define EEPROM_WRITE 0
#define EEPROM_READ  1

#define EEPROMINDEX_U 0x10
#define EEPROMINDEX_O 0x20
#define EEPROMINDEX_M 0x30

#define EEPROMLEVELSETTINGS  0x40
#define EEPROMEXPOSETTINGS  0x48


#define BLINKRATE 0x00fF

// defines for PINS
// links
#define PITCH_PIN     A6
#define YAW_PIN       A3

// rechts
#define ROLL_PIN      A1
#define THROTTLE_PIN  A0  

#define TASTATUR_PIN A7
#define TASTE_OFF  0
#define TASTE_ON  1


#define BATT_PIN         A2



uint16_t loopcounter = 0;
uint8_t blinkcounter = 0;
uint8_t impulscounter = 0;
uint16_t throttlecounter = 0;
uint16_t throttlesekunden = 0;

// RC_22
//#define POT0LO 620  // Min wert vom ADC Pot 0
//#define POT0HI 3400 // Max wert vom ADC Pot 0

#define POTLO   0
#define POTHI  710

//Impulslaenge, ms
#define PPMLO  850  // Minwert ms fuer Impulslaenge
#define PPMHI  2150 // Maxwert ms fur Impulslaenge

#define MINDIFF 4



uint16_t schritt = 32;



uint16_t                   impulstimearray[NUM_SERVOS] = {};
const int                  adcpinarray[NUM_SERVOS] = {A3,A6,A1,A0};    // pins der Pots

uint8_t                    kanalsettingarray[ANZAHLMODELLE][NUM_SERVOS][KANALSETTINGBREITE] = {};

uint16_t                   servomittearray[NUM_SERVOS] = {}; // Werte fuer Mitte

uint8_t levelwert= 0;
//uint8_t levelwerta = 0;
//uint8_t levelwertb = 0;

uint8_t levelwertarray[NUM_SERVOS] = {}; // leelwert pro servo


uint8_t levelwertayaw = 0;
uint8_t levelwertbyaw = 0;

uint16_t potwertyaw = 0;


uint8_t expowert = 0;
//uint8_t expowerta = 0;
//uint8_t expowertb = 0;

uint8_t expowertarray[NUM_SERVOS] = {}; // expowert pro Servo


uint16_t          potwertarray[NUM_SERVOS] = {}; // Werte fuer Mitte
//uint16_t          externpotwertarray[NUM_SERVOS] = {}; // Werte von extern  pro servo

//uint16_t currentexpoarray[5][513] = {};

uint8_t                                  curr_pfeil = 0;

uint16_t      blink_cursorpos=0xFFFF;
uint8_t blinkstatus = 0;

uint16_t stopsekunde=0;
uint16_t stopminute=0;
uint16_t motorsekunde=0;
uint16_t motorminute=0;
uint8_t motorstunde=0;

uint16_t sendesekunde=0;
uint16_t sendeminute=0;
uint8_t sendestunde=0;

uint8_t curr_steuerstatus = 0;

uint8_t calibstatus = 0;

uint8_t savestatus = 0;


float potlo = POTLO; // min pot
float pothi = POTHI; // max pot
float ppmlo = PPMLO; // min ppm
float ppmhi = PPMHI; // max ppm

uint16_t diffa = 0;

uint16_t potwertpitch = 0;
uint16_t diffapitch = 0;
uint16_t diffbpitch = 0;

uint16_t diffb = 0;
float expofloat = 0;
uint16_t expoint = 0;
uint16_t levelint = 0;

uint16_t levelintraw = 0;

uint16_t levelintcheck = 0;


uint16_t levelintpitcha = 0;

uint16_t levelintpitchb = 0;

uint16_t batteriespannung = 0;
uint16_t batteriearray[8] = {};
uint16_t batteriemittel = 0;
uint8_t batteriemittelwertcounter = 0;
uint16_t batterieanzeige = 0;
float UBatt = 0;
uint8_t eepromstatus = 0;
uint16_t eepromprelltimer = 0;
Bounce2::Button eepromtaste = Bounce2::Button();
uint16_t intdiff = 0;
uint16_t intdiffpitch = 0;

uint16_t tastaturwert = 0;
uint8_t tastencounter = 0;
uint8_t tastaturstatus = 0;
uint8_t Taste = 0;
uint8_t taste5counter = 0;

// balken
#define VBX   64
#define VBY    12
#define HBX    6
#define HBY    54

uint8_t taskarray[4] = {'Y', 'P', 'R', 'T'};

uint16_t potgrenzearray[NUM_SERVOS][2]; // obere und untere Grenze von adc

float quot = (ppmhi - ppmlo)/(pothi - potlo);

float expoquot = (ppmhi - ppmlo)/2/0x200; // umrechnen der max expo (512) auf PPM  

// float quotarray[NUM_SERVOS] = {}; // Umrechnungsfaktor pro Pot



// OLED > in display.cpp
uint16_t pot0 = 0;

uint16_t potwert = 0;

uint16_t errcounter = 0;
uint16_t radiocounter = 0;

// uint16_t                posregister[8][8]={}; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem

uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor

unsigned char char_x = 0;
unsigned char char_y = 0;

// Menu
uint8_t                 curr_model=0; // aktuelles modell
uint8_t                 speichermodel=0;
uint8_t                 curr_funktion=0; // aktuelle funktion
uint8_t                 curr_aktion=0; // aktuelle aktion

uint8_t                  curr_wert = 0;

uint8_t                 curr_impuls=0; // aktueller impuls

uint8_t                 curr_modus=0; // Modell oder Sim oder Calib


uint8_t                 curr_setting=0; // aktuelles Setting fuer Modell
uint8_t                          speichersetting=0;

uint8_t                 curr_trimmkanal=0; // aktueller  Kanal fuerTrimmung
uint8_t                 curr_trimmung=0; // aktuelle  Trimmung fuer Trimmkanal


uint8_t                 curr_screen = 0; // aktueller screen
uint8_t                 last_screen=0; // letzter screen

uint8_t                 curr_page=7; // aktuelle page
uint8_t                 curr_col=0; // aktuelle colonne

uint8_t                 curr_cursorzeile=0; // aktuelle zeile des cursors
uint8_t                 curr_cursorspalte=0; // aktuelle colonne des cursors
uint8_t                 last_cursorzeile=0; // letzte zeile des cursors
uint8_t                 last_cursorspalte=0; // letzte colonne des cursors

// Tastatur
uint8_t                 Tastenindex=0;
uint16_t                Tastenwert=0;
uint8_t                 adcswitch=0;
uint16_t                lastTastenwert=0;
int16_t                 Tastenwertdiff=0;
uint16_t                tastaturcounter=0;
uint16_t                tastaturdelaycounter=0;

elapsedMillis   zeitintervall;
uint8_t           sekundencounter = 0;
elapsedMillis   sinceLastBlink = 0;


Signal data;
void ResetData() 
{
   data.throttle = 0;                  
   data.pitch = 127;
   data.roll = 127;
   data.yaw = 127;
   data.aux1 = 0;                       
   data.aux2 = 0;
   
}

// PPM decode
const byte PPM_PIN = 2; // PPM-Eingang an Pin 2
volatile unsigned long lastTime = 0;
volatile unsigned long pulseLength = 0;
volatile byte channel = 0;
const byte maxChannels = 8;
volatile uint16_t ppmValues[maxChannels];
void ppmISR() 
{
  unsigned long now = micros();
  pulseLength = now - lastTime;
  lastTime = now;

  if (pulseLength > 3000) {
    // Sync-Pause erkannt: neues Frame beginnt
    channel = 0;
  } else if (channel < maxChannels) {
    ppmValues[channel] = pulseLength;
    channel++;
  }
}


void updatemitte(void)
{
   
   
}// updatemitte

void printgrenzen()
{
   Serial.print("\nprintgrenzen\n");  
   for (uint8_t i = 0;i<NUM_SERVOS;i++)
   {
      Serial.print("grenzen i:\t");
      Serial.print(i);
      Serial.print("\t");
      Serial.write(taskarray[i]);
      Serial.print("\t");
      Serial.print("potgrenze HI:\t");
      Serial.print(potgrenzearray[i][0]);
      Serial.print("\t");
      Serial.print("potgrenze LO:\t");
      Serial.print(potgrenzearray[i][1]);
      Serial.print("\t");
      Serial.print("servomitte:\t");
      Serial.print(servomittearray[i]);
      Serial.print("\n");
   }
   Serial.print("end printgrenzen\n");  
}

void printeeprom(uint8_t zeilen)
{
   Serial.print("printeeprom\n");
   for (uint8_t i=0;i<zeilen;i++)
   {
      //Serial.write(taskarray[i]);
      //Serial.print("\t");
      uint8_t f = EEPROM.read(i);
      
      if ((i+1)%8==0 )
      {
         //Serial.print(i);
         //Serial.print(": ");
         
         Serial.print(f);
         Serial.print("\n");
      }
      else
      {
         //Serial.print(i);
         //Serial.print(": ");
         
         Serial.print(f);
         Serial.print("\t");
      }
      
   }
   Serial.print("\n");
   uint8_t eepromyawlo = EEPROM.read(2*(0 + EEPROMINDEX_U));
   uint8_t eepromyawhi = EEPROM.read(2*(0 + EEPROMINDEX_U)+1);
   uint16_t eepromyaw = (eepromyawhi << 8) | eepromyawlo;
   
   Serial.print("eeprompitch U: \t");
   Serial.print(eepromyawlo);
   Serial.print("\t");
   Serial.print(eepromyawhi);
   Serial.print("\t");
   Serial.print(eepromyaw);
   Serial.print("\n");
   
   eepromyawlo = EEPROM.read(2*(0 + EEPROMINDEX_O));
   eepromyawhi = EEPROM.read(2*(0 + EEPROMINDEX_O)+1);
   eepromyaw = (eepromyawhi << 8) | eepromyawlo;
   
   Serial.print("eepromyaw O: \t");
   Serial.print(eepromyawlo);
   Serial.print("\t");
   Serial.print(eepromyawhi);
   Serial.print("\t");
   Serial.print(eepromyaw);
   Serial.print("\n");
}

void eepromread()
{
   Serial.print("eepromread \t");
   //Serial.print("kontrolle Adresse A: ");
   //Serial.print(EEPROM.read(0));
   //Serial.print(" Adresse B: ");
   //Serial.println(EEPROM.read(0));
   for (uint8_t i = 0;i<NUM_SERVOS;i++)
   {
      Serial.write(taskarray[i]);
      Serial.print("\t");
      uint8_t l = (potgrenzearray[i][0] & 0x00FF); // lo byte
      uint8_t h = (potgrenzearray[i][0] & 0xFF00)>>8; // hi byte
      Serial.print("potgrenzearray 0\t");
      Serial.print(potgrenzearray[i][0]);
      Serial.print("\t");
      uint16_t grenzeU = (h << 8) | l;
      Serial.print("grenzeU\t");
      
      uint8_t el = 0;
      uint8_t  eh = 0;
      el = EEPROM.read(2*(i + EEPROMINDEX_U)); // lo byte
      eh = EEPROM.read(2*(i + EEPROMINDEX_U)+1); // hi byte
      Serial.print(el);
      Serial.print("\t");
      Serial.print(eh);
      Serial.print("\t");
      
      potgrenzearray[i][1] = (eh << 8) | el;
      
      el = EEPROM.read(2*(i + EEPROMINDEX_O)); // lo byte
      eh = EEPROM.read(2*(i + EEPROMINDEX_O)+1); // hi byte
      Serial.print(el);
      Serial.print("\t");
      Serial.print(eh);
      Serial.print("\t");
      
      potgrenzearray[i][0] = (eh << 8) | el;
      
      el = EEPROM.read(2*(i + EEPROMLEVELSETTINGS));
      kanalsettingarray[0][i][1] = el; // modell 0
      
      eh = EEPROM.read(2*(i + EEPROMEXPOSETTINGS));
      kanalsettingarray[0][i][2] = eh; // modell 0
      
      if(i==0)
      {
         Serial.print("\n");
         Serial.print("level\t");
         Serial.print(el);
         Serial.print("\t");
         Serial.print("expo\t");
         Serial.println(eh);

 
      }
      
      
      
      
   } // for i
   Serial.print("\n");  
}

void clearsettings(void)
{
   Serial.print("clearsettings\n");  
   for (uint8_t i = 0;i<NUM_SERVOS;i++)
   {
      kanalsettingarray[curr_model][i][1] = 0x00; // level
      kanalsettingarray[curr_model][i][2] = 0x00; // level
      
   } // for i
}

void cleargrenzen(void)
{
   Serial.print("cleargrenzen\n");  
   for (uint8_t i = 0;i<NUM_SERVOS;i++)
   {
      potgrenzearray[i][0] = 127; // 
      potgrenzearray[i][1] = 127; 
      
   } // for i
}

void eepromwrite(void)
{
   Serial.print("eepromwrite\n");  
   for (uint8_t i = 0;i<NUM_SERVOS;i++)
   {
      Serial.print("potgrenzearray raw i:\t");
      Serial.print(i);
      Serial.print("\t");
      Serial.write(taskarray[i]);
      Serial.print("\t");
      Serial.print("potgrenze HI:\t");
      Serial.print(potgrenzearray[i][0]);
      Serial.print("\t");
      Serial.print("potgrenze LO:\t");
      Serial.print(potgrenzearray[i][1]);
      Serial.print("\t");
      Serial.print("servomitte:\t");
      Serial.print(servomittearray[i]);
      Serial.print("\t");
      Serial.print("adresse U:\t");
      uint8_t addresseU_LO = 2*(i + EEPROMINDEX_U);
      Serial.print(addresseU_LO);
      Serial.print("\t");
      Serial.print("adresse H:\t");
      uint8_t addresseU_HI = 2*(i + EEPROMINDEX_U)+1;
      Serial.print(addresseU_HI);
      
      
      Serial.print("\n");
      
      
      
      
      
   
   }
   
   Serial.print("eepromwrite end\n");
}
uint8_t Joystick_Tastenwahl(uint16_t Tastaturwert)
{
   //return 0;
   if (Tastaturwert < JOYSTICKTASTE1) 
      return 2;
   if (Tastaturwert < JOYSTICKTASTE2)
      return 1;
   if (Tastaturwert < JOYSTICKTASTE3)
      return 4;
   if (Tastaturwert < JOYSTICKTASTE4)
      return 7;
   if (Tastaturwert < JOYSTICKTASTE5)
      return 8;
   if (Tastaturwert < JOYSTICKTASTE6)
      return 3;
   if (Tastaturwert < JOYSTICKTASTE7)
      return 6;
   if (Tastaturwert < JOYSTICKTASTE8)
      return 9;
   if (Tastaturwert < JOYSTICKTASTE9)
      return 5;
   /*
    if (Tastaturwert < JOYSTICKTASTEL)
    return 10;
    if (Tastaturwert < JOYSTICKTASTE0)
    return 0;
    if (Tastaturwert < JOYSTICKTASTER)
    return 12;
    */
   return 0;
}
// tastenwahl

void tastenfunktion(uint16_t Tastenwert)
{  
   tastaturcounter++;   
   if (Tastenwert>10) // ca Minimalwert der Matrix
   {      
      //Serial.print(Tastenwert);
      //Serial.print("\t");
      //Serial.print(tastaturcounter);
      
      //Serial.print("\n");
      
      if (tastaturcounter>=400)   //   Prellen
      {        
         
         tastaturcounter=0x00;
         //Serial.println("Taste down");
         if (!(tastaturstatus & (1<<TASTE_OK))) // Taste noch nicht gedrueckt
         {
            
            //Serial.println(Tastenwert);
            //Taste = 0;
            
            //tastaturstatus |= (1<<TASTE_ON); // nur einmal   
            tastaturstatus |= (1<<TASTE_OK); // nur einmal   
            Taste= Joystick_Tastenwahl(Tastenwert);
            tastaturstatus |= (1<<AKTION_OK);
            if(OLED && Taste) // Taste und Tastenwert anzeigen
            {
               //oled_delete(0,62,20);
               //u8g2.setCursor(0,42);
               //u8g2.print(tastaturwert);
               //u8g2.print("T ");
               //u8g2.setCursor(0,62);
               //u8g2.print(Taste);
               
               //u8g2.sendBuffer(); 
               
            }
            
            
            //;
         }
         else // Taste neu gedrückt
         {
            /*
             Taste = 0;
             //tastaturstatus |= (1<<TASTE_ON); // nur einmal 
             
             Taste= Joystick_Tastenwahl(Tastenwert);
             tastaturstatus |= (1<<AKTION_OK);
             if(OLED && Taste) // Taste und Tastenwert anzeigen
             {
             oled_delete(0,62,40);
             u8g2.setCursor(0,62);
             //u8g2.print(tastaturwert);
             u8g2.print("T ");
             u8g2.print(Taste);
             
             u8g2.sendBuffer(); 
             
             }
             */
            
         }
      }
      
      
   }// if tastenwert
   else 
   {
      //if (tastaturstatus & (1<<TASTE_ON))
      {
         
         //tastaturstatus &= ~(1<<TASTE_OK);
      }
   }
   
   
}//tastenfunktion

void setModus(void)
{
   switch (curr_modus)
   {
      case MODELL:
      {
         eepromread();
      }break;
         
      case SIM:
      {
         // Joystick-Settings auf neutral stellen
         for (uint8_t i=0;i<NUM_SERVOS;i++)
         {
            //Serial.print(adcpinarray[i]);
            //Serial.print("\t");
            //Serial.print(servomittearray[i]);
            //Serial.print("\t");
            
            kanalsettingarray[0][i][1] = 0x00; // level
            kanalsettingarray[0][i][2] = 0x00; // expo
         }
      }break;

      case CALIB:
      {

      }break;
   }// switch curr_steuerstatus
}

void setCalib(void)
{

}

void setup()
{
   
   uint8_t ee[16];
   delay(50);
   for (uint8_t i=0;i<64;i++)
   {
      //ee[i] = EEPROM.read(i);
      //EEPROM.write(i,0);
      
   }
   
   delay(50);
   
   //Serial.begin(9600);

   // PPM decode
   pinMode(PPM_PIN, INPUT);
   attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);
   
   curr_steuerstatus = MODELL;
   //savestatus = 0xFF;
   
   delay(500);
   /*
    for (uint8_t i=0;i<16;i++)
    {
    ee[i] = EEPROM.read(i);
    Serial.print(" i: ");
    Serial.print(i);
    Serial.print(" ee: *");
    Serial.print(ee[i]);
    
    }
    Serial.print("\n");
    */
   Serial.println(__DATE__);
   Serial.println(__TIME__);
   
   printeeprom(160);
   
   //eepromread();
   
   
   pinMode(BUZZPIN,OUTPUT);
   
   pinMode(LOOPLED,OUTPUT);
   
   pinMode(BATT_PIN,INPUT);
   
   pinMode(TASTATUR_PIN,INPUT);
   
   /*
   //pinMode(EEPROMTASTE,INPUT_PULLUP);
   eepromtaste.attach( EEPROMTASTE ,  INPUT_PULLUP ); 
   eepromtaste.interval(5);
   eepromtaste.setPressedState(LOW);
   */
   
   //digitalWrite(EEPROMTASTE, HIGH);
   
   // https://registry.platformio.org/libraries/adafruit/Adafruit%20LiquidCrystal/installation
   // set up the LCD's number of rows and columns: 
   //lcd.begin(20, 4);
   // Print a message to the LCD.
   //lcd.print("hello, world!");
   
   // OLED
   //u8x8.setBusClock(4000000);
   //u8x8.setI2CAddress(2*0x3D);
   //u8x8.begin();
   //u8g2.setBusClock(4000000);
   
   // 0.96"
   //u8g2.begin(); 
   initDisplay();
   
   /*
    u8g2.clearDisplay(); 
    //u8g2.setFont(u8g2_font_helvR14_tr); // https://github.com/olikraus/u8g2/wiki/fntlist12
    u8g2.setFont(u8g2_font_t0_15_mr);  
    u8g2.setCursor(4, 14);
    u8g2.print(F("nRF24 T"));
    //u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.setFontMode(0);
    oled_vertikalbalken(VBX,VBY,balkenvb,balkenvh);
    
    oled_horizontalbalken(HBX,HBY,balkenhb,balkenhh);
    */
   oled_vertikalbalken(BATTX,BATTY,BATTB,BATTH);
   
   
   setHomeScreen();
   
   
   
   u8g2.sendBuffer(); 
   
   
   //                Configure the NRF24 module  | NRF24 modül konfigürasyonu
   radio.begin();
   radio.openWritingPipe(pipeOut);
   //radio.setChannel(100);
   radio.setChannel(124);
   radio.setAutoAck(false);
   //radio.setDataRate(RF24_250KBPS);    // The lowest data rate value for more stable communication  | Daha kararlı iletişim için en düşük veri hızı.
   radio.setDataRate(RF24_2MBPS); // Set the speed of the transmission to the quickest available
   
   
   radio.setPALevel(RF24_PA_MAX);      // Output power is set for maximum range  |  Çıkış gücü maksimum menzil için ayarlanıyor.
   
   radio.setPALevel(RF24_PA_MIN); 
   radio.setPALevel(RF24_PA_MAX); 
   
   radio.stopListening();              // Start the radio comunication for Transmitter | Verici için sinyal iletişimini başlatır.
   if (radio.failureDetected) 
   {
      radio.failureDetected = false;
      delay(250);
      Serial.println("Radio failure detected, restarting radio");
   }
   else
   {
      Serial.println("Radio OK");
   }
   ResetData();
   
   
   
   
   //printeeprom(160);
   
} // setup

int Throttle_Map(int val, int fromlow, int fromhigh,int tolow, int tohigh, bool reverse)
{
   val = constrain(val, fromlow, fromhigh);
   val = map(val, fromlow,fromhigh, tolow, tohigh);




   return ( reverse ? 255 - val : val );
}

int Throttle_Map255(int val, int fromlow, int fromhigh,int tolow, int tohigh, bool reverse)
{
   val = constrain(val, fromlow, fromhigh);
   val = map(val, fromlow,fromhigh, tolow, tohigh);

   uint8_t levelwerta = levelwertarray[THROTTLE] & 0x07;
   uint8_t levelwertb = (levelwertarray[THROTTLE] & 0x70)>>4;

   uint8_t expowerta = expowertarray[THROTTLE] & 0x07;

   uint16_t expoint = 3;
   uint16_t levelint = 0;

   expoint = expoarray8[expowerta][val];
   levelint = expoint * (8-levelwerta);
   levelint /= 4;



   return ( reverse ? 255 - levelint : levelint );
}



// Joystick center and its borders 
int Border_Map(int val, int lower, int middle, int upper, bool reverse)
{
   val = constrain(val, lower, upper);
   if ( val < middle )
      val = map(val, lower, middle, 0, 128);
   else
      val = map(val, middle, upper, 128, 255);
   return ( reverse ? 255 - val : val );
}

// Joystick center and its borders 
int Border_Map10(int val, int lower, int middle, int upper, bool reverse)
{
   val = constrain(val, lower, upper);
   if ( val < middle )
      val = map(val, lower, middle, 0, 254); // normieren auf 0-254
   
   else
      val = map(val, middle, upper, 255, 512); // normieren auf 255 - 512
   return ( reverse ? 512 - val : val );
}


int Border_Mapvar255(uint8_t servo, int val, int lower, int middle, int upper, bool reverse)
{
   val = constrain(val, lower, upper); // Grenzen einhalten
   uint8_t levelwerta = levelwertarray[servo] & 0x07;
   uint8_t levelwertb = (levelwertarray[servo] & 0x70)>>4;

   uint8_t expowerta = expowertarray[servo] & 0x07;
   uint8_t expowertb = (expowertarray[servo] & 0x70)>>4;
  
  //levelwerta = 0;
  //levelwertb = 0;
  //expowerta = 0;
  //expowertb = 0;
  
   if ( val < middle )
   {
      
      val = map(val, lower, middle, 0, 127); // normieren auf 0-127
      //intdiff = val;
      intdiff =  (127 - val);// Abweichung von mitte, 
      //levelintraw = intdiff;
      //diffa = map(intdiff,0,(middle - lower), 0,512);
      diffa = intdiff;
      
      expoint = expoarray8[expowerta][diffa];
      levelint = expoint * (8-levelwerta);
      levelint /= 8;
      levelintcheck = 127 + levelint;
      levelint = 127 + levelint;
   }  
   else
   {
      val = map(val, middle, upper, 128, 255); // normieren auf 128 - 255
      //intdiff = val;
      
      intdiff =  (val - 127);// Abweichung von mitte, 
      //diffb = map(intdiff,0,(upper - middle),0,512);
      diffb = intdiff;
      if(diffb >= 127 )
      {
         diffb = 127;
      }
      expoint = expoarray8[expowertb][diffb];
      levelint = expoint * (8-levelwertb) ;     
      levelint /= 8;
      levelintcheck= 127 - levelint;
      levelint= 127 - levelint;
   }
   
   return ( reverse ? 255 - levelint : levelint );
}



uint16_t map_uint16(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) 
{
   if (in_max == in_min) return out_min; // prevent division by zero
   return (uint16_t)(((uint32_t)(x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min);
}

double mapd(double x, double in_min, double in_max, double out_min, double out_max) 
{
   if (in_max == in_min) return out_min;
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void loop()
{                
   //            
   loopcounter++;
   //digitalWrite(BUZZPIN,!(digitalRead(BUZZPIN)));
   //tastaturwert = analogRead(TASTATUR_PIN);

   tastenfunktion(tastaturwert);
   
   if (zeitintervall > 500) 
   { 
      zeitintervall = 0;
      
      sekundencounter++;
      if (sekundencounter%2)
      {
         
         throttlecounter += (data.throttle);
         throttlesekunden = throttlecounter >> 8;
         blinkstatus = 1;

         stopsekunde++;
         if(stopsekunde == 60)
         {
            stopsekunde = 0;
            stopminute++;
         }
         refreshScreen();


      }
      else
      {
         blinkstatus = 0;
      }
      if(curr_screen == 5)
      {
         updateModusScreen();
         u8g2.sendBuffer();
      }
      
   }  
   
   if(loopcounter >= 2*BLINKRATE)
   {
      if(Taste)
      {
         //Serial.print(tastaturwert);
         //Serial.print(" Taste: ");
         Serial.print(Taste);
         Serial.print("\n");
         
         
      }
      loopcounter = 0;
      blinkcounter++;
      impulscounter+=16;
      digitalWrite(LOOPLED, ! digitalRead(LOOPLED));
      
      
      
      //eepromread();
      ///*
      //u8g2.clearBuffer();                   // Clear display.
      //u8x8.setFont(u8g2_font_ncenB08_tr);    // choose a suitable font
      //u8x8.drawString(0, 24, "Hello OLED!");    // write something to the buffer
      //u8x8.sendBuffer();   
      //u8g2.sendBuffer(); // Transfer buffer to screen.
      //*/
      
      
      //Serial.print(" M: ");
      //if(abs(servomittearray[ROLL] - potwertarray[ROLL]) > 2)
      
      
      // 0.96
      loopcounter1++;
      uint8_t charindex = loopcounter1  & 0x7F;
      //u8g2.setDrawColor(0);
      charh = u8g2.getMaxCharHeight() ;
      //oled_delete(4,44,64);
      
      //u8g2.drawGlyph(32,44,'A'+(charindex));
      char buf0[4];
      /*
      
       // Yaw
       //u8g2.setCursor(4,30);
       //u8g2.print(data.yaw);
       sprintf(buf0, "%3d", data.yaw);
       u8g2.drawStr(4,30,buf0);
       
       // Pitch
       // u8g2.setCursor(36,30);
       // u8g2.print(data.pitch);
       sprintf(buf0, "%3d", data.pitch);
       u8g2.drawStr(32,30,buf0);
       
       // Roll
       //u8g2.setCursor(4,46);
       //u8g2.print(data.roll);
       sprintf(buf0, "%3d", data.roll);
       u8g2.drawStr(4,42,buf0);
       
       // Throttle
       //u8g2.setCursor(36,46);
       //u8g2.print(data.throttle);
       sprintf(buf0, "%3d", data.throttle);
       u8g2.drawStr(32,42,buf0);
       */
      
      uint8_t wertv = map(data.pitch,0,255,2,balkenvh-2); // Platz fuer 3 pixel dicke
      //oled_vertikalbalken_setwert(VBX,VBY,balkenvb,balkenvh,wertv);
      
      uint8_t werth = map(data.yaw,0,255,2,balkenhb-2); // Platz fuer 3 pixel dicke
      
      //oled_horizontalbalken_setwert(HBX,HBY,balkenhb,balkenhh,werth);
      
      batterieanzeige = (0x50*batteriespannung)/0x6B/8; // resp. /107
      
      
      /*
       //char buf1[4];
       // Batt
       //sprintf(buf1, "%1.2f", UBatt);
       u8g2.setCursor(90,62);
       u8g2.setDrawColor(0);
       u8g2.print(UBatt,2);
       u8g2.setDrawColor(1);
       */
      if(curr_screen == 0)
      {
         updateHomeScreen();
         u8g2.sendBuffer();
      }
      
      if(loopcounter1 > 25)
      {
         loopcounter1 = 0;
      }
      
      
      
      
       if(calibstatus & (1<CALIB_START))
       {

         Serial.print(" YAW: ");
         Serial.print(potwertarray[YAW]);
         
         Serial.print(" yaw: ");
         Serial.print(data.yaw);   
            
         Serial.print(" PITCH: ");
         Serial.print(potwertarray[PITCH]);
         
         Serial.print(" pitch: ");
         Serial.print(data.pitch);   

         Serial.print(" ROLL: ");
         Serial.print(potwertarray[ROLL]);
         
         Serial.print(" roll: ");
         Serial.print(data.roll);

         Serial.print(" Throttle: ");
         Serial.print(potwertarray[THROTTLE]);
         
         
         Serial.print(" data.throttle: ");
         Serial.print(data.throttle);

         Serial.print("\n");
         
       }
       
       //Serial.print(" throttlemitte: ");
       //Serial.print(servomittearray[THROTTLE]);
       
       
      //Serial.print(" throttlecounter: ");
       //Serial.print(throttlecounter);
      

      /*
       Serial.print(" A1: ");
       Serial.print(potwertarray[PITCH]);
       Serial.print(" A3: ");
       Serial.print(potwertarray[ROLL]);
       Serial.print(" A6: ");
       Serial.print(potwertarray[THROTTLE]);
       */
      
      //Serial.print(" *\n");
   }
   // EEPROM
   eepromtaste.update();
   

   
   
      
   data.yaw = map(ppmValues[YAW],850,2200,0, 255); 
   data.pitch = map(ppmValues[PITCH],850,2200,0, 255); 
   data.roll = map(ppmValues[ROLL],850,2200,0, 255); 
   data.throttle =  map(ppmValues[THROTTLE],850,2200,0, 255); 
   
   radiocounter++;
   
   if (radio.write(&data, sizeof(Signal)))
   {
      //Serial.print("radio ok ");
      //Serial.println(data.yaw);

      radiocounter++; 

   }
   else
   {
      Serial.print("radio error\n");
      //digitalWrite(BUZZPIN,!(digitalRead(BUZZPIN)));
      errcounter++;
      Serial.println(errcounter);
   }
}
