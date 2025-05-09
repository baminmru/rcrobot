/*    based on   AlexGyver 2016 sample code
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
//#include <Servo.h>
#include "Wire.h"
#include "MCP4725.h"
#include <avr/wdt.h>

MCP4725 MCP0(0x60);
MCP4725 MCP1(0x61);


bool connected0 = false;
bool connected1 = false;
bool isSleep = true;


RF24 radio(9, 10);  // "создать" модуль на пинах 9 и 10 Для Уно
//RF24 radio(9,53); // для Меги

const byte Controller   =2;
const byte ReverseLeft  =3;
const byte ReverseRight =4;
const byte Breaker      =5;
const byte TurnLeftLight  =6;
const byte TurnRightLight =7;
const byte Bell = 8;

const byte powerLeftTest =15;
const byte powerRightTest =14;


#define MY_PERIOD 2000  // период в мс

#define MY_SLEEP_PERIOD 10000  // период в мс


struct RoboCtrl{
  byte Break:1;
  byte  TurnLeftLight:1;
  byte  TurnRightLight:1;
  byte Bell:1;
  byte Q:4;
  byte RL:1;  // rear right
  byte RR:1;  // rear left
  int VL;   
  int VR;
  int MV;
  uint32_t clock;
  byte CRC;
};

byte getCRC(byte* data, int length) {
  byte CRC = 0x55;
  int i = 0;
  while (length--) {
    CRC += *(data + i);
    i++;
  }
  return CRC;
}


uint32_t tmr1;
uint32_t tmr2;
bool flag =false;

struct RoboCtrl cur;  // массив, хранящий передаваемые данные
void InitRadio();
void(* resetFunc) (void) = 0;

uint32_t myClock = 0;



uint64_t addr =0xFABEDA0001;

void setup() {
  Serial.begin(57600);       // открываем порт для связи с ПК
  Serial.println(__FILE__);

  printf_begin();

  pinMode(Controller, OUTPUT);   
  pinMode(ReverseLeft, OUTPUT);  
  pinMode(ReverseRight, OUTPUT);  
  pinMode(Breaker, OUTPUT);  
  pinMode(TurnLeftLight, OUTPUT);  
  pinMode(TurnRightLight, OUTPUT);  
  pinMode(Bell, OUTPUT);  
  pinMode(powerLeftTest,INPUT);
  pinMode(powerRightTest,INPUT);
  

 Serial.println("MCP4725 init");
 Wire.begin();

  if (MCP0.begin() == false)
  {
    Serial.println("Could not find DAC at 60 addr");
  }else {
   Serial.println("Found DAC at 60 addr");
  }


  if (MCP1.begin() == false)
  {
    Serial.println("Could not find DAC at 61 addr");
  }else {
   Serial.println("Found DAC at 61 addr");
  }


  // initial staus of robot  
  digitalWrite(Controller,LOW);
  digitalWrite(Breaker,LOW);

  digitalWrite(TurnLeftLight,HIGH);
  digitalWrite(TurnRightLight,HIGH);

  // forward
  digitalWrite(ReverseLeft,HIGH);  
  digitalWrite(ReverseRight,HIGH);   
  
  // no move
  MCP0.setValue(500 );
  MCP1.setValue(500 );


  isSleep =false;
  tmr1 = millis();
  InitRadio();
  Serial.println("end of setup()");
   Serial.println(sizeof(cur));
   myClock =  0;
}

void InitRadio(){
  radio.begin(); //активировать модуль
  radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);    // (время между попыткой достучаться, число попыток)
  //radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(16);   // размер пакета, в байтах
  radio.enableDynamicPayloads();

  radio.openReadingPipe(1, addr);     // хотим слушать трубу 0
  radio.setChannel(0x60);  // выбираем канал (в котором нет шумов!)

  radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS

  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!
 
  radio.powerUp();          // начать работу
  radio.startListening();   // начинаем слушать эфир, мы приёмный модуль


}



bool CheckCLC(){
  bool OK =false;
 
  if(myClock==0){
    myClock = cur.clock;
    OK = true;
  }else{
    if (cur.clock - myClock > 1000 ) {
      OK = false;
      myClock = 0;
    }
    else{
      OK = true;
      myClock = cur.clock;
    }
      
  }


  return OK;
}

void loop() {
  flag = false;
  byte pipeNo=1;
  int cnt = 60;
  //while (cnt &&   radio.available(&pipeNo)) { // есть входящие данные  radio.getPayloadSize() >0) { //}
    while (cnt &&   radio.getPayloadSize() >0) {  
    // чиатем входящий сигнал
    radio.read(&cur, sizeof(cur));

    cnt--;

    
    if(cur.CRC = getCRC((byte*) &cur,sizeof(cur)-1) 
    && cur.Q >=1 && cur.Q<=9 && cur.MV >=-2000 && cur.MV<=4000 && cur.VL >=0 && cur.VR >=0 && cur.VR <= abs(cur.MV) and cur.VL <= abs(cur.MV)
    && CheckCLC()
    ){


      if(isSleep){
        isSleep = false;
        Serial.println("WakeUp");
        digitalWrite(Controller,LOW);
      }


      int testLeft = analogRead(powerLeftTest);
      int testRight = analogRead(powerRightTest);

         // process only if payload CRC is OK
   
      printf("B=%d\tMV=%d\tQ=%d\tRL=%d\tVL=%d\tRR=%d\tVR=%d\tTR=%d\tTL=%d\tCLC=%lu\tL=%d\tR=%d\r\n", cur.Break, cur.MV, cur.Q, cur.RL, cur.VL,cur.RR,cur.VR,cur.TurnRightLight,cur.TurnLeftLight,cur.clock,testLeft,testRight);

      // lights
      if(cur.Break)
        digitalWrite(Breaker,LOW);
      else
        digitalWrite(Breaker,HIGH);

      if(cur.Bell)
        digitalWrite(Bell,LOW);
      else
        digitalWrite(Bell,HIGH);
      
      if(cur.TurnRightLight)
        digitalWrite(TurnRightLight,LOW);
      else
        digitalWrite(TurnRightLight,HIGH);    

      if(cur.TurnLeftLight)
        digitalWrite(TurnLeftLight,LOW);
      else
        digitalWrite(TurnLeftLight,HIGH); 


      // motor control
      if(cur.RL)
        digitalWrite(ReverseLeft,LOW);
      else
        digitalWrite(ReverseLeft,HIGH);  

      if(cur.RR)
        digitalWrite(ReverseRight,LOW);
      else
        digitalWrite(ReverseRight,HIGH);   

      // speed control
      if(cur.VL >0)
        MCP0.setValue(700 + abs(cur.VL) );
      else
        MCP0.setValue(500 );

      if(cur.VR >0)
        MCP1.setValue(700 + abs(cur.VR) );
      else
        MCP1.setValue(500 );



      flag= true;
    }
     
  }
  
  
  if(!flag){
    if (millis() - tmr1 >= MY_PERIOD) {  // останавливаемся поскольку нет инфы, что делать дальше
        
        if(!isSleep){
          if (millis() - tmr2 >= MY_PERIOD){
              Serial.println("No signal");
              tmr2=millis();
          }
          MCP0.setValue(500 );
          MCP1.setValue(500 );
          digitalWrite(TurnRightLight,LOW);
          digitalWrite(TurnLeftLight,LOW);
          digitalWrite(Breaker,LOW);
          InitRadio();
          
        }
    } 
  
    if (millis() - tmr1 >= MY_SLEEP_PERIOD) {  // останавливаемся поскольку нет инфы, что делать дальше
        if(!isSleep){
          Serial.println("resetting bot");
          isSleep =true;
          digitalWrite(Controller,HIGH);
          resetFunc();
         
        }
    } 
  }else{
    tmr1 = millis();    // последний раз когда получили данные от передатчика
    tmr2 = tmr1;
  }
  

}
int serial_putc( char c, FILE * ) {
  Serial.write( c );
  return c;
}
void printf_begin(void) {
  fdevopen( &serial_putc, 0 );
}

