/*   based on   AlexGyver 2016 sample code
*/

#include <MCP4725.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

RF24 radio(9, 10); // "создать" модуль на пинах 9 и 10 Для Уно
//RF24 radio(9,53); // для Меги

byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб

byte button = 3;  // кнопка на 3 цифровом
byte vx = 0;  // потенциометр на 0 аналоговом
byte vy = 1;  // движковый потенциометр на 1 аналоговом пине
byte quadrant = 0;


uint32_t tmr1;

/*
        A      B          C
  X: 0 - 480 480-506-532  532-1012
  Y: 0 - 480 480-506-532  532-1012

  AA -7    AB - 8    AC - 9
  BA -4    BB - 5    BC - 6
  AA -1    AB - 2    AC - 3

*/

#define MY_PERIOD 1000  // период в мс

int newX=512;
int newY=512;

int oldX=512;
int oldY=512;

const int d =150;
const int dv =10; // spped inc \ dec step
const int MAX_SPEED = 2100;  // max speed
const int MIN_SPEED =-1000;
const int ROT_SPEED = 1200;
const int ROT_SPEEDR = 1200;


float f;


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
  byte CRC = 0;
  int i = 0;
  while (length--) {
    CRC += *(data + i);
    i++;
  }
  return CRC;
}

struct RoboCtrl cur;  // массив, хранящий передаваемые данные
struct RoboCtrl prev;    // массив, хранящий последние переданные данные

boolean flag;           // флажок отправки данных

void setup() {
  Serial.begin(57600);   //открываем порт для связи с ПК
   Serial.println(__FILE__);
  printf_begin();

 memset(&cur, 0 ,sizeof(RoboCtrl));
 memset(&prev, 0 ,sizeof(RoboCtrl));

  pinMode(button, INPUT_PULLUP); // настроить пин кнопки

  radio.begin(); //активировать модуль
  radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 4);    // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(16);   // размер пакета, в байтах

  radio.openReadingPipe(1, address[0]);     // хотим слушать трубу 0
  radio.setChannel(0x60);  // выбираем канал (в котором нет шумов!)

  radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS

  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  radio.powerUp();        //начать работу
  radio.stopListening();  //не слушаем радиоэфир, мы передатчик
    Serial.println("end of setup()");
   Serial.println(sizeof(cur));
}

void loop() {


  // инвертированный (!) сигнал с кнопки
  cur.Break = !digitalRead(button);
  cur.clock = millis() >> 5;
 
  newX = analogRead(vx); // получить значение X
  newY = analogRead(vy); // получить значение Y
  

  
  if ( newX < 512-d){
    if ( newY < 512-d){

        cur.Q=7;
        if(cur.MV <=0){
          cur.MV = dv;
        }
        cur.RL=0;
        cur.RL=0;
        cur.VR = cur.MV;
        cur.VL = cur.MV * 3 /4;
        cur.TurnLeftLight =1;
        cur.TurnRightLight =0;
       

    }else if ( newY < 512+d) {
        cur.Q=4;

        if(cur.MV <=0){
          cur.MV = dv;
        }

        cur.RL=0;
        cur.RL=0;
        cur.VR = cur.MV;
        cur.VL = cur.MV  / 2;
        cur.TurnLeftLight =1;
        cur.TurnRightLight =0;

    }else if(newY <= 1024){
        cur.Q=1;
        cur.RL=1;
        cur.RR=0;

        cur.MV =0;
        cur.VR = ROT_SPEED  ;
        cur.VL = ROT_SPEEDR ;
        
        cur.TurnLeftLight =1;
        cur.TurnRightLight =0;
    }
  }else if ( newX < 512 +d) {
    if ( newY < 512-d){
        cur.Q=8;
        if(cur.MV + dv <= MAX_SPEED)
          cur.MV +=dv;

        if(cur.MV >=0){
          cur.RL=0;
          cur.RR=0;
          cur.TurnLeftLight =0;
          cur.TurnRightLight =0;
        }else{
          cur.RL=1;
          cur.RR=1;
          cur.TurnLeftLight =1;
          cur.TurnRightLight =1;
        }

        cur.VR = abs(cur.MV);
        cur.VL = abs(cur.MV);

    }else if ( newY < 512 +d) {
        cur.Q=5;
        
        if(cur.MV >=0){
          cur.RL=0;
          cur.RR=0;
          cur.TurnLeftLight =0;
          cur.TurnRightLight =0;
        }else{
          cur.RL=1;
          cur.RR=1;
          cur.TurnLeftLight =1;
          cur.TurnRightLight =1;
        }
        cur.VR = abs(cur.MV);
        cur.VL = abs(cur.MV);

    }else if(newY <= 1024){
        
        cur.Q=2;
        if(cur.MV - dv >=MIN_SPEED)
          cur.MV -=dv;

        if(cur.MV >=0){
          cur.RL=0;
          cur.RR=0;
          cur.TurnLeftLight =0;
          cur.TurnRightLight =0;
        }else{
          cur.RL=1;
          cur.RR=1;
          cur.TurnLeftLight =1;
          cur.TurnRightLight =1;

        }

        cur.VR = abs(cur.MV);
        cur.VL = abs(cur.MV);

    }
  }else if(newX <= 1024){
    if ( newY < 512-d){
     
        cur.Q=9;

        if(cur.MV <=0){
          cur.MV = dv;
        }
        cur.RL=0;
        cur.RL=0;
        cur.VL = cur.MV;
        cur.VR = cur.MV * 3 /4;
        cur.TurnLeftLight =0;
        cur.TurnRightLight =1;

    }else if ( newY < 512+d) {
        
        cur.Q=6;
        if(cur.MV <=0){
          cur.MV = dv;
        }
        cur.RL=0;
        cur.RL=0;
        cur.VL = cur.MV;
        cur.VR = cur.MV  / 2;
        cur.TurnLeftLight =0;
        cur.TurnRightLight =1;

    }else if(newY <= 1024){
        
        cur.Q=3;
        cur.RL=0;
        cur.RR=1;
        cur.MV =0;
        cur.VR = ROT_SPEEDR  ;
        cur.VL = ROT_SPEED ;
        cur.TurnLeftLight =0;
        cur.TurnRightLight =1;

    }
  }

  if (cur.Break){
      
      cur.MV =0;
      cur.RL=0;
      cur.RL=0;
      cur.VR = 0;
      cur.VL = 0;
      cur.TurnLeftLight =0;
      cur.TurnRightLight =0;

  }


  if (memcmp (&cur, &prev, sizeof(RoboCtrl)-1) == 0){ 
        flag=0;
  }
  else{
        flag=1;
        memcpy ( &prev, &cur, sizeof(RoboCtrl)-1);
  }

  if (flag==0 && millis() - tmr1 >= MY_PERIOD) {  // ищем разницу
    tmr1 = millis();                   // сброс таймера
    flag =1;
  }

  if (flag == 1) {
    cur.CRC = getCRC((byte*) &cur,sizeof(RoboCtrl)-1);
    radio.powerUp();    // включить передатчик
    radio.write(&cur, sizeof(RoboCtrl)); // отправить по радио
    //printf("B=%d\tMV=%d\tQ=%d\tRL=%d\tVL=%d\tRR=%d\tVR=%d\tTR=%d\tTL=%d\tCLC=%lu\r\n", cur.Break, cur.MV, cur.Q, cur.RL, cur.VL,cur.RR,cur.VR,cur.TurnRightLight,cur.TurnLeftLight, cur.clock);
    flag = 0;           //опустить флаг
    radio.powerDown();  // выключить передатчик
    
  }
  delay(1);
}
int serial_putc( char c, FILE * ) {
  Serial.write( c );
  return c;
}
void printf_begin(void) {
  fdevopen( &serial_putc, 0 );
}
