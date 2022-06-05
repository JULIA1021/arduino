#include <esp_now.h>       //引用函示庫
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>

#include "RC.h"

#define WIFI_CHANNEL 4                                          //定義變數(8~13)
#define PWMOUT  // normal esc, uncomment for serial esc            
#define LED 2
#define CALSTEPS 256 // gyro and acc calibration steps
//#define externRC // use of external RC receiver in ppmsum mode
//#define webServer // use of webserver to change PID

extern int16_t accZero[3];     //變數要在多個檔案之間共用時用extern ，且15~23宣告變數
extern float yawRate;
extern float rollPitchRate;
extern float P_PID;
extern float I_PID;
extern float D_PID;
extern float P_Level_PID;
extern float I_Level_PID;
extern float D_Level_PID;

volatile boolean recv;    //volatile用來提醒編譯器它後面所定義的變數隨時有可能改變，因此編譯後的程式每次需要儲存或讀取這個變數的時候，都會直接從變數位址中讀取資料
//volatile int peernum = 0;
//esp_now_peer_info_t slave;

void recv_cb(const uint8_t *macaddr, const uint8_t *data, int  )  //定義recv_cb函式
{
  recv = true;
  //Serial.print("recv_cb ");把資料列印到Serial port
  //Serial.println(len); 把資料列印到Serial port 在資料尾端加上換行字元
  if (len == RCdataSize)     //當len == RCdataSize執行迴圈
  {
    for (int i=0;i<RCdataSize;i++)
         RCdata.data[i] = data[i];
  }
  /*
  if (!esp_now_is_peer_exist(macaddr)) //不存在於macaddr
  {
    Serial.println("adding peer ");     
    esp_now_add_peer(macaddr, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);  //執行 esp_now_add_peer(to add the device to the paired device list before you send data to this device)
    peernum++;
  }
  */
};

#define ACCRESO 4096      //定義變數
#define CYCLETIME 3
#define MINTHROTTLE 1090
#define MIDRUD 1495
#define THRCORR 19

enum ang { ROLL,PITCH,YAW };  //enum(列舉主要是用在宣告僅有少數值的型別，像是一星期內的日期)此處ROLL(翻滾),PITCH(俯仰),YAW(偏擺)

static int16_t gyroADC[3];    //static可以控制變數的可見範圍
static int16_t accADC[3];
static int16_t gyroData[3];
static float angle[2]    = {0,0};  
extern int calibratingA;       //變數要在多個檔案之間共用時用extern 

#ifdef flysky     //處理程序指令檢查宏是否由#define定義
  #define ROL 0
  #define PIT 1
  #define THR 2
  #define RUD 3
#else //orangerx   //#ifdef若不執行 則else下執行
  #define ROL 1
  #define PIT 2
  #define THR 0
  #define RUD 3
#endif

#define AU1 4    
#define AU2 5
static int 16_t rcCommand[] = {0,0,0};    //static可以控制變數的可見範圍

#define GYRO     0
#define STABI    1
static int8_t flightmode;          //static可以控制變數的可見範圍  (  int8_t:typedef signed char)
static int8_t oldflightmode;       

boolean armed = false;   //
uint8_t armct = 0;      //uint8_t: typedef unsigned char
int debugvalue = 0;

void setup() 
{
  Serial.begin(115200); Serial.println();   //序列埠使用115200 且println

  delay(3000); // give it some time to stop shaking after battery plugin
  MPU6050_init();   //執行MPU6050_init()
  MPU6050_readId(); // must be 0x68, 104dec
  
  EEPROM.begin(64);  // 將那部分內存分配給RAM ，size 必須介於 0 和 4096
  if (EEPROM.read(63) != 0x55) Serial.println("Need to do ACC calib");   //當EEPROM.read(63) 不等於0x55 顯示Need to do ACC calib
     else ACC_Read(); // eeprom is initialized                           //等於0x55 ，則執行ACC_Read()
  if (EEPROM.read(62) != 0xAA) Serial.println("Need to check and write PID");    //當EEPROM.read(62)  不等於0xAA   顯示Need to check and write PID
     else PID_Read(); // eeprom is initialized                              //等於0xAA5 ，則執行PID_Read()    

  
  WiFi.mode(WIFI_STA); // Station mode for esp-now   //以工作站（Station）模式啟動，ESP32用來上網讀取資料
  #if defined webServer  
    setupwebserver();     // 若webServer被定義，則執行setupwebserver()
    delay(500); 
  #endif


  #if defined externRC
   init_RC();           // 若externRC被定義，則執行init_RC()
  #else
    Serial.printf("This mac: %s, ", WiFi.macAddress().c_str());    //若externRC未被定義，則顯示字串，且
    Serial.printf(", channel: %i\n", WIFI_CHANNEL);   
    if (esp_now_init() != 0) Serial.println("*** ESP_Now init failed");
    esp_now_register_recv_cb(recv_cb);
  #endif

  delay(500); 
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  initServo();
}

uint32_t rxt; // receive time, used for falisave

void loop() 
{
  uint32_t now,mnow,diff; 
  now = millis(); // actual time
  if (debugvalue == 5) mnow = micros();

  #if defined webServer
    loopwebserver();
  #endif

  if (recv)
  {
    recv = false;  
    #if !defined externRC  
      buf_to_rc();
    #endif

    if (debugvalue == 4) Serial.printf("%4d %4d %4d %4d \n", rcValue[0], rcValue[1], rcValue[2], rcValue[3]); 
  
    if      (rcValue[AU1] < 1300) flightmode = GYRO;
    else                          flightmode = STABI;   
    if (oldflightmode != flightmode)
    {
      zeroGyroAccI();
      oldflightmode = flightmode;
    }

    if (armed) 
    {
      rcValue[THR]    -= THRCORR;
      rcCommand[ROLL]  = rcValue[ROL] - MIDRUD;
      rcCommand[PITCH] = rcValue[PIT] - MIDRUD;
      rcCommand[YAW]   = rcValue[RUD] - MIDRUD;
    }  
    else
    {  
      if (rcValue[THR] < MINTHROTTLE) armct++;
      if (armct >= 25) 
      { 
        digitalWrite(LED,HIGH); 
        armed = true;
      }
    }

    if (debugvalue == 5) Serial.printf("RC input ms: %d\n",now - rxt);
    rxt = millis();
  }

  Gyro_getADC();
  
  ACC_getADC();

  getEstimatedAttitude();

  pid();

  mix();

  writeServo();
  
  // Failsave part
  if (now > rxt+90)
  {
    rcValue[THR] = MINTHROTTLE;
    if (debugvalue == 5) Serial.printf("RC Failsafe after %d \n",now-rxt);
    rxt = now;
  }

  // parser part
  if (Serial.available())
  {
    char ch = Serial.read();
    // Perform ACC calibration
    if (ch == 10) Serial.println();
    else if (ch == 'A')
    { 
      Serial.println("Doing ACC calib");
      calibratingA = CALSTEPS;
      while (calibratingA != 0)
      {
        delay(CYCLETIME);
        ACC_getADC(); 
      }
      ACC_Store();
      Serial.println("ACC calib Done");
    }
    else if (ch == 'R')
    {
      Serial.print("Act Rate :  ");
      Serial.print(yawRate); Serial.print("  ");
      Serial.print(rollPitchRate); Serial.println();
      Serial.println("Act PID :");
      Serial.print(P_PID); Serial.print("  ");
      Serial.print(I_PID); Serial.print("  ");
      Serial.print(D_PID); Serial.println();
      Serial.print(P_Level_PID); Serial.print("  ");
      Serial.print(I_Level_PID); Serial.print("  ");
      Serial.print(D_Level_PID); Serial.println();
    }
    else if (ch == 'D')
    {
      Serial.println("Loading default PID");
      yawRate = 6.0;
      rollPitchRate = 5.0;
      P_PID = 0.15;    // P8
      I_PID = 0.00;    // I8
      D_PID = 0.08; 
      P_Level_PID = 0.35;   // P8
      I_Level_PID = 0.00;   // I8
      D_Level_PID = 0.10;
      PID_Store();
    }
    else if (ch == 'W')
    {
      char ch = Serial.read();
      int n = Serial.available();
      if (n == 3)
      {
        n = readsernum();        
        if      (ch == 'p') { P_PID       = float(n) * 0.01 + 0.004; Serial.print("pid P ");       Serial.print(P_PID); }
        else if (ch == 'i') { I_PID       = float(n) * 0.01 + 0.004; Serial.print("pid I ");       Serial.print(I_PID); }
        else if (ch == 'd') { D_PID       = float(n) * 0.01 + 0.004; Serial.print("pid D ");       Serial.print(D_PID); }
        else if (ch == 'P') { P_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level P "); Serial.print(P_Level_PID); }
        else if (ch == 'I') { I_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level I "); Serial.print(I_Level_PID); }
        else if (ch == 'D') { D_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level D "); Serial.print(D_Level_PID); }
        else Serial.println("unknown command");
      }
      else if (ch == 'S') { PID_Store(); Serial.print("stored in EEPROM"); }
      else 
      {
        Serial.println("Input format wrong");
        Serial.println("Wpxx, Wixx, Wdxx - write gyro PID, example: Wd13");
        Serial.println("WPxx, WIxx, WDxx - write level PID, example: WD21");
      }
    }
    else if (ch >= '0' && ch <='9') debugvalue = ch -'0';
    else
    {
      Serial.println("A - acc calib");
      Serial.println("D - write default PID");
      Serial.println("R - read actual PID");
      Serial.println("Wpxx, Wixx, Wdxx - write gyro PID");
      Serial.println("WPxx, WIxx, WDxx - write level PID");
      Serial.println("WS - Store PID in EEPROM");
      Serial.println("Display data:");
      Serial.println("0 - off");
      Serial.println("1 - Gyro values");
      Serial.println("2 - Acc values");
      Serial.println("3 - Angle values");
      Serial.println("4 - RC values");
      Serial.println("5 - Cycletime");
    }
  }

  if      (debugvalue == 1) Serial.printf("%4d %4d %4d \n", gyroADC[0], gyroADC[1], gyroADC[2]);  
  else if (debugvalue == 2) Serial.printf("%5d %5d %5d \n", accADC[0], accADC[1], accADC[2]);
  else if (debugvalue == 3) Serial.printf("%3f %3f \n", angle[0], angle[1]); 
  
  delay(CYCLETIME-1);  

  if (debugvalue == 5) 
  {
    diff = micros() - mnow;
    Serial.println(diff); 
  }
}

int readsernum()
{
  int num;
  char numStr[3];  
  numStr[0] = Serial.read();
  numStr[1] = Serial.read();
  return atol(numStr);
}
