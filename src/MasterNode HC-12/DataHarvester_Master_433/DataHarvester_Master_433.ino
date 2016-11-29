// MediaFlow,
// TinyFarmer MasterNode Impletation

#include <ArduinoJson.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

#define TRUE 1
#define FALSE 0


#define USE_HC12                 /* HC12 모듈 사용 시 */

#define DHID                         "1" // 멀티마스터 적용 시 마스터 구분 용 ID


#define HEARTBEAT_TIMER               30000 /* HEARTBEAT 갱신 주기 */

// 타이니파머 설정
#define CHANNEL                       97 /* 기본 채널 97 */
#define MAX_BITMOSS                   9 /* 최대 비트모스 갯수 */
#define DEFAULT_BITMOSS_PERIOD        1 /* 기본 비트모스 수집 주기 (분단위) */

#define  RESETID_ADDR                 100

// EEPROM 관련 설정
#define INITIALIE_EEPOM_ON_BOOTUP     FALSE /* EEPROM 초기화 */

// 기타 설정
#define SERIAL_SPEED                  9600 //115200 /* 115200 /* 시리얼 속도 */
#define JSON_BUFFER_SIZE              300 /* JSON 버퍼 사이즈 */
#define NUM_OF_RETRY                  5 /* 주기 정보 전송 재시도 횟수 */
#define DELAY_BETWEEN_MESSAGE         100 /* 메세지 사이의 최소 시간 */

// HC12 시리얼 설정
#define HC12_TX 5
#define HC12_RX 6
#define HC12_SET 7
SoftwareSerial HC12(HC12_RX, HC12_TX);

// 기타 전역 변수
unsigned long heartbeatTimer;
bool sentResult = false;
int retryNum = NUM_OF_RETRY;
String sensorMessage = "";
String HC12Data = "";

int ID_sensor = 0 ;
int channel = CHANNEL;

bool string_compelete = false;
bool send_compelete = true;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is 

struct sensor_data_t {
  int dhId;                      //채널번호
  int id;
  float temp;
  float hum;
  long ill;
  int co2;
  float ph;
  float ec;
  float soil_temp;
  float soil_hum;
  int wind_dir;
  float wind_vol;
  float rainfall;
  bool isBooted;
};

// 리셋 함수
void(* resetFunc) (void) = 0; //declare reset function at address 0

int getChan()
{
   int ch = 0;
 
   ch = 8 * digitalRead(8) + 4 * digitalRead(4) + 2 * digitalRead(3) + digitalRead(2) ;

   return ch;
}

void setup()
{
  delay(5000);
  Serial.begin(SERIAL_SPEED);
  HC12.begin(SERIAL_SPEED);
  
  channel = getChan();

  // 로터리 스위치
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(8, INPUT);

#ifdef USE_HC12
  ///////////////////////////////////////////
  // 초기화 (HC-12)
  ///////////////////////////////////////////
  String chString = "AT+C00" + String(channel) +"\r\n";
  
  pinMode(HC12_SET, OUTPUT);
  digitalWrite(HC12_SET, LOW);  // AT 설정 모드 진입
  delay(200);

  HC12.print(chString);          // 채널  설정
  delay(200);
  
  HC12.print(F("AT+B9600\r\n")); // 속도  설정
  delay(200);
  
  digitalWrite(HC12_SET, HIGH); // AT 설정 모드 빠져나옴

  Serial.println("HC-12 Init. Done");
  Serial.println(chString);
#endif


  // EEPROM 초기화 여부
  if (INITIALIE_EEPOM_ON_BOOTUP == TRUE)
    ResetEEPROM();

  if (long(EEPROMReadlong(RESETID_ADDR)) != 0)
  {
    StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["ptCd"] = "10";
    //root["dhId"] = MAC;
    root["resetHistoryId"] = EEPROMReadlong(RESETID_ADDR);
    root.printTo(Serial);
    Serial.println("");

    EEPROMWritelong(RESETID_ADDR, 0);
  }
  inputString.reserve(200);
}


void loop()
{
  sensor_data_t sensorDataT;

  if(ID_sensor >= MAX_BITMOSS)
  {
    ID_sensor = 0;
  }

  ////////////////////////////////////////////////////////////////////
  // 센서 노드에 센서 데이터 요청
  ////////////////////////////////////////////////////////////////////
  if(send_compelete == true)
  {
    ID_sensor ++; // 전송할 센서 아이디 값 증가
    //ID_sensor = 2;
    
    StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["ch"] = String(channel);
  
    root["id"] = String(ID_sensor);
      
    root["req"] = "1";
    root.printTo(HC12);
    HC12.println("");
    Serial.print("request: ");
    Serial.println(ID_sensor);

    HC12Data = "";
  
    delay(2500);
  }

  ////////////////////////////////////////////////////////////////////
  // 센서 데이터 처리 및 주기 정보 반환
  ////////////////////////////////////////////////////////////////////
  if (HC12.available())
  {
    HC12Data = HC12.readString(); // Receive a single character from the software serial port
    string_compelete = true;
  }

  ////////////////////////////////////////////////////////////////////
  // 수신된 데이터 처리
  ////////////////////////////////////////////////////////////////////
  if (string_compelete == true)
  {
    Serial.print("Received: ");
    Serial.println(HC12Data);

    // JSON 파싱
    StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(HC12Data);

    if (root.success())
    {
      Serial.println("Parsing Done");
      // 제대로 받았을때
      sentResult  = false;

      // 데이터 파싱
      sensorDataT.dhId = root["dhId"];
      sensorDataT.id = root["id"];
      sensorDataT.temp = root["temp"];
      sensorDataT.hum = root["hum"];
      sensorDataT.ill = root["ill"];
      sensorDataT.co2 = root["co2"];
      
      sensorDataT.ph = root["ph"];
      sensorDataT.ec = root["ec"];
      sensorDataT.soil_temp = root["soil_temp"];
      sensorDataT.soil_hum = root["soil_hum"];
      sensorDataT.wind_dir = root["wind_dir"];
      sensorDataT.wind_vol = root["wind_vol"];
      sensorDataT.rainfall = root["rainfall"];
      sensorDataT.isBooted = root["isBooted"];

      // 부팅된 첫번째 데이터 전송인지 여부 확인
      if (sensorDataT.isBooted == true)
      {
        StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        root["ptCd"] = "10";
        root["dhId"] = String(channel);
        root["id"] = sensorDataT.id;
        root.printTo(Serial);
        Serial.println("");
      }




      StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
      JsonObject& root = jsonBuffer.createObject();

      // 수신된 데이터의 채널정보가 일치할 때만 처리
      //if(root["dhId"] == String(channel))
      {
        Serial.println("Sending to Server");
        // 시리얼로 해당 내용 그대로 전달
        root["ptCd"] = "06";
        root["dhId"] = String(channel);
        root["id"] = sensorDataT.id;
        root["temp"] = sensorDataT.temp;
        root["hum"] = sensorDataT.hum;
        root["ill"] = sensorDataT.ill;
        root["co2"] = sensorDataT.co2;
        root["ph"] = sensorDataT.ph;
        root["ec"] = sensorDataT.ec;
        root["soilTemp"] = sensorDataT.soil_temp;
        root["soilHum"] = sensorDataT.soil_hum;
        root["windDir"] = sensorDataT.wind_dir;
        root["windVol"] = sensorDataT.wind_vol;
        root["rainfall"] = sensorDataT.rainfall;
        root.printTo(Serial);
        Serial.println("");

        //delay(2000);
      }
    }
    else
    {
      Serial.println("Parsing FAILED");
    }

    delay(1000);
   
    // HC-12 버퍼 클리어
    HC12Data = "";
    string_compelete = false;
    send_compelete = true;
  }

  
  //  heartbeatTimer (ignored)
  if ((unsigned long)(millis() - heartbeatTimer) > HEARTBEAT_TIMER)
  {
    heartbeatTimer = millis();
  }

}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '}') {
      stringComplete = true;
    }
  }
}






























// EEPROM 초기화 함수
void ResetEEPROM()
{
  for (int index = 0 ; index < MAX_BITMOSS ; index++)
    EEPROM.write(index, DEFAULT_BITMOSS_PERIOD);

  EEPROMWritelong(RESETID_ADDR, 0);
}

// EEPROM에 센서 노드와 해당 주기를 설정
void WriteToEEPROM(int sensorNodeID, int period)
{
  EEPROM.write(sensorNodeID - 1, period);
}

// EEPROM에 센서 노드 주기 정보를 요청
int ReadFromEEPROM(int sensorNodeID)
{
  return int(EEPROM.read(sensorNodeID - 1));
}

void EEPROMWritelong(int address, long value)
{
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

long EEPROMReadlong(long address)
{
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}


