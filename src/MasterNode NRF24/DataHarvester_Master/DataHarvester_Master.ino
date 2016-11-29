// MediaFlow,
// TinyFarmer MasterNode Impletation

#include <ArduinoJson.h>

#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <EEPROM.h>

#define TRUE 1
#define FALSE 0

#define DHID                         "1" // 멀티마스터 적용 시 마스터 구분 용 ID

// RF 관련 설정
#define CE_PIN                        9    /* CE_PIN */
#define CS_PIN                        10   /* CS_PIN */
#define HEARTBEAT_TIMER               30000 /* HEARTBEAT 갱신 주기 */

#define MAX_MESSAGE_LENGTH            150  /* 센서 메시지 최대 길이 */

RF24 radio(CE_PIN, CS_PIN);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

// 타이니파머 설정
#define CHANNEL                       97 /* 기본 채널 97 */
#define MAX_BITMOSS                   99 /* 최대 비트모스 갯수 */
#define DEFAULT_BITMOSS_PERIOD        1  /* 기본 비트모스 수집 주기 (분단위) */

#define  RESETID_ADDR                 100
#define MAC                           "0008DC000000"

// EEPROM 관련 설정
#define INITIALIE_EEPOM_ON_BOOTUP     FALSE /* EEPROM 초기화 */

// 기타 설정
#define SERIAL_SPEED                  9600 //115200 /* 115200 /* 시리얼 속도 */
#define JSON_BUFFER_SIZE              200 /* JSON 버퍼 사이즈 */
#define MASTER_NODE_ID                0 /* 마스터 노드 아이디 (고정) */
#define NUM_OF_RETRY                  5 /* 주기 정보 전송 재시도 횟수 */
#define DELAY_BETWEEN_MESSAGE         100 /* 메세지 사이의 최소 시간 */

// 프로토콜
String ptCd = "01";
String dtCd = "04";
String id = "1";
String Sleep = "";

// 기타 전역 변수
unsigned long heartbeatTimer;
bool sentResult = false;
int retryNum = NUM_OF_RETRY;
String sensorMessage = "";

// Test
//long testTimes[21];

struct sensor_data_t {
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
  String _sleep;
};

// 리셋 함수
void(* resetFunc) (void) = 0; //declare reset function at address 0




int getChan()
{
   int ch = 0;
   
   //ch = 2^3 * digitalRead(31) + 2^2 * digitalRead(30) +2^1 * digitalRead(29) * 2^0 * digitalRead(28) ;
   Serial.println("Channel : " + ch);
   return ch;
}

void setup()
{

  delay(5000);
  Serial.begin(SERIAL_SPEED);

  mesh.setNodeID(MASTER_NODE_ID);
  mesh.begin(getChan(), RF24_250KBPS);

  // EEPROM 초기화 여부
  if (INITIALIE_EEPOM_ON_BOOTUP == TRUE)
    ResetEEPROM();

  // Data Harvester(Master Node)가 Tinyfarmer Hub 에 TCP Socket 연결 후 보내는 프로토콜
  // #Protocol 2.3.1
  StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["ptCd"] = ptCd;
  root["dtCd"] = dtCd;
  //root["id"] = MAC;
  root.printTo(Serial);
  Serial.println("");

  delay(300);

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

  //  for (int i = 0; i <= 20 ; i++)
  //    testTimes[i] = 0;
}


void loop()
{
  sensor_data_t sensorDataT;

  // Mesh 업데이트
  mesh.update();
  mesh.DHCP();

  // 센서 데이터 처리 및 주기 정보 반환
  if (network.available())
  {
    RF24NetworkHeader header;
    network.peek(header);

    // MESSAGE 종류
    // M: Sensor 정보 (JSON Format)
    // P: 센서 취득 주기 정보 (INT)
    switch (header.type)
    {
      // 센서 노드로부터의 센서 정보 취득 및 주기 정보 반환
      case 'M':
        sentResult  = false;

        // 센서 정보 읽어 들이기
        network.read(header, &sensorDataT, sizeof(sensorDataT));

        // 부팅된 첫번째 데이터 전송인지 여부 확인
        if (sensorDataT.isBooted == true)
        {
          StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
          JsonObject& root = jsonBuffer.createObject();
          root["ptCd"] = "10";
          //root["dhId"] = MAC;
          root["id"] = sensorDataT.id;
          root.printTo(Serial);
          Serial.println("");
        }

        StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();

        // 시리얼로 해당 내용 그대로 전달
        root["ptCd"] = "06";
        //root["dhId"] = MAC;
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
        root["sleep"] = sensorDataT._sleep;
        root.printTo(Serial);
        Serial.println("");

        // 센서 정보를 보내온 센서 노드에게 센서 정보 취득 주기 정보를 반환
        int period = ReadFromEEPROM(sensorDataT.id);
        sentResult = mesh.write(&period, 'P', sizeof(int), sensorDataT.id);
        //        testTimes[sensorDataT.id] += 1;
        mesh.update();

        // 실패 했을 경우 재 실행
        retryNum = NUM_OF_RETRY;

        while (!sentResult && retryNum >= 0)
        {
          // printDHCP();
          delay(DELAY_BETWEEN_MESSAGE);
          mesh.update();
          sentResult = mesh.write(&period, 'P', sizeof(int), sensorDataT.id);
          retryNum -= 1;
          //Serial.println("Retrying to send Period Info to " + String(sensorDataT.id) + " " + String(period));
        }

        // Clear the Address to 00 (NULL)
        mesh.setAddress(sensorDataT.id, 00);
        break;
    }
  }

  // 서버로 부터의 센서 취득 주기 메시지 파싱 및 해당 내용 EEPROM 기록
  if (Serial.available() > 0)
  {
    String peoridRequest = Serial.readString();
    StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(peoridRequest);

    if (root.success())
    {
      String commandStr = root["ptCd"];

      // Period Info parsing
      if (commandStr.equals("05"))
      {
        int sensorID = root["id"];
        int sensorPeriod = root["period"];
        WriteToEEPROM(sensorID, sensorPeriod);
        // Serial.println("Period info received " + String(sensorID) + " " + String(sensorPeriod));
      }
      // Reset Command
      else if (commandStr.equals("09"))
      {
        long resetId = root["resetHistoryId"];

        // WRITE IT BACK
        EEPROMWritelong(RESETID_ADDR, resetId);
        delay(3000);
        resetFunc(); // call reset
      }
    }
  }

  //  heartbeatTimer (ignored) , #Protocol 3.3.1
  if ((unsigned long)(millis() - heartbeatTimer) > HEARTBEAT_TIMER)
  {
    heartbeatTimer = millis();
    StaticJsonBuffer<JSON_BUFFER_SIZE> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();

    root["ptCd"] = "02";
    root["dtCd"] = "04";
    root["dhId"] = DHID;
    root.printTo(Serial);
    Serial.println("");
    heartbeatTimer = millis();
    printTest();
    printDHCP();
    clearDHCP();
  }

  delay(30);
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

void printTest()
{
  Serial.println("---------------------");
  int projected = millis() / (DEFAULT_BITMOSS_PERIOD * 60000);
  Serial.println("Projected : " + String(projected) + " Times");
  for (int i = 0; i <= 20 ; i++)
  {
    //    if(testTimes[i] != 0)
    {
      //      Serial.println(String(i) + " : " + String(testTimes[i]) + " Times");
    }
  }
  Serial.println("---------------------");
}

void printDHCP()
{
  Serial.println(" ");
  Serial.println(F("********Assigned Addresses********"));
  for (int i = 0; i < mesh.addrListTop; i++) {
    Serial.print("NodeID: ");
    Serial.print(mesh.addrList[i].nodeID);
    Serial.print(" RF24Network Address: 0");
    Serial.println(mesh.addrList[i].address, OCT);
  }
  Serial.println(F("**********************************"));
}


void clearDHCP()
{
  for (int i = 0; i < mesh.addrListTop; i++)
  {
    mesh.setStaticAddress(mesh.addrList[i].nodeID, 00);
  }
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


