#include <Arduino.h>
#include <checksum.h>
#include <mavlink_commands.hpp>
#include <mavlink_conversions.h>
#include <mavlink_get_info.h>
#include <mavlink_helpers.h>
#include <mavlink_sha256.h>
#include <mavlink_types.h>
#include <protocol.h>
#include <math.h>
#include <stdlib.h>

//para nao bugar os heartbeat
#include <TaskScheduler.h>
Scheduler taskScheduler;

//configurações do filtro
#define windowSize 5
#define minVal 10
#define maxVal 50

//configuração polinômio
#define C4 (6.03)
#define C3 (-40.82)
#define C2 (105.64)
#define C1 (-156.45)
#define C0 (134.25)

//configuração de pino
#define adcPin 34

//configuração mavlink
std::shared_ptr<Task> heartbeat_task;
std::shared_ptr<Task> sys_status_task;
HardwareSerial mavSerial(2); 
#define SYS_ID 255
#define COMP_ID 1
#define TARGET_SYS_ID 1 

int validSamples = 0;
float voltage = 0, rawTemp = 0, filteredTemp = 0;
float tempHistory[windowSize] = {0};
uint16_t adcVal = 0;

//configuração da frequência de aquisição de dados
unsigned long previousMillis = 0;  
const long interval = 10;  

void send_sys_status();
void heartbeat_task_callback();
int compare(const void *a, const void *b);
void medianFilter(float inputValue, float *outputValue);
float readVoltage(uint16_t rawAdcVal);
float readTemperature(float voltage);
void tempReading(void);
void process_mavlink_messages();
void send_land_command();

void setup() {
  Serial.begin(9600);
  mavSerial.begin(57600, SERIAL_8N1, 16, 17);

  heartbeat_task = std::make_shared<Task>(TASK_SECOND, TASK_FOREVER, &heartbeat_task_callback);
  taskScheduler.addTask(*heartbeat_task);
  heartbeat_task->enable();

  sys_status_task = std::make_shared<Task>(2000, TASK_FOREVER, &send_sys_status);
  taskScheduler.addTask(*sys_status_task);
  sys_status_task->enable();
  Serial.println("Sistema inicializado!");
  delay(3000);
}

void loop() {
  process_mavlink_messages();
  taskScheduler.execute();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;  // Atualiza o tempo de leitura
      tempReading();
      if(filteredTemp>=50){
        send_land_command();
      }
  }
}

void heartbeat_task_callback() {
  Serial.println("Enviando heartbeat...");

  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &msg,
      MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
      MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);

  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  mavSerial.write(buffer, len);
}
void send_sys_status() {
  Serial.println("Enviando SYS_STATUS...");
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t voltage_battery = 12000;  // 12V em mV
  int16_t current_battery = 1000;    // 1A em cA
  int8_t battery_remaining = 80;     // 80% da bateria
  uint16_t load = 500;               // Carga da CPU (5%)

  mavlink_msg_sys_status_pack(SYS_ID, COMP_ID, &msg,
      0, 0, 0,  
      load, voltage_battery, current_battery, battery_remaining,
      0, 0, 0, 0, 0, 0, 
      0, 0, 0  // Consumo de bateria e temperatura
  );

  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  mavSerial.write(buffer, len);
}
int compare(const void *a, const void *b) {
  return (*(float*)a - *(float*)b);
}
void medianFilter(float inputValue, float *outputValue) {
  tempHistory[0] = inputValue;

  if (validSamples < windowSize) {
      validSamples++;
  }

  float temp[windowSize];
  for (int i = 0; i < validSamples; i++) {
      temp[i] = tempHistory[i];
  }

  qsort(temp, validSamples, sizeof(float), compare);

  int medianIndex = validSamples / 2;
  float medianVal = temp[medianIndex];

  if ((minVal != -1 && medianVal < minVal) || 
      (maxVal != -1 && medianVal > maxVal)) {
      return;
  }

  *outputValue = medianVal;
}
float readVoltage(uint16_t rawAdcVal) {
  return (rawAdcVal * 3.3) / 4095.0; 
}
float readTemperature(float voltage) {
  return C0 + C1 * voltage + C2 * pow(voltage, 2) + C3 * pow(voltage, 3) + C4 * pow(voltage, 4);
}
void tempReading(void) {
  adcVal = analogRead(adcPin); 
  voltage = readVoltage(adcVal);
  rawTemp = readTemperature(voltage);


  medianFilter(rawTemp, &filteredTemp);

  
  Serial.print("Temperatura: ");
  Serial.println(filteredTemp);
}
void process_mavlink_messages() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (mavSerial.available()) {
      uint8_t c = mavSerial.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
          if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
              mavlink_sys_status_t sys_status;
              mavlink_msg_sys_status_decode(&msg, &sys_status);
              Serial.print("SYS_STATUS: ");
              Serial.print("Bateria: ");
              Serial.print(sys_status.voltage_battery / 1000.0);
              Serial.println("V");
              } 
      if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
          mavlink_command_ack_t command_ack;
          mavlink_msg_command_ack_decode(&msg, &command_ack);

          if (command_ack.command == 20) {
              Serial.println(" Comando de return to lauch recebido ");
              if (command_ack.result == MAV_RESULT_ACCEPTED) {
                  Serial.println(" Drone iniciando pouso...");
              } else {
                  Serial.print("Falha ao processar return to lauch: Código ");
                  Serial.println(command_ack.result);
              }
          }
        }
      }
  }
}
void send_land_command() {
  Serial.println("Enviando comando de LAND...");

  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
      SYS_ID, COMP_ID, &msg, 
      1,  // Target System (1 = Drone)
      0,  // Target Component (0 = Default)
      21,  // Comando LAND
      0,  // Confirmation
      0, 0, 0, 0, 0, 0, 0  // Parâmetros não utilizados
  );
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  mavSerial.write(buffer, len);
}