#include <ESP8266WiFi.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

// WiFi & MQTT instellingen
const char* ssid = "Your wifi access point";
const char* password = "password";
const char* mqtt_server = "192.168.14.30";  // IP MQTT broker
const char* mqtt_topic = "dtsu666/data";
const char* client_id = "DTSU666_MQTT";

// RS485 control
#define RS485_CTRL D5       // GPIO14
#define RS485_RX   D6       // GPIO13
#define RS485_TX   D7       // GPIO12

SoftwareSerial rs485(RS485_RX, RS485_TX);  // RX, TX
ModbusMaster node;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Voor RS485 richting
void preTransmission() {
  digitalWrite(RS485_CTRL, HIGH);
}

void postTransmission() {
  digitalWrite(RS485_CTRL, LOW);
}

void setup() {
  Serial.begin(115200);  // Voor debug
  pinMode(RS485_CTRL, OUTPUT);
  digitalWrite(RS485_CTRL, LOW);  // Begin in ontvangmodus

  rs485.begin(9600);  // Modbus RTU
  node.begin(4, rs485);  // Modbus slave ID
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Verbind met WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi verbonden!");
  Serial.print("IP adres: ");
  Serial.println(WiFi.localIP());
  
  // MQTT
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setBufferSize(768);
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Verbinden met MQTT...");
    if (mqttClient.connect("ESP8266_RS485")) {
      Serial.println("verbonden!");
    } else {
      Serial.print("Fout: ");
      Serial.print(mqttClient.state());
      delay(2000);
    }
  }
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  StaticJsonDocument<768> metingDoc;  // Voor opslaan registers
    
  union { uint32_t i; float f; } raw;
  float waarde;
  uint8_t result;

  auto readAndStore = [&](uint16_t reg, float divisor, const char* key, JsonDocument& doc) {
    result = node.readInputRegisters(reg, 2);
    if (result == node.ku8MBSuccess) {
      raw.i = ((uint32_t)node.getResponseBuffer(0) << 16) | node.getResponseBuffer(1);
      waarde = raw.f / divisor;
      doc[key] = waarde;
      Serial.print(String(key) + ": "); Serial.println(waarde, 3);
    } else {
      Serial.print("Fout bij uitlezen "); Serial.println(key);
    }
  };

  // === Realtime metingen ===
  readAndStore(8192, 10.0, "voltage_ab", metingDoc); //Uab
  readAndStore(8194, 10.0, "voltage_bc", metingDoc); //Ubc
  readAndStore(8196, 10.0, "voltage_ca", metingDoc); //Uca
  readAndStore(8198, 10.0, "voltage_a", metingDoc); //Ua
  readAndStore(8200, 10.0, "voltage_b", metingDoc); //Ub
  readAndStore(8202, 10.0, "voltage_c", metingDoc); //Uc
  readAndStore(8204, 1000.0, "current_a", metingDoc); //Ia
  readAndStore(8206, 1000.0, "current_b", metingDoc); //Ib
  readAndStore(8208, 1000.0, "current_c", metingDoc); //Ic
  readAndStore(8210, 10.0, "active_power_total", metingDoc); //Pt
  readAndStore(8212, 10.0, "active_power_a", metingDoc); //Pa
  readAndStore(8214, 10.0, "active_power_b", metingDoc); //Pb
  readAndStore(8216, 10.0, "active_power_c", metingDoc); //Pc
  readAndStore(8218, 10.0, "reactive_power_total", metingDoc); //Qt
  readAndStore(8220, 10.0, "reactive_power_a", metingDoc); //Qa
  readAndStore(8222, 10.0, "reactive_power_b", metingDoc); //Qb
  readAndStore(8224, 10.0, "reactive_power_c", metingDoc); //Qc
  readAndStore(8234, 1000.0, "power_factor_total", metingDoc); //PFt
  readAndStore(8236, 1000.0, "power_factor_a", metingDoc); //PFa
  readAndStore(8238, 1000.0, "power_factor_b", metingDoc); //PFb
  readAndStore(8240, 1000.0, "power_factor_c", metingDoc); //PFc
  readAndStore(8260, 100.0, "frequency", metingDoc); //Freq
  readAndStore(4126, 1.0, "forward_active_energy_total", metingDoc); //ImpEp
  //readAndStore(4128, 1.0, "forward_active_energy_a", metingDoc); //ImpEpA
  //readAndStore(4130, 1.0, "forward_active_energy_b", metingDoc); //ImpEpB
  //readAndStore(4132, 1.0, "forward_active_energy_c", metingDoc); //ImpEpC
  //readAndStore(4134, 1.0, "net_forward_active_energy", metingDoc); //NetImpEp
  readAndStore(4136, 1.0, "reverse_active_energy_total", metingDoc); //ExpEp
  //readAndStore(4138, 1.0, "reverse_active_energy_a", metingDoc); //ExpEpA
  //readAndStore(4140, 1.0, "reverse_active_energy_b", metingDoc); //ExpEpB
  //readAndStore(4142, 1.0, "reverse_active_energy_c", metingDoc); //ExpEpC
  //readAndStore(4144, 1.0, "net_reverse_active_energy", metingDoc); //NetExpEp

  // === Publish realtime metingen ===
  char buffer1[768];
  serializeJson(metingDoc, buffer1);
  mqttClient.publish("dtsu666/data", buffer1);
  Serial.println("JSON verzonden (data): ");
  Serial.println(buffer1);
  Serial.print("JSON length: ");
  Serial.println(strlen(buffer1));

  delay(1000);
}
