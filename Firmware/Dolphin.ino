#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <PID_v1.h>
#include <ArduinoJson.h>
#include <vector>
#include <math.h>

#define tolerance 30

#define M1A 3
#define M1B 4
#define M1EA 5
#define M1EB 6
#define M2A 41
#define M2B 42
#define M2EA 39
#define M2EB 40
#define LED 16
#define VBAT 1
#define SDA 35
#define SCL 36

#define INA3221_ADDR 0x40

// INA3221寄存器地址
#define INA3221_CONFIG_REG 0x00
#define INA3221_SHUNT_VOLTAGE_CH1 0x01
#define INA3221_BUS_VOLTAGE_CH1 0x02
#define INA3221_SHUNT_VOLTAGE_CH2 0x03
#define INA3221_BUS_VOLTAGE_CH2 0x04
#define INA3221_SHUNT_VOLTAGE_CH3 0x05
#define INA3221_BUS_VOLTAGE_CH3 0x06

// 给功率计算使用的变量
#define record_number 1200
volatile int32_t lastTick_ms = 0;
struct Power {
  uint16_t VBUS_BUS;
  uint16_t VBUS_TOP;
  uint16_t VBUS_BOT;
  uint16_t VSHUNT_BUS;
  uint16_t VSHUNT_TOP;
  uint16_t VSHUNT_BOT;
};
static volatile Power PowerData[record_number];
volatile uint32_t power_counter = 0;

// PID参数
double M1_Kp = 0.1, M1_Ki = 0.01, M1_Kd = 0;
double M2_Kp = 0.1, M2_Ki = 0.01, M2_Kd = 0;
double M1_input, M1_output, M1_setpoint;
double M2_input, M2_output, M2_setpoint;
PID M1_PID(&M1_input, &M1_output, &M1_setpoint, M1_Kp, M1_Ki, M1_Kd, DIRECT);
PID M2_PID(&M2_input, &M2_output, &M2_setpoint, M2_Kp, M2_Ki, M2_Kd, DIRECT);

double M1_velocity_Kp = 0.1, M1_velocity_Ki = 0.01, M1_velocity_Kd = 0;
double M2_velocity_Kp = 0.1, M2_velocity_Ki = 0.01, M2_velocity_Kd = 0;
double M1_velocity_input, M1_velocity_output, M1_velocity_setpoint;
double M2_velocity_input, M2_velocity_output, M2_velocity_setpoint;
PID M1_velocity_PID(&M1_velocity_input, &M1_velocity_output, &M1_velocity_setpoint, M1_velocity_Kp, M1_velocity_Ki, M1_velocity_Kd, DIRECT);
PID M2_velocity_PID(&M2_velocity_input, &M2_velocity_output, &M2_velocity_setpoint, M2_velocity_Kp, M2_velocity_Ki, M2_velocity_Kd, DIRECT);

// 轨迹参数
struct Segment {
  double params[6];
};
std::vector<Segment> M1_trajectoryParameters;
std::vector<Segment> M2_trajectoryParameters;

// 简单模式传参
struct SimpleParm {
  uint8_t power;
  int32_t target;
};

// 时间变量
int32_t startTime = -2147483648;
int32_t totalTime = 0;

static const char *ssid = "BotTest";
static const char *password = "123456789";

// Set web server port number to 80
WiFiServer server(80);
// Variable to store the HTTP request
String header;

volatile int32_t M1_target = 0;
volatile int32_t M1_current = 0;
volatile int32_t M1_error = 0;
volatile uint8_t M1_power = 0;
volatile int32_t M2_target = 0;
volatile int32_t M2_current = 0;
volatile int32_t M2_error = 0;
volatile uint8_t M2_power = 0;
volatile uint32_t pause_bot = 0;

volatile uint8_t M1_rec, M2_rec;

void motorControl() {
  if (pause_bot == 0) {
    // 电机1控制
    if (M1_velocity_output > 0) {
      ledcWrite(M1A, abs(M1_velocity_output));
      ledcWrite(M1B, 0);
    } else if (M1_velocity_output < 0) {
      ledcWrite(M1A, 0);
      ledcWrite(M1B, abs(M1_velocity_output));
    } else {
      ledcWrite(M1A, 0);
      ledcWrite(M1B, 0);
    }

    // 电机2控制
    if (M2_velocity_output > 0) {
      ledcWrite(M2A, abs(M2_velocity_output));
      ledcWrite(M2B, 0);
    } else if (M2_velocity_output < 0) {
      ledcWrite(M2A, 0);
      ledcWrite(M2B, abs(M2_velocity_output));
    } else {
      ledcWrite(M2A, 0);
      ledcWrite(M2B, 0);
    }
  } else {
    // 如果暂停，停止所有电机
    ledcWrite(M1A, 0);
    ledcWrite(M1B, 0);
    ledcWrite(M2A, 0);
    ledcWrite(M2B, 0);
  }
}

void motorSimple()
{
  if (M1_target - tolerance * 3 > M1_current && !pause_bot)
  {
    ledcWrite(M1A, M1_power);
    ledcWrite(M1B, 0);
  }
  else if (M1_target - tolerance * 2 > M1_current && !pause_bot)
  {
    ledcWrite(M1A, (M1_power - 20) / 10 + 20);
    ledcWrite(M1B, 0);
  }
  else if (M1_target - tolerance > M1_current && !pause_bot)
  {
    ledcWrite(M1A, (M1_power < 10) ? M1_power : 10);
    ledcWrite(M1B, 0);
  }
  else if (M1_target + tolerance * 3 < M1_current && !pause_bot)
  {
    ledcWrite(M1A, 0);
    ledcWrite(M1B, M1_power);
  }
  else if (M1_target - tolerance * 2 > M1_current && !pause_bot)
  {
    ledcWrite(M1A, 0);
    ledcWrite(M1B, (M1_power - 20) / 10 + 20);
  }
  else if (M1_target + tolerance < M1_current && !pause_bot)
  {
    ledcWrite(M1A, 0);
    ledcWrite(M1B, (M1_power < 10) ? M1_power : 10);
  }
  else
  {
    ledcWrite(M1A, 255);
    ledcWrite(M1B, 255);
  }

  if (M2_target - tolerance * 3 > M2_current && !pause_bot)
  {
    ledcWrite(M2A, M2_power);
    ledcWrite(M2B, 0);
  }
  else if (M2_target - tolerance * 2 > M2_current && !pause_bot)
  {
    ledcWrite(M2A, (M2_power - 20) / 10 + 20);
    ledcWrite(M2B, 0);
  }
  else if (M2_target - tolerance > M2_current && !pause_bot)
  {
    ledcWrite(M2A, (M2_power < 10) ? M2_power : 10);
    ledcWrite(M2B, 0);
  }
  else if (M2_target + tolerance * 3 < M2_current && !pause_bot)
  {
    ledcWrite(M2A, 0);
    ledcWrite(M2B, M2_power);
  }
  else if (M2_target + tolerance * 2 < M2_current && !pause_bot)
  {
    ledcWrite(M2A, 0);
    ledcWrite(M2B, (M2_power - 20) / 10 + 20);
  }
  else if (M2_target + tolerance < M2_current && !pause_bot)
  {
    ledcWrite(M2A, 0);
    ledcWrite(M2B, (M2_power < 10) ? M2_power : 10);
  }
  else
  {
    ledcWrite(M2A, 255);
    ledcWrite(M2B, 255);
  }
}

void IRAM_ATTR isr() {
  uint8_t M1_now = (digitalRead(M1EA) << 1) | digitalRead(M1EB);
  uint8_t M2_now = (digitalRead(M2EA) << 1) | digitalRead(M2EB);

  // 00,01,11,10
  // ++--++--=dolphin
  // --++--++=on the water
  if ((M1_rec == 0b00 && M1_now == 0b01) || (M1_rec == 0b01 && M1_now == 0b11) || (M1_rec == 0b11 && M1_now == 0b10) || (M1_rec == 0b10 && M1_now == 0b00))
    M1_current++;
  else if ((M1_rec == 0b10 && M1_now == 0b11) || (M1_rec == 0b11 && M1_now == 0b01) || (M1_rec == 0b01 && M1_now == 0b00) || (M1_rec == 0b00 && M1_now == 0b10))
    M1_current--;
  else if (M1_rec != M1_now)
    M1_error++;
  if ((M2_rec == 0b00 && M2_now == 0b01) || (M2_rec == 0b01 && M2_now == 0b11) || (M2_rec == 0b11 && M2_now == 0b10) || (M2_rec == 0b10 && M2_now == 0b00))
    M2_current++;
  else if ((M2_rec == 0b10 && M2_now == 0b11) || (M2_rec == 0b11 && M2_now == 0b01) || (M2_rec == 0b01 && M2_now == 0b00) || (M2_rec == 0b00 && M2_now == 0b10))
    M2_current--;
  else if (M2_rec != M2_now)
    M2_error++;

  M1_rec = M1_now;
  M2_rec = M2_now;
}

double segmentedFunction(int64_t t, const std::vector<Segment> &parameters) {
  if (parameters.size() == 0) {
    return 0;
  }
  if (totalTime == -1) {
    t %= (int32_t)parameters[parameters.size() - 1].params[0];
  }
  for (size_t i = 0; i < parameters.size(); i++) {
    if (t <= parameters[i].params[0]) {
      uint8_t functionType = (uint8_t)parameters[i].params[1];
      double a = parameters[i].params[2];
      double b = parameters[i].params[3];
      double c = parameters[i].params[4];
      double d = parameters[i].params[5];

      if (functionType == 0) { // 三次函数
        double t2 = t * t;
        double t3 = t2 * t;

        return a * t3 + b * t2 + c * t + d;
      } else if (functionType == 1) { // Sin函数
        return sin(a * t + b) * c + d;
      }
      else if (functionType == 2) { // 简易控制（直接控制power和位置）
        return NAN;
      }
      else {
        return 0;
      }
    }
  }

  // 如果 t 超出范围，返回最后一段函数在其终点的值
  size_t last = parameters.size() - 1;
  int64_t lastT = (int64_t)parameters[last].params[0];
  if ((uint8_t)parameters[last].params[1] == 0) {
    return parameters[last].params[2] * lastT * lastT * lastT + parameters[last].params[3] * lastT * lastT + parameters[last].params[4] * lastT + parameters[last].params[5];
  } else if ((uint8_t)parameters[last].params[1] == 1) {
    return sin(parameters[last].params[2] * lastT + parameters[last].params[3]) * parameters[last].params[4] + parameters[last].params[5];
  } else if ((uint8_t)parameters[last].params[1] == 2) {
    return NAN;
  }
  return 0;
}

SimpleParm simpleControl(int64_t t, const std::vector<Segment> &parameters) {
  SimpleParm simpleParm;
  simpleParm.power = 0;
  simpleParm.target = 0;

  if (parameters.size() == 0) {
    return simpleParm;
  }
  if (totalTime == -1) {
    t %= (int32_t)parameters[parameters.size() - 1].params[0];
  }
  for (size_t i = 0; i < parameters.size(); i++) {
    if (t <= parameters[i].params[0]) {
      uint8_t functionType = (uint8_t)parameters[i].params[1];
      double a = parameters[i].params[2]; // dummy
      double b = parameters[i].params[3]; // dummy
      double c = parameters[i].params[4]; // power
      double d = parameters[i].params[5]; // target
      int32_t power = (int32_t)c;
      // 缓启动，改善电流冲击
      if ((i == 0 && t <= 65) || (i > 0 && t - 65 < parameters[i - 1].params[0])) {
        power = power > 32 ? 32 : power;
      }
      else if ((i == 0 && t <= 105) || (i > 0 && t - 105 < parameters[i - 1].params[0])) {
        power = power > 64 ? 64 : power;
      }
      int32_t target = (int32_t)d;
      simpleParm.power = (uint8_t)power;
      simpleParm.target = target;
      return simpleParm;
    }
  }
  return simpleParm;
}

void parseTrajectoryJson(String json) {
  DynamicJsonDocument doc(8192);  // 根据预期的JSON大小调整容量

  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.println("JSON解析失败");
    return;
  }

  totalTime = doc["totalTime"].as<int32_t>();

  M1_trajectoryParameters.clear();
  M2_trajectoryParameters.clear();

  JsonArray M1_segments = doc["M1_trajectory"];
  for (JsonArray segment : M1_segments) {
    Segment seg;
    for (int i = 0; i < 6 && i < segment.size(); i++) {
      seg.params[i] = segment[i].as<double>();
    }
    M1_trajectoryParameters.push_back(seg);
  }

  JsonArray M2_segments = doc["M2_trajectory"];
  for (JsonArray segment : M2_segments) {
    Segment seg;
    for (int i = 0; i < 6 && i < segment.size(); i++) {
      seg.params[i] = segment[i].as<double>();
    }
    M2_trajectoryParameters.push_back(seg);
  }
}

void parsePIDJson(String json) {
  DynamicJsonDocument doc(256);  // 根据预期的JSON大小调整容量

  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.println("JSON解析失败");
    return;
  }

  M1_Kp = M2_Kp = doc["Kp"].as<double>();
  M1_Ki = M2_Ki = doc["Ki"].as<double>();
  M1_Kd = M2_Kd = doc["Kd"].as<double>();
  M1_velocity_Kp = M2_velocity_Kp = doc["vKp"].as<double>();
  M1_velocity_Ki = M2_velocity_Ki = doc["vKi"].as<double>();
  M1_velocity_Kd = M2_velocity_Kd = doc["vKd"].as<double>();
  M1_PID.SetTunings(M1_Kp, M1_Ki, M1_Kd);
  M2_PID.SetTunings(M2_Kp, M2_Ki, M2_Kd);
  M1_velocity_PID.SetTunings(M1_velocity_Kp, M1_velocity_Ki, M1_velocity_Kd);
  M2_velocity_PID.SetTunings(M2_velocity_Kp, M2_velocity_Ki, M2_velocity_Kd);
}

String urlDecode(String input) {
  String result = "";
  char c;
  int len = input.length();

  for (int i = 0; i < len; i++) {
    c = input.charAt(i);
    if (c == '+') {
      result += ' ';
    } else if (c == '%') {
      if (i + 2 < len) {
        int hex = 0;
        for (int j = 1; j <= 2; j++) {
          c = input.charAt(i + j);
          if (c >= '0' && c <= '9') {
            hex += (c - '0') * (j == 1 ? 16 : 1);
          } else if (c >= 'A' && c <= 'F') {
            hex += (c - 'A' + 10) * (j == 1 ? 16 : 1);
          } else if (c >= 'a' && c <= 'f') {
            hex += (c - 'a' + 10) * (j == 1 ? 16 : 1);
          }
        }
        result += char(hex);
        i += 2;
      }
    } else {
      result += c;
    }
  }

  return result;
}

float getBattery() {
  int32_t temp = 0;
  for (int32_t i = 0; i < 8; i++)
    temp += analogRead(VBAT);
  return (float)temp * 0.0000808068060588885f - 0.00308826876628743f;
}

String getPower() {
  String result = "";
  for (int i = 0; i < record_number; i++) {
    int index = (power_counter + i + 1) % record_number;
    char hexStr[5];
    snprintf(hexStr, sizeof(hexStr), "%04X", PowerData[index].VBUS_BUS);
    result += hexStr;
    snprintf(hexStr, sizeof(hexStr), "%04X", PowerData[index].VBUS_TOP);
    result += hexStr;
    snprintf(hexStr, sizeof(hexStr), "%04X", PowerData[index].VBUS_BOT);
    result += hexStr;
    snprintf(hexStr, sizeof(hexStr), "%04X", PowerData[index].VSHUNT_BUS);
    result += hexStr;
    snprintf(hexStr, sizeof(hexStr), "%04X", PowerData[index].VSHUNT_TOP);
    result += hexStr;
    snprintf(hexStr, sizeof(hexStr), "%04X", PowerData[index].VSHUNT_BOT);
    result += hexStr;
  }
  return result;
}

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA, SCL);
  Serial.println("I2C Inited");

  initINA3221();
  Serial.println("Power Sensor Inited");

  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  Serial.println("IND Inited");

  ledcAttach(M1A, 50000, 8);
  ledcAttach(M1B, 50000, 8);
  ledcAttach(M2A, 50000, 8);
  ledcAttach(M2B, 50000, 8);
  Serial.println("LEDC Inited");

  pinMode(VBAT, INPUT);
  Serial.println("ADC Inited");
  attachInterrupt(M1EA, isr, CHANGE);
  attachInterrupt(M1EB, isr, CHANGE);
  attachInterrupt(M2EA, isr, CHANGE);
  attachInterrupt(M2EB, isr, CHANGE);
  Serial.println("Encoder Interrupt Inited");
  pinMode(M1EA, INPUT_PULLUP);
  pinMode(M1EB, INPUT_PULLUP);
  pinMode(M2EA, INPUT_PULLUP);
  pinMode(M2EB, INPUT_PULLUP);
  Serial.println("Encoder Inited");

  M1_rec = (digitalRead(M1EA) << 1) | digitalRead(M1EB);
  M2_rec = (digitalRead(M2EA) << 1) | digitalRead(M2EB);

  M1_PID.SetMode(AUTOMATIC);
  M2_PID.SetMode(AUTOMATIC);
  M1_PID.SetOutputLimits(-9000, 9000);
  M2_PID.SetOutputLimits(-9000, 9000);
  M1_velocity_PID.SetMode(AUTOMATIC);
  M2_velocity_PID.SetMode(AUTOMATIC);
  M1_velocity_PID.SetOutputLimits(-255, 255);
  M2_velocity_PID.SetOutputLimits(-255, 255);
  Serial.println("PID Inited");

  delay(500);
  digitalWrite(LED, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED, !digitalRead(LED));
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
  server.begin();

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      digitalWrite(LED, !digitalRead(LED));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  Serial.println("OTA Inited");
  digitalWrite(LED, LOW);
}

void loop() {
  ArduinoOTA.handle();
  WiFiClient client = server.available();  // Listen for incoming clients

  if (client) {                      // If a new client connects,
    Serial.println("New Request.");  // print a message out in the serial port
    String currentLine = "";         // make a String to hold incoming data from the client
    while (client.connected()) {     // loop while the client's connected
      if (client.available()) {      // if there's bytes to read from the client,
        char c = client.read();      // read a byte, then
        Serial.write(c);             // print it out the serial monitor
        header += c;
        if (c == '\n') {  // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          header = urlDecode(header);
          if (currentLine.length() == 0) {
            Serial.println(header);
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:application/json");
            client.println("Connection: close");
            // 添加CORS头，允许所有域访问
            client.println("Access-Control-Allow-Origin: *");
            client.println("Access-Control-Allow-Methods: GET");
            client.println("Access-Control-Allow-Headers: Content-Type");
            client.println();

            // GET /?status HTTP/1.1
            if (header.indexOf("status") >= 0) {
              Serial.println("status requested");
              // Get the status of the system
              client.println("{\"status\":\"ok\", \"ip\":\"" + WiFi.localIP().toString() + "\", \"mac\":\"" + String(WiFi.macAddress()) + "\", \"m1_current\":" + M1_current + ", \"m2_current\":" + M2_current + ", \"m1_target\":" + M1_target + ", \"m2_target\":" + M2_target + ", \"m1_power\": " + M1_power + ", \"m2_power\": " + M2_power + ", \"m1_error\": " + M1_error + ", \"m2_error\": " + M1_error + ", \"battery\": " + getBattery() + ", \"pause\":" + (pause_bot ? String("true") : String("false")) + "}");
            } else if (header.indexOf("calibrate") >= 0)
            {
              Serial.println("calibrate requested");
              // Set target and current position to 0
              // Format: /calibrate=[M1, bit][M2, bit]
              // Totally 2 characters in the parameter
              int pos = header.indexOf('=');
              if (header.substring(pos + 1, pos + 2)[0] == '1')
              {
                M1_target = M1_current = 0;
              }
              if (header.substring(pos + 2, pos + 3)[0] == '1')
              {
                M2_target = M2_current = 0;
              }
              client.println("{\"status\":\"calibrate ok\", \"ip\":\"" + WiFi.localIP().toString() + "\", \"mac\":\"" + String(WiFi.macAddress()) + "\", \"m1_current\":" + M1_current + ", \"m2_current\":" + M2_current + ", \"m1_target\":" + M1_target + ", \"m2_target\":" + M2_target + ", \"m1_power\": " + M1_power + ", \"m2_power\": " + M2_power + ", \"m1_error\": " + M1_error + ", \"m2_error\": " + M1_error + ", \"battery\": " + getBattery() + ", \"pause\":" + (pause_bot ? String("true") : String("false")) + "}");
            } else if (header.indexOf("pid") >= 0) {
              Serial.println("pid requested");
              // Set the target and power of motors
              int start_pos = header.indexOf('`');
              int end_pos = header.lastIndexOf('`');
              String pid_json = header.substring(start_pos + 1, end_pos);
              parsePIDJson(pid_json);
              client.println("{\"status\":\"pid ok\", \"ip\":\"" + WiFi.localIP().toString() + "\", \"mac\":\"" + String(WiFi.macAddress()) + "\", \"m1_current\":" + M1_current + ", \"m2_current\":" + M2_current + ", \"m1_target\":" + M1_target + ", \"m2_target\":" + M2_target + ", \"m1_power\": " + M1_power + ", \"m2_power\": " + M2_power + ", \"m1_error\": " + M1_error + ", \"m2_error\": " + M1_error + ", \"battery\": " + getBattery() + ", \"pause\":" + (pause_bot ? String("true") : String("false")) + "}");
            } else if (header.indexOf("set") >= 0) {
              Serial.println("set requested");
              // Set the target and power of motors
              // Format: `{"M1_trajectory": [[...], [...], ...], "M2_trajectory": [[...], [...], ...], "totalTime": 1000}`
              int start_pos = header.indexOf('`');
              int end_pos = header.lastIndexOf('`');
              String trajectory_json = header.substring(start_pos + 1, end_pos);
              parseTrajectoryJson(trajectory_json);
              client.println("{\"status\":\"set ok\", \"ip\":\"" + WiFi.localIP().toString() + "\", \"mac\":\"" + String(WiFi.macAddress()) + "\", \"m1_current\":" + M1_current + ", \"m2_current\":" + M2_current + ", \"m1_target\":" + M1_target + ", \"m2_target\":" + M2_target + ", \"m1_power\": " + M1_power + ", \"m2_power\": " + M2_power + ", \"m1_error\": " + M1_error + ", \"m2_error\": " + M1_error + ", \"battery\": " + getBattery() + ", \"pause\":" + (pause_bot ? String("true") : String("false")) + "}");
            } else if (header.indexOf("run") >= 0) {
              Serial.println("run requested");
              startTime = millis();
              pause_bot = 0;
              client.println("{\"status\":\"run ok\", \"ip\":\"" + WiFi.localIP().toString() + "\", \"mac\":\"" + String(WiFi.macAddress()) + "\", \"m1_current\":" + M1_current + ", \"m2_current\":" + M2_current + ", \"m1_target\":" + M1_target + ", \"m2_target\":" + M2_target + ", \"m1_power\": " + M1_power + ", \"m2_power\": " + M2_power + ", \"m1_error\": " + M1_error + ", \"m2_error\": " + M1_error + ", \"battery\": " + getBattery() + ", \"pause\":" + (pause_bot ? String("true") : String("false")) + "}");
            } else if (header.indexOf("pause") >= 0) {
              Serial.println("pause requested");
              if (startTime > 0) {
                pause_bot = millis() - startTime;
              }
              client.println("{\"status\":\"resume ok\", \"ip\":\"" + WiFi.localIP().toString() + "\", \"mac\":\"" + String(WiFi.macAddress()) + "\", \"m1_current\":" + M1_current + ", \"m2_current\":" + M2_current + ", \"m1_target\":" + M1_target + ", \"m2_target\":" + M2_target + ", \"m1_power\": " + M1_power + ", \"m2_power\": " + M2_power + ", \"m1_error\": " + M1_error + ", \"m2_error\": " + M1_error + ", \"battery\": " + getBattery() + ", \"pause\":" + (pause_bot ? String("true") : String("false")) + "}");
            } else if (header.indexOf("resume") >= 0) {
              Serial.println("resume requested");
              if (pause_bot > 0) {
                startTime = millis() - pause_bot;
                pause_bot = 0;
              }
              client.println("{\"status\":\"pause ok\", \"ip\":\"" + WiFi.localIP().toString() + "\", \"mac\":\"" + String(WiFi.macAddress()) + "\", \"m1_current\":" + M1_current + ", \"m2_current\":" + M2_current + ", \"m1_target\":" + M1_target + ", \"m2_target\":" + M2_target + ", \"m1_power\": " + M1_power + ", \"m2_power\": " + M2_power + ", \"m1_error\": " + M1_error + ", \"m2_error\": " + M1_error + ", \"battery\": " + getBattery() + ", \"pause\":" + (pause_bot ? String("true") : String("false")) + "}");
            } else if (header.indexOf("stop") >= 0) {
              Serial.println("resume requested");
              startTime = -2147483648;
              pause_bot = 0;
              client.println("{\"status\":\"stop ok\", \"ip\":\"" + WiFi.localIP().toString() + "\", \"mac\":\"" + String(WiFi.macAddress()) + "\", \"m1_current\":" + M1_current + ", \"m2_current\":" + M2_current + ", \"m1_target\":" + M1_target + ", \"m2_target\":" + M2_target + ", \"m1_power\": " + M1_power + ", \"m2_power\": " + M2_power + ", \"m1_error\": " + M1_error + ", \"m2_error\": " + M1_error + ", \"battery\": " + getBattery() + ", \"pause\":" + (pause_bot ? String("true") : String("false")) + "}");
            } else if (header.indexOf("power") >= 0) {
              client.println("{\"status\":\"power ok\", \"ip\":\"" + WiFi.localIP().toString() + "\", \"mac\":\"" + String(WiFi.macAddress()) + "\", \"m1_current\":" + M1_current + ", \"m2_current\":" + M2_current + ", \"m1_target\":" + M1_target + ", \"m2_target\":" + M2_target + ", \"m1_power\": " + M1_power + ", \"m2_power\": " + M2_power + ", \"m1_error\": " + M1_error + ", \"m2_error\": " + M1_error + ", \"battery\": " + getBattery() + ", \"pause\":" + (pause_bot ? String("true") : String("false")) + ", \"power\": \"" + getPower() + "\"}");
            } else if (header.indexOf("sensor") >= 0) {
              Serial.println("sensor requested");
              // Temp function to debug the current sensor
              float shuntVoltage[3], busVoltage[3], current[3];
              float shuntResistance[3] = { 0.04f, 0.02f, 0.04f };
              readINA3221Data(shuntVoltage, busVoltage, current, shuntResistance);

              for (int channel = 0; channel < 3; channel++) {
                client.print("Channel ");
                client.println(channel + 1);

                client.print("  Shunt Voltage: ");
                client.print(shuntVoltage[channel], 6);
                client.println(" V");

                client.print("  Bus Voltage: ");
                client.print(busVoltage[channel], 3);
                client.println(" V");

                client.print("  Current: ");
                client.print(current[channel], 6);
                client.println(" A");

                client.println();
              }
              client.println("{\"status\":\"reach ok\", \"ip\":\"" + WiFi.localIP().toString() + "\", \"mac\":\"" + String(WiFi.macAddress()) + "\", \"m1_current\":" + M1_current + ", \"m2_current\":" + M2_current + ", \"m1_target\":" + M1_target + ", \"m2_target\":" + M2_target + ", \"m1_power\": " + M1_power + ", \"m2_power\": " + M2_power + ", \"m1_error\": " + M1_error + ", \"m2_error\": " + M1_error + ", \"battery\": " + getBattery() + ", \"pause\":" + (pause_bot ? String("true") : String("false")) + "}");
            } else {
              Serial.println("error requested");
              client.println("{\"status\":\"error\", \"ip\":\"" + WiFi.localIP().toString() + "\", \"mac\":\"" + String(WiFi.macAddress()) + "\", \"m1_current\":" + M1_current + ", \"m2_current\":" + M2_current + ", \"m1_target\":" + M1_target + ", \"m2_target\":" + M2_target + ", \"m1_power\": " + M1_power + ", \"m2_power\": " + M2_power + ", \"m1_error\": " + M1_error + ", \"m2_error\": " + M1_error + ", \"battery\": " + getBattery() + ", \"pause\":" + (pause_bot ? String("true") : String("false")) + "}");
            }
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else {  // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }

  int32_t tick_ms = millis();
  if (tick_ms >= lastTick_ms + 100) {
    lastTick_ms += 100;
    int16_t raw_shunt[3], raw_bus[3];
    readINA3221DataRaw(raw_shunt, raw_bus);
    power_counter++;
    power_counter %= record_number;
    PowerData[power_counter].VBUS_BUS = raw_bus[1];
    PowerData[power_counter].VBUS_TOP = raw_bus[2];
    PowerData[power_counter].VBUS_BOT = raw_bus[0];
    PowerData[power_counter].VSHUNT_BUS = raw_shunt[1];
    PowerData[power_counter].VSHUNT_TOP = raw_shunt[2];
    PowerData[power_counter].VSHUNT_BOT = raw_shunt[0];
  }
  int32_t currentTime = tick_ms - startTime;
  double t = (double)currentTime / (totalTime + 1000);
  if (totalTime == -1) {
    t = 1.0;
  }

  if (t <= 1.0 && t >= 0 && startTime >= 0) {
    static int32_t lastM1Position = 0;
    static int32_t lastM2Position = 0;
    static double M1_velocity, M2_velocity;
    static uint32_t lastVelocityUpdateTime = 0;
    const uint32_t VELOCITY_UPDATE_INTERVAL = 20;  // 20ms

    // 计算速度
    volatile uint32_t currentMillis = millis();
    if (currentMillis - lastVelocityUpdateTime >= VELOCITY_UPDATE_INTERVAL) {
      // 计算期望位置
      M1_setpoint = segmentedFunction(currentTime, M1_trajectoryParameters);
      M2_setpoint = segmentedFunction(currentTime, M2_trajectoryParameters);

      if (isnan(M1_setpoint) || isnan(M2_setpoint)) {
        SimpleParm M1_control = simpleControl(currentTime, M1_trajectoryParameters);
        M1_power = M1_control.power;
        M1_target = M1_control.target;
        SimpleParm M2_control = simpleControl(currentTime, M2_trajectoryParameters);
        M2_power = M2_control.power;
        M2_target = M2_control.target;
        motorSimple();
        goto bypassPID;
      }
      
      static double deltaTime = (currentMillis - lastVelocityUpdateTime) / 1000.0f;  // 转换为秒
      M1_velocity = (M1_current - lastM1Position) / deltaTime;
      M2_velocity = (M2_current - lastM2Position) / deltaTime;

      // 更新上一次的值
      lastM1Position = M1_current;
      lastM2Position = M2_current;
      lastVelocityUpdateTime = currentMillis;

      static double M1_derivative = segmentedFunction(currentTime + 1, M1_trajectoryParameters) - M1_setpoint;
      static double M2_derivative = segmentedFunction(currentTime + 1, M2_trajectoryParameters) - M2_setpoint;

      // 动态调整PID
      if (M1_derivative < 0) {
        M1_Kp = 5.45428968775176 - 3.65416275758547 * M1_derivative - 0.17101356497169 * M1_derivative * M1_derivative;
        M1_Ki = 3.62045028936399 - 0.566288451562346 * M1_derivative - 0.0205751960059309 * M1_derivative * M1_derivative;
        M1_velocity_Kp = 0.0147461576950953 - 0.00256744296420574 * M1_derivative - 0.000136757136602617 * M1_derivative * M1_derivative;
      } else {
        M1_Kp = 10.0;
        M1_Ki = 5.0;
        M1_velocity_Kp = 0.025;
      }
      if (M2_derivative < 0) {
        M2_Kp = 5.45428968775176 - 3.65416275758547 * M2_derivative - 0.17101356497169 * M2_derivative * M2_derivative;
        M2_Ki = 3.62045028936399 - 0.566288451562346 * M2_derivative - 0.0205751960059309 * M2_derivative * M2_derivative;
        M2_velocity_Kp = 0.0147461576950953 - 0.00256744296420574 * M2_derivative - 0.000136757136602617 * M2_derivative * M2_derivative;
      } else {
        M2_Kp = 14.0;
        M2_Ki = 6.0;
        M2_velocity_Kp = 0.025;
      }
      M1_PID.SetTunings(M1_Kp, M1_Ki, M1_Kd);
      M2_PID.SetTunings(M2_Kp, M2_Ki, M2_Kd);
      M1_velocity_PID.SetTunings(M1_velocity_Kp, M1_velocity_Ki, M1_velocity_Kd);
      M2_velocity_PID.SetTunings(M2_velocity_Kp, M2_velocity_Ki, M2_velocity_Kd);

      // 位置环（外环）
      M1_input = M1_current;
      M2_input = M2_current;
      M1_PID.Compute();
      M2_PID.Compute();

      // 使用位置 PID 输出作为速度设定点
      M1_velocity_setpoint = M1_output;
      M2_velocity_setpoint = M2_output;

      // 速度环（内环）
      M1_velocity_input = M1_velocity;
      M2_velocity_input = M2_velocity;
      M1_velocity_PID.Compute();
      M2_velocity_PID.Compute();

      // 使用速度 PID 输出控制电机
      motorControl();
      bypassPID:;
    }

  } else {
    // 轨迹结束，停止电机
    ledcWrite(M1A, 0);
    ledcWrite(M1B, 0);
    ledcWrite(M2A, 0);
    ledcWrite(M2B, 0);
    startTime = -2147483648;
    pause_bot = 0;
  }
}

// 初始化INA3221
void initINA3221() {
  Wire.beginTransmission(INA3221_ADDR);
  Wire.write(INA3221_CONFIG_REG);
  // 配置寄存器: 使能所有三个通道, 13位ADC分辨率, 连续转换模式
  Wire.write(0b0111010100100111);  // 16个采样平均，其他默认设置
  // Wire.write(0x00);
  Wire.endTransmission();
}

// 读取INA3221数据
void readINA3221DataRaw(int16_t raw_shunt[3], int16_t raw_bus[3]) {
  for (int channel = 0; channel < 3; channel++) {
    // 读取分流电压
    Wire.beginTransmission(INA3221_ADDR);
    Wire.write(INA3221_SHUNT_VOLTAGE_CH1 + channel * 2);
    Wire.endTransmission();

    Wire.requestFrom(INA3221_ADDR, 2);
    raw_shunt[channel] = Wire.read() << 8 | Wire.read();

    // 读取总线电压
    Wire.beginTransmission(INA3221_ADDR);
    Wire.write(INA3221_BUS_VOLTAGE_CH1 + channel * 2);
    Wire.endTransmission();

    Wire.requestFrom(INA3221_ADDR, 2);
    raw_bus[channel] = Wire.read() << 8 | Wire.read();
  }
}

// 读取INA3221数据
void readINA3221Data(float shuntVoltage[3], float busVoltage[3], float current[3], float shuntResistance[3]) {
  for (int channel = 0; channel < 3; channel++) {
    // 读取分流电压
    Wire.beginTransmission(INA3221_ADDR);
    Wire.write(INA3221_SHUNT_VOLTAGE_CH1 + channel * 2);
    Wire.endTransmission();

    Wire.requestFrom(INA3221_ADDR, 2);
    int16_t raw_shunt = Wire.read() << 8 | Wire.read();
    shuntVoltage[channel] = (float)(raw_shunt >> 3) * 0.000040f;  // 40.0µV/LSB

    // 读取总线电压
    Wire.beginTransmission(INA3221_ADDR);
    Wire.write(INA3221_BUS_VOLTAGE_CH1 + channel * 2);
    Wire.endTransmission();

    Wire.requestFrom(INA3221_ADDR, 2);
    int16_t raw_bus = Wire.read() << 8 | Wire.read();
    busVoltage[channel] = (float)(raw_bus >> 3) * 0.008f;  // 8.0mV/LSB

    // 计算电流
    current[channel] = shuntVoltage[channel] / shuntResistance[channel];
  }
}
