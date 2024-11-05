#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>  // 添加 MQTT 库

#define LED_PIN 48
#define NUM_LEDS 1
#define BUTTON_PIN 0
#define ADC_PIN 4  // ADC引脚定义

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// 新增变量用于存储临时参数
char newSsid[32];
char newPassword[64];
char newMqttServer[32];
char newDeviceId[32];


// WiFi 配置
char ssid[32] = "TS-MES";  // 替换为你的WiFi SSID
char password[64] = "ts888888";  // 替换为你的WiFi密码

// MQTT 服务器配置
char mqtt_server[32] = "60.204.251.153";  // 替换为你的MQTT服务器的IP
const int mqtt_port = 1883;  // MQTT服务器端口


WiFiClient espClient;
PubSubClient client(espClient);  // 创建PubSubClient实例

const float referenceVoltage = 1;  // 设置的参考电压阈值

int ele_run = 0;  // 动作电压计数变量
int adcValue;
float voltage;

int prev_ele_run = 0;  // 记录前一个ele_run的值
int run_flag = 1;      // 初始化时为运行状态
unsigned long lastCheckTime = 0; // 上次检查ele_run变化的时间戳

char deviceId[32] = "ESP32_009";  // 设备ID

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//新增一个远程升级服务器/WIFI连接功能
void callback(char* topic, byte* message, unsigned int length) {
  char incomingMessage[length + 1];
  memcpy(incomingMessage, message, length);
  incomingMessage[length] = '\0';  // 转换为字符串

  // 解析接收到的 JSON 数据
  StaticJsonDocument<200> doc;
  deserializeJson(doc, incomingMessage);

  // 如果收到的是配置更新消息
  if (strcmp(topic, "device/config") == 0) {
    // 获取新的 WiFi、MQTT 参数
    const char* tempSsid = doc["ssid"];
    const char* tempPassword = doc["password"];
    const char* tempMqttServer = doc["mqtt_server"];
    const char* tempDeviceId = doc["deviceId"];

    // 临时存储新参数
    strcpy(newSsid, tempSsid);
    strcpy(newPassword, tempPassword);
    strcpy(newMqttServer, tempMqttServer);
    strcpy(newDeviceId, tempDeviceId);

    // 将接收到的参数发送回 MQTT 服务器，等待确认
    StaticJsonDocument<200> returnDoc;
    returnDoc["ssid"] = newSsid;
    returnDoc["password"] = newPassword;
    returnDoc["mqtt_server"] = newMqttServer;
    returnDoc["deviceId"] = newDeviceId;

    char buffer[200];
    serializeJson(returnDoc, buffer);
    
    // 发布到返回参数的主题
    client.publish("device/config_response", buffer);
  }

  // 如果收到确认消息 "OK"
  if (strcmp(topic, "device/config_confirmation") == 0 && strcmp(incomingMessage, "OK") == 0) {
    // 确认无误，应用新配置
    Serial.println("Configuration confirmed, applying changes...");
    
    // 覆盖新的 WiFi 和 MQTT 配置
    strcpy(ssid, newSsid);
    strcpy(password, newPassword);
    strcpy(mqtt_server, newMqttServer);
    strcpy(deviceId, newDeviceId);

    // 重新启动 WiFi 和 MQTT 连接
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);

    Serial.println("New configuration applied.");
  }
}

void reconnect() {
  // 重连 MQTT 服务器
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {  // 尝试连接 MQTT
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void sendMQTTData() {
  // 创建JSON对象
  StaticJsonDocument<200> doc;
  
  // 添加设备ID、冲压次数和run_flag
  doc["deviceId"] = deviceId;
  doc["ele_run"] = ele_run;
  doc["run_flag"] = run_flag;

  // 序列化JSON对象为字符串
  char buffer[200];
  serializeJson(doc, buffer);

  // 发布JSON数据到MQTT主题
  client.publish("device/ele_run", buffer);
}

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

int detectLowVoltageTwice() {
  int lowVoltageCount = 0;  // 低电压计数
  bool isLow = false;  // 记录电压是否为低电压状态

  while (lowVoltageCount < 2) {
    adcValue = analogRead(ADC_PIN);  // 读取ADC值
    voltage = (adcValue / 4095.0) * 3.3;  // 将ADC值转换为电压

    // 如果电压低于参考电压，并且之前是高电压
    if (voltage < referenceVoltage && !isLow) {
      lowVoltageCount++;  // 记录低电压
      isLow = true;  // 标记为低电压状态
      Serial.print("Detected low voltage. Count: ");
      Serial.println(lowVoltageCount);
    }

    // 如果电压恢复高电压状态
    if (voltage >= referenceVoltage && isLow) {
      isLow = false;  // 重置为高电压状态，准备检测下一个低电压
    }

    delay(1);  // 添加延时，避免频繁读取造成误差
  }

  // 检测到两次低电压后，返回
  return lowVoltageCount;
}

void setup() {
  Serial.begin(115200);  // 初始化串口监视器
  setup_wifi();  // 连接 WiFi
  client.setServer(mqtt_server, mqtt_port);  // 设置 MQTT 服务器
  client.setCallback(callback);  // 设置回调

  analogReadResolution(12);  // 设置ADC分辨率为12位（0-4095）
  analogSetAttenuation(ADC_11db);  // 设置ADC衰减，测量范围为0-3.3V

  // 输出初始化信息到串口监视器
  Serial.println("System initialized. Ready to read ADC values...");

  setColor(0, 255, 255);  // 蓝色LED
  delay(200);  // 延迟1秒
  setColor(255, 0, 0);   // 红色LED
  delay(200);  // 延迟1秒
  setColor(255, 255, 0);   // 黄色LED
  delay(200);  // 延迟1秒
}

void loop() {
  if (!client.connected()) {
    reconnect();  // 如果未连接MQTT服务器，尝试重连
  }
  client.loop();  // 保持客户端连接

  setColor(0, 255, 255);  // 蓝色LED，表示开始检测

  // 检测低电压两次
  detectLowVoltageTwice();

  // 完成两次低电压检测后，执行其他逻辑
  ele_run++;  // 增加计数
  Serial.print("Voltage cycle detected, ele_run: ");
  Serial.println(ele_run);

  // 发送数据到MQTT
  sendMQTTData();

  setColor(255, 255, 0);  // 黄色LED，表示动作完成

  // 每2秒检查ele_run值是否变化
  if (millis() - lastCheckTime >= 2000) {
    if (ele_run == prev_ele_run) {
      run_flag = 0;  // 待机状态
    } else {
      run_flag = 1;  // 运行状态
    }

    prev_ele_run = ele_run;  // 更新上次记录的ele_run值
    lastCheckTime = millis();  // 更新检查时间

    // 再次发送数据到MQTT
    sendMQTTData();
  }
}
