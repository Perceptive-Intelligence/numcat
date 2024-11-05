#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
/* S3
#define LED_PIN 48
#define NUM_LEDS 1
#define BUTTON_PIN 0
*/
//C3
#define LED_PIN 8 // 8号引脚用于控制灯
#define NUM_LEDS 1
#define BUTTON_PIN 9 // Boot按键对应9号引脚

/*下面是功能列举
 * 1.提示信息，通过LED指示灯颜色  
 * 2.WIFI连接，连接成功显示蓝色
 * 3.POST请求上传数据，上传成功显示红色
 * 4.USB HOST通过CDC或者其他方式对USB Device发送数据请求：十六进制的0xaa 0x01（固定两个字节进行数据请求）
 * 5.数据返回（固定五个字节的十六进制数据），通过解析函数对数据进行转换，并调用时间戳获取函数获取当前数据获取时间戳，与转换好的数据进行封装绑定成POST请求中的固定格式再进行POST请求上传
 * 6.boot函数，通过绑定固定引脚，当我们按下时可以对转换后的相关数据进行删除
 * 7.读取主机端发送的http的configure.json文件对WIFI，url等参数进行相关配置
 * 下面是流程列举
 * 总体流程：1.先根据板端初始的参数进行WIFI连接（读取板载configure.json文件）-2.数据请求发送-3.返回数据并进行转换-4.获取时间戳进行绑定-5.POST请求上传数据
 * 后续若是参数有变动，流程变为：1.原始WIFI连接-2.http发送configure.json进行更改配置-3.数据请求发送-4.返回数据并进行转换-5.获取时间戳进行绑定-5.POST请求上传数据
 * 测试流程列举：
 * 现在请给出单个模块的单独测试，比如说读取板载配置文件，先创建一个配置文件，然后进行根据下面的参数进行初始化，然后在终端进行打印，后续在这个上面再加上WIFI连接
 * 有关文件系统的暂时先别做，先把其他的代码逻辑进行仿照实现即可
 * */
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

const char* ssid = "tjj";
const char* password = "tjj445408";

String serverUrl = "http://192.168.43.106:8888//api/SpectroscopicArchives/ApplyArchives";
String deviceId = "your_device_id";
DynamicJsonDocument receiveData(1024);
IPAddress local_IP(192, 168, 43, 111);  // 设置为固定IP地址
IPAddress gateway(192, 168, 43, 1);     // 网关地址
IPAddress subnet(255, 255, 255, 0);     // 子网掩码
IPAddress dns(8, 8, 8, 8);              // 首选DNS

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  

  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected!");
  setColor(0,0 , 255);   // 绿色LED
    // WiFi连接成功后配置固定IP
  if (!WiFi.config(local_IP, gateway, subnet, dns)) {
    Serial.println("STA Failed to configure");
  }
  
  setColor(0, 255, 0); // 连接成功，点亮绿色LED
}

String getCurrentTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "";
  }
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  return String(buffer);
}

String getData(const byte* data) {
  int value = (data[2] << 8) | data[3];
  int sign = (data[4] >> 4) == 8 ? 1 : -1;
  int decimal_places = data[4] & 0x0F;
  float final_value = sign * value / pow(10, decimal_places);
  return String(final_value);
}

void processReceivedData(const byte* buffer) {
  String result = getData(buffer);
  String timestamp = getCurrentTime();
  receiveData.clear();
  receiveData["deviceId"] = deviceId;
  receiveData["data"].add(JsonObject());
  receiveData["data"][0]["result"] = result;
  receiveData["data"][0]["time"] = timestamp;
   // 打印准备上传的数据
  String payload;
  serializeJson(receiveData, payload);
  Serial.print("Payload to be sent: ");
  Serial.println(payload);
}

void sendPostRequest() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    String payload;
    serializeJson(receiveData, payload);

    int httpResponseCode = http.POST(payload);
    if (httpResponseCode == 200) {
      Serial.println("POST request successful");
      setColor(255, 255, 0);  // 200响应，点亮红色LED
      delay(500);
      setColor(0, 0, 0);  // 关闭LED
    } else {
      Serial.printf("POST request failed with status code: %d\n", httpResponseCode);
      setColor(0, 0, 255);  // 响应失败，点亮蓝色LED
    }
    http.end();
  } else {
    Serial.println("WiFi not connected");
  }
}

void setup() {
  Serial.begin(115200); // 在USB CDC On Boot使能（Enabled）情况下，这里是USBCDC
  Serial.setDebugOutput(true);

  // 初始化Adafruit NeoPixel
  strip.begin();
  strip.show(); // 初始化所有像素为'off'

  // 连接WiFi
  connectToWiFi();

  // 初始化按钮引脚
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // 设置时间服务器
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

void loop() {
  /*
setColor(255,0 , 0);   // 色LED
delay(1000);
setColor(0,255 , 0);   // 色LED
delay(1000);
setColor(0,0 , 255);   // 色LED
  delay(1000);
  */
  
  static unsigned long lastPostTime = 0;
  unsigned long currentTime = millis();

  // 每隔3秒发送一次POST请求
  if (currentTime - lastPostTime >= 3000) {
    lastPostTime = currentTime;

    byte data[] = {0xAA, 0x01}; // 定义要发送的十六进制数据
    Serial.write(data, sizeof(data)); // 发送十六进制数据
    //Serial.write("hellow esp32");

    // 检查是否有可用的返回数据
    if (Serial.available() >= 5) { // 等待5个字节的数据
      byte response[5];
      Serial.readBytes(response, 5); // 读取5个字节的数据

      // 检查返回数据是否符合预期
      if (response[0] == 0xAA && response[1] == 0x01) { // 根据您的预期数据检查条件
        setColor(0, 0, 255);  // 点亮蓝色LED
        processReceivedData(response);
        sendPostRequest();
      } else {
        setColor(0, 255, 255);   // 蓝色LED
      }
    }
  }

  // 按钮删除数据
  if (digitalRead(BUTTON_PIN) == LOW) {
    while (digitalRead(BUTTON_PIN) == LOW) {
      delay(10); // 按钮防抖
    }
   // 删除数据操作
    if (receiveData.containsKey("data") && !receiveData["data"].isNull() && receiveData["data"].size() > 0) {
      receiveData["data"].remove(receiveData["data"].size() - 1); // 删除末尾的数据项
      Serial.println("\nData deleted");
    }
    setColor(255, 255, 255); // 连接成功，点亮白色LED
  }
}
    
