/**
 ******************************************************************************
 * @file           : demo_final.ino
 * @brief          : 远程OTA更新、MQTT通信及电压监测功能
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 工业物联网团队.
 * 保留所有权利。</center></h2>
 *
 * 本软件组件由工业物联网团队根据BSD 3-Clause许可协议授权，
 * 您只能在遵守许可协议的情况下使用此文件。
 * 您可以在以下网址获取许可协议：
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 * @file           : demo_final.ino（code 2.0）
 * @author         : tianzhun
 * @brief          : 该程序基于ESP32微控制器，实现通过MQTT协议接收远程固件，并执行OTA更新，同时进行电压监测。
 *
 * @details        : 主要功能包括：
 *                   - WiFi连接
 *                   - MQTT通信，用于数据上传和接收固件更新指令
 *                   - OTA更新，通过fireware主题接收固件数据，并写入Flash完成固件更新
 *                   - 分块传输机制，确保大文件固件的稳定更新(共享RAM和共享固件)
 *                   - 电压监测，通过ADC接口实时采集设备电压状态，检测高低电平并上传至服务器
 *                   - 重试机制，确保OTA更新的成功率
 *
 * @note           : OTA固件更新采用MQTT通信方式，设备接收指定的固件大小，并根据固件数据包依次写入，
 *                   完成后进行校验并重启以激活新固件。支持对单设备或所有设备的批量更新。
 *                   同时，设备进行电压监测，定期上传电压状态，并进行相应动作。
 *
 *******************************************************************************
 */
#include "esp32-hal-timer.h" // 确保包含正确的定时器库
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Update.h> // OTA 更新库
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_ota_ops.h"      // 用于获取 OTA 操作
#include "esp_partition.h"    // 用于访问分区表
#include "light.h"

// 设置 CLK 和 DIO 引脚
#define CLK 4
#define DIO 5

#define LED_PIN 8 // （S3-C3需变化）
#define NUM_LEDS 1
#define ADC_PIN 2 // ADC引脚定义（S3-C3需变化）
#define tiaoshi 1

// 创建 LightDisplay 对象
LightDisplay lightDisplay(CLK, DIO);
// 任务队列
TaskHandle_t task1Handle = NULL; // 任务1（电压检测）句柄
TaskHandle_t task2Handle = NULL; // 任务2（OTA更新）句柄
TaskHandle_t task3Handle = NULL; // 任务3（数据上传）句柄

// 监测电压相关变量
int adcValue = 0;
float voltage = (adcValue / 4095.0) * 3.3;

// 定时器相关变量
hw_timer_t *task3Timer = NULL;                             // 定时器句柄
portMUX_TYPE task3TimerMux = portMUX_INITIALIZER_UNLOCKED;  // 定时器互斥锁

int ele_run = 0;                    // 冲压动作计数
int ele_run_i = 0;                  // 电压动作计数
int ele_run_n = 1;                  // 电压触发值+
int prev_ele_run = 0;               // 记录前一个ele_run的值   （S3-C3需变化）
int run_flag = 0;                   // 初始化为运行状态
const float referenceVoltage = 1.5; // 设置的参考电压阈值
// OTA更新相关变量 
unsigned long last_sendtime = 0;
unsigned long lastCheckTime = 0; // 上次检查ele_run变化的时间戳
bool otaInProgress = false;      // 标记是否处于 OTA 更新状态]
// 待机状态变量
int runtime = 10000;
// 发送标志位
bool send_flag = false;
// 当前开发板的设备ID
const char *deviceId = "ESP32_015"; // 每块开发板的独特设备ID（如 ESP32_001 到 ESP32_100）  64往前烧
// WiFi及MQTT服务器配置  （不同工厂需变化）
char ssid[32] = "TS-MES";
char password[64] = "ts888888";
//char mqtt_server[32] = "60.204.251.153";
char mqtt_server[32] = "192.168.19.1";
const int mqtt_port = 1883;
// 相关初始化
/// LED 灯配置
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
WiFiClient espClient;
PubSubClient client(espClient);
const char *commandTopic = "device/command";   // 指令和固件更新主题
const char *dataTopic = "device/ele_run";      // 数据上传主题
const char *debugTopic = "device/debug";       // 调试反馈主题
const char *firmwareTopic     = "device/firmware"; // 固件更新主题

// 固件传输数据
unsigned long totalBytesReceived = 0;
unsigned long totalBytesExpected = 0; // 动态设置接收的固件大小
const int bufferSize = 4096;          // OTA数据缓冲区大小
byte buffer[bufferSize];              // OTA数据缓冲区
int bufferIndex = 0;                  // OTA数据缓冲区索引
// 函数声明
void setup_wifi();
void sendDebugMessage(const char *message,int runtime);
void handleOTAUpdate(byte *payload, unsigned int length);
void callback(char *topic, byte *payload, unsigned int length);
void reconnect();  
int detectHighVoltage();
void sendMQTTData();
void setColor(uint8_t r, uint8_t g, uint8_t b);
void test_runtime();
void check_run();
void Quest_1();
void Quest_2();
void setBufferSizeAndCheck(int size);
void restoreDefaultBufferSize();
void send_debug(const char *message);
void IRAM_ATTR onTask3Timer();
void stopTask3Timer();
void task1(void *pvParameters); // task函数为声明：Guru Meditation Error: Core panic'ed（空间未初始化，没分配空间执行任务）
void task2(void *pvParameters);
void task3(void *pvParameters);
void monitorTaskStack(const char *taskName);
void checkOTAPartitionSize();
/**************************************  主函数功能 实现 **************************************/

/**************************************  函数实现: setup() **************************************
 * 函数功能：
 * - 系统启动时，完成WiFi连接、MQTT服务器初始化和LED状态指示。
 * - 创建任务1和任务2，任务1用于电压检测，任务2用于OTA更新。
 * 输入：无
 * 返回：无
 **************************************************************************************************/

void setup()
{

    setColor(255, 255, 255); // 白色LED
    Serial.begin(115200);
    
    setup_wifi();
  
    client.setServer(mqtt_server, mqtt_port);
    
      // 确保MQTT连接成功
  while (!client.connected()) {
      reconnect();  // 重连MQTT
  }
    test_runtime();//阻塞式运行时间检测[开始时执行一次即可]
  
    // client.setCallback(callback);
    //   client.setBufferSize(8192);  #BUG将缓冲区大小设置大了无法正常发送数据，要填满才会发送，设置小了接收不了OTA的块

    adcValue = analogRead(ADC_PIN);      // 读取ADC值
    voltage = (adcValue / 4095.0) * 3.3; // 将ADC值转换为电压

    setColor(0, 255, 255); // 初始化成功 蓝色LED
    
    lightDisplay.begin(); // 初始化显示屏
    
    client.setBufferSize(1024);//设置发送缓存区
    
    // 检查系统堆内存
    Serial.printf("Free heap size before task creation: %u bytes\n", xPortGetFreeHeapSize());
    
  // 创建任务1（电压检测任务）
    xTaskCreate(
    task1,         // 任务函数
    "Task1",       // 任务名称
    40960,          // 堆栈大小（以字节为单位）
    NULL,          // 任务输入参数
    1,             // 任务优先级
    &task1Handle); // 任务句柄
 

    // 创建任务2（OTA更新任务）互斥，任务二执行，任务一三都需要停止

    xTaskCreate(
        task2,         // 任务函数
        "Task2",       // 任务名称
        51200,         // 堆栈大小
        NULL,          // 任务输入参数
        1,             // 任务优先级
        &task2Handle); // 任务句柄

    // 创建任务3（上传任务）
    xTaskCreate(
        task3,         // 任务函数
        "Task3",       // 任务名称
        40960,         // 堆栈大小（以字节为单位）
        NULL,          // 任务输入参数
        1,             // 任务优先级
        &task3Handle); // 任务句柄，用于管理任务暂停和恢复
}
/**************************************  函数实现: loop() **************************************
 * 函数功能：
 * - 主循环中保持MQTT连接，如果连接断开，重新连接MQTT服务器。
 * 输入：无
 * 返回：无
 **************************************************************************************************/
void loop()
{

   /* 
    if (!client.connected())
    {
        reconnect();
    }

    // client.loop();
   
     if (tiaoshi)
     {
         Quest_1();
     }
     else
     {
         Quest_2();
     }
     */
     
}
/**************************************  函数实现: task1() **************************************
 * 函数功能：
 * - 执行电压检测任务，持续监控ADC值并执行电压相关任务。
 * - 如果正在进行OTA更新，暂停电压检测任务。
 * 输入：void *pvParameters：任务的输入参数
 * 返回：无
 **************************************************************************************************/

void task1(void *pvParameters)
{

    setColor(255, 0, 0); // runtime函数检测时间中
    //test_runtime();
    sendDebugMessage("初始化成功", runtime);
    Serial.printf(" runtime是: %d\n", runtime);
    monitorTaskStack("Task1");

    while (1)
    {
        if (!otaInProgress)
        {
            Quest_1();                 // 执行电压检测任务
            monitorTaskStack("任务1"); // 检查任务1堆栈使用情况
        }
        else
        {
            vTaskSuspend(NULL); // 如果 OTA 更新进行中，暂停任务1
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // 延时避免任务过于频繁
        send_debug("任务一正在运行...");
    }
}
/**************************************  函数实现: task2() **************************************
 * 函数功能：
 * - 执行OTA更新任务，保持MQTT客户端的连接，并监听OTA更新指令。
 * - 当OTA更新开始时，暂停任务1，执行固件下载和更新，完成后恢复任务1。
 * 输入：void *pvParameters：任务的输入参数
 * 返回：无
 **************************************************************************************************/

void task2(void *pvParameters)
{
    while (1)
    {
        // 设置 OTA 更新回调函数
        client.setCallback(callback); // 里面已集成OTA更新功能
        client.loop();                // MQTT客户端保持连接
        monitorTaskStack("Task2");
        if (otaInProgress)
        {
            Serial.println("开始 OTA 更新...");
            vTaskSuspend(task1Handle);  // 暂停其他任务
            vTaskSuspend(task3Handle); // OTA 更新期间暂停任务3
            
            stopTask3Timer();          // 停止任务3的定时器
            
            Quest_2();                 // 执行 OTA 更新任务
            monitorTaskStack("任务2"); // 检查任务2堆栈使用情况
            // 判断固件是否烧录完毕再恢复任务1
            if (totalBytesReceived == totalBytesExpected)
            {
                restoreDefaultBufferSize(); // 恢复缓存区默认大小并清空缓存区

                String successMsg = String(deviceId) + "任务二执行 OTA 更新完成";
                send_debug(successMsg.c_str());

                vTaskResume(task3Handle); // OTA 更新完成后恢复任务3
                vTaskResume(task1Handle);
                otaInProgress = false;    // 结束 OTA 状态
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // 延时避免任务过于频繁
        Serial.printf("task2 正在运行...");
    }
}

/**************************************  函数实现: task3() **************************************
 * 函数功能：
 * - 初始化定时器并绑定中断处理函数。
 * - 定期检查任务3的栈空间使用情况并发送任务状态。
 * - 使用定时器定时执行相关功能（如调用 `check_run` 函数）。
 *
 * 函数细节：
 * - 初始化定时器，设置定时器频率为1MHz。
 * - 使用 `timerAttachInterrupt` 绑定中断函数。
 * - 每秒触发一次定时器告警并执行相关功能。
 * 
 * 输入：void *pvParameters：任务的输入参数
 * 返回：无
 **************************************************************************************************/

void task3(void *pvParameters)
{

 
        // 初始化定时器，设置频率为1 MHz
    task3Timer = timerBegin(1000000); // 只传入频率，单位为 Hz

    // 绑定中断处理函数
    timerAttachInterrupt(task3Timer, &onTask3Timer);

    // 设置定时器告警时间为1秒
    timerWrite(task3Timer, 2000000); // 1000000微秒 = 1秒

    // 设置定时器告警时间为1秒，并允许无限次重载
    timerAlarm(task3Timer, 2000000, true, 0); // 自动重载次数为0，表示无限次

    if (task3Timer != NULL)
    {
        Serial.println("任务3定时器初始化成功");
    }
    else
    {
        Serial.println("任务3定时器初始化失败");
    }

    while (1)
    {
        // 确保MQTT连接成功
    while (!client.connected()) {
        reconnect();  // 重连MQTT
    }
            if (send_flag == true)
            {
                check_run();
                send_flag = false;
            }
            
        monitorTaskStack("Task3");
        vTaskDelay(100 / portTICK_PERIOD_MS); // 延时避免任务过于频繁
    }
}

/**************************************  函数实现: Quest_1() **************************************
 * 函数功能：
 * - 任务1的主要逻辑，负责电压检测及相关的MQTT数据发送。
 * - 持续读取ADC值，检测电压高低，并上传电压检测结果。
 * 输入：无
 * 返回：无
 **************************************************************************************************/

void Quest_1()
{
    setColor(0, 255, 255); // 初始化成功 蓝色LED

    /*
    if (millis() - lastCheckTime >= runtime)
    {
        // 间隔runtime 检查runflag

        if (ele_run == prev_ele_run)
        {
            run_flag = 0; // 待机状态
        }
        else
        {
            run_flag = 1; // 运行状态
        }

        prev_ele_run = ele_run;   // 更新上次记录的ele_run值
        lastCheckTime = millis(); // 更新检查时间
    }

    if (millis() - last_sendtime >= 2000)
    {
        Serial.printf(" run_flag: %lu \n", run_flag);
        sendMQTTData();
    }
    check_run();*/
              // 确保MQTT连接成功
    while (!client.connected()) {
        reconnect();  // 重连MQTT
    }
    
    adcValue = analogRead(ADC_PIN);      // 读取ADC值
    voltage = (adcValue / 4095.0) * 3.3; // 将ADC值转换为电压
    if (voltage > referenceVoltage)
    {
        // sendMQTTData();
        // 检测一次高电平
        delay(10);
        adcValue = analogRead(ADC_PIN);      // 读取ADC值
        voltage = (adcValue / 4095.0) * 3.3; // 将ADC值转换为电压

        if (voltage > referenceVoltage)
        {
            // 检测二次高电平
            delay(10);
            adcValue = analogRead(ADC_PIN);      // 读取ADC值
            voltage = (adcValue / 4095.0) * 3.3; // 将ADC值转换为电压

            if (voltage > referenceVoltage)
            {
                // 检测第三次进入程序
                detectHighVoltage(); // 正常情况下进行电压检测并上传数据
                ele_run_i++;
                if (ele_run_i == ele_run_n)
                {
                    ele_run++;
                    ele_run_i = 0;
                    // 检测完毕
                    setColor(255, 255, 0); // 黄色LED
                    delay(50);
                }
            }
        }
    }
    send_debug("电压检测结束"); // 检测结束
    lightDisplay.showNumber(ele_run); // 显示数字 ele_run
    /*
      if (millis() - lastCheckTime >= runtime) {
      // 间隔runtime 检查runflag


         if (ele_run == prev_ele_run) {
           run_flag = 0;  // 待机状态

         } else {
           run_flag = 1;  // 运行状态

       }

      prev_ele_run = ele_run;  // 更新上次记录的ele_run值
      lastCheckTime = millis();  // 更新检查时间
      }

      if (millis() - last_sendtime >= 2000)
      {
      Serial.printf(" run_flag: %lu \n", run_flag);
      sendMQTTData();
      }

      Serial.printf("ele_RUN: %lu \n",ele_run);
      Serial.printf(" run_flag: %lu \n", run_flag);
    // 封装
    check_run();
    */
}
/**************************************  函数实现: Quest_2() **************************************
 * 函数功能：
 * - 任务2的主要逻辑，负责执行OTA更新。
 * - 设置缓存区大小为40KB，处理OTA更新，并在完成后恢复默认设置。
 * 输入：无
 * 返回：无
 **************************************************************************************************/

void Quest_2()
{
    // 内存设置
    setColor(0, 0, 255); // 初始化成功 绿色LED

    setBufferSizeAndCheck(16384); // 设置成16KB
}
/**************************************  各个函数  实现 **************************************/

/**************************************  函数实现: setup_wifi() **************************************
 * 函数功能：
 * - 连接WiFi网络，直到成功连接为止。
 * - 打印WiFi连接状态和本地IP地址。
 * 输入：无
 * 返回：无
 **************************************************************************************************/

void setup_wifi()
{
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

/**************************************  函数实现: sendDebugMessage() **************************************
 * 函数功能：
 * - 发送调试信息到MQTT服务器的debug主题。
 * - 使用格式化字符串，将消息和运行时间发送到服务器。
 * 输入：
 * - const char *message：调试信息
 * - int runtime：运行时间
 * 返回：无
 **************************************************************************************************/

void sendDebugMessage(const char *message, int runtime)
{
    char buffer[100];                                             // 创建一个足够大的缓冲区来存储格式化的字符串
    snprintf(buffer, sizeof(buffer), "%s: %d", message, runtime); // 使用 snprintf 构造包含 runtime 的字符串
    client.publish(debugTopic, buffer);                           // 发送格式化的字符串到 MQTT 的 debug 主题
    Serial.println(buffer);                                       // 在串口监视器中输出消息
}

/**************************************  函数实现: handleOTAUpdate() **************************************
 * 函数功能：
 * - 处理OTA更新过程中接收到的固件数据。
 * - 将数据写入更新缓冲区，最终完成更新并重新启动设备。
 * 输入：
 * - byte *payload：接收到的固件数据
 * - unsigned int length：接收到的数据长度
 * 返回：无
 **************************************************************************************************/

void handleOTAUpdate(byte *payload, unsigned int length)
{

    if (otaInProgress)
    {

        totalBytesReceived = 0;
        // bug缓存区内存错误：OTA更新，初始化缓存区
        bufferIndex = 0;                   // 缓冲区索引重置
        memset(buffer, 0, sizeof(buffer)); // 清空缓冲区，避免数据混乱

        Serial.println("开始 OTA 更新...");
        size_t freeSketchSpace = ESP.getFreeSketchSpace();
        Serial.printf("可用的闪存空间: %u 字节\n", freeSketchSpace);
        Serial.println(String("totalBytesExpected: ") + totalBytesExpected);
        checkOTAPartitionSize();
        if (!Update.begin(totalBytesExpected))
        {
            Update.printError(Serial);
            delay(1000);
            client.publish(debugTopic, "OTA 更新启动失败");
            return;
        }
    }

    // 将接收到的数据写入缓冲区
    for (unsigned int i = 0; i < length; i++)
    {
        buffer[bufferIndex++] = payload[i];

        // 如果缓冲区满了，写入 Update
        if (bufferIndex == bufferSize)
        {
            if (Update.write(buffer, bufferSize) != bufferSize)
            {
                Update.printError(Serial);
                client.publish(debugTopic, "OTA 写入失败");
                return;
            }

            bufferIndex = 0; // 清空缓冲区
        }
    }

    totalBytesReceived += length;
    Serial.printf("接收到固件数据: %d 字节, 总共接收: %d 字节\n", length, totalBytesReceived);

    // 如果固件接收完成，结束更新
    if (totalBytesReceived >= totalBytesExpected)
    {
        // 将剩余的未满缓冲区的数据写入
        if (bufferIndex > 0)
        {
            if (Update.write(buffer, bufferIndex) != bufferIndex)
            {
                Update.printError(Serial);
                client.publish(debugTopic, "OTA 写入失败");
                return;
            }
        }
        // bug,挂起信息+延时确保信息准确发送，加了之后行了一次，现在又不行了，终端又不显示了
        if (Update.end(true))
        {
            // 拼接设备ID和更新成功的消息
            String successMsg = String(deviceId) + " OTA 更新完成";

            // 发布更新成功信息到服务器，并判断是否成功发送
            if (client.publish(debugTopic, successMsg.c_str()))
            {
                Serial.println(successMsg);

                // 让 MQTT 客户端处理所有挂起的消息，确保它们被发送
                client.loop(); // 处理可能还没发出的消息
                delay(2000);   // 延迟2秒，确保消息传输时间充足

                // 发送成功后重启设备
                ESP.restart();
            }
            else
            {
                // 发送失败时打印错误信息
                Serial.println("更新成功消息发送失败，设备未重启");
            }
        }
        else
        {
            // 打印并发布更新失败信息
            Update.printError(Serial);
            client.publish(debugTopic, "OTA 更新失败");
        }
    }
}

/**************************************  函数实现: callback() **************************************
 * 函数功能：
 * - 处理接收到的MQTT消息，根据主题不同，执行不同的操作。
 * - 对firmwareTopic进行处理时，调用OTA更新函数。
 * 输入：
 * - char *topic：接收到的MQTT主题
 * - byte *payload：接收到的数据
 * - unsigned int length：接收到的数据长度
 * 返回：无
 **************************************************************************************************/

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.println("收到消息");
    Serial.println(topic);

    if (strcmp(topic, firmwareTopic) == 0)
    {
        if (otaInProgress == true)
        {
            Serial.println("收到固件数据...");
            handleOTAUpdate(payload, length); // 调用函数处理固件更新
        }
    }
    else if (strcmp(topic, commandTopic) == 0)
    {
        // 处理从 `commandTopic` 收到的指令，解析 JSON
        StaticJsonDocument<512> doc; // 使用 ArduinoJson 库解析 JSON
        DeserializationError error = deserializeJson(doc, payload, length);

        if (error)
        {
            Serial.print("JSON 解析错误: ");
            Serial.println(error.c_str());
            return;
        }

        const char *receivedDeviceId = doc["deviceId"];
        const char *receivedCommand = doc["command"];

        unsigned long receivedSize = doc["size"]; // 获取固件大小

        // 检查 deviceId 是否匹配，并判断是否为 OTA 更新指令
        if ((strcmp(receivedDeviceId, "ALL") == 0 || strcmp(receivedDeviceId, deviceId) == 0) && strcmp(receivedCommand, "0x01") == 0)
        {
            // 设置动态的固件大小
            totalBytesExpected = receivedSize;
           
            otaInProgress = true; // OTA更新开始
            Serial.println("收到 OTA 更新指令");
            if (client.publish(debugTopic, "0x01"))
            { // 向服务器发送确认消息
                Serial.println("确认消息发送成功: 0x01");
            }
            else
            {
                Serial.println("确认消息发送失败");
            }
        }
    }
}

/**************************************  函数实现: reconnect() **************************************
 * 函数功能：
 * - 尝试连接MQTT服务器，直到成功为止。
 * - 订阅所有必要的MQTT主题。
 * 输入：无
 * 返回：无
 **************************************************************************************************/

void reconnect()
{
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(deviceId))
        {
            Serial.println("connected");
            client.subscribe(commandTopic);  // 订阅指令主题
            client.subscribe(firmwareTopic); // 订阅固件更新主题
            client.subscribe(debugTopic);    // 订阅调试反馈主题
            client.subscribe(dataTopic);     // 添加：订阅数据上传主题
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" 5 秒后重试...");
            delay(5000);
        }
    }
}

/**************************************  函数实现: detectHighVoltage() **************************************
 * 函数功能：
 * - 检测高电压，判断电压是否超过设定阈值。
 * - 根据电压变化情况，返回检测结果。
 * 输入：无
 * 返回：
 * - int：返回检测结果，0表示电压未达到阈值，1表示电压超过阈值
 **************************************************************************************************/

int detectHighVoltage()
{
    setColor(255, 0, 255); // 进入检测 紫色LED
    int n = 0;
    /*
               if (millis() - lastCheckTime >= runtime) {
            // 间隔runtime 检查runflag


                  if (ele_run == prev_ele_run) {
                    run_flag = 0;  // 待机状态

                  } else {
                    run_flag = 1;  // 运行状态

                }

            prev_ele_run = ele_run;  // 更新上次记录的ele_run值
            lastCheckTime = millis();  // 更新检查时间
           }

        if (millis() - last_sendtime >= 2000)
          {
            Serial.printf(" run_flag: %lu \n", run_flag);
            sendMQTTData();
          }

    check_run(); // #避免延时影响发送数据
    */
    while (1)
    {
        for (int i = 0; i < 30; i++)
        {
            // 60ms周期内读取60次
            adcValue = analogRead(ADC_PIN);      // 读取ADC值
            voltage = (adcValue / 4095.0) * 3.3; // 将ADC值转换为电压

            if (referenceVoltage <= voltage)
                n++;
            delay(1);
        }

        Serial.printf(" n: %d \n", n);
        if (n == 0)
        {
            return 0;
        }
        n = 0;
        /*
        if (millis() - lastCheckTime >= runtime)
        {
            // 间隔runtime 检查runflag

            if (ele_run == prev_ele_run)
            {
                run_flag = 0; // 待机状态
            }
            else
            {
                run_flag = 1; // 运行状态
            }

            prev_ele_run = ele_run;   // 更新上次记录的ele_run值
            lastCheckTime = millis(); // 更新检查时间
        }

        if (millis() - last_sendtime >= 2000)
        {
            Serial.printf(" run_flag: %lu \n", run_flag);
            sendMQTTData();
        }

        check_run(); // #避免延时影响发送数据
        */
    } // while
}

// int adcValue = analogRead(ADC_PIN);
// float voltage = (adcValue / 4095.0) * 3.3;

// static bool isHigh = false;

// if (voltage > 3.0 && !isHigh) {  // 检测到高电压且之前为低电压
//     isHigh = true;
//     ele_run++;
//     sendMQTTData();  // 每次检测到高电压后发送数据
// }

// if (voltage <= 3.0 && isHigh) {  // 电压恢复到低电压状态
//     isHigh = false;
// }

/**************************************  函数实现: sendMQTTData() **************************************
 * 函数功能：
 * - 创建JSON对象，包含设备ID、冲压次数和运行状态。
 * - 将JSON对象序列化为字符串，发布到MQTT数据主题。
 * 输入：无
 * 返回：无
 **************************************************************************************************/
/*
void sendMQTTData()
{
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
    client.publish(dataTopic, buffer);
    
}
*/
void sendMQTTData() {
    // 使用 DynamicJsonDocument 动态分配内存
    DynamicJsonDocument doc(1024);  // 1KB 动态内存（根据实际需求调整大小）

    // 设置数据包格式 "type" 和 "sn"（相当于 ClientID）
    doc["type"] = "desc";
    doc["sn"] = deviceId;  // 使用 deviceId 作为 ClientID

    // 创建嵌套的 "info" 对象
    JsonObject info = doc.createNestedObject("info");

    // 创建 "JJ31_D1" 数组
    JsonArray JJ31_D1 = info.createNestedArray("JJ31_D1");

    // 添加各项数据
    JsonObject ioStatus = JJ31_D1.createNestedObject();
    ioStatus["id"] = "_io_status";
    ioStatus["desc"] = "设备状态";
    ioStatus["value"] = run_flag;  // 示例值，替换为实际值

    JsonObject deviceAlarm = JJ31_D1.createNestedObject();
    deviceAlarm["id"] = "Mitsubishi_Device_alarm";
    deviceAlarm["desc"] = "设备报警";
    deviceAlarm["value"] = 0;  // 示例值，替换为实际值

    // 添加更多字段，例如总生产件数、运行时间等
    JsonObject partsTotalCount = JJ31_D1.createNestedObject();
    partsTotalCount["id"] = "Mitsubishi_Parts_total_counts";
    partsTotalCount["desc"] = "累计生产件数";
    partsTotalCount["value"] = ele_run;  // 示例值，替换为实际值

    // 序列化JSON对象为字符串
    char buffer[512];
    serializeJson(doc, buffer);

    // 发布JSON数据到MQTT主题
    client.publish(dataTopic, buffer);
}




/**************************************  函数实现: setColor() **************************************
 * 函数功能：
 * - 设置LED的颜色，通过RGB值控制LED显示不同颜色。
 * 输入：
 * - uint8_t r：红色值
 * - uint8_t g：绿色值
 * - uint8_t b：蓝色值
 * 返回：无
 **************************************************************************************************/

void setColor(uint8_t r, uint8_t g, uint8_t b)
{
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
}
/**************************************  函数实现: test_runtime() **************************************
 * 函数功能：
 * - 通过两个ADC采样循环，检测设备的运行时间。
 * - 计算并设置检测到的运行时间，若小于2000ms，强制设为2000ms。
 * 输入：无
 * 返回：无
 **************************************************************************************************/

void test_runtime()
{
    // 第一个循环
    int i = 0;
    int pre_adcValue = analogRead(ADC_PIN);      // 读取ADC值
    int pre_voltage = (adcValue / 4095.0) * 3.3; // 将ADC值转换为电压
    int pre_state = 0;
    int test_state = 0;
    //需要做滤波处理嘛
    if (referenceVoltage <= pre_voltage)
    {
        pre_state = 1;
        test_state = 1;
    }
    while (pre_state == test_state)
    {
        // 读取ADC
        pre_adcValue = analogRead(ADC_PIN);          // 读取ADC值
        pre_voltage = (pre_adcValue / 4095.0) * 3.3; // 将ADC值转换为电压
        if (referenceVoltage <= pre_voltage)
        {
            test_state = 1;
        }
        else
            test_state = 0;
    }
    // 第二个循环 检测runtime
    setColor(0, 255, 0); // 绿灯_初始函数检测时间中
    i = 0;
    pre_adcValue = analogRead(ADC_PIN);          // 读取ADC值
    pre_voltage = (pre_adcValue / 4095.0) * 3.3; // 将ADC值转换为电压
    pre_state = 0;
    test_state = 0;

    if (referenceVoltage <= pre_voltage)
    {
        pre_state = 1;
        test_state = 1;
    }

    while (pre_state == test_state)
    {
        // 读取ADC
        adcValue = analogRead(ADC_PIN);          // 读取ADC值
        pre_voltage = (adcValue / 4095.0) * 3.3; // 将ADC值转换为电压
        if (referenceVoltage <= pre_voltage)
        {
            test_state = 1;
        }
        else
            test_state = 0;
        i++;
        delay(1);
    }
    runtime = i;
    if (runtime <= 2000)
    {
        runtime = 2000;
        ele_run++;
    }
    else
        runtime = runtime - runtime % 1000;
        ele_run++;
        
}
/**************************************  函数实现: check_run() **************************************
 * 函数功能：
 * - 每隔一定时间（runtime）检查电压状态，并根据变化更新运行标志。
 * - 检查时间间隔是否满足发送数据条件，满足时上传MQTT数据。
 * 输入：无
 * 返回：无
 **************************************************************************************************/

void check_run()
{
    if (millis() - lastCheckTime >= runtime)
    {
        // 间隔runtime 检查runflag

        if (ele_run == prev_ele_run)
        {
            run_flag = 0; // 待机状态
        }
        else
        {
            run_flag = 1; // 运行状态
        }

        prev_ele_run = ele_run;   // 更新上次记录的ele_run值
        lastCheckTime = millis(); // 更新检查时间
    }

    if (millis() - last_sendtime >= 2000)
    {
        Serial.printf(" run_flag: %lu \n", run_flag);
        sendMQTTData();
    }

    Serial.printf("ele_RUN: %lu \n", ele_run);
    
    Serial.printf(" run_flag: %lu \n", run_flag);
}

/**************************************  函数实现: monitorTaskStack() **************************************
 * 函数功能：
 * - 检查当前任务剩余的堆栈空间，输出堆栈剩余字节数。
 * 输入：
 * - const char *taskName：任务名称，用于输出时标识任务
 * 返回：无
 **************************************************************************************************/

void monitorTaskStack(const char *taskName)
{
    UBaseType_t stackLeft = uxTaskGetStackHighWaterMark(NULL); // 检查当前任务剩余栈空间
    size_t freeHeap = xPortGetFreeHeapSize();                  // 检查当前系统剩余堆内存
    Serial.printf("任务 %s 正在运行...\n", taskName);
    Serial.printf("任务 %s 剩余栈空间: %u 字节\n", taskName, stackLeft);
    Serial.printf("当前剩余堆内存: %u 字节\n", freeHeap);
}

/**************************************  函数实现: restoreDefaultBufferSize() **************************************
 * 函数功能：
 * - 恢复默认缓存区大小为256字节，重置缓冲区索引，并清空缓冲区数据。
 * 输入：无
 * 返回：无
 **************************************************************************************************/

void restoreDefaultBufferSize()
{
    setBufferSizeAndCheck(1024);        // 恢复默认缓存区大小为 256字节
    bufferIndex = 0;                   // 重置缓冲区索引
    memset(buffer, 0, sizeof(buffer)); // 清空缓冲区，避免数据残留
}

/**************************************  函数实现: setBufferSizeAndCheck() **************************************
 * 函数功能：
 * - 设置缓存区大小并检查是否设置成功。
 * - 打印缓存区大小设置的结果。
 * 输入：
 * - int size：要设置的缓存区大小
 * 返回：无
 **************************************************************************************************/

void setBufferSizeAndCheck(int size)
{
    client.setBufferSize(size);
    if (client.getBufferSize() == size)
    {
        Serial.printf("缓存区设置为 %d 字节成功...\n", size);
    }
    else
    {
        Serial.printf("缓存区设置失败，当前缓存区大小为: %d 字节...\n", client.getBufferSize());
    }
}
/**************************************  函数实现: send_debug() **************************************
 * 函数功能：
 * - 通过串口和 MQTT 同时发送反馈信息。
 * - 函数接收一个字符串作为参数，向指定的主题发布消息。
 * 输入：
 * - const char *message：要发送的字符串消息。
 * 返回：无
 **************************************************************************************************/
void send_debug(const char *message)
{
    // 通过串口发送调试信息
    Serial.println(message);

    // 通过 MQTT 发送信息到指定的主题
    client.publish("topic/feedback", message);
}

/**************************************  函数实现: onTask1Timer() **************************************
 * 函数功能：
 * - 定时器回调函数，用于定时调用 `check_run` 函数。
 *
 * 函数细节：
 * - 该函数由定时器中断触发，每秒调用一次。
 * - 使用 `portENTER_CRITICAL_ISR` 和 `portEXIT_CRITICAL_ISR` 确保定时器的互斥访问。
 * - 定时器触发时，直接执行 `check_run` 函数来检查电压并更新运行状态。
 *
 * 注意事项：
 * - 此函数为定时器中断回调，无需外部函数声明。
 * - 函数运行在中断上下文中，尽量减少处理时间。
 *
 * 输入：无
 * 返回：无
 **************************************************************************************************/
// 定时器回调函数（任务3专用）
void IRAM_ATTR onTask3Timer()
{
    send_flag = true;  // 定时器到时后触发发送标志位
}


/**************************************  函数实现: stopTask3Timer() **************************************
 * 函数功能：
 * - 停止并销毁定时器，释放定时器相关资源。
 *
 * 函数细节：
 * - 调用 `timerDetachInterrupt` 分离中断，并使用 `timerEnd` 停止定时器。
 * - 清空定时器指针，确保定时器不会再次使用。
 *
 * 注意事项：
 * - 在任务结束或不再需要定时器时调用，防止资源泄漏。
 *
 * 输入：无
 * 返回：无
 **************************************************************************************************/
void stopTask3Timer()
{
    if (task3Timer != NULL)
    {
        timerDetachInterrupt(task3Timer); // 分离中断
        timerEnd(task3Timer);             // 停止并销毁定时器
        task3Timer = NULL;                // 清空定时器指针
    }
}

void checkOTAPartitionSize() {
  // 获取下一个 OTA 更新分区的信息
  const esp_partition_t *otaPartition = esp_ota_get_next_update_partition(NULL);

  if (otaPartition != NULL) {
    Serial.printf("OTA 分区大小: %d 字节\n", otaPartition->size);
  } else {
    Serial.println("未找到 OTA 分区");
  }
}
