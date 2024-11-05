// light.h
#ifndef LIGHT_H
#define LIGHT_H

#include <Arduino.h>
#include <TM1637TinyDisplay6.h>

// 定义一个类来封装 TM1637 的显示功能
class LightDisplay {
public:
    LightDisplay(uint8_t clk, uint8_t dio);
    void begin();                // 初始化显示
    void showNumber(int number); // 显示数字

private:
    TM1637TinyDisplay6 display;  // TM1637 显示屏实例
};

#endif
