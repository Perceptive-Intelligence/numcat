// light.cpp
#include "light.h"

// 构造函数，用于初始化显示模块的 CLK 和 DIO 引脚
LightDisplay::LightDisplay(uint8_t clk, uint8_t dio) : display(clk, dio) {}

// 初始化显示屏
void LightDisplay::begin() {
    display.begin();
    display.setBrightness(BRIGHT_HIGH);
}

// 显示数字
void LightDisplay::showNumber(int number) {
    display.clear();            // 清空显示
    display.showNumber(number); // 显示数字
}
