// OLED.h

#ifndef OLED_H
#define OLED_H

#include <U8x8lib.h>

// 引脚定义
#define OLED_MOSI   4  // SDA (MOSI)
#define OLED_CLK    5  // SCL (SCK)
#define OLED_DC     6  // Data/Command
#define OLED_RESET  7  // Reset
#define OLED_CS     18 // Chip Select

// 初始化 SSD1309 显示屏，使用软件SPI
extern U8X8_SSD1309_128X64_NONAME0_4W_SW_SPI u8x8;

// OLED 初始化函数声明
void OLED_init(void);

// OLED 显示函数声明
void OLED_show(int ele_run);

#endif
