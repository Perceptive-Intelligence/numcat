// OLED.cpp

#include "OLED.h"

// 创建 U8x8 对象
U8X8_SSD1309_128X64_NONAME0_4W_SW_SPI u8x8(OLED_CLK, OLED_MOSI, OLED_CS, OLED_DC, OLED_RESET);

// OLED 初始化函数
void OLED_init(void) {
  u8x8.begin();
  u8x8.setPowerSave(0);  // 打开显示屏电源
}

// OLED 显示函数，接受一个整数并显示字符串+变量值
void OLED_show(int ele_run) {
  char buffer[20];  // 创建一个足够大的缓冲区存储整行内容
  snprintf(buffer, sizeof(buffer), "work: %d", ele_run);  // 将字符串和变量值拼接成完整的字符串
  
  u8x8.setFont(u8x8_font_chroma48medium8_r);  // 设置字体
  u8x8.clearLine(1);  // 清除第1行，避免残留内容
  u8x8.drawString(0, 1, buffer);  // 在第1行打印完整的字符串
}
