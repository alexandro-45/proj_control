#ifndef STM32_LIB_LCD_H
#define STM32_LIB_LCD_H

void lcd_send_cmd(char cmd);

void lcd_send_data(char data);

void lcd_clear(void);

void lcd_put_cur(int row, int col);

void lcd_init(void);

void lcd_send_string(char *str);

#endif //STM32_LIB_LCD_H
