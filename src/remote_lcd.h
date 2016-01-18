#ifndef _REMOTE_LCD_H
#define _REMOTE_LCD_H

void lcd_reset();
void lcd_init();
void lcd_contrast(uint8_t);
void lcd_clear();
void lcd_update();
void lcd_putchar(char character, uint8_t x, uint8_t y);
void lcd_puts(char *str, uint8_t x, uint8_t y);
void lcd_setpos(uint8_t x, uint8_t y);

#endif

