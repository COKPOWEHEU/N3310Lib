/*
 * Описание     :  Это драйвер для графического LCD от Nokia 5110, а также его китайских клонов.
 * Автор        :  Xander Gresolio <xugres@gmail.com>
 * Лицензия     :  GPL v3.0
 *
 * Доработка:
 *     - kobzar aka kobraz для http://cxem.net/ maodzedun@gmail.com
 *     - Alex для http://forum.cxem.net/
 *     - COKPOWEHEU для http://forum.cxem.net/
 */
/*
 * Description  :  Nokia 5110 LCD driver
 * Authors      :  Xander Gresolio <xugres@gmail.com>
 *                 kobzar aka kobraz for http://cxem.net/ maodzedun@gmail.com
 *                 Alex for http://forum.cxem.net/
 *                 COKPOWEHEU for http://forum.cxem.net/
 * License      :  GPL v3.0
 */
/************************************************************************************/
#ifndef __N5110_H__
#define __N5110_H__

#include <inttypes.h>

// закомментируйте эту директиву, если ваш дисплей оригинальный
//#define CHINA_LCD

//использование аппаратного SPI.
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// If you will use this option - be careful! Lines of hardware SPI must be the same with
// their description in settings
// Если будете использовать - внимательно проверьте соответствие назначенных ниже выводов
// реальным линиям SPI в контроллере
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//#define LCD_HARDWARE_SPI

//usage full CP1251 table
//хранение всех 256 символов таблицы CP1251
#define FULL_CP1251_TABLE

//usage tiny 3x5 font for numbers
//использование маленького шрифта 3х5, только для цифр
#define LCD_FONT3x5

//disabling function working with float numbers (recommends)
//отключение нерекомендуемой функции работы с плавающей точкой
#define DISABLE_DOUBLE
//отключение работы с графикой (точки, линии и прочее)
#define DISABLE_GRAPHICS

//update screen method
//  LCD_CONTINUOUS - 1 byte per 1 call Lcd_update() function. It is smaller and process load is more uniform
//  LCD_DIRECT - all screen updates by 1 call Lcd_update() function
//способ обновления экрана (можно выбрать что-то одно):
//  LCD_CONTINUOUS - один байт за вызов Lcd_update. Занимает меньше места, и расход скорости более равномерный, но требует вызывать Lcd_update постоянно.
//  LCD_DIRECT - обновление всей изменившейся области за один вызов Lcd_update. Занимает больше места, но вызывать Lcd_update надо только один раз для отрисовки экрана
#define LCD_CONTINUOUS
//#define LCD_DIRECT

// Ports binding
// syntax: "PORT_NAME, BIT_NUM". Example : "B, 1" - PB1
// Назначение портов для дисплея.
// Синтаксис: "PORT_NAME, BIT_NUM". Пример : "B, 1" - вывод будет задействован на PB1
#define LCD_DATA	B, 3	// SDIN / MOSI
#define LCD_CLK		B, 5	// SCLK / SCK
#define LCD_DC		B, 1	// DC
#define LCD_CE		B, 2	// SCE
#define LCD_RST		B, 0	// RESET

#define LCD_RESOLUTION	84,48

#define LCD_OK		0
#define LCD_OUT		-1
#define LCD_OK_WRAP	1

#define PIXEL_OFF	0   // Погасить пиксели дисплея
#define PIXEL_ON	1   // Включить пиксели дисплея
#define PIXEL_XOR	2   // Инвертировать пиксели

void Lcd_init();   // Инициализация
void Lcd_clear();   // Очистка буфера
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
extern volatile unsigned char lcd_fontsize; //размер шрифта, 1 или 2 //font size, 1 or 2
extern volatile unsigned char lcdtfdn; //lcd_tiny_font_down - сдвиг шрифта 3х5 относительно обычного
inline void LcdSize(unsigned char size){lcd_fontsize = size;}
inline void Lcd3x5shift(unsigned char sh){lcdtfdn = sh;}
//обновление экрана. См. режимы работы
void Lcd_update();
void LcdContrast( uint8_t contrast );   // Установка контрастности дисплея
int8_t LcdGotoXY( uint8_t x, uint8_t y );   // Установка текстового курсора в позицию x,y
int8_t LcdChr( char ch );   // Вывод символа в текущей позиции
int8_t Lcd_puts( const char *str );   // Вывод строки из переменной
int8_t Lcd_puts_P( const char *str);   // Вывод статичной строки 
#ifndef DISABLE_DOUBLE
//lcd_put_double - вывод числа с плавающей точкой. Не рекомендуется, потому что медленно
void Lcd_putd( double data, int accuracy );
#endif
//lcd_put_fixed_point - вывод числа с фиксированной точкой
void Lcd_putf( int32_t value, int8_t dot);
int8_t Lcd_pixel( uint8_t x, uint8_t y, uint8_t color );
int8_t Lcd_line( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
int8_t Lcd_circle( uint8_t x, uint8_t y, uint8_t radius, uint8_t color);
int8_t Lcd_rect_empty( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
int8_t Lcd_rect( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);

#endif
