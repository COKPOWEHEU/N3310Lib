#include "n5110.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdlib.h>
#include "pinmacro.h"

#ifndef LCD_CONTINUOUS
#ifndef LCD_DIRECT
#error "Select either continous or direct mode"
#endif
#elif defined LCD_DIRECT
#error "Do not select both continuous and direct modes"
#endif

PROGMEM const uint8_t glyph[][5] = {
  #include "lcd_chars.inc"
};
#ifdef LCD_FONT3x5
PROGMEM const uint8_t glyph3x5[][3]={
  {0b00100, 0b00100, 0b00100}, //-
  {0b00000, 0b10000, 0b00000}, //dot
  {0b11000, 0b01110, 0b00011}, //slash
  {0b11111, 0b10001, 0b11111}, //0
  {0b00000, 0b00000, 0b11111}, //1
  {0b11101, 0b10101, 0b10111}, //2
  {0b10101, 0b10101, 0b11111}, //3
  {0b00111, 0b00100, 0b11111}, //4
  {0b10111, 0b10101, 0b11101}, //5
  {0b11111, 0b10101, 0b11101}, //6
  {0b00001, 0b00001, 0b11111}, //7
  {0b11111, 0b10101, 0b11111}, //8
  {0b10111, 0b10101, 0b11111}  //9
};
#endif

PROGMEM const uint8_t bitmask[8]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
PROGMEM const float exp_data[]={1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000};

#define LCD_X_RES	arg1(LCD_RESOLUTION)
#define LCD_Y_RES	arg2(LCD_RESOLUTION)
#define LCD_VIDEO_SIZE		( ( arg1(LCD_RESOLUTION) * arg2(LCD_RESOLUTION) ) / 8 )
//видеобуфер, 84*48 бит, или 504 байта
//videobuffer, 84x48 bit = 504 bytes
uint8_t  lcd_videobuf[ LCD_VIDEO_SIZE ];
//текущая позиция на экране
//current "cursor" position on screen
int   LcdCacheIdx;
volatile unsigned char lcd_fontsize=1;
volatile unsigned char lcdtfdn = 0;

#ifdef LCD_DIRECT
//в direct-режиме отслеживается измененная область экрана
//if direct-mode we use these variables to control the changed part of screen
  int lcd_lowaddr;
  int lcd_hiaddr;
  char lcd_update_flag;
  #define LCDLOWADDR(x) do{if(x < lcd_lowaddr)lcd_lowaddr = x;}while(0)
  #define LCDHIADDR(x) do{if(x > lcd_hiaddr)lcd_hiaddr = x;}while(0)
  #define LCDUPD() do{lcd_update_flag = 1;}while(0)
#else
  #define LCDLOWADDR(x)
  #define LCDHIADDR(x)
  #define LCDUPD()
#endif


void Lcd_init();
void Lcd_update();
//////////////////////////////////////////////////////////////////////
/* Name         :  LcdSend
 * Description  :  local function to send 1 byte to LCD by SPI
 * Argument(s)  :  data - data byte
 * Return value :  -
 */
/* Имя                   :  LcdSend
 * Описание              :  посылает 1 байт дисплею по SPI. Не предназначена для использования вне этого файла
 * Аргумент(ы)           :  data - байт для передачи
 * Возвращаемое значение :  -
 */
#ifndef LCD_HARDWARE_SPI
void LcdSend( uint8_t data ){
  uint8_t i;
  PORT_0(LCD_CE);
  for( i=0;i<8;i++ ){
    if(data & (1<<7))PORT_1(LCD_DATA);else PORT_0(LCD_DATA);
    data<<=1;
    PORT_1(LCD_CLK);
    PORT_0(LCD_CLK);	// Strob
  }
  PORT_1(LCD_CE);
}
inline void LcdCmd(uint8_t data){
  PORT_0(LCD_DC);
  LcdSend(data);
}
inline void LcdData(uint8_t data){
  PORT_1(LCD_DC);
  LcdSend(data);
}
#else
void LcdSend( uint8_t data){
  while(!(SPSR & (1<<SPIF))){}
  SPDR = data;
}
inline void LcdCmd(uint8_t data){
  while(!(SPSR & (1<<SPIF))){}
  PORT_0(LCD_DC);
  SPDR = data;
}
inline void LcdData(uint8_t data){
  while(!(SPSR & (1<<SPIF))){}
  PORT_1(LCD_DC);
  SPDR = data;
}
#endif
////////////////////////////////////////////////////////////////////////////////
/* Name         :  lcd_clear
 * Description  :  clears screen. After calling this function you must call LcdUpdate()
 */
/* Имя                   :  Lcd_clear
 * Описание              :  Очищает дисплей. Далее необходимо выполнить LcdUpdate
 */
void Lcd_clear(){
  uint8_t *ptr = lcd_videobuf, *end = &lcd_videobuf[LCD_VIDEO_SIZE];
  for(;ptr<end;ptr++)*ptr=0;
#ifdef LCD_DIRECT
  lcd_lowaddr = 0;
  lcd_hiaddr = LCD_VIDEO_SIZE-1;
  lcd_update_flag = 1;
#endif
}
//////////////////////////////////////////////////////////////////////////////
/* Name         :  LcdContrast
 * Description  :  sets the LCD contrast
 * Argument(s)  :  contrast, value between 0x00 and 0x7F
 * Return value :  -
 */
/* Имя                   :  LcdContrast
 * Описание              :  Устанавливает контрастность дисплея
 * Аргумент(ы)           :  контраст -> значение от 0x00 к 0x7F
 */
void LcdContrast( uint8_t contrast ){
  LcdCmd( 0x21 );              // Расширенный набор команд
  LcdSend( 0x80 | contrast );   // Установка уровня контрастности
  LcdSend( 0x20 );
}
//////////////////////////////////////////////////////////////////////////////
/* Name         :  LcdGotoXY
 * Description  :  sets "cursor" to character position (x,y), font size is 1
 * Argument(s)  :  x,y - position from (0, 0) to (13, 5)
 * Return value :  -
 */
/* Имя                   :  LcdGotoXY
 * Описание              :  Устанавливает курсор в позицию x,y относительно стандартного размера шрифта
 * Аргумент(ы)           :  x,y -> координаты новой позиции курсора. Значения: 0,0 .. 13,5
 * Возвращаемое значение :  смотри возвращаемое значение в n5110.h
 */
int8_t LcdGotoXY( uint8_t x, uint8_t y ){
  // Проверка границ
  if( x > 13 || y > 5 ) return LCD_OUT;
  //  Вычисление указателя. Определен как адрес в пределах 504 байт
  LcdCacheIdx = x * 6 + y * 84;
  return LCD_OK;
}
//////////////////////////////////////////////////////////////////////////////
/* Name         :  LcdGotoAbs
 * Description  :  sets "cursor" to absolute position (x,y)
 * Argument(s)  :  x,y - position
 * Return value :  -
 */
/* Имя                   :  LcdGotoAbs
 * Описание              :  Устанавливает курсор в позицию x,y относительно стандартного размера шрифта
 * Аргумент(ы)           :  x,y -> координаты новой позиции курсора
 * Возвращаемое значение :  -
 */
int8_t LcdGotoAbs(uint8_t x, uint8_t y){
  // Защита от выхода за пределы
  if ( x >= LCD_X_RES || y >= LCD_Y_RES) return LCD_OUT;
  // Пересчет индекса и смещения
#ifdef LCD_FONT3x5
  lcdtfdn = (y & 0x03);
#endif
  y = (y>>1)&~0x03; //index = (y / 8)*2
  LcdCacheIdx = (y<<4) + (y<<2) + y + x; //index = ( ( y / 8 ) * 84 ) + x;
  return LCD_OK;
}
////////////////////////////////////////////////////////////////////////////////
/* Name         :  LcdChr
 * Description  :  output 1 character to "cursor" position and increment it
 * Argument(s)  :  ch - character
 * Return value :  see at n5110.h
 */
/* Имя                   :  LcdChr
 * Описание              :  Выводит символ в текущей позиции курсора, затем инкрементирует положение курсора
 * Аргумент(ы)           :  ch   -> символ для вывода
 * Возвращаемое значение :  смотри возвращаемое значение в n5110.h
 */
int8_t LcdChr( char ch ){
  uint8_t i, c;
  uint8_t b1, b2;
  int  tmpIdx;
  LCDUPD();
  LCDLOWADDR(LcdCacheIdx); //специфично для direct-режима, отслеживание нижней границы
#ifndef FULL_CP1251_TABLE
  if( (ch >= 0x20) && (ch <= 0x7F) )ch -= 32;
    else if ( ch >= 0xC0 )ch -= 96;
    else ch = 95;
#endif
#ifdef LCD_FONT3x5
  if( lcd_fontsize == 0 ){
    #ifdef FULL_CP1251_TABLE
      ch -= '-';
    #else
      ch -= ('0'-'-');
    #endif
    for( i = 0; i < 3; i++ ){
      c = lcd_videobuf[LcdCacheIdx];
      b1 = pgm_read_byte( &glyph3x5[(uint8_t)ch][i] );
      switch(lcdtfdn){
        case 1: c &= 0b11000001; c |= (b1<<1); break;
        case 2: c &= 0b10000011; c |= (b1<<2); break;
        case 3: c &= 0b00000111; c |= (b1<<3); break;
        default: c &= 0b11100000; c |= b1;
      }
      lcd_videobuf[LcdCacheIdx++] = c;
    }
  }else
#endif
  if( lcd_fontsize == 1 )
    for( i = 0; i < 5; i++ )lcd_videobuf[LcdCacheIdx++] = pgm_read_byte( &glyph[(uint8_t)ch][i] );
  else{
    tmpIdx = LcdCacheIdx - 84;
    LCDLOWADDR(tmpIdx);
    if( tmpIdx < 0 ) return LCD_OUT;
    for( i = 0; i < 5; i++ ){
      // Копируем вид символа из таблицы у временную переменную
      c = pgm_read_byte(&glyph[(uint8_t)ch][i]);
      //resize the image
      // Увеличиваем картинку
      asm volatile(
        "bst %[ival],0 \n\t""bld %[outlow],0 \n\t""bld %[outlow],1 \n\t"
        "bst %[ival],1 \n\t""bld %[outlow],2 \n\t""bld %[outlow],3 \n\t"
        "bst %[ival],2 \n\t""bld %[outlow],4 \n\t""bld %[outlow],5 \n\t"
        "bst %[ival],3 \n\t""bld %[outlow],6 \n\t""bld %[outlow],7 \n\t"
        "bst %[ival],4 \n\t""bld %[outhi],0 \n\t""bld %[outhi],1 \n\t"
        "bst %[ival],5 \n\t""bld %[outhi],2 \n\t""bld %[outhi],3 \n\t"
        "bst %[ival],6 \n\t""bld %[outhi],4 \n\t""bld %[outhi],5 \n\t"
        "bst %[ival],7 \n\t""bld %[outhi],6 \n\t""bld %[outhi],7 \n\t"
        : [outlow]"=&r"(b1), [outhi]"=&r"(b2)
        : [ival]"r"(c)
      );
      
      // Копируем две части в кэш
      lcd_videobuf[tmpIdx++] = b1;
      lcd_videobuf[tmpIdx++] = b1;
      lcd_videobuf[tmpIdx + 82] = b2;
      lcd_videobuf[tmpIdx + 83] = b2;
    }
    // Обновляем x координату курсора
    LcdCacheIdx = (LcdCacheIdx + 11) % LCD_VIDEO_SIZE;
  }
  LCDHIADDR(LcdCacheIdx);
  // Горизонтальный разрыв между символами
  lcd_videobuf[LcdCacheIdx] = 0x00;
  // Если достигли позицию указателя LCD_CACHE_SIZE - 1, переходим в начало
  if(LcdCacheIdx == (LCD_VIDEO_SIZE - 1) ){
    LcdCacheIdx = 0;
    return LCD_OK_WRAP;
  }
  // Иначе просто инкрементируем указатель
  LcdCacheIdx++;
  return LCD_OK;
}
/////////////////////////////////////////////////////////////////////////////////////
/* Name         :  Lcd_puts
 * Description  :  output C-style string from RAM
 * Argument(s)  :  str - pointer to string in RAM
 * Return value :  see in n5110.h
 * Example      :  Lcd_puts("RAM string"); Lcd_puts(filename);
 */
/* Имя                   :  Lcd_puts
 * Описание              :  Эта функция предназначена для печати строки из переменной
 * Аргумент(ы)           :  str       -> массив содержащий строку которую нужно напечатать
 * Возвращаемое значение :  смотри возвращаемое значение в n5110.h
 * Пример                :  Lcd_puts(some_char);
 *                       :  Lcd_puts("RAM string");
 */
int8_t Lcd_puts( const char *str ){
  int8_t resp;
  do{
    resp = LcdChr(*str);
    str++;
    if(resp == LCD_OUT)return LCD_OUT;
  }while(*str);
  return LCD_OK;
}
//output string from flash memory. Same as Lcd_puts()
//аналогично предыдущему, только строка располагается во flash-памяти
//Например. Lcd_puts_P(PSTR("Str"));
int8_t Lcd_puts_P(const char *str){
  int8_t resp;
  char ch= pgm_read_byte(str);
  do{
    resp = LcdChr(ch);
    str++;    
    ch = pgm_read_byte(str);
    if(resp == LCD_OUT)return LCD_OUT;
  }while(ch);
  return LCD_OK;
}
#ifndef DISABLE_DOUBLE
/* Name         :  Lcd_putd
 * Description  :  output 'double' value
 * Argument(s)  :  data - data byte
 *                 accuracy - number of digits after decimal point
 * Return value :  -
 */
/* Имя                   :  Lcd_putd
 * Описание              :  Эта функция предназначена для печати числа с плавающей запятой
 * Аргумент(ы)           :  data -> число
 *                       :  accuracy -> число знаков после запятой
 */
//lcd_put_double - вывод числа с плавающей точкой. Не рекомендуется, потому что медленно
void Lcd_putd(double data, int accuracy ){
  int32_t val = data*pgm_read_float(&exp_data[accuracy]);
  Lcd_putf(val, accuracy);
}
}
#endif
/* Name         :  Lcd_putf
 * Description  :  output fixed-point number
 * Argument(s)  :  value - output number
 *                 dot - number of dogits after decimal point
 * Return value :  -
 */
/* Имя                   :  Lcd_putf
 * Описание              :  Эта функция предназначена для печати числа с фиксированной точкой
 * Аргумент(ы)           :  value -> число
 *                       :  dot   -> число знаков после запятой
 */
//lcd_put_fixed_point - вывод числа с фиксированной точкой
void Lcd_putf( int32_t value, int8_t dot){
  char lcd_numbuffer[13];
  char *buf = lcd_numbuffer+13;
  uint32_t temp,qq;
  uint8_t res;
  int8_t sign=0;
  if(value<0){sign=1; temp = -value;}else temp=value;
  *--buf=0;
  do{
    value = temp;
    temp>>=1;
    temp += temp>>1;
    temp += temp>>4;
    temp += temp>>8;
    temp += temp>>16;
    qq = temp;
    temp >>= 3;
    res = (uint8_t)((uint32_t)value - ((temp<<1) + (qq & ~7ul)));
    if(res > 9){
      res -= 10;
      temp++;
    }
    *--buf = res+'0';
    if(--dot == 0)*--buf='.';
  }while(temp != 0);
  if(dot == 0)*--buf='0';
  if(dot>0){
    do{*--buf='0';}while(--dot);
    *--buf='.';
    *--buf='0';
  }
  if(sign)*--buf='-';
  Lcd_puts(buf);
}
#ifndef DISABLE_GRAPHICS
/* Имя                   :  Lcd_pixel
 * Описание              :  Отображает пиксель по абсолютным координатам (x,y)
 * Аргумент(ы)           :  x,y  -> абсолютные координаты пикселя
 *                          color -> Off, On или Xor. Смотри enum в n5110.h
 * Возвращаемое значение :  смотри возвращаемое значение в n5110.h
 */
int8_t Lcd_pixel( uint8_t x, uint8_t y, uint8_t color ){
  int index;
  uint8_t offset;
  uint8_t data;
  // Защита от выхода за пределы
  if ( x >= LCD_X_RES || y >= LCD_Y_RES) return LCD_OUT;
  // Пересчет индекса и смещения
  index = (y>>1)&~0x03; //index = (y / 8)*2
  index = (index<<4) + (index<<2) + index + x; //index = ( ( y / 8 ) * 84 ) + x;
  offset = pgm_read_byte(&bitmask[y & 0x07]);
  data = lcd_videobuf[ index ];
  // Обработка битов
  if( color == PIXEL_OFF )data &=~ offset ;
    else if( color == PIXEL_ON )data |= offset;
    else if( color  == PIXEL_XOR )data ^= offset;
  // Окончательный результат копируем в буфер
  lcd_videobuf[ index ] = data;
  LCDLOWADDR(index);
  LCDHIADDR(index);
  LCDUPD();
  return LCD_OK;
}
/* Имя                   :  Lcd_line
 * Описание              :  Рисует линию между двумя точками на дисплее (алгоритм Брезенхэма)
 * Аргумент(ы)           :  x1, y1  -> абсолютные координаты начала линии
 *                          x2, y2  -> абсолютные координаты конца линии
 *                          color   -> Off, On или Xor. Смотри в n5110.h
 * Возвращаемое значение :  смотри возвращаемое значение в n5110.h
 */
int8_t Lcd_line( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color){
  int dx, dy, stepx, stepy, fraction;
  int8_t response;
  // dy   y2 - y1
  // -- = -------
  // dx   x2 - x1
  dy = y2 - y1;
  dx = x2 - x1;
  if( dy < 0 ){dy = -dy; stepy = -1;}else{stepy = 1;}
  if( dx < 0 ){dx = -dx; stepx = -1;}else{stepx = 1;}
  dx <<= 1;
  dy <<= 1;
  // Рисуем начальную точку
  response = Lcd_pixel( x1, y1, color );
  if(response != LCD_OK)return response;
  // Рисуем следующие точки до конца
  if( dx > dy ){
    fraction = dy - ( dx >> 1);
    while( x1 != x2 ){
      if( fraction >= 0 ){
        y1 += stepy;
        fraction -= dx;
      }
      x1 += stepx;
      fraction += dy;
      response = Lcd_pixel( x1, y1, color );
      if(response != LCD_OK)return response;
    }
  }else{
    fraction = dx - ( dy >> 1);
    while( y1 != y2 ){
      if( fraction >= 0 ){
        x1 += stepx;
        fraction -= dy;
      }
      y1 += stepy;
      fraction += dx;
      response = Lcd_pixel( x1, y1, color );
      if(response != LCD_OK)return response;
    }
  }
  return LCD_OK;
}
/* Имя                   :  Lcd_circle
 * Описание              :  Рисует окружность (алгоритм Брезенхэма)
 * Аргумент(ы)           :  x, y   -> абсолютные координаты центра
 *                          radius -> радиус окружности
 *                          color  -> Off, On или Xor. Смотри enum в n5110.h
 * Возвращаемое значение :  смотри возвращаемое значение в n5110.h
 */
int8_t Lcd_circle( uint8_t x, uint8_t y, uint8_t radius, uint8_t color){
  int8_t xc = 0;
  int8_t yc = 0;
  int8_t p = 0;
  if ( x >= LCD_X_RES || y >= LCD_Y_RES) return LCD_OUT;
  yc = radius;
  p = 3 - (radius<<1);
  while(xc <= yc){
    Lcd_pixel(x + xc, y + yc, color);
    Lcd_pixel(x + xc, y - yc, color);
    Lcd_pixel(x - xc, y + yc, color);
    Lcd_pixel(x - xc, y - yc, color);
    Lcd_pixel(x + yc, y + xc, color);
    Lcd_pixel(x + yc, y - xc, color);
    Lcd_pixel(x - yc, y + xc, color);
    Lcd_pixel(x - yc, y - xc, color);
    if(p < 0)p += (xc++ << 2) + 6;else p += ((xc++ - yc--)<<2) + 10;
  } 

  return LCD_OK;
}
/* Имя                   :  Lcd_rect  (rectangle)
 * Описание              :  Рисует один закрашенный прямоугольник
 * Аргумент(ы)           :  x1    -> абсолютная координата x левого верхнего угла
 *                          y1    -> абсолютная координата y левого верхнего угла
 *                          x2    -> абсолютная координата x правого нижнего угла
 *                          y2    -> абсолютная координата y правого нижнего угла
 *                          color -> Off, On или Xor. Смотри enum в n5110.h
 * Возвращаемое значение :  смотри возвращаемое значение в n5110.h
 */
int8_t Lcd_rect( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color){
  uint8_t x,y;
  if(x1>x2){x = x1; x1 = x2; x2 = x;}
  if(y1>y2){y = y1; y1 = y2; y2 = y;}
  if(x2 >= LCD_X_RES || y2 >= LCD_Y_RES)return LCD_OUT;
  for(x=x1; x<=x2; x++)
    for(y=y1; y<=y2; y++)Lcd_pixel(x,y,color);
  return LCD_OK;
}
/* Имя                   :  Lcd_rect_empty
 * Описание              :  Рисует незакрашенный прямоугольник
 * Аргумент(ы)           :  x1    -> абсолютная координата x левого верхнего угла
 *                          y1    -> абсолютная координата y левого верхнего угла
 *                          x2    -> абсолютная координата x правого нижнего угла
 *                          y2    -> абсолютная координата y правого нижнего угла
 *                          color -> Off, On или Xor. Смотри enum в n5110.h
 * Возвращаемое значение :  смотри возвращаемое значение в n5110.h
 */
int8_t Lcd_rect_empty( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color){
  uint8_t tmpIdx;
  if( ( x1 >= LCD_X_RES) || ( x2 >= LCD_X_RES) || ( y1 >= LCD_Y_RES) || ( y2 >= LCD_Y_RES) )return LCD_OUT;
  if(x1 > x2){tmpIdx = x1; x1 = x2; x2 = tmpIdx;}
  if(y1 > y2){tmpIdx = y1; y1 = y2; y2 = tmpIdx;}
  // Рисуем горизонтальные линии
  for( tmpIdx = x1; tmpIdx <= x2; tmpIdx++ ){
    Lcd_pixel( tmpIdx, y1, color );
    Lcd_pixel( tmpIdx, y2, color );
  }
  // Рисуем вертикальные линии
  for( tmpIdx = y1; tmpIdx <= y2; tmpIdx++ ){
    Lcd_pixel( x1, tmpIdx, color );
    Lcd_pixel( x2, tmpIdx, color );
  }
  return LCD_OK;
}
#else //DISABLE_GRAPHICS
int8_t Lcd_pixel( uint8_t x, uint8_t y, uint8_t color ){return LCD_OK;}
int8_t Lcd_line( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color){return LCD_OK;}
int8_t Lcd_circle( uint8_t x, uint8_t y, uint8_t radius, uint8_t color){return LCD_OK;}
int8_t Lcd_rect_empty( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color){return LCD_OK;}
int8_t Lcd_rect( uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color){return LCD_OK;}
#endif

/* Имя                   :  LcdDelay
 * Описание              :  Некалиброванная задержка для процедуры инициализации LCD
 */
inline void LcdDelay(){
  unsigned char i=1;
  while(i){i++;}
}
/* Имя                   :  Lcd_init
 * Описание              :  Производит инициализацию порта и SPI МК, контроллера LCD
 */
void Lcd_init(){
  DDR_1(LCD_DATA);
  DDR_1(LCD_CLK);
  DDR_1(LCD_DC);
  DDR_1(LCD_CE);
  DDR_1(LCD_RST);
  
  PORT_0(LCD_DATA);
  PORT_1(LCD_RST);
  PORT_0(LCD_CLK);
  PORT_0(LCD_DC);
  PORT_0(LCD_CE);
  PORT_1(LCD_RST);
  LcdDelay();		// Некалиброванная задержка
  PORT_0(LCD_RST);	// Дергаем reset
  LcdDelay();
  PORT_1(LCD_RST);
  PORT_1(LCD_CE);	// Отключаем LCD контроллер - высокий уровень на SCE
#ifdef LCD_HARDWARE_SPI
  SPCR = (1<<SPE | 1<<MSTR);
  #if (F_CPU < 8000000UL)
  SPSR = (1<<SPI2X);
  #else
  SPSR = 0;
  #endif
  SPDR = 0;
  PORT_0(LCD_CE);
#endif
  // Отправляем команды дисплею
  LcdCmd ( 0x20 | (0<<2 | 0<<1 | 1<<0) );// Включаем расширенный набор команд (LCD Extended Commands)
  LcdSend( 0x80 | 0x48 );  // Установка контрастности (LCD Vop)
  LcdSend( 0x04 | 0b10 );  // Установка температурного коэффициента (Temp coefficent)
  LcdSend( 0x10 | 0b011 ); // Настройка питания (LCD bias mode 1:48)
  LcdSend( 0x20 ); // Включаем стандартный набор команд и горизонтальную адресацию (LCD Standard Commands,Horizontal addressing mode)
  LcdSend( 0x08 | (1<<2 | 0<<0) ); // Нормальный режим (LCD in normal mode)
  // Первичная очистка дисплея
  Lcd_clear();
}

/* Имя                   :  Lcd_update
 * Описание              :  Копирует видеобуфер в ОЗУ дисплея
 */
#ifdef LCD_CONTINUOUS
#ifndef CHINA_LCD
void Lcd_update(){
  static unsigned int curpos = 0;
  if(curpos>=LCD_VIDEO_SIZE){
    LcdCmd(0x80); //goto(x=0)
    LcdSend(0x40);//goto(y=0)
    LcdData(lcd_videobuf[0]);
    curpos=1;
  }
  LcdData(lcd_videobuf[curpos++]);
}
#else
#warning понятия не имею как это будет работать
void Lcd_update(){
  static unsigned char x=0,y=0;
  static unsigned int curpos=0;
  LcdData(lcd_videobuf[curpos]);
  x++; curpos++;
  if(x >= LCD_X_RES){
    x=0;                
    LcdCmd( 0x80);
    LcdSend( 0x40 | y);
    y++;
    if(y >= LCD_Y_RES){
      LcdCmd( 0x21 );    // Включаем расширенный набор команд
      LcdSend( 0x45 );    // Сдвигаем картинку на 5 пикселей вверх (нестандартная команда китайца, оригинал её игнорирует)
      LcdSend( 0x20 );    // Включаем стандартный набор команд и горизонтальную адресацию
      LcdSend( 0x80);
      LcdSend( 0x40);
      x=0; y=0; curpos=0;
    }
  }
}
#endif
#endif
#ifdef LCD_DIRECT
void Lcd_update(){
  int i;
  if(!lcd_update_flag)return;
  if( lcd_lowaddr < 0 )lcd_lowaddr = 0;
    else if( lcd_lowaddr >= LCD_VIDEO_SIZE )lcd_lowaddr = LCD_VIDEO_SIZE - 1;
  if( lcd_hiaddr < 0 )lcd_hiaddr = 0;
    else if ( lcd_hiaddr >= LCD_VIDEO_SIZE )lcd_hiaddr = LCD_VIDEO_SIZE - 1;
#ifdef CHINA_LCD  // Алгоритм для китайского ЖК из нестандартным контроллером
  uint8_t x,y;
  // 102 x 64 - таково предполагаемое разрешение буфера китайского ЖК, при чем
  // память буфера отображается на дисплей со сдвигом вверх на 3 пикселя.
  // Поэтому выводим картинку ниже - начиная с второй строки y+1, а потом
  // сдвинем вверх (опять таки фича китайца, полезная в данном случае)
  x = lcd_lowaddr % LCD_X_RES;      // Устанавливаем начальный адрес x
  LcdCmd( 0x80 | x );     // относительно нижней границы LoWaterMark
  y = lcd_lowaddr / LCD_X_RES + 1;  // Устанавливаем начальный адрес y+1
  LcdSend( 0x40 | y );     // относительно нижней границы LoWaterMark
  for( i = lcd_lowaddr; i <= lcd_hiaddr; i++ ){
    // передаем данные в буфер дисплея
    LcdData( lcd_videobuf[i] );
    x++;                 // заодно подсчитываем координату x, чтобы вовремя перейти на новую строку
    if (x >= LCD_X_RES){ // если вышли за предел, то переходим на следующую строку (x=0; y++)
      // проще говоря, чтобы верно заполнить нужную часть нестандартного буфера,
      // придется явно указывать требуемый адрес, иначе все поплывет :)
      x=0;                
      LcdCmd( 0x80);
      y++;
      LcdSend( 0x40 | y);
    }
  }
  LcdCmd( 0x21 );    // Включаем расширенный набор команд
  LcdSend( 0x45 );    // Сдвигаем картинку на 5 пикселей вверх (нестандартная команда китайца, оригинал её игнорирует)
  LcdSend( 0x20 );    // Включаем стандартный набор команд и горизонтальную адресацию
#else  // Алгоритм для оригинального дисплея
  // Устанавливаем начальный адрес в соответствии к LoWaterMark
  LcdCmd( 0x80 | ( lcd_lowaddr % LCD_X_RES ) );
  LcdSend( 0x40 | ( lcd_lowaddr / LCD_X_RES ) );
  // Обновляем необходимую часть буфера дисплея
  for( i = lcd_lowaddr; i <= lcd_hiaddr; i++ ){
    // Для оригинального дисплея не нужно следить за адресом в буфере,
    // можно просто последовательно выводить данные
    LcdData( lcd_videobuf[i]);
  }
#endif
  // Сброс указателей границ в пустоту
  lcd_lowaddr = LCD_VIDEO_SIZE - 1;
  lcd_hiaddr = 0;
  // Сброс флага изменений кэша
  lcd_update_flag = 0;
}
#endif
