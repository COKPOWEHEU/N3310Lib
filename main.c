#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "n5110.h"

PROGMEM const char romstr[]={215, 232, 242, 224, 233, 32, 109, 97, 110, 96, 251, 0};
int main(){
  Lcd_init();
  Lcd_clear();
  LcdGotoXY(0,0);
  Lcd_puts_P(romstr);
  LcdSize(2);
  LcdGotoXY(0,2);
  Lcd_putf(31415,4);
  Lcd_line(10,30,50,40,1);
  Lcd_circle(42,36,12,1);
  while(1){
    Lcd_update();
  }
  return 0;
} 
