/**
 * @file gpf_display.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-28
 * 
 * Voir fichier gpf_display.cpp pour plus d'informations.
 * 
 */

#ifndef GPF_DISPLAY_H
#define GPF_DISPLAY_H

#include "Arduino.h"
//#include <Adafruit_GFX.h> 
#include "ILI9341_t3.h"
#include "gpf.h"


#define GPF_DISPLAY_MODE_NORMAL                0
#define GPF_DISPLAY_MODE_LOGGING_AUTO_SCROLL   1

class GPF_DISPLAY  : public Print {

    public:
        GPF_DISPLAY();
        void initialize();
        ILI9341_t3 * get_tft();
        virtual size_t write(uint8_t); //Pour la class Print        

        void setMode(uint8_t);        
        
        void    setTextSize(uint8_t);
        int16_t getDisplayWidth();
        int16_t getDisplayHeight();
        void    setRotation(uint8_t);
        uint8_t getRotation();

        void    modeLogging_print_hw(uint8_t);
        void    modeLogging_print_sw(uint8_t);
        void    clearScreen();

    private:
        ILI9341_t3 tft = ILI9341_t3(GPF_SPI_DISPLAY_CS, GPF_SPI_DISPLAY_DC, GPF_DISPLAY_RESET_PIN, GPF_SPI_MOSI, GPF_SPI_SCLK, GPF_SPI_MISO);
        int16_t  scroll = 0;
        uint16_t charHeight = 0;
        uint16_t charWidth = 0;
        
        int8_t   currentMode = GPF_DISPLAY_MODE_NORMAL;
        unsigned long cpt = 0;        
        
};

class Adafruit_GFX_Button_2 : public Adafruit_GFX_Button { //Le butde créer cette classe dérivée est de seulement pourvoir agrandir le label du bouton de 10 à 30.
   public:
    void initButton(ILI9341_t3 *gfx, int16_t x, int16_t y,
		uint8_t w, uint8_t h,
		uint16_t outline, uint16_t fill, uint16_t textcolor,
		const char *label, uint8_t textsize);

    void drawButton(bool inverted = false);

   private:
    ILI9341_t3 *_gfx_2;
    int16_t _x_2, _y_2;
	//uint16_t _w, _h;
	uint8_t _textsize_2;
    char _label_2[30]; 
};

#endif
