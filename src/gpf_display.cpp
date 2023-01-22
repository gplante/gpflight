/**
 * @file gpf_display.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-11-06
 * 
 * Class pour écran de type TFT d'une résolution de 320x240 avec un chip ili9341
 * Cette classe s'utilise conjointement avec la librairie ILI9341_t3 (https://github.com/PaulStoffregen/ILI9341_t3)
 * 
 */
 
#include "Arduino.h"
#include "Math.h"
#include "gpf.h"
#include "gpf_dshot.h"
#include "gpf_debug.h"
#include "gpf_util.h"
#include "gpf_display.h"
#include "ILI9341_t3.h"


GPF_DISPLAY::GPF_DISPLAY() {
 

}

void GPF_DISPLAY::initialize() {    
    // *** Finalement, cette patch ne marche pas...
    // Patch sinon l'écran reste blanc et ne fonctionne pas
    // Ca ne bug pas si je ne branche pas les pin du touchscreen mais j'ai besoin du touchscreen :-)
    // https://forum.pjrc.com/threads/69740-ILI9341-White-screen-with-touch-pins-connected-Teensy-4-1-amp-audio-shield
    pinMode(GPF_SPI_TOUCH_DC,OUTPUT); 
    digitalWrite(GPF_SPI_TOUCH_DC,HIGH); 
    //Fin patch
    //delay(500);
    tft.begin();    

    // Note: you can now set the SPI speed to any value
    // the default value is 30Mhz, but most ILI9341 displays
    // can handle at least 60Mhz and as much as 100Mhz
    //  tft.setClock(60000000);
    //tft.setClock(1000000);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW);  
    setTextSize(2); //4
    setRotation(0);
    tft.setScroll(0);    
    
}

ILI9341_t3 * GPF_DISPLAY::get_tft() { 
 return &tft;
}

size_t GPF_DISPLAY::write(uint8_t character) {     

 if(currentMode != GPF_DISPLAY_MODE_LOGGING_AUTO_SCROLL) {
    tft.write(character);
 } else {
    if (tft.getRotation() == 0) {
     modeLogging_print_hw(character); //Si orientation 0 (mode portrait) on peut se service du hardware vertical scroll. C'est rapide.
    } else {
     modeLogging_print_sw(character); //Malheureusement, le hardware vertical scroll est disponible seulement si orientation est à 0 donc on doit simuler un scroll et c'est extrèmement beaucoup plus long!!!
    }
 } 

 return 1;
}

void GPF_DISPLAY::setMode(uint8_t newMode) {
 currentMode = newMode;
 if(newMode == GPF_DISPLAY_MODE_LOGGING_AUTO_SCROLL) {
    scroll = 0;
 }
}

int16_t GPF_DISPLAY::getDisplayWidth() {
    return tft.width();
}

int16_t GPF_DISPLAY::getDisplayHeight() {
    return tft.height();
}

void GPF_DISPLAY::setRotation(uint8_t newRotation) {
    tft.setRotation(newRotation);
}

uint8_t GPF_DISPLAY::getRotation() {
    return tft.getRotation();
}

void GPF_DISPLAY::setTextSize(uint8_t newSize) {
 tft.setTextSize(newSize);
 tft.measureChar('X',&charWidth,&charHeight); 
}

void GPF_DISPLAY::modeLogging_print_hw(uint8_t character) {
   static  bool    myScroll_started  = false;
   static  int16_t x = 0;
   static  int16_t y = 0;
   static  int16_t y_previous = 0;

   if (character == '\r') { //On ignore ce caractère
     return;
   }

   if ((x == 0) && (y == 0)) {
     tft.fillRect(0,0,getDisplayWidth(), charHeight, ILI9341_BLACK); //efface le contenu de l'ancienne ligne           
     
     /*
     DEBUG_GPF_DISPLAY_PRINT("*********************************GREEN*** x=");
     DEBUG_GPF_DISPLAY_PRINT(x);
     DEBUG_GPF_DISPLAY_PRINT(" y=");
     DEBUG_GPF_DISPLAY_PRINT(y);
     DEBUG_GPF_DISPLAY_PRINTLN("");
     */
   }

   if (character == '\n') { //On incrémente le y mais on sort tout de suite. On veut que le scroll soit pris en considération seulement lorsqu'on va afficher le prochain caractère.
       y = y + charHeight; 
       x = 0;
       return;
   }

   
 
   //Si pas assez de place horizontale, on scroll tout de suite dans ce cas.
   if ((getDisplayWidth() - x ) >= charWidth) {
       //Ok, il y a assez de place horizontale
   } else {       
      //Oups pas assez de place horizontale donc on force un changement de ligne.
      y = y + charHeight; 
      x = 0;  //On remet le curseur X au début de la ligne également.
   }

   bool assez_de_place = true;

   if (y_previous != y) { //Si on doit changer de ligne
      if (myScroll_started) { // Une fois qu'on a commencé à utiliser le scroll, on va toujours devoir l'utiliser car ca veut dire qeu'on est rendu en bas de l'écran et on va toujours rester en bas de l'écran. Bref, comme du texte qui défile sur un terminal.
            assez_de_place = false; //Il n'y a jamais assez de place
      } else {
         //On est pas encore rendu en bas de l'écran car on a pas commencer à scroller encore.
         if (((getDisplayHeight() - y ) >= charHeight)) {
            //Assez de place
         } else {
            assez_de_place = false;
            myScroll_started = true;
         }
      }
   }

   if (!assez_de_place) { //Ca veut dire que le texte est rendu en bas de l'écran
      //On se sert du Vertical Hardware Scroll du display
      //myScroll_started = true;
      scroll = scroll + charHeight;  

      scroll = scroll % getDisplayHeight(); //Si on est rendu à scroller plus que la hauteur du display on s'assure de revenir au début du display

      if (scroll > 0) { 
       if (scroll < charHeight) { // On s'assure qu'on scroll d'au moin la hauteur qu'on a besoin. En effet, ca peut arriver à cause de la ligne modulo ci-haut.
       
        // Si on est rendu pas mal en bas de l'écran, puis qu'il n'y a pas assez de place pour écrire le caractère on doit repartir en haut de l'écran.
        // Sinon il aurait fallu que j'écrive par exemple une demie ligne en bas de l'écran puis l'autre demi ligne en haut de l'écran.       
        // Ca ne me tente pas de commencer à écrire du code pour gérer celà alors dans ce temps là, je skip jusqu'au début du display.       
        // Voir section 8.2.30. Vertical Scrolling Start Address (37h) du datasheet du display ILI9341 (fichier ILI9341-2.pdf) pour plus d'informations sur la manière que le scrolling du display fonctionne.
        //
        // Ceci arrive seulement lorsque ma police n'est pas un multiple de la hauteur de l'écran.
        // (en passant le vertical scrolling du display fonctionne seulement de manière "verticale". Le display n'a pas de fonction pour scroller de manière horizontale)
        // Par exemple, en mode portrait, la hauteur de l'écran a 320 pixels. Si ma police mesure 40px de haut, tout va toujour bien fonctionner.
        // Cependant, si ma police a 48px de haut, une fois la 6ième ligne affichée, le curseur Y va être rendu à la position 288. Il va ne rester que
        // 32 pixels avant la fin de l'écran donc puisqi'il n'y a pas assez de place pour écrire ma ligne qui a besoin de 48 pixels et bien je recommence en haut de l'écran.

        tft.fillRect(0,getDisplayHeight() - (charHeight - scroll),getDisplayWidth(), (charHeight - scroll), ILI9341_BLACK); //On efface ce qu'il y avait d'imprimé à cet endroit avant de retourner en haut de l'écran.
        /*
        DEBUG_GPF_DISPLAY_PRINT("*********************************RED*** x=");
        DEBUG_GPF_DISPLAY_PRINT(x);
        DEBUG_GPF_DISPLAY_PRINT(" y=");
        DEBUG_GPF_DISPLAY_PRINT(y);
        DEBUG_GPF_DISPLAY_PRINTLN("");
        */
        scroll = charHeight;
       }
      }

      tft.setScroll(scroll);    


      y = scroll - charHeight; // Y est toujours ajusté par rapport à scroll (Toujours la ligne d'avant dans la mémoire du display puisque scroll change uniquement le pointer dans la mémoire du display. Le pointeur est l'adresse à partir de laquelle le display sera affiché. Voir section 8.2.30. Vertical Scrolling Start Address (37h) du datasheet du display ILI9341) 
      if (y < 0) {
        y = y + getDisplayHeight();
      }
   }

   //rendu ici, c'est certain qu'on a assez de place pour imprimer le caractere

   //Si scroll a changé, on efface la ligne puis on imprime le caractere. Si c'est un auttre \n on réincrimente scroll puis on resort ici après.
   if (y_previous != y) {
     //Si on scroll, on efface le contenue de l'ancienne ligne avant de réimprimer par dessus. 
     tft.fillRect(0,y,getDisplayWidth(), charHeight, ILI9341_BLACK); //efface le contenu de l'ancienne ligne
     /*
     DEBUG_GPF_DISPLAY_PRINT("*********************************PINK*** x=");
     DEBUG_GPF_DISPLAY_PRINT(x);
     DEBUG_GPF_DISPLAY_PRINT(" y=");
     DEBUG_GPF_DISPLAY_PRINT(y);
     DEBUG_GPF_DISPLAY_PRINTLN("");
     */
   }

   tft.setCursor(x, y); //Positionne le curseur X et Y
   tft.fillRect(x,y,charWidth, charHeight, ILI9341_BLACK);
   tft.print((char)character);

     DEBUG_GPF_DISPLAY_PRINT("x=");
     DEBUG_GPF_DISPLAY_PRINT(x);
     DEBUG_GPF_DISPLAY_PRINT(" y=");
     DEBUG_GPF_DISPLAY_PRINT(y);
     DEBUG_GPF_DISPLAY_PRINT(" scroll=");
     DEBUG_GPF_DISPLAY_PRINT(scroll);
     DEBUG_GPF_DISPLAY_PRINT(" enter=");
     if (character == '\n') {
      DEBUG_GPF_DISPLAY_PRINT("oui");
     } else {
      DEBUG_GPF_DISPLAY_PRINT("non");
     }
     DEBUG_GPF_DISPLAY_PRINT(" character=");
     DEBUG_GPF_DISPLAY_PRINT((char)character);
     DEBUG_GPF_DISPLAY_PRINTLN("");


   x = x + charWidth;

   y_previous = y;

   return;
 
}

void GPF_DISPLAY::modeLogging_print_sw(uint8_t character) {
     // Attention, au début j'avais branché les pins du touchscreen également mais sans callé le 
     // constructeur XPT2046_Touchscreen(GPF_SPI_TOUCH_DC) donc finalement la fonction tft.readRect() ne fonctionnait pas.
     // Donc en simple, si vous ne vous servez pas du touchscreen, ne brancher pas les fils. Si vous sous en servez et bien caller le constructeur pour le touchscreen
     // et tout va bien fonctionner.
    
     static int16_t x = 0;
     static int16_t y = 0;
     static int16_t y_previous = 0;
     static bool    myScroll_started  = false;
            bool    pleaseScroll = false;

     uint16_t rect_height = 0;
     uint16_t* screenContent = NULL;

     if (character == '\r') {
       return;
     }
    
     if (character == '\n') { //On incrémente le y mais on sort tout de suite. On veut que le scroll soit pris en considération seulement lorsqu'on va afficher le prochain caractère.
       y = y + charHeight;
       x = 0;
       return;
     }

     if (y != y_previous) { //Par exemple, si le caractère d'avant c'était un \n
       if (myScroll_started) {
         pleaseScroll = true;  
       }
     }

     //Check si assez de place horizontale pour afficher le caractère à la colonne en cours     
     if ((getDisplayWidth() - x ) >= charWidth) {
             //Ok, il y a assez de place horizontale                
     } else {
         //Oups, pas assez de place horizontale donc on doit changer de ligne.
         y = y + charHeight;
         x = 0;
         if (myScroll_started) {
          pleaseScroll = true;  
         }
     }

     //Check si assez de place verticale pour afficher le caractère à la rangée en cours
     if (((getDisplayHeight() - y ) >= charHeight)) {
        //Ok, assez de place donc pas besoin de scroller.
     } else {       
        //Oups, pas assez de place vertical donc on doit changer de ligne.
        pleaseScroll = true;           
        myScroll_started = true;       
     }

     if (pleaseScroll) {
      rect_height = getDisplayHeight() - charHeight;
      screenContent = ( uint16_t*) malloc(getDisplayWidth() * rect_height * sizeof(uint16_t));
      tft.readRect(0,charHeight - (getDisplayHeight() - y ),getDisplayWidth(),rect_height,screenContent);
      tft.writeRect(0,0,getDisplayWidth(),rect_height,screenContent);
      free(screenContent);
      tft.fillRect(0,rect_height,getDisplayWidth(), charHeight, ILI9341_BLACK); //efface l'amcienne ligne
      y = rect_height;
      x = 0;
     }

     tft.setCursor(x, y);     
     tft.fillRect(x,y,charWidth, charHeight, ILI9341_BLACK);
     tft.print((char)character);     

     /*
      DEBUG_GPF_DISPLAY_PRINT("x=");
      DEBUG_GPF_DISPLAY_PRINT(x);
      DEBUG_GPF_DISPLAY_PRINT(" y=");
      DEBUG_GPF_DISPLAY_PRINT(y);
      DEBUG_GPF_DISPLAY_PRINT(" character=");
      DEBUG_GPF_DISPLAY_PRINT((char)character);
      DEBUG_GPF_DISPLAY_PRINTLN("");
     */

     x = x + charWidth;
     y_previous = y;

     return;
 
}

void GPF_DISPLAY::clearScreen() {
   get_tft()->setFontAdafruit();
   setTextSize(2); //4
   setRotation(0);
   get_tft()->setScroll(0);
   get_tft()->fillScreen(ILI9341_BLACK);
   get_tft()->setTextColor(ILI9341_YELLOW);
}

void Adafruit_GFX_Button_2::initButton(ILI9341_t3 *gfx, int16_t x, int16_t y,
		uint8_t w, uint8_t h,
		uint16_t outline, uint16_t fill, uint16_t textcolor,
		const char *label, uint8_t textsize) {

      _x_2 = x;
	   _y_2 = y;
	   _textsize_2 = textsize;
	   _gfx_2 = gfx;
	   strncpy(_label_2, label, 29);
	   _label_2[29] = 0;

      // Appel la fonction de la classe de base sans le label
      Adafruit_GFX_Button::initButton(gfx, x, y, w, h, outline, fill, textcolor, "", textsize);

}

void Adafruit_GFX_Button_2::drawButton(bool inverted) {
     // Appel la fonction de la classe de base
     Adafruit_GFX_Button::drawButton(inverted);

     //Ensuite j'imprime mon label
     _gfx_2->setCursor(_x_2 - strlen(_label_2)*3*_textsize_2, _y_2-4*_textsize_2);
	  _gfx_2->setTextSize(_textsize_2);
	  _gfx_2->print(_label_2);

}
