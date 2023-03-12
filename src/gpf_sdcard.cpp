/**
 * @file gpf_sdcard.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-12-02
 * 
 * Class utilitaire pour communiquer avec le lecteur de carte sd intégré sur le board Teensy 4.1
 * 
 */
 
#include "Arduino.h"
#include "gpf.h"
#include "gpf_sdcard.h"
#include "gpf_debug.h"
#include "gpf_util.h"
#include <SD.h>
#include <TimeLib.h>

GPF_SDCARD::GPF_SDCARD() {

}

void GPF_SDCARD::initialize() {  
    
    for (size_t i = 0; i < GPF_SDCARD_FILE_TYPE_ITEM_COUNT; i++) {
      thefileIsOpen[i] = false;
    }

    DEBUG_GPF_SDCARD_PRINT("Initializing SD card...");
    sdCardInitOk = false;

    if (SD.begin(BUILTIN_SDCARD)) {
      sdCardInitOk = true;
      DEBUG_GPF_SDCARD_PRINTLN("Ok :-)");

      openFile(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG);
      getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->print(gpf_util_get_dateTimeString(GPF_MISC_FORMAT_DATE_TIME_FRIENDLY,true));
      getFileObject(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG)->println("Démarrage du teensy");
      closeFile(GPF_SDCARD_FILE_TYPE_INFORMATION_LOG);
     
      //test
      DEBUG_GPF_SDCARD_PRINTLN("***** TEST - Liste des dossiers et fichiers de la carte sd *****");
      File root = SD.open("/");
      debugPrintDirectory(root, 0);
      DEBUG_GPF_SDCARD_PRINTLN("***** FIN TEST *****");
      root.close();
     
    } else {
      DEBUG_GPF_SDCARD_PRINTLN("Oups, lecteur SD non détecté!");
    }    

}

void GPF_SDCARD::debugPrintDirectory(File dir, int numTabs) {
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      // No more files
      break;
    }

    for (uint8_t i = 0; i < numTabs; i++) {
      DEBUG_GPF_SDCARD_PRINT('\t');
    }

    DEBUG_GPF_SDCARD_PRINT(entry.name());
    if (entry.isDirectory()) {
      DEBUG_GPF_SDCARD_PRINTLN("/");
      debugPrintDirectory(entry, numTabs + 1);
    } else {
      // Files have sizes, directories do not
      DEBUG_GPF_SDCARD_PRINT("\t\t");
      DEBUG_GPF_SDCARD_PRINTLN(entry.size(), DEC);
    }
  }
}

bool GPF_SDCARD::openFile(gpf_sdcard_file_type_type_enum myFileType) {
    int i;
    char tmpBuffer[6] = "";

    thefileIsOpen[myFileType] = false;
    
    if (sdCardInitOk) {
     
     strcpy(theFileName[myFileType],"");

     strcat(theFileName[myFileType], theFileNamePrefix[myFileType]);
     strcat(theFileName[myFileType],"-");

     itoa(year(), tmpBuffer, 10);
     strcat(theFileName[myFileType], tmpBuffer);    
    
     i = month();
     if (i < 10) {
      strcat(theFileName[myFileType],"0");    
     }
     itoa(i, tmpBuffer, 10);
     strcat(theFileName[myFileType], tmpBuffer);

     i = day();
     if (i < 10) {
      strcat(theFileName[myFileType],"0");    
     }
     itoa(i, tmpBuffer, 10);
     strcat(theFileName[myFileType], tmpBuffer);

     if (myFileType == GPF_SDCARD_FILE_TYPE_BLACK_BOX) {
      //Les fichiers de types blackbox on le format  bb-aaaammjj-hhmmss.log
      //tandis que les autres fichiers ont le format ??-aammjj.log

      strcat(theFileName[myFileType], "-");

      i = hour();
      if (i < 10) {
       strcat(theFileName[myFileType],"0");    
      }
      itoa(i, tmpBuffer, 10);
      strcat(theFileName[myFileType], tmpBuffer);

      i = minute();
      if (i < 10) {
       strcat(theFileName[myFileType],"0");    
      }
      itoa(i, tmpBuffer, 10);
      strcat(theFileName[myFileType], tmpBuffer);

      i = second();
      if (i < 10) {
       strcat(theFileName[myFileType],"0");    
      }
      itoa(i, tmpBuffer, 10);
      strcat(theFileName[myFileType], tmpBuffer);    
     }

     strcat(theFileName[myFileType], ".log");

     theFile[myFileType] = SD.open(theFileName[myFileType], FILE_WRITE);
     if (theFile[myFileType]) {
      thefileIsOpen[myFileType] = true;
      //DEBUG_GPF_SDCARD_PRINT("Ouverture du fichier ");
      //DEBUG_GPF_SDCARD_PRINTLN(theFileName[myFileType]);
     } else {
      DEBUG_GPF_SDCARD_PRINT("Oups, ne peut ouvrir le fichier ");
      DEBUG_GPF_SDCARD_PRINTLN(theFileName[myFileType]);
     }
    }
    
    return thefileIsOpen[myFileType];
}

void GPF_SDCARD::closeFile(gpf_sdcard_file_type_type_enum myFileType) {
    if (sdCardInitOk) {
     //theFile[myFileType].close();
     
     if (thefileIsOpen[myFileType]) {
      theFile[myFileType].close();
      //DEBUG_GPF_SDCARD_PRINT("Fermeture du fichier ");
      //DEBUG_GPF_SDCARD_PRINTLN(theFileName[myFileType]);
     }

     thefileIsOpen[myFileType] = false;
    }
}

File * GPF_SDCARD::getFileObject(gpf_sdcard_file_type_type_enum myFileType) {
 return &theFile[myFileType];
}

/*
size_t GPF_SDCARD::write(uint8_t character) {     
 if (!logFileIsOpen) {
    //openFile();
 }

 if (logFile) {
    logFile.write(character);
    //DEBUG_GPF_SDCARD_PRINT(character);     
 } 

 return 1;
}
*/