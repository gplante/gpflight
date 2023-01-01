/**
 * @file gpf_dshot.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-28
 * 
 * Voir fichier gpf_dshot.cpp pour plus d'informations.
 * 
 */

#ifndef GPF_SDCARD_H
#define GPF_SDCARD_H

#include <SD.h>

typedef enum { 
    GPF_SDCARD_FILE_TYPE_INFORMATION_LOG,
    GPF_SDCARD_FILE_TYPE_ERROR_LOG,
    GPF_SDCARD_FILE_TYPE_DEBUG_LOG,
    GPF_SDCARD_FILE_TYPE_BLACK_BOX,

    GPF_SDCARD_FILE_TYPE_ITEM_COUNT // MUST BE LAST
} gpf_sdcard_file_type_type_enum;

//class GPF_SDCARD : public Print {
class GPF_SDCARD {

    public:
        GPF_SDCARD();
        void initialize();
        File * getFileObject(gpf_sdcard_file_type_type_enum myFileType);
        //virtual size_t write(uint8_t); //Pour la class Print        
        bool openFile(gpf_sdcard_file_type_type_enum myFileType);
        void closeFile(gpf_sdcard_file_type_type_enum myFileType);
        
    private:
        
        bool sdCardInitOk  = false;
        char logFileName[30] = "default-test-log.log"; //Ex.: 20220313-145301.log"

        //blackbox.log est un nom par défaut car le nom du fichier blackbox est dynamique et correspond à la date et l'heure en cours comme par exemple:
        // bb-20220124-163201.log
        char theFileName[GPF_SDCARD_FILE_TYPE_ITEM_COUNT][30] = {"info.log", "error.log", "debug.log", "nom-fichier-dynamique.log"}; //Max 29 carac. sinon augmenter taille tableau
        char theFileNamePrefix[GPF_SDCARD_FILE_TYPE_ITEM_COUNT][3] = {"in", "er", "de", "bb"}; //Max 2 carac. sinon augmenter taille tableau
        File theFile[GPF_SDCARD_FILE_TYPE_ITEM_COUNT];
        bool thefileIsOpen[GPF_SDCARD_FILE_TYPE_ITEM_COUNT];
        
        void debugPrintDirectory(File dir, int numTabs);
        
        
};

#endif