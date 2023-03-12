/**
 * @file gpf_music_player.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2023-02-27
 * 
 * Class "wrapper" pour communiquer avec le player dy-hv20t via la librairie https://github.com/SnijderC/dyplayer de Chris SnijderC.
 * 
 */
 
#include "Arduino.h"
#include "gpf.h"
#include "gpf_debug.h"
#include "gpf_util.h"

GPF_MUSIC_PLAYER::GPF_MUSIC_PLAYER() {

}

void GPF_MUSIC_PLAYER::initialize() {  
 //DY::Player DY_Player(&Serial8);
 //DY_Player(&Serial8);
 DY_Player.port = &Serial8;
 DY_Player.isSoftSerial = false;
 DY_Player.begin();
 //char path[] = "/metalli.mp3";
  
 //DY_Player.playSpecifiedDevicePath(DY::Device::Sd, path);
 //DY_Player.playSpecifiedDevicePath(DY::Device::Sd, path);
 
 stop();
 setVolume(0);

 DY_Player.setCycleMode(DY::PlayMode::RepeatOne);

}

void GPF_MUSIC_PLAYER::select(gpf_music_player_song_name_type_enum song) {
   char path[20] = "";

   switch (song) {
    case GPF_MUSIC_PLAYER_SONG_NAME_MACHINE_GUN_1:
     strcpy(path,"/mgun1c.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_MACHINE_GUN_2:
     strcpy(path,"/mgun2c.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_METALLICA:
     strcpy(path,"/metalli.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_LA_BOLDUC:
     strcpy(path,"/bolduc.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_RUSH:
     strcpy(path,"/rush.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_CELINE:
     strcpy(path,"/celine.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_DRAGON:
     strcpy(path,"/dragon.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_ELVIS:
     strcpy(path,"/elvis.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_GERARD:
     strcpy(path,"/gerard.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_JOURNEY:
     strcpy(path,"/journey.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_KATY:
     strcpy(path,"/katy.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_TOTO:
     strcpy(path,"/toto.mp3");
     break;

    case GPF_MUSIC_PLAYER_SONG_NAME_SANTANA:
     strcpy(path,"/santana.mp3");
     break;

    default:
     break;
   }

   DY_Player.playSpecifiedDevicePath(DY::Device::Sd, path);
   DY_Player.stop();
}

void GPF_MUSIC_PLAYER::play() {
  DY_Player.play();
}

void GPF_MUSIC_PLAYER::pause() {
  DY_Player.pause();
}

void GPF_MUSIC_PLAYER::stop() {
  DY_Player.stop();
}

void GPF_MUSIC_PLAYER::setVolume(uint8_t volumeLevel) {
  DY_Player.setVolume(volumeLevel);
}

void GPF_MUSIC_PLAYER::updatePlaying(uint16_t pwmValue_Volume, uint16_t pwmValue_Track, uint16_t pwmValue_List) {
   stickPosType_MusicList_previousValue = stickPosType_MusicList;
   musicVolumeLevel_previousValue       = musicVolumeLevel;
   musicTrack_previousValue             = musicTrack;

   stickPosType_MusicList = gpf_util_getPwmChannelToPosType(pwmValue_List);
   musicVolumeLevel       = map(constrain(pwmValue_Volume, 1000, 2000), 1000, 2000, 0, 30);
   musicTrack             = map(constrain(pwmValue_Track, 1000, 2000), 1000, 2000, 0, 5); //Switch 6 positions sur radio RadioMaster TX16S

   if (stickPosType_MusicList == GPF_RC_CHANNEL_POSITION_LOW) {
     musicTrack = 255; 
   } else {
     if (stickPosType_MusicList == GPF_RC_CHANNEL_POSITION_HIGH) {
       musicTrack = musicTrack + 6; //6 chansons par liste (Pot 6 positions)
     }
   }

   // Stick Volume //On update seulement si il a changé pour ne pas ralentir la main thread
   if (musicVolumeLevel != musicVolumeLevel_previousValue) {
    setVolume(musicVolumeLevel);      
   }

/*
   // Stick List //On update seulement si il a changé pour ne pas ralentir la main thread
   if (stickPosType_MusicList != stickPosType_MusicList_previousValue) {
     if (stickPosType_MusicList == GPF_RC_CHANNEL_POSITION_LOW) {
      stop();
     }
   }
*/

   if (musicTrack != musicTrack_previousValue) { //On update seulement si il a changé pour ne pas ralentir la main thread

     //if ((stickPosType_MusicList != GPF_RC_CHANNEL_POSITION_LOW)) {
      switch (musicTrack) {
       case 0: //Track 0 et 6, on ne joue rien. C'est la position par défaut du pot 6 positions lorsqu'on met la TX16S sous tension.
        stop();
        break;

       case 1:
        select(GPF_MUSIC_PLAYER_SONG_NAME_MACHINE_GUN_2); 
        play();
        break;  

       case 2:
        select(GPF_MUSIC_PLAYER_SONG_NAME_DRAGON); 
        play();
        break;   

       case 3:
        select(GPF_MUSIC_PLAYER_SONG_NAME_KATY); 
        play();
        break;  

       case 4:
        select(GPF_MUSIC_PLAYER_SONG_NAME_JOURNEY); 
        play();
        break;  

       case 5:
        select(GPF_MUSIC_PLAYER_SONG_NAME_GERARD); 
        play();
        break;     

       case 6: //Track 0 et 6, on ne joue rien. C'est la position par défaut du pot 6 positions lorsqu'on met la TX16S sous tension.
        stop();
        break; 

       case 7:
        select(GPF_MUSIC_PLAYER_SONG_NAME_MACHINE_GUN_1); 
        play();
        break; 

       case 8:
        select(GPF_MUSIC_PLAYER_SONG_NAME_METALLICA); 
        play();
        break;  

       case 9:
        select(GPF_MUSIC_PLAYER_SONG_NAME_SANTANA); 
        play();
        break;  

       case 10:
        select(GPF_MUSIC_PLAYER_SONG_NAME_ELVIS); 
        play();
        break;  

       case 11:
        select(GPF_MUSIC_PLAYER_SONG_NAME_LA_BOLDUC); 
        play();
        break; 

       case 255: 
        stop();
        break; 
     
       default:
        stop();
        break;
      }
     //}
   }

   if ((stickPosType_MusicList != stickPosType_MusicList_previousValue) || (musicTrack != musicTrack_previousValue) ) {
     DEBUG_GPF_MUSIC_PLAYER_PRINT("musicVolumeLevel=");
     DEBUG_GPF_MUSIC_PLAYER_PRINT(musicVolumeLevel);
     DEBUG_GPF_MUSIC_PLAYER_PRINT(" stickPosType_MusicList=");
     DEBUG_GPF_MUSIC_PLAYER_PRINT(stickPosType_MusicList);
     DEBUG_GPF_MUSIC_PLAYER_PRINT(" musicTrack=");
     DEBUG_GPF_MUSIC_PLAYER_PRINT(musicTrack);
     DEBUG_GPF_MUSIC_PLAYER_PRINT(" pwmValue_List=");
     DEBUG_GPF_MUSIC_PLAYER_PRINT(pwmValue_List);
     
     DEBUG_GPF_MUSIC_PLAYER_PRINTLN("");
   }
   
}
