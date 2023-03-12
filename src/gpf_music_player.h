/**
 * @file gpf_music_player.h
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2023-02-26
 */

#include "Arduino.h"
#include "gpf_cons.h"
#include "gpf_debug.h"
#include "DYPlayerArduino.h"

#ifndef GPF_MUSIC_PLAYER_H
#define GPF_MUSIC_PLAYER_H


typedef enum { 
    GPF_MUSIC_PLAYER_SONG_NAME_MACHINE_GUN_1,
    GPF_MUSIC_PLAYER_SONG_NAME_MACHINE_GUN_2,
    GPF_MUSIC_PLAYER_SONG_NAME_METALLICA,    
    GPF_MUSIC_PLAYER_SONG_NAME_LA_BOLDUC,
    GPF_MUSIC_PLAYER_SONG_NAME_RUSH, //Pas utilisé pour le moment
    GPF_MUSIC_PLAYER_SONG_NAME_CELINE, //Pas utilisé pour le moment
    GPF_MUSIC_PLAYER_SONG_NAME_DRAGON,
    GPF_MUSIC_PLAYER_SONG_NAME_ELVIS,
    GPF_MUSIC_PLAYER_SONG_NAME_GERARD,
    GPF_MUSIC_PLAYER_SONG_NAME_JOURNEY, 
    GPF_MUSIC_PLAYER_SONG_NAME_KATY,
    GPF_MUSIC_PLAYER_SONG_NAME_TOTO, //Pas utilisé pour le moment
    GPF_MUSIC_PLAYER_SONG_NAME_SANTANA, 

    GPF_MUSIC_PLAYER_SONG_NAME_ITEM_COUNT // MUST BE LAST
} gpf_music_player_song_name_type_enum;

class GPF_MUSIC_PLAYER {

    public:
        GPF_MUSIC_PLAYER();
        void initialize();
        void select(gpf_music_player_song_name_type_enum song);
        void play();
        void pause();
        void stop();
        void setVolume(uint8_t volumeLevel);
        void updatePlaying(uint16_t pwmValue_Volume, uint16_t pwmValue_Track, uint16_t pwmValue_List);
        DY::Player DY_Player;
        
    private:        
        uint16_t pwmValue_Volume;
        uint16_t pwmValue_Track;
        gpf_rc_channel_position_type_enum stickPosType_MusicList               = GPF_RC_CHANNEL_POSITION_LOW;
        gpf_rc_channel_position_type_enum stickPosType_MusicList_previousValue = GPF_RC_CHANNEL_POSITION_LOW;

        uint8_t  musicVolumeLevel               = 0; //0 à 30
        uint8_t  musicVolumeLevel_previousValue = 0; 

        uint8_t  musicTrack               = 0; 
        uint8_t  musicTrack_previousValue = 0; 

        bool isPlaying  = false;                        
        gpf_music_player_song_name_type_enum currentSong = GPF_MUSIC_PLAYER_SONG_NAME_MACHINE_GUN_1;
};

#endif