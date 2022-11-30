/**
 * @file gpf_dshot.cpp
 * @author Guylain Plante (gplante2@gmail.com)
 * @version 0.1
 * @date 2022-10-28
 * 
 * Class pour communiquer avec un ESC qui supporte le protocole DSHOT.
 * Je n'ai trouvé aucune librairie "simple" ou "toute faite" sur Internet j'ai donc décidé de coder le tout moi même.
 * On ne peut pas utiliser de uart pour celà. 
 * Je doit utiliser des DMA sinon écrire sur les pins simplement à l'aide du code via la loop() principale ne serait pas assez rapide à ce que j'ai pu lire sur Internet.
 * Je me suis quand même inspiré du code de certains projets trouvés sur GitHub. En voici une liste non exhaustive:  
 * todo
 * 
 */
 
#include "Arduino.h"
#include "Math.h"
#include "gpf.h"
#include "gpf_dshot.h"
#include "gpf_debug.h"
#include "gpf_util.h"

GPF_DSHOT::GPF_DSHOT() {

}

void GPF_DSHOT::initialize() {    
    
}

