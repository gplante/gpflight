# GPFlight (*** En cours de développement ***)

## Contrôleur de vol pour stabiliser un quadricoptère (drone de 4 moteurs)
<br />
Microcontrôleur: Teensy 4.1
<br />
ESC (Electronic Speed Controller): 4 in 1 Skystars BL32 KO60A (Protocole DSHOT)
<br />
Receveur: TBS Crossfire Nano (Protocole CRSF)
<br />
IMU: MPU-6050 (3 axes gyroscope et 3 axes accéléromètre)
<br />
ou
<br />
IMU: BMI088 (3 axes gyroscope et 3 axes accéléromètre)
<br />
<br />
Note: Simplement pour le plaisir, j'ai installé un lecteur MP3 avec amplificateur puis deux haut-parleurs sur le drone. Malheureusement, le MPU-6050 est trop sensible aux vibrations des haut-parleurs au point que le MPU-6050 devient dans un état instable ou gèle brusquement puis plus moyen de communiquer avec lui. Après quelques recherches, j'ai découvert que le IMU BMI088 de Bosh est moins sensible aux vibrations (selon leur documentation). J'ai donc finalement commandé une plaquette avec un BMI088 chez Seeed Studio. Résultat: ce IMU semble en effet ne pas être affecté par les vibrations des haut-parleurs. Tout semble bien fonctionner. Vous pouvez voir mon premier test extérieur (avec de la musique et des sons de mitraillettes) ici: [https://youtu.be/wTCoblbzLhY] (https://youtu.be/wTCoblbzLhY)
<br />
<br />
Note: J'ai dû cependant diminuer la vitesse de communication (i2c) entre le IMU et le Teensy à 400 KHz. En effet, je pouvais avant communiquer avec le MPU-6050 à 1 MHz ce que je n'ai pas réussi à faire avec le BMI088. Donc j'ai également dû diminuer la vitesse de la boucle principale de 2 Khz à 500 Hz mais tout fonctionne bien quand même.
<br />
<br />
Lecteur MP3 avec amplificateur: HV20T
<br />
Haut-parleurs: AIYIMA 2" 10W 4 Ohm
<br />
<br />
<br />
<br />
30 avril 2023: J'ai fait mon premier test extérieur avec un drone que j'ai construit d'environ 770mm et des hélices de 12 pouces.
<br />
[https://youtu.be/wTCoblbzLhY] (https://youtu.be/wTCoblbzLhY)
<br />
<br />
<br />

#### Avis de non-responsabilité: Ce code source est concédé à titre gratuit sans aucune garantie. Utilisez à vos risques et périls.
#### Disclaimer: This source code is granted free of charge without any guarantee. Use at your own risk.