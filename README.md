# DAWA-6.1 - FR (See below for US)

Site web officiel (FR) : http://dawa.panik-po.com/

DAWA est un système de chronométrage et d'acquisition de données pour votre moto.
Il enregistre les données 10x par seconde dans un fichier CSV.

# Q&A rapide
* __Comment sont récupérées les données ?__  
Pas de prise OBD, protocole trop lent sur ma "veille" moto de 2009. Je récupère les informations directement sur le faisceau du calculateur.
Chaque moto est différente, il faudra sur une autre moto :
    - Identifier les signaux (RPM, vitesse, position poignée de gaz, rapport engagé et frein)
    - Vérifier la forme des signaux (RPM = signal carré, poignée de gaz = signal analogique, etc.)
    - Eventuellement adapter le code au niveau de la programmation si le signal est complètement différent

* __Je peux l'assembler moi même ?__  
Aucun problème, attention néanmoins, il vous faut au minimum :
    - Le circuit imprimé (je peux le fournir, j'en ai quelques uns en stock)
    - Un programmateur pour les 2 µproc Atmel SAMD21 (Atmel ICE BASIC par exemple)
    - Une station de soudure à l'air chaud (indispensable pour souder l'IMU)

* __Combien ça coute ?__  
Grossièrement, voici ce que ça me coûte :
    - PCB : ~10€
    - Composants : ~90€
    - Autres (impression 3D, résine pour les sondes de t°, la journée passée à assembler un exemplaire) : à vous de juger

Certes, ça reste moins cher que ce que l'on peut trouver dans le commerce (Starlane, Alfano, 3DMS) mais ce n'est clairement comparable.  
Vous voulez un vrai chrono, utilisable à la sortie de sa boite, passez votre chemin et dirigez vous vers les produits du commerce.  
Vous voulez un maximum de fonctionnalités, pouvoir faire évoluer un projet libre et avez quelques notions en électronique/développement, ce projet est pour vous :)

# Les nouveautés de la v6
* __Nouveau circuit imprimé__ : DAWA v6 est désormais un circuit imprimé autonome (puce Atmel SAMD21 comme l'Arduino M0). La v5 était un module qui se branchait sur un Arduino M0
* __Nouveau capteur de position__ : Le capteurs 9 axes MPU-9250 est maintenant utilisé. Ce capteur est monté sur un circuit imprimé secondaire avec son propre µproc (Atmel SAMD21)
* __Nouvel écran OLED__ : Un peu plus grand (1.3" vs 0.96"). Le connecteur d’écran (SPI) est compatible avec les écrans LCD couleur 2,4" 240x320
* __Nouveau boitier__ : Un tout nouveau boîtier imprimé en 3D
* __Autodetection des capteurs__ : Les capteurs de température infrarouges sont maintenant détectés automatiquement
* __Navigation__ : Ajout d'un système de menu de navigation et 4 boutons pour permettre la configuration et l'utilisation sans smartphone ni Bluetooth
* __LEDs__ : Ajout de 4 LEDs pour des événements spécifiques (meilleur temps au tour par exemple)
* __Connecteurs__ : tous les connecteurs sont maintenant du même côté (en bas), le connecteur GPS est maintenant de type SMA (à visser)
* __Batterie GPS__ : Batterie rechargeable LIR 2025

# Quelles informations sont enregistrées ?
* __Acquisition de données bruts__ : Sur les motos Triumph et beaucoup d’autres, les valeurs du calculateur peuvent être lues directement (j’utilise personnellement: RPM, vitesse, position poignée de gaz, rapport engagé et frein)
* __Atitude de la moto__ : Un capteur à 9 axes (MPU-9250) est utilisé pour stocker les G et j'espère que bientôt, le roulis et le tangage
* __Position de la moto__ : Une puce GPS UBLOX 10Hz récupère les coordonnées GPS en temps réel
* __Température infrarouge__ : Vous pouvez brancher jusqu'à 6 capteurs de température infrarouge (pneus ou t° au sol, par exemple).
* __Entrées additionnelles__ : Vous pouvez mesurer 2 entrées analogiques et 2 entrées numériques supplémentaires (capteurs de suspension par exemple)

# Où est-ce enregistré ?
Tout est stocké sur une carte micro SD.  
10 fois par seconde, une nouvelle ligne est créée dans un fichier CSV. Cette ligne contient toutes les valeurs de données séparées par un point-virgule.  
Les valeurs actuelles sont affichées en temps réel sur l’écran OLED joint.

# Peut-il être utilisé comme un chronomètre ?
OUI ! Depuis la v4 et l'intégration d'une puce GPS 10Hz, des fonctions de chronométrage sont disponibles.

# Comment ça marche ?
Rien de plus simple !  
Un bouton pour démarrer l'enregistrement et un pour l'arrêter :)  
Un fichier CSV est créé à chaque nouvel enregistrement.  

# Qu'en est-il des fonctions de chronométrage
Un peu plus compliqué, vous devez mettre un fichier nommé "TRACKS.CSV" sur la carte SD.  
Ce fichier contiendra le nom du circuit et les coordonnées de la ligne d'arrivée, une ligne par circuit:  
`<trackname>;<finishline lat. A>;<finishline lon. A>;<finishline lat. B>;<finishline lon. B>`  
`CAROLE;489799930;25224350;489800230;25226330`  
Pour conserver la précision, la latitude et la longitude doivent être converties en nombres entiers (multiplier par 10 000 000).  
Au début de l’enregistrement, la piste la plus proche est automatiquement choisie.

# Vous avez dit Bluetooth?
La connexion Bluetooth est utile dans ces 2 cas:  
- Vous venez de terminer votre session sur piste et vous voulez connaître les temps/meilleurs tours  
- Avant d'utiliser DAWA, certains paramètres peuvent être ajustés, utilisez la console Bluetooth pour les configurer !  
  
J'utilise "Serial Bluetooth Terminal" sur Androïd. Connectez-vous et tapez "help" pour voir toutes les commandes disponibles

# Evolutions
* __Temps intermédiaire__ : Ajout de la gestion des temps intermédiaires

# Bugs connus
* __Angle d'inclinaison__ : Le composant est présent, l'algorithme aussi mais les valeurs dérivent dans le temps. C'est une problématique récurrente.

# Contenu du référentiel
* /3D Case - Pièces Fusion 360 et fichiers STL pour l'impression 3D
* /Arduino - Le fichier .ino que vous devez insérer dans l’Arduino M0
* /Documentation - Quelques informations sur ce projet (FR)
* /Eagle - Tous les fichiers Eagle (schémas, PCB/Gerber, bibliothèques, liste de composants)

# DAWA-6.1 - US

French official website project : http://dawa.panik-po.com/

DAWA is an Arduino datalogger (and laptimer) shield for your motorbike.
It records 10 times per seconds in a CSV file lots of information.

# What's new in v6
* __New PCB__ : DAWA v6 is now a standalone PCB (Atmel SAMD21 chip like Arduino M0). v5 was a shield which needs an Arduino M0
* __New IMU__ : MPU-9250 is now used. This IMU is on a secondary PCB with his own µproc (Atmel SAMD21)
* __New OLED screen__ : A little bit bigger (1.3" vs 0.96"). The screen connector (SPI) is compatible with 2.4" 240x320 color LCD
* __New enclosure__ : A brand new 3D printed enclosure
* __Sensors autodetection__ : Infrared temperature sensors are now autodetected
* __Navigation__ : Add a navigation menu system and 4 buttons to allow configuration and use without smartphone and Bluetooth
* __Lights__ : Add 4 LEDS for specific events (best time on lap for example)
* __Connectors__ : All connectors are now on the same side (bottom), GPS connector is now SMA
* __GPS battery__ : Rechargeable LIR 2025 Battery

# What information is logged ?
* __Raw data acquisition__ : On Triumph bikes and many others, ECU values can be directly read (I personnaly use : RPM, SPEED, GEAR POSITION, THROTTLE and BRAKE state)
* __Environement values__ : A 9-axis sensor (MPU-9250) is used to store G-forces and I hope soon, roll and pitch
* __Position values__ : A UBLOX 10Hz GPS chip gets realtime coordinates
* __Infrared temperatures__ : You can plug up to 6 infrared temperature sensors (tyres or ground t° for example)
* __Additional inputs__ : You can measure 2 analog inputs and 2 digital inputs (suspension sensors for exemple)

# Where is it logged ?
Everything is stored on a micro SD card.  
10 times per seconds, a new line is created in a CSV file. This line contains every data values separated by a semicolon.  
Current values are displayed in realtime on the OLED screen attached.

# Could it be used as a laptimer ?
YES ! Since v4 and the integration of a 10Hz GPS chip, laptimer functions are available.

# How does it work ?
I couldn't make it easier !  
Press the button start recording, press again stop recording :)  
One CSV file is created on each new record.

# What about laptimer functions
A little bit more difficult, you have to put a file named "TRACKS.CSV" on the sdcard.  
This file will contain track name and finishline coordinates, one line per track :  
`<trackname>;<finishline lat. A>;<finishline lon. A>;<finishline lat. B>;<finishline lon. B>`  
`CAROLE;489799930;25224350;489800230;25226330`  
To keep precision, latitude and longitude should be converted to integers (multiply by 10 000 000).  
When start recording the closer track is automatically chosen.

# You said Bluetooth ?
Bluetooth connection is usefull in these 2 cases :
- You just finish your track session and want to know lap times / best lap
- Before using DAWA, some parameters could be adjusted, use the bluetooth console to setup them !

I'm using "Serial Bluetooth Terminal" on Androïd. Connect and type "help" to view all available commands

# What's next  
* __Laptimer split time__ : Add split time management

# Known bugs
* __Lean angle__ : Seems easy but in fact very complicated to obtain good values without drift

# Repository Contents
* /3D Case - Fusion 360 parts and STL files for 3D printing
* /Arduino - The .ino file you need to put in the Arduino M0
* /Documentation - Some brief explanations about this shield (french - not translated)
* /Eagle - All Eagle files (schematics, PCB/Gerber, libraries, parts list)

