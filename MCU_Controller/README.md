# Chef d'orchestre robotique
Le code responsable du fonctionnement du robot principal.

# Todo
- [] Check Watermarks of tasks and optimize
- [] Check if pcnt_mutex is necessary in  ledc_stepper
- [] Check their is no bug with xTaskToNotify in ledc_stepper (rewrite wait system ?)
- [] Make ld19p uart use interrupts: rx_flow_thrhd

# Guide de développement

À rédiger. Il faut refaire le repo proprement.

## Attention !
When adding new files, do a clean run or they won't get picked up by CMake !

## Ressources utiles

- [ESP32-S3 Pin Reference and warnings about them](http://wiki.fluidnc.com/en/hardware/ESP32-S3_Pin_Reference)
- [ESP32-S3 datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)

# Documentation succincte
## Architecture Hardware
### Microcontrôleur
Le code est destiné à un ESP32-S3, choisi pour sa puissance de calcul (2 cœurs à 240Mhz), son nombre de broches (36 GPIO), et ses nombreux périphériques qui peuvent être connectés à n'importe quelle sortie à l'aide de la GPIO MATRIX.

Il fait usage de plusieurs périphériques spécialisés :
- 3 UART : 1 pour le LiDAR, 1 pour la communication avec les 4 TMC2209 des roues, 1 pour communiquer avec les 2 TMC2209 des monte-charges. Il faut donc utiliser l'UART0, ce qui force donc le flash de l'ESP32-S3 via l'interface USB.
- 2 LEDC et 2 PCNT, pour le contrôle des 2 moteurs pas-à-pas des montes charges, à l'aide de la librairie LedcStepper.
- 4 RMT_modules, pour le contrôle de 4 moteurs pas-à-pas, à l'aide de la librairie FastAccelStepper.

Il devrait être plus ou moins facilement adaptable à un autre microcontrôleur utilisant l'API ESP-IDF.

### Électronique
Les roues motrices du robot ainsi que les deux monte-charges sont des moteurs pas-à-pas Nema17.

Chaque monte-charge est constitué :
- D'un système de ventouses constitué de deux ventouses, un solénoïde et une pompe.
- De 4 électroaimants en parallèle, deux par canette.

La carte peut être trouvée ici : [Centrale-Lille-Makers/CdFdR-Carte-electronique](https://github.com/Centrale-Lille-Makers/CdFdR-Carte-electronique)

### Mécanique
Le robot est une base holonome, constituée de 4 roues agencées dans un carré (les dimensions peuvent être trouvées dans le fichier [config.h](main/src/config.h)).

Le modèle CAO peut être trouvé ici : [Onshape/Centrale Lille Makers - CdFR 2025](https://cad.onshape.com/documents/1dd859d76dc0697bb206ffb4/w/155620088c2d8d686e407f28/e/31f5d0088d568de8c32fdbd0?renderMode=0&uiState=68458cdf4c795e4f05b684ab)

## Architecture Software
Le code fait usage de l'API ESP-IDF, destinée à la programmation des microcontrôleurs du constructeur Espressif.
Il se base sur l'utilisation de FreeRTOS pour la gestion des tâches en parallèle.
Il est couplé à [arduino-esp32](https://github.com/espressif/arduino-esp32) qui permet l'usage de fonctions et de librairies Arduino.

### config.h

Fichier de définitions contenant les pins de chaque composant, les paramètres des moteurs pas-à-pas, les paramètres géométriques du robot, et plus.

### main.cpp

Tâche principale, qui fait appel aux sous-modules, et gère la sélection du programme (stratégie, position sur la table).

### Modules

#### Ihm

Chargé de la gestion haut niveau de l'IHM, et notamment de son accès simultané par plusieurs tâches à l'aide de sémaphores. Elle utilise la librairie TM1638plus.

#### Lidar

Chargé de la détection des obstacles, et sera chargé de la localisation (pas encore au point), en interprétant les points du LiDAR. Elle utilise la librairie Ld19p.

#### Lift

Chargé du contrôle des monte-charges : calibration de l'axe linéaire en début de partie, contrôle de la hauteur, contrôle des ventouses et contrôle des électroaimants. Elle utilise la librairie LedcStepper.

#### Motion

Chargé des déplacements de la base roulante. Implémente la cinématique holonome, et permet la réalisation de déplacements relatifs avec une certaine vitesse, accélération et jerk.

Implémente également la mise en pause et la reprise des déplacements, en cas de détection d'un obstacle par le LiDAR. Il est capable d'estimer la position future du robot en fonction de son déplacement actuel, afin d'indiquer au LiDAR où chercher les obstacles. Elle utilise la librairie FastAccelStepper.

### Librairies

#### Ld19p

Librairie pour l'interface avec le LiDAR Ld19p, inspirée du guide [LudovaTech/lidar-LD19-tutorial](https://github.com/LudovaTech/lidar-LD19-tutorial).

Elle fait usage d'une interface UART, et utilise un buffer et des interruptions afin de stocker pouvoir à tout moment récupérer le dernier scan complet du LiDAR. Elle s'occupe de la vérification et de l'interprétation des données, et fournit une liste de points avec leur angle et leur distance.

#### LedcStepper

Librairie sur mesure pour le contrôle des 2 moteurs pas-à-pas des monte-charges.

Elle est capable de contrôler en position ou en vitesse un moteur pas à pas, de façon asynchrone, et de manière performante, en utilisant très peu d'interruptions.

Elle utilise un périphérique LEDC pour générer les impulsions de contrôle à la bonne fréquence, et un périphérique PCNT (Pulse Counter) pour compter les impulsions et générer une interruption afin d'arrêter le moteur à la position souhaitée. Les deux sont reliées au même pin à l'aide de la GPIO Matrix de l'ESP32.

#### Externes

##### FastAccelStepper

[gin66/FastAccelStepper](https://github.com/gin66/FastAccelStepper)

Utilisée pour le contrôle des 4 moteurs pas-à-pas des roues.

##### TM1638plus

[gavinlyonsrepo/TM1638plus](https://github.com/gavinlyonsrepo/TM1638plus)

Utilisée pour l'Interface Homme-Machine (IHM) ([modèle ici](https://github.com/gavinlyonsrepo/TM1638plus?tab=readme-ov-file#model-one)).