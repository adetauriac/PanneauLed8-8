# PanneauLed8-8
Panneau Matrix NeoLed 8*8. Idée d'origine, permettre d'afficher sa disponibilité sur un OpenSpace, mais également la température / Humidité 

 Sur la base : 
  - D'une Matrice de led adressable 8*8 5v 
  - d'un DHT22
  - Support de d'accus 18650 (Shield V") (https://www.banggood.com/fr/Geekcreit-ESP32-ESP32S-18650-Battery-Charge-Shield-V3-Micro-USB-Type-A-USB-0_5A-Test-Charging-Protection-Board-For-Raspberry-Pi-Arduino-p-1265088.html?rmmds=search&cur_warehouse=CN)
       ==> reste a voir avec accus neuf si c'est OK
   
   - 3 STL
      ==> Le corps
      ==> Le fond
      ==> La grille des led a imprimer acec 4-5 couche blanche (coté face du Panneau ) puis en sombre, cela pour éviter que les Pixel "bavent" sur les voisint
        A tester en PETG, avec la chaleur des LED, le PLA se "bouge"
        
     - Le code adruino pour un adruino Nano

La gestion de l'affichage des temperature et humidité est réaliser par bargraphe
A gauche sur 4 de large la temperature
   La temperature demarre a 16° et chaque ligne represente 2°
A droite, sur 3 led de large (pour separation avec temperature) l'humidité, 
  Chaque ligne represente 12%
