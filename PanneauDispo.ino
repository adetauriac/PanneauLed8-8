#include "FastLED.h"
#include "DHT.h"

//DHT22 Pin
#define DHTPIN 7     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);


// How many leds in your strip?
#define NUM_LEDS 64

byte pixelType = 0;
char drawIn[4];
char frameIn[768];

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 2
//#define CLOCK_PIN 13

// Define the array of leds
CRGB leds[NUM_LEDS];


int BRIGHTNESS=10;


#define NumberMenu 4 // Nombre de Menu 1er niveau
int posMenu = 1; // Variable pour identifier les menus


//Init Selecteur
//Encoder rotatif Pin definition Attention D3 est necessaire pour l'intteruption 
const int PinA = 3;  // Used for generating interrupts using CLK signal
const int PinB = 4;  //Used for reading DT signal
const int PinSW = 10;  // Used for the push button switch

// Updated by the ISR (Interrupt Service Routine)
volatile int virtualPosition = 1000;
volatile int lastPosition = 1000;


// ------------------------------------------------------------------
// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT
// ------------------------------------------------------------------
void isr ()  {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 5) {
    if (digitalRead(PinB) == LOW)
    {
      virtualPosition-- ; // Could be -5 or -10
    }
    else {
      virtualPosition++ ; // Could be +5 or +10
    }

    // Restrict value from 0 to +2000
    virtualPosition = min(2000, max(0, virtualPosition));

    // Keep track of when we were here last (no more than every 5ms)
    lastInterruptTime = interruptTime;
  }
}

const uint8_t kSquareWidth = 8;
const uint8_t kBorderWidth = 1;



// Temporisation de la mise à jour du NEOPIXEL 

unsigned long interval = 5000; //Tempo à régler
unsigned long previousMillis = 0; //

bool fist_select = true;
 

void setup() {
  // Uncomment/edit one of the following lines for your leds arrangement.
  // FastLED.addLeds<TM1803, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<TM1804, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<TM1809, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  // FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  // FastLED.addLeds<APA104, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<UCS1903, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<UCS1903B, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<GW6205, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<GW6205_400, DATA_PIN, RGB>(leds, NUM_LEDS);

  // FastLED.addLeds<WS2801, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<SM16716, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<LPD8806, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<P9813, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<APA102, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<DOTSTAR, RGB>(leds, NUM_LEDS);

  // FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<SM16716, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<P9813, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<DOTSTAR, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  Serial.begin(500000);

  // Rotary pulses are INPUTs
  pinMode(PinA, INPUT_PULLUP);
  pinMode(PinB, INPUT_PULLUP);
  // Switch is floating so use the in-built PULLUP so we don't need a resistor
  pinMode(PinSW, INPUT_PULLUP);
  // Attach the routine to service the interrupts
  attachInterrupt(digitalPinToInterrupt(PinA), isr, LOW);

  dht.begin();
  FastLED.setBrightness(BRIGHTNESS);



  
}

void loop() {

 navigation() ; // Verification du mouvement du selecteur afin de naviguer
 

  switch (posMenu){
    case 0 :
       Occupe();
     break;
    case 1:
      dispo();
     break;
    case 2:
      test();
     break;
    case 3:
      unsigned long currentMillis = millis();
      if (fist_select == true){
        previousMillis = millis();
        ReadDHT22();
        fist_select = false;
      }
      if ((unsigned long)(currentMillis - previousMillis) >= interval ) {// tempo de mise à jour des données capteur et de l'afficheur.
        previousMillis = millis();
        ReadDHT22();
    
      }
     break;  
  }
  
  if ((!digitalRead(PinSW))) { //Sortire de la boucle 
        delay(250);
        Serial.print("Click");
        BRIGHTNESS = BRIGHTNESS - 10;
        if (BRIGHTNESS < 0){
          BRIGHTNESS = 64;
        }
        
        FastLED.setBrightness(BRIGHTNESS);
       }
    

}


void navigation() { 
  if ( virtualPosition != lastPosition) {
  
    if  (virtualPosition > lastPosition ) {  //menu suivant
      posMenu = (posMenu + 1 ) % NumberMenu ;
    } else if (virtualPosition < lastPosition ) { //menu precedent
      if (posMenu == 0) {
        posMenu = (NumberMenu);
      }
      posMenu = (posMenu - 1) % NumberMenu;
    }
    lastPosition = virtualPosition;
    Serial.print(F("Position menu : "));
    Serial.println(posMenu);
    fist_select=true;
  
  }

}


void ReadDHT22(){
  
    // Lecture du taux d'humidité
    float h = dht.readHumidity();
    // Lecture de la température en Celcius
    float t = dht.readTemperature();
    // Pour lire la température en Fahrenheit
    float f = dht.readTemperature(true);
    
    // Stop le programme et renvoie un message d'erreur si le capteur ne renvoie aucune mesure
    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println("Echec de lecture !");
      return;
    }
  
    // Calcul la température ressentie. Il calcul est effectué à partir de la température en Fahrenheit
    // On fait la conversion en Celcius dans la foulée
    float hi = dht.computeHeatIndex(f, h);
    
  
    Serial.print("Humidite: "); 
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: "); 
    Serial.print(t);
    Serial.print(" *C ");
    Serial.print("Temperature ressentie: ");
    Serial.print(dht.convertFtoC(hi));
    Serial.println(" *C");

   //********  Led pour affichage Temperature   *******
    for (int i = 0 ; i < 8; i++){
      CleanLigne(1,i);
      CleanLigne(2,i);
    }
   if (t >= 16 )  {//si la température est supérieure à 16°C
      //   Serial.println("Rang 1 allumée");
      for (int i = 56; i < 60; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(138, 0, 255); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (t >= 18 )  {//si la température est supérieure à 17°C
      //   Serial.println("Rang 2 allumée");
      for (int i = 48; i < 52; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(84, 0, 255); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (t >= 20 )  {//si la température est supérieure à 18°C
      //   Serial.println("Rang 3 allumée");
      for (int i = 40; i < 44; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(0, 101, 253); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (t >= 22 )  {//si la température est supérieure à 19°C
      //   Serial.println("Rang 4 allumée");
      for (int i = 32; i < 36; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(0, 253, 253); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (t >= 24 )  {//si la température est supérieure à 20°C
      //   Serial.println("Rang 5 allumée");
      for (int i = 24; i < 28; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(0, 253, 179); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (t >= 26 )  {//si la température est supérieure à 21°C
      //   Serial.println("Rang 6 allumée");
      for (int i = 16; i < 20; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(50, 251, 0); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (t >= 28 )  {//si la température est supérieure à 22°C
      //   Serial.println("Rang 7 allumée");
      for (int i = 8; i < 12; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(255, 252, 0); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (t >= 30 )  {//si la température est supérieure à 23°C
      //   Serial.println("Rang 8 allumée");
      for (int i = 0; i < 4; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(248, 149, 0); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }



   //********  Led pour affichage Humidity   *******

   if (h >= 12 )  {//si la température est supérieure à 16°C
      //   Serial.println("Rang 1 allumée");
      for (int i = 61; i < 64; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(138, 0, 255); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (h >= 24 )  {//si la température est supérieure à 17°C
      //   Serial.println("Rang 2 allumée");
      for (int i = 53; i < 56; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(84, 0, 255); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (h >= 36 )  {//si la température est supérieure à 18°C
      //   Serial.println("Rang 3 allumée");
      for (int i = 45; i < 48; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(0, 101, 253); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (h >= 48 )  {//si la température est supérieure à 19°C
      //   Serial.println("Rang 4 allumée");
      for (int i = 37; i < 40; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(0, 253, 253); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (h >= 60 )  {//si la température est supérieure à 20°C
      //   Serial.println("Rang 5 allumée");
      for (int i = 29; i < 32; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(0, 253, 179); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (h >= 72 )  {//si la température est supérieure à 21°C
      //   Serial.println("Rang 6 allumée");
      for (int i = 21; i < 24; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(50, 251, 0); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (h >= 84 )  {//si la température est supérieure à 22°C
      //   Serial.println("Rang 7 allumée");
      for (int i = 13; i < 16; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(255, 252, 0); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }
   if (h >= 96 )  {//si la température est supérieure à 23°C
      //   Serial.println("Rang 8 allumée");
      for (int i = 5; i < 8; i++) {// on allume les leds de 0 à 1. i étant la led correspondante, il est reconnu dans la fonction suivante.
        leds[i] = CRGB(248, 149, 0); // on a i qui correspond aux leds, 0, 0, 255 --> rouge, vert, bleu.
      }
    }


    
    
    FastLED.show();
    
 
}

void Occupe(){
  leds[0] = CRGB(0, 0, 0);
  leds[1] = CRGB(0, 0, 0);
  leds[2] = CRGB(255, 0, 0);
  leds[3] = CRGB(255, 0, 0);
  leds[4] = CRGB(255, 0, 0);
  leds[5] = CRGB(255, 0, 0);
  leds[6] = CRGB(0, 0, 0);
  leds[7] = CRGB(0, 0, 0);
  leds[8] = CRGB(0, 0, 0);
  leds[9] = CRGB(255, 0, 0);
  leds[10] = CRGB(255, 0, 0);
  leds[11] = CRGB(255, 0, 0);
  leds[12] = CRGB(255, 0, 0);
  leds[13] = CRGB(255, 0, 0);
  leds[14] = CRGB(255, 0, 0);
  leds[15] = CRGB(0, 0, 0);
  leds[16] = CRGB(255, 0, 0);
  leds[17] = CRGB(255, 0, 0);
  leds[18] = CRGB(255, 0, 0);
  leds[19] = CRGB(255, 0, 0);
  leds[20] = CRGB(255, 0, 0);
  leds[21] = CRGB(255, 0, 0);
  leds[22] = CRGB(255, 0, 0);
  leds[23] = CRGB(255, 0, 0);
  leds[24] = CRGB(255, 0, 0);
  leds[25] = CRGB(255, 255, 255);
  leds[26] = CRGB(255, 255, 255);
  leds[27] = CRGB(255, 255, 255);
  leds[28] = CRGB(255, 255, 255);
  leds[29] = CRGB(255, 255, 255);
  leds[30] = CRGB(255, 255, 255);
  leds[31] = CRGB(255, 0, 0);
  leds[32] = CRGB(255, 0, 0);
  leds[33] = CRGB(255, 255, 255);
  leds[34] = CRGB(255, 255, 255);
  leds[35] = CRGB(255, 255, 255);
  leds[36] = CRGB(255, 255, 255);
  leds[37] = CRGB(255, 255, 255);
  leds[38] = CRGB(255, 255, 255);
  leds[39] = CRGB(255, 0, 0);
  leds[40] = CRGB(255, 0, 0);
  leds[41] = CRGB(255, 0, 0);
  leds[42] = CRGB(255, 0, 0);
  leds[43] = CRGB(255, 0, 0);
  leds[44] = CRGB(255, 0, 0);
  leds[45] = CRGB(255, 0, 0);
  leds[46] = CRGB(255, 0, 0);
  leds[47] = CRGB(255, 0, 0);
  leds[48] = CRGB(0, 0, 0);
  leds[49] = CRGB(255, 0, 0);
  leds[50] = CRGB(255, 0, 0);
  leds[51] = CRGB(255, 0, 0);
  leds[52] = CRGB(255, 0, 0);
  leds[53] = CRGB(255, 0, 0);
  leds[54] = CRGB(255, 0, 0);
  leds[55] = CRGB(0, 0, 0);
  leds[56] = CRGB(0, 0, 0);
  leds[57] = CRGB(0, 0, 0);
  leds[58] = CRGB(255, 0, 0);
  leds[59] = CRGB(255, 0, 0);
  leds[60] = CRGB(255, 0, 0);
  leds[61] = CRGB(255, 0, 0);
  leds[62] = CRGB(0, 0, 0);
  leds[63] = CRGB(0, 0, 0);
  FastLED.show();


}


void dispo(){
  leds[0] = CRGB(0, 0, 0);
  leds[1] = CRGB(0, 0, 0);
  leds[2] = CRGB(0, 0, 0);
  leds[3] = CRGB(0, 0, 0);
  leds[4] = CRGB(0, 0, 0);
  leds[5] = CRGB(0, 0, 0);
  leds[6] = CRGB(0, 0, 0);
  leds[7] = CRGB(0, 0, 0);
  leds[8] = CRGB(0, 0, 0);
  leds[9] = CRGB(0, 0, 0);
  leds[10] = CRGB(0, 0, 0);
  leds[11] = CRGB(0, 0, 0);
  leds[12] = CRGB(0, 0, 0);
  leds[13] = CRGB(0, 0, 0);
  leds[14] = CRGB(0, 0, 0);
  leds[15] = CRGB(0, 255, 0);
  leds[16] = CRGB(0, 0, 0);
  leds[17] = CRGB(0, 0, 0);
  leds[18] = CRGB(0, 0, 0);
  leds[19] = CRGB(0, 0, 0);
  leds[20] = CRGB(0, 0, 0);
  leds[21] = CRGB(0, 0, 0);
  leds[22] = CRGB(0, 255, 0);
  leds[23] = CRGB(0, 255, 0);
  leds[24] = CRGB(0, 255, 0);
  leds[25] = CRGB(0, 0, 0);
  leds[26] = CRGB(0, 0, 0);
  leds[27] = CRGB(0, 0, 0);
  leds[28] = CRGB(0, 0, 0);
  leds[29] = CRGB(0, 255, 0);
  leds[30] = CRGB(0, 255, 0);
  leds[31] = CRGB(0, 0, 0);
  leds[32] = CRGB(0, 255, 0);
  leds[33] = CRGB(0, 255, 0);
  leds[34] = CRGB(0, 0, 0);
  leds[35] = CRGB(0, 0, 0);
  leds[36] = CRGB(0, 255, 0);
  leds[37] = CRGB(0, 255, 0);
  leds[38] = CRGB(0, 0, 0);
  leds[39] = CRGB(0, 0, 0);
  leds[40] = CRGB(0, 0, 0);
  leds[41] = CRGB(0, 255, 0);
  leds[42] = CRGB(0, 255, 0);
  leds[43] = CRGB(0, 255, 0);
  leds[44] = CRGB(0, 255, 0);
  leds[45] = CRGB(0, 0, 0);
  leds[46] = CRGB(0, 0, 0);
  leds[47] = CRGB(0, 0, 0);
  leds[48] = CRGB(0, 0, 0);
  leds[49] = CRGB(0, 0, 0);
  leds[50] = CRGB(0, 255, 0);
  leds[51] = CRGB(0, 255, 0);
  leds[52] = CRGB(0, 0, 0);
  leds[53] = CRGB(0, 0, 0);
  leds[54] = CRGB(0, 0, 0);
  leds[55] = CRGB(0, 0, 0);
  leds[56] = CRGB(0, 0, 0);
  leds[57] = CRGB(0, 0, 0);
  leds[58] = CRGB(0, 0, 0);
  leds[59] = CRGB(0, 0, 0);
  leds[60] = CRGB(0, 0, 0);
  leds[61] = CRGB(0, 0, 0);
  leds[62] = CRGB(0, 0, 0);
  leds[63] = CRGB(0, 0, 0);
  FastLED.show();


}

void showProgramCleanUp(long delayTime) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(delayTime);
}

void CleanLigne(int type, int numline){
  int j=0;
  j=numline*8;
  if (type =1) //Temperature, 4 première colonne
  {
    int xxx = j+4;
    for ( int i = j; i < xxx;i++){
      leds[i] = CRGB::Black ;
    }
  }
    if (type =2) //Temperature, 4 première colonne
  {
    int xxx = j+8;
    for ( int i = j; i < xxx;i++){
      leds[i] = CRGB::Black ;
    }
  }

}


void test()
{
  // Apply some blurring to whatever's already on the matrix
  // Note that we never actually clear the matrix, we just constantly
  // blur it repeatedly.  Since the blurring is 'lossy', there's
  // an automatic trend toward black -- by design.
  uint8_t blurAmount = dim8_raw( beatsin8(3,64,192) );
  blur2d( leds, kSquareWidth, kSquareWidth, blurAmount);

  // Use two out-of-sync sine waves
  uint8_t  i = beatsin8(  91, kBorderWidth, kSquareWidth-kBorderWidth);
  uint8_t  j = beatsin8( 109, kBorderWidth, kSquareWidth-kBorderWidth);
  uint8_t  k = beatsin8(  73, kBorderWidth, kSquareWidth-kBorderWidth);
  
  // The color of each point shifts over time, each at a different speed.
  uint16_t ms = millis();  
  leds[XY( i, j)] += CHSV( ms / 29, 200, 255);
  leds[XY( j, k)] += CHSV( ms / 41, 200, 255);
  leds[XY( k, i)] += CHSV( ms / 73, 200, 255);
  
  FastLED.show();
}

void serialEvent() {
  pixelType = Serial.read();

  switch (pixelType) {
    case 0:
    //draw mode
      Serial.readBytes(drawIn, 4);

      leds[drawIn[0]] = CRGB(drawIn[1], drawIn[2], drawIn[3]);

      FastLED.show();
      Serial.flush();
      break;

    case 1:
      //clear mode
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CRGB::Black;
      }

      FastLED.show();
      Serial.flush();
      break;

    case 2:
    //frame in mode
      Serial.readBytes(frameIn, (NUM_LEDS * 3));
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CRGB(frameIn[i * 3], frameIn[(i * 3) + 1], frameIn[(i * 3) + 2]);
      }

      FastLED.show();
      Serial.flush();
      break;

    case 3:
      int brightnessLED = Serial.read();
      FastLED.setBrightness(brightnessLED);
      Serial.flush();

      break;
  }
}

uint16_t XY( uint8_t x, uint8_t y) { return (y * kSquareWidth) + x; }
