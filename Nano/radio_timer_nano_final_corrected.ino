/*
 * Radio Zendtijd Timer - Arduino Nano (VERSIMPELDE VERSIE)
 * ALLEEN D4 GEBRUIKT VOOR AUDIO - GEEN BUZZER
 * 
 * GEBRUIK NIET VOOR NORMALE AMATEUR RADIO UITZENDING!
 * Deze versie is alleen voor testdoeleinden met langere tijden.
 */

// Arduino headers voor Arduino Nano borden
#include <Arduino.h>
#include <avr/pgmspace.h>  // Voor PROGMEM

// Pin definities (ARDUINO NANO - ATmega328P)
#define PTT_SWITCH_PIN     2     // PTT input (met pull-up)
#define TONE_PIN          4      // 1kHz toon output (tone functie) - ALLEEN AUDIO OUTPUT
#define PTT_RELAY_PIN     7      // PTT relay driver
#define LED_STATUS_PIN    13     // Status LED (Nano onboard LED)
#define TIMER_POT_PIN     A0     // Timer instelling potmeter (analoog)

// 7-Segment Display pinnen (Common Cathode)
#define SEGMENT_A_PIN     8      // Segment A (boven) 
#define SEGMENT_B_PIN     9      // Segment B (rechtsboven)
#define SEGMENT_C_PIN     10     // Segment C (rechtsonder)
#define SEGMENT_D_PIN     11     // Segment D (onder)
#define SEGMENT_E_PIN     12     // Segment E (linksonder)
#define SEGMENT_F_PIN     3      // Segment F (linksboven)
#define SEGMENT_G_PIN     A2     // Segment G (midden) - verplaatst naar A2
#define SEGMENT_DP_PIN    A1     // Decimal Point (analog pin als digitaal)

// Array met alle segment pinnen voor eenvoudige aansturing
const int segmentPins[7] PROGMEM = {
  SEGMENT_A_PIN,    // 0: Segment A
  SEGMENT_B_PIN,    // 1: Segment B  
  SEGMENT_C_PIN,    // 2: Segment C
  SEGMENT_D_PIN,    // 3: Segment D
  SEGMENT_E_PIN,    // 4: Segment E
  SEGMENT_F_PIN,    // 5: Segment F
  SEGMENT_G_PIN     // 6: Segment G
};

// Digit patterns voor 0-9 (Common Cathode - HIGH = aan) - NAAR PROGMEM
const byte digitPatterns[10] PROGMEM = {
  0b01111110, // 0: A,B,C,D,E,F aan (G uit)
  0b00110000, // 1: B,C aan (alle andere uit)
  0b01101101, // 2: A,B,G,E,D aan
  0b01111001, // 3: A,B,C,D,G aan
  0b00110011, // 4: F,G,B,C aan
  0b01011011, // 5: A,F,G,C,D aan
  0b01011111, // 6: A,F,G,C,D,E aan
  0b01110000, // 7: A,B,C aan
  0b01111111, // 8: Alle segmenten aan
  0b01111011  // 9: A,B,C,D,F,G aan
};

// Letter patterns voor letters (Common Cathode - HIGH = aan) - NAAR PROGMEM
const byte letterPatterns[26] PROGMEM = {
  0b01110111, // A: A,B,C,E,F,G aan (D uit)
  0b00111111, // B: C,D,E,F,G aan
  0b01011110, // C: A,D,E,F aan
  0b00111101, // D: B,C,D,E,G aan (zoals O maar met G aan)
  0b01001111, // E: A,D,E,F,G aan (alleen B,C uit)
  0b01000111, // F: A,E,F,G aan (alleen B,C,D uit)
  0b00111111, // G: B,C,D,E,F,G aan (A uit)
  0b00110111, // H: B,C,E,F,G aan (D uit)
  0b00001110, // I: E,F,G aan (alleen basis)
  0b00111110, // J: B,C,D,E,F aan (A uit)
  0b00000000, // K: (niet implementeerbaar op 7-seg)
  0b00001110, // L: D,E,F aan (A,B,C,G uit)
  0b00000000, // M: (niet implementeerbaar op 7-seg)
  0b00010101, // N: C,E,G aan (D,E,F uit)
  0b00111110, // O: B,C,D,E,F aan (A,G uit)
  0b01100111, // P: A,B,E,F,G aan (C,D uit)
  0b01100111, // Q: A,B,C,F,G aan (D,E uit)
  0b01100111, // R: A,B,E,F,G aan (IDENTIEK aan P)
  0b01111011, // S: A,C,D,F,G aan (B,E uit)
  0b00011100, // T: D,E,F,G aan (A,B,C uit)
  0b00111110, // U: B,C,D,E,F aan (A,G uit)
  0b00111110, // V: B,C,D,E,F aan (A,G uit)
  0b00000000, // W: (niet implementeerbaar op 7-seg)
  0b00110111, // X: B,C,E,F,G aan (zoals H)
  0b00110111, // Y: B,C,E,F,G aan (zoals H)
  0b01101101  // Z: A,B,D,E,G aan (zoals 2)
};

// Timer bereik (seconden) - TEST MODE voor snelle validatie
#define MIN_TIMER_SECONDS  30.0     // Minimum 30 seconden
#define MAX_TIMER_SECONDS  300.0    // Maximum 300 seconden
#define WARNING_TIME_FROM_END 15   // Waarschuwings toon 15 sec voor einde
#define BEEP_TIME_FROM_END    6    // Beeps 6 sec voor einde
#define WARNING_TONE_DURATION 1000  // 1 seconde waarschuwings toon
#define BEEP_DURATION          5000  // 5 seconden durende beeps

// Audio constanten
#define TONE_FREQUENCY     1000     // 1kHz toon (warning tone)
#define BEEP_FREQUENCY     500      // 500Hz beeps (snel pulserend)
#define BEEP_PULSE_INTERVAL 100     // 100ms interval voor pulserende beeps

// Status variabelen
bool timerActive = false;
bool warningToneActive = false;
bool warningTonePlayed = false;       // EENMALIGE tone tracking
bool warningBeepsActive = false;
bool maxTimeReached = false;
unsigned long timerStartTime = 0;
unsigned long lastBeepTime = 0;
bool pttCurrent = false;
bool pttPrevious = false;

// VASTE TIJDSTIPPEN (berekend bij TX start)
unsigned long warningToneMs = 0;     // Wanneer warning tone start (t-15)
unsigned long warningBeepsMs = 0;    // Wanneer beeps starten (t-6)
unsigned long timerEndMs = 0;        // Wanneer timer eindigt

// Timer instellingen (dynamisch via potmeter)
float totalTimerSeconds = 0;
unsigned long totalTimerMs = 0;
unsigned long lastPotReadTime = 0;
const unsigned long POT_READ_INTERVAL = 500;  // Potmeter elke 500ms lezen

// Decimal Point (DP) knipper variabelen
unsigned long lastDPToggleTime = 0;
bool dpState = false;
const unsigned long DP_BLINK_INTERVAL = 500;  // DP knippert elke 0.5 seconden

// Debounce variabelen
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

// PROGMEM helper variabelen
byte currentPattern = 0;
int segmentPinArray[7];

void setup() {
  // Pin configuratie voor Arduino Nano
  pinMode(PTT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(TONE_PIN, OUTPUT);       // Alleen D4 voor audio
  pinMode(PTT_RELAY_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  pinMode(TIMER_POT_PIN, INPUT);         // A0 als analoge input
  pinMode(SEGMENT_DP_PIN, OUTPUT);       // A1 als digitale output voor DP
  pinMode(SEGMENT_G_PIN, OUTPUT);        // A2 als digitale output voor Segment G
  
  // 7-Segment Display pinnen configureren
  for (int i = 0; i < 7; i++) {
    // Lees segment pins van PROGMEM
    segmentPinArray[i] = pgm_read_word(&segmentPins[i]);
    pinMode(segmentPinArray[i], OUTPUT);
    digitalWrite(segmentPinArray[i], LOW);  // Alle segmenten uit bij start
  }
  digitalWrite(SEGMENT_DP_PIN, LOW);   // DP uit bij start
  
  // Initialiseer uitgangen
  digitalWrite(PTT_RELAY_PIN, LOW);  // PTT uit bij start
  digitalWrite(LED_STATUS_PIN, LOW);
  
  // Lees potmeter en stel timer in
  readTimerPot();
  updateTimerSettings();
  
  // GEEN SERIELE OUTPUT - VOOR MINDER TX LED ACTIVITEIT
  // Serial.begin(9600);  // UITGESCHAKELD
  
  // Welcome LED sequence (zonder output)
  welcomeSequence();
  
  // 7-Segment Display test bij opstarten (zonder output)
  displayStartupTest();
}

void loop() {
  // Potmeter periodiek uitlezen
  if (millis() - lastPotReadTime >= POT_READ_INTERVAL) {
    readTimerPot();
    updateTimerSettings();
    lastPotReadTime = millis();
  }
  
  // Debounced PTT reading
  bool currentPTT = digitalRead(PTT_SWITCH_PIN);
  
  // Debounce logic
  if (currentPTT != pttCurrent) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // PTT state is stable
    if (currentPTT != pttPrevious) {
      if (currentPTT == LOW) {  // PTT ingedrukt (active LOW)
        if (!timerActive) {
          startRadioSession();
        }
      } else {  // PTT losgelaten
        resetTimer();
      }
      pttPrevious = currentPTT;
    }
  }
  pttCurrent = currentPTT;
  
  // Timer processing met VASTE TIJDSTIPPEN
  if (timerActive) {
    unsigned long currentMs = millis();
    unsigned long elapsed = currentMs - timerStartTime;
    
    // PTT WATCHER: Zorg ervoor dat PTT HIGH blijft tijdens hele timer
    ensurePTTStaysHigh();
    
    // 7-Segment Display update met percentage
    updateDisplayPercentage(elapsed);
    
    // WAARSCHUWING TONE (t-15): Speel EENMALIG 1 seconde toon
    if (!warningTonePlayed && currentMs >= warningToneMs) {
      warningTonePlayed = true;  // Marker dat we dit hebben gedaan
      tone(TONE_PIN, TONE_FREQUENCY);  // 1kHz toon
      
      // DEBUG: Snelle LED flits om aan te geven dat warning tone start
      digitalWrite(LED_STATUS_PIN, HIGH);
      delay(50);
      digitalWrite(LED_STATUS_PIN, LOW);
      
      // Start watchdog timer voor 1 seconde duur
      warningToneActive = true;
      warningToneMs = currentMs;  // Reset naar huidige tijd als startpunt
    }
    
    // Stop toon na 1 seconde (watchdog)
    if (warningToneActive && (currentMs - warningToneMs >= WARNING_TONE_DURATION)) {
      warningToneActive = false;
      noTone(TONE_PIN);
    }
    
    // WAARSCHUWINGS BEEPS (t-6): Start pulserende beeps
    if (!warningBeepsActive && currentMs >= warningBeepsMs) {
      warningBeepsActive = true;
      lastBeepTime = currentMs - BEEP_PULSE_INTERVAL;  // Reset zodat eerste beep direct speelt
      
      // DEBUG: Snelle LED flits om aan te geven dat beeps starten
      digitalWrite(LED_STATUS_PIN, HIGH);
      delay(50);
      digitalWrite(LED_STATUS_PIN, LOW);
    }
    
    // Beep generatie (wordt elke cyclus aangeroepen)
    if (warningBeepsActive && timerActive) {
      handleWarningBeeps();
    }
    
    // EINDE VAN TIMER (na 5-sec beeps): PTT UIT
    if (currentMs >= timerEndMs) {
      timerActive = false;
      warningBeepsActive = false;
      digitalWrite(PTT_RELAY_PIN, LOW);  // PTT UIT bij einde
      noTone(TONE_PIN);
      digitalWrite(LED_STATUS_PIN, LOW);
      
      // Display reset naar 0
      displayDigit(0);
      
      // DP resetten
      digitalWrite(SEGMENT_DP_PIN, LOW);
      dpState = false;
      lastDPToggleTime = millis();
    }
  } else {
    // Timer niet actief: display toont 0
    displayDigit(0);
  }
  
  // Status LED
  updateStatusLED();
  
  // Decimal Point knipperen tijdens PTT actief (timerActive)
  updateDPBlink();
  
  // Small delay for stability
  delay(10);
}

void startRadioSession() {
  // Reset alle states
  timerActive = true;
  maxTimeReached = false;
  warningToneActive = false;
  warningTonePlayed = false;   // EENMALIGE tone marker
  warningBeepsActive = false;
  timerStartTime = millis();
  lastBeepTime = 0;
  
  // BEREKEN VASTE TIJDSTIPPEN BIJ START (gebruik current potmeter waarde)
  unsigned long currentTimerMs = (unsigned long)(totalTimerSeconds * 1000.0);
  warningToneMs = timerStartTime + (currentTimerMs - 15000);   // t-15 seconden
  warningBeepsMs = timerStartTime + (currentTimerMs - 6000);   // t-6 seconden
  timerEndMs = timerStartTime + currentTimerMs;               // Einde timer
  
  // KRITIEK: Activeer PTT en hou HIGH tijdens hele sessie
  digitalWrite(PTT_RELAY_PIN, HIGH);
  
  // Reset audio outputs
  noTone(TONE_PIN);  // Alle audio uit
  
  // Display start met 0% (nog geen percentage getoond)
  displayDigit(0);
}

void resetTimer() {
  // Stop alle timers
  timerActive = false;
  maxTimeReached = false;
  warningToneActive = false;
  warningTonePlayed = false;
  warningBeepsActive = false;
  
  // Reset alle outputs
  digitalWrite(PTT_RELAY_PIN, LOW);  // PTT UIT (manual reset)
  noTone(TONE_PIN);       // Alle audio uit
  digitalWrite(LED_STATUS_PIN, LOW);
  
  // Display weer op 0 bij reset
  displayDigit(0);
  
  // DP ook resetten
  digitalWrite(SEGMENT_DP_PIN, LOW);
  dpState = false;
  lastDPToggleTime = millis();
}

void startWarningBeeps() {
  warningBeepsActive = true;
  // Beeps worden gegenereerd in loop()
}

void handleWarningBeeps() {
  unsigned long currentTime = millis();
  
  // Snel pulserende toon op D4 (100ms interval)
  if (currentTime - lastBeepTime >= BEEP_PULSE_INTERVAL) {
    static bool beepOn = false;  // Toggle aan/uit
    
    if (beepOn) {
      // TOON UIT
      noTone(TONE_PIN);
    } else {
      // TOON AAN (500Hz)
      tone(TONE_PIN, BEEP_FREQUENCY);
    }
    
    beepOn = !beepOn;  // Toggle voor volgende keer
    lastBeepTime = currentTime;
  }
}

void updateStatusLED() {
  if (timerActive) {
    if (warningBeepsActive) {
      // Snel knipperen bij beep (500ms)
      digitalWrite(LED_STATUS_PIN, (millis() / 500) % 2);
    } else if (warningToneActive) {
      // Middelmatig knipperen bij 1sec warning tone (500ms)
      digitalWrite(LED_STATUS_PIN, (millis() / 500) % 2);
    } else if (warningTonePlayed) {
      // Langzaam knipperen tussen tone en beeps (1000ms)
      digitalWrite(LED_STATUS_PIN, (millis() / 1000) % 2);
    } else {
      // Continu aan tijdens normale operatie
      digitalWrite(LED_STATUS_PIN, HIGH);
    }
  } else {
    digitalWrite(LED_STATUS_PIN, LOW);
  }
}

void readTimerPot() {
  // Lees analoog (0-1023 voor Arduino Nano) - ABSOLUTE TIMING seconden
  int potValue = analogRead(TIMER_POT_PIN);
  
  // Converteer naar seconden (30-300 sec voor Nano)
  float potFraction = (float)potValue / 1023.0;
  totalTimerSeconds = MIN_TIMER_SECONDS + (potFraction * (MAX_TIMER_SECONDS - MIN_TIMER_SECONDS));
}

void updateTimerSettings() {
  // Bereken totale tijd in milliseconden - ABSOLUTE TIMING
  totalTimerMs = (unsigned long)(totalTimerSeconds * 1000.0);
  
  // De warning tijden worden berekend in startRadioSession()
  // op basis van de actuele timerStartTime
}

void welcomeSequence() {
  // LED test - GEEN SERIELE OUTPUT
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(200);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(200);
  }
  
  // TEST toon (1kHz)
  tone(TONE_PIN, TONE_FREQUENCY);
  delay(500);
  noTone(TONE_PIN);
  
  // TEST beeps (500Hz pulserend)
  for (int i = 0; i < 6; i++) {
    tone(TONE_PIN, BEEP_FREQUENCY);
    delay(100);
    noTone(TONE_PIN);
    delay(100);
  }
}

// NIEUWE FUNCTIES VOOR 7-SEGMENT DISPLAY

void displayDigit(int digit) {
  // Controleer of digit geldig is (0-9)
  if (digit < 0 || digit > 9) {
    // Bij ongeldige waarde: alle segmenten uit
    for (int i = 0; i < 7; i++) {
      digitalWrite(segmentPinArray[i], LOW);
    }
    return;
  }
  
  // Haal pattern op van PROGMEM voor het gevraagde cijfer
  currentPattern = pgm_read_byte(&digitPatterns[digit]);
  
  // Zet segmenten aan volgens pattern (Common Cathode: HIGH = aan)
  for (int i = 0; i < 7; i++) {
    bool segmentOn = (currentPattern >> (6-i)) & 0x01;
    digitalWrite(segmentPinArray[i], segmentOn ? HIGH : LOW);
  }
}

void displayLetter(char letter) {
  // Converteer naar hoofdletters als dat nog niet zo is
  letter = toupper(letter);
  
  // Controleer of letter geldig is (A-Z)
  if (letter < 'A' || letter > 'Z') {
    // Bij ongeldige waarde: alle segmenten uit
    for (int i = 0; i < 7; i++) {
      digitalWrite(segmentPinArray[i], LOW);
    }
    return;
  }
  
  int letterIndex = letter - 'A';
  
  // Controleer of letter implementeerbaar is - VAN PROGMEM
  currentPattern = pgm_read_byte(&letterPatterns[letterIndex]);
  if (currentPattern == 0) {
    // Letter niet implementeerbaar - knipper als waarschuwing
    // Knipper alle segmenten 3 keer
    for (int blink = 0; blink < 3; blink++) {
      for (int i = 0; i < 7; i++) {
        digitalWrite(segmentPinArray[i], HIGH);
      }
      delay(100);
      for (int i = 0; i < 7; i++) {
        digitalWrite(segmentPinArray[i], LOW);
      }
      delay(100);
    }
    return;
  }
  
  // Zet segmenten aan volgens pattern (Common Cathode: HIGH = aan)
  for (int i = 0; i < 7; i++) {
    bool segmentOn = (currentPattern >> (6-i)) & 0x01;
    digitalWrite(segmentPinArray[i], segmentOn ? HIGH : LOW);
  }
}

void updateDPBlink() {
  // Decimal Point knippert alleen tijdens PTT actief (timerActive = true)
  if (timerActive) {
    unsigned long currentTime = millis();
    
    // Check of het tijd is om de DP status te wijzigen
    if (currentTime - lastDPToggleTime >= DP_BLINK_INTERVAL) {
      dpState = !dpState;  // Toggle DP status
      digitalWrite(SEGMENT_DP_PIN, dpState ? HIGH : LOW);
      lastDPToggleTime = currentTime;
    }
  } else {
    // Timer niet actief: DP uit
    if (dpState) {  // Alleen als het nog niet uit is
      digitalWrite(SEGMENT_DP_PIN, LOW);
      dpState = false;
    }
    lastDPToggleTime = millis();  // Reset timing voor volgende activatie
  }
}

void updateDisplayPercentage(unsigned long elapsed) {
  if (elapsed >= totalTimerMs) {
    // Timer klaar, toon 0
    displayDigit(0);
    return;
  }
  
  // Bereken percentage
  float percentage = (float)elapsed / totalTimerMs * 100.0;
  int currentDigit = (int)(percentage / 10.0 + 0.5); // Afronden naar nearest integer
  
  // Clamp naar 0-9 range
  if (currentDigit > 9) currentDigit = 9;
  if (currentDigit < 0) currentDigit = 0;
  
  // Update display (alleen als waarde veranderd is)
  static int lastDisplayDigit = -1;
  if (currentDigit != lastDisplayDigit) {
    displayDigit(currentDigit);
    lastDisplayDigit = currentDigit;
  }
}

void displayStartupTest() {
  // PE2PVD sequence - elk character 0.5 seconden
  const char callSign[] = "PE2PVD";  // Array in plaats van String
  
  for (int i = 0; i < 6; i++) {  // Direct 6 in plaats van callSign.length()
    char ch = callSign[i];
    
    if (ch >= '0' && ch <= '9') {
      // Cijfer - gebruik digitPatterns
      int digit = ch - '0';
      displayDigit(digit);
    } else if (ch >= 'A' && ch <= 'Z') {
      // Letter - gebruik letterPatterns
      int letterIndex = ch - 'A';
      byte pattern = pgm_read_byte(&letterPatterns[letterIndex]);
      
      displayLetter(ch);
    }
    
    delay(500); // 0.5 seconde per character
  }

  // Alle segmenten uit na test
  for (int i = 0; i < 7; i++) {
    digitalWrite(segmentPinArray[i], LOW);
  }

  // Korte pauze 
  delay(1000);
    
  // Toon 0 als standaard display na test
  displayDigit(0);
}

// NIEUWE FUNCTIE: PTT WATCHER - Zorgt ervoor dat PTT HIGH blijft tijdens warning fasen
void ensurePTTStaysHigh() {
  // Als timer actief is en PTT per ongeluk LOW zou zijn, maak het weer HIGH
  if (timerActive && !maxTimeReached) {
    if (!digitalRead(PTT_RELAY_PIN)) {
      digitalWrite(PTT_RELAY_PIN, HIGH);
    }
  }
}