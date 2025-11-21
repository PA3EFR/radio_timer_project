/*
 * Radio Zendtijd Timer - ESP32 Lolin32 Lite met 7-Segment Display
 * TEST MODE - ABSOLUTE TIMING VERSIE
 * SOFTWARE PWM VERSIE (betrouwbaar voor Arduino IDE)
 * 
 * GEBRUIK NIET VOOR NORMALE AMATEUR RADIO UITZENDING!
 * Deze versie is alleen voor testdoeleinden met langere tijden.
 * 
 * TEST TIMER BEREIK: 20-60 seconden (potmeter instelbaar)
 * GEBRUIKT ARDUINO tone() FUNCTIES (GEEN ESP32 LEDC)
 * ABSOLUTE TIMING:
 * - Timer - 15 sec = Waarschuwings toon (1 sec)
 * - Timer - 6 sec = Snel pulserende beeps (5 sec durend)
 * - Einde timer = PTT automatisch uit
 * - Reset = bij PTT loslaten
 * 
 * 7-SEGMENT DISPLAY FUNCTIONALITEIT:
 * - Bij opstarten: LED test (alle segmenten 1-voor-1)
 * - In rust (geen PTT): display toont "0"
 * - Bij PTT actief: percentage van maximale zendtijd (10%="1", 20%="2", etc.)
 * - Bij 100%: display weer "0"
 * 
 * Auteur: MiniMax Agent
 * Bord: Lolin32 Lite (ESP32)
 */

// ESP32 headers voor LEDC (PWM) functionaliteit
// Voor Lolin32 Lite borden
#include <Arduino.h>

// Pin definities (ESP32 Lolin32 Lite - SOFTWARE PWM)
#define PTT_SWITCH_PIN     4    // PTT input (met pull-up)
#define TONE_PIN          17    // 1kHz toon output (tone functie)
#define BEEP_PIN          18    // Beep piepjes output (tone functie)
#define PTT_RELAY_PIN     16    // PTT relay driver
#define LED_STATUS_PIN    22    // Status LED (ESP32 onboard)
#define TIMER_POT_PIN     34    // Timer instelling potmeter (analoog)

// 7-Segment Display pinnen (Common Cathode)
// Aangepast om conflict met PTT_RELAY_PIN (GPIO 16) te vermijden
#define SEGMENT_A_PIN     2     // Segment A (boven) 
#define SEGMENT_B_PIN     12    // Segment B (rechtsboven)
#define SEGMENT_C_PIN     13    // Segment C (rechtsonder)
#define SEGMENT_D_PIN     14    // Segment D (onder)
#define SEGMENT_E_PIN     27    // Segment E (linksonder)
#define SEGMENT_F_PIN     26    // Segment F (linksboven)
#define SEGMENT_G_PIN     25    // Segment G (midden)
#define SEGMENT_DP_PIN    0     // Decimal Point (optioneel)

// Array met alle segment pinnen voor eenvoudige aansturing
const int segmentPins[7] = {
  SEGMENT_A_PIN,    // 0: Segment A
  SEGMENT_B_PIN,    // 1: Segment B  
  SEGMENT_C_PIN,    // 2: Segment C
  SEGMENT_D_PIN,    // 3: Segment D
  SEGMENT_E_PIN,    // 4: Segment E
  SEGMENT_F_PIN,    // 5: Segment F
  SEGMENT_G_PIN     // 6: Segment G
};

// Digit patterns voor 0-9 (Common Cathode - HIGH = aan)
const byte digitPatterns[10] = {
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

// Letter patterns voor letters (Common Cathode - HIGH = aan)
// Geoptimaliseerd voor "PE2PVD" call sign
const byte letterPatterns[26] = {
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
#define MIN_TIMER_SECONDS  30.0     // Minimum  seconden
#define MAX_TIMER_SECONDS  300.0     // Maximum  seconden
#define WARNING_TIME_FROM_END 15   // Waarschuwings toon 15 sec voor einde
#define BEEP_TIME_FROM_END    6    // Beeps 6 sec voor einde
#define WARNING_TONE_DURATION 1000  // 1 seconde waarschuwings toon
#define BEEP_DURATION          5000  // 5 seconden durende beeps
// TEST MODE: Timer instelbaar 20-60 sec, absolute timing van eindtijd

// Audio constanten
#define TONE_FREQUENCY     1000     // 1kHz toon
#define BEEP_FREQUENCY     500      // 500Hz beeps
#define BEEP_ON_TIME       200      // Beep aan tijd
#define BEEP_INTERVAL      1000     // Tijd tussen beeps

// Software PWM configuratie (Arduino tone() functies)
#define TONE_FREQUENCY     1000     // 1kHz toon
#define BEEP_FREQUENCY     500      // 500Hz beeps

// Status variabelen
bool timerActive = false;
bool warningToneActive = false;
bool warningToneCompleted = false;
bool warningBeepsActive = false;
bool warningTimeReached = false;
bool maxTimeReached = false;
unsigned long timerStartTime = 0;
unsigned long lastBeepTime = 0;
unsigned long warningToneStartTime = 0;  // Voor 1-seconde timing
bool pttCurrent = false;
bool pttPrevious = false;

// Timer instellingen (dynamisch via potmeter) - TEST MODE seconden
float totalTimerSeconds = 0;             // Totale tijd in seconden
unsigned long totalTimerMs = 0;          // Totale tijd in milliseconden
unsigned long warningTimeMs = 0;         // Waarschuwings tijd
unsigned long beepTimeMs = 0;            // Beep start tijd
unsigned long lastPotReadTime = 0;
const unsigned long POT_READ_INTERVAL = 500;  // Potmeter elke 500ms lezen

// LEDC (LED Control) voor PWM tonen
volatile bool toneState = false;
volatile unsigned long lastToggleTime = 0;

// Decimal Point (DP) knipper variabelen
unsigned long lastDPToggleTime = 0;
bool dpState = false;
const unsigned long DP_BLINK_INTERVAL = 500;  // DP knippert elke 0.5 seconden

// Debounce variabelen
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

void setup() {
  // Pin configuratie
  pinMode(PTT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(PTT_RELAY_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  
  // 7-Segment Display pinnen configureren
  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
    digitalWrite(segmentPins[i], LOW);  // Alle segmenten uit bij start
  }
  pinMode(SEGMENT_DP_PIN, OUTPUT);
  digitalWrite(SEGMENT_DP_PIN, LOW);   // DP uit bij start
  
  // Potmeter (analoge input - geen pinMode nodig voor ESP32 analoge pins)

  
  // Initialiseer uitgangen
  digitalWrite(PTT_RELAY_PIN, LOW);  // PTT uit bij start
  digitalWrite(LED_STATUS_PIN, LOW);
  
  // Lees potmeter en stel timer in
  readTimerPot();
  updateTimerSettings();

  
  // Setup PWM kanalen voor tonen
  setupPWMChannels();
  
  // Seriële communicatie voor debugging
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Radio Timer ESP32 met 7-Segment Display - Lolin32 Lite ===");
  Serial.println("TEST MODE: Potmeter instelbaar: 30-300 SECONDEN timer");
  Serial.println("Timer-15sec = 1kHz toon (1 sec), Timer-6sec = snel beeps (5 sec)");
  Serial.println("7-Segment Display: Opstarttest, Rust=0, PTT actief=percentage");
  Serial.println("*** GEBRUIK NIET VOOR NORMALE UITZENDT ***");
  Serial.println("Klaar voor gebruik...\n");
  
  // Welcome LED sequence
  welcomeSequence();
  
  // 7-Segment Display test bij opstarten
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
  
  // Timer processing
  if (timerActive) {
    unsigned long elapsed = millis() - timerStartTime;
    
    // PTT WATCHER: Zorg ervoor dat PTT HIGH blijft tijdens hele timer
    ensurePTTStaysHigh();
    
    // 7-Segment Display update met percentage
    updateDisplayPercentage(elapsed);
    
    // ABSOLUTE TIMING: Waarschuwings tijd bereikt (Timer - 15 sec) - activeer EENMALIGE 1sec waarschuwings toon
    if (!warningTimeReached && elapsed >= warningTimeMs) {
      warningTimeReached = true;
      warningToneActive = true;
      warningToneStartTime = elapsed;
      Serial.print("🔊 Waarschuwings toon gestart (1 seconde) - PTT BLIJFT HIGH (");
      Serial.print(totalTimerSeconds);
      Serial.print(" sec timer - ");
      Serial.print(WARNING_TIME_FROM_END);
      Serial.println(" sec voor einde)");
    }
    
    // EENMALIGE TONE: Stop toon na 1 seconde
    if (warningToneActive && (elapsed - warningToneStartTime >= WARNING_TONE_DURATION)) {
      warningToneActive = false;
      warningToneCompleted = true;
      noTone(TONE_PIN);
      Serial.println("🔇 Waarschuwings toon klaar - Wachten op beeps...");
    }
    
    // ABSOLUTE TIMING: Beep tijd bereikt (Timer - 6 sec) - start snel pulserende beeps
    if (!warningBeepsActive && elapsed >= beepTimeMs) {
      warningBeepsActive = true;
      Serial.print("⚡ SNEL PULSERENDE BEEPS gestart - PTT BLIJFT HIGH (nog ");
      Serial.print(BEEP_TIME_FROM_END);
      Serial.println(" sec)");
    }
    
    // Na 5-seconden durende beeps: stop beeps MAAR timer blijft actief tot einde
    if (warningBeepsActive && (elapsed - beepTimeMs >= BEEP_DURATION)) {
      // 5-sec beep sequence klaar - stop beeps maar timer blijft actief tot einde
      warningBeepsActive = false;
      noTone(BEEP_PIN);
      
      Serial.println("⚡ 5-SECONDEN SNEL PULSERENDE BEEPS KLAAR - PTT BLIJFT HIGH tot einde!");
    }
    
    // Einde van timer: PTT UIT en display reset naar 0
    if (elapsed >= totalTimerMs) {
      timerActive = false;
      digitalWrite(PTT_RELAY_PIN, LOW);  // PTT UIT bij einde van timer
      warningBeepsActive = false;
      noTone(BEEP_PIN);
      digitalWrite(LED_STATUS_PIN, LOW);
      
      // Display weer op 0 bij einde
      displayDigit(0);
      
      // DP ook resetten bij timer einde
      digitalWrite(SEGMENT_DP_PIN, LOW);
      dpState = false;
      lastDPToggleTime = millis();
      
      Serial.println("⏹️  TIMER EINDE - PTT UIT! Display reset naar 0");
    }
  } else {
    // Timer niet actief: display toont 0
    displayDigit(0);
  }
  
  // Beep generatie met timer (beeps doen PTT relay UIT blijven)
  if (warningBeepsActive && timerActive) {
    handleWarningBeeps();
  }
  
  // Generate 1-second warning tone ONLY during active period
  if (warningToneActive && timerActive) {
    generateWarningTone();  // Dit handelt de 1-seconde timing af
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
  warningTimeReached = false;
  maxTimeReached = false;
  warningToneActive = false;
  warningToneCompleted = false;
  warningBeepsActive = false;
  timerStartTime = millis();
  lastBeepTime = 0;
  warningToneStartTime = 0;
  
  // KRITIEK: Activeer PTT en hou HIGH tijdens hele sessie
  digitalWrite(PTT_RELAY_PIN, HIGH);
  Serial.println("🚀 PTT RELAY ACTIEF (HIGH) - Timer gestart");
  
  // Reset audio outputs
  noTone(TONE_PIN);  // Tone uit
  noTone(BEEP_PIN);  // Beeps uit

  
  Serial.print("📡 Radio sessie - Timer: ");
  Serial.print(totalTimerSeconds, 1);
  Serial.println(" SECONDEN");
  Serial.println("📈 ABSOLUTE TIMING: Timer-15sec=1kHz toon, Timer-6sec=snel beeps, einde=PTT uit");
  
  // Display start met 0% (nog geen percentage getoond)
  displayDigit(0);
}

void resetTimer() {
  // Stop alle timers
  timerActive = false;
  warningTimeReached = false;
  maxTimeReached = false;
  warningToneActive = false;
  warningToneCompleted = false;
  warningBeepsActive = false;
  
  // Reset alle outputs
  digitalWrite(PTT_RELAY_PIN, LOW);  // PTT UIT (manual reset)
  noTone(TONE_PIN);       // Tone uit
  noTone(BEEP_PIN);       // Beeps uit

  digitalWrite(LED_STATUS_PIN, LOW);
  
  // Display weer op 0 bij reset
  displayDigit(0);
  
  // DP ook resetten
  digitalWrite(SEGMENT_DP_PIN, LOW);
  dpState = false;
  lastDPToggleTime = millis();
  
  Serial.println("⏹️  TIMER GERESET - PTT UIT (Manual reset - PTT losgelaten)");
}

void activateWarningTone() {
  warningToneActive = true;
  warningToneStartTime = millis() - timerStartTime;  // Record start time
  // Tone wordt gegenereerd door generateWarningTone()
  // (voor 1 seconde van 80% tot 95%)
}

void startWarningBeeps() {
  warningBeepsActive = true;
  // Beeps worden gegenereerd in loop()
}

void handleWarningBeeps() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastBeepTime >= 200) {  // Snellere interval voor pulserende beeps
    // Generate snel pulserende beep
    tone(BEEP_PIN, BEEP_FREQUENCY);
    
    // Debug output (elke 2 seconden om spam te voorkomen)
    if ((currentTime - timerStartTime) % 2000 < 200) {
      Serial.print("⚡ SNEL PULSERENDE BEEP - PTT: ");
      Serial.print(digitalRead(PTT_RELAY_PIN) ? "HIGH ✅" : "LOW ❌");
      Serial.print(" | Tijd: ");
      Serial.print((currentTime - timerStartTime) / 1000);
      Serial.print("/");
      Serial.print(totalTimerSeconds);
      Serial.println(" sec");
    }
    
    delay(100);  // Korte beep tijd
    noTone(BEEP_PIN);    // Korte pauze tussen beeps
    lastBeepTime = currentTime;
  }
}

void generateWarningTone() {
  // Genereer 1-seconde 1kHz waarschuwings toon
  // Timing wordt afgehandeld in de hoofdlus via WARNING_TONE_DURATION
  tone(TONE_PIN, TONE_FREQUENCY);
  
  // DEBUG: Controleer PTT status tijdens tone
  if (millis() % 2000 < 10) {  // Elke 2 seconden (voor debugging)
    Serial.print("🔊 1sec WAARSCHUWING TONE actief - PTT: ");
    Serial.print(digitalRead(PTT_RELAY_PIN) ? "HIGH ✅" : "LOW ❌");
    Serial.print(" | Timer: ");
    Serial.print((millis() - timerStartTime) / 1000);
    Serial.print("/");
    Serial.print(totalTimerSeconds);
    Serial.println(" sec");
  }
}

void forcePttOff() {
  // Emergency stop - alles uit (ALLEEN voor debugging, niet gebruikt in normale flow)
  digitalWrite(PTT_RELAY_PIN, LOW);
  noTone(TONE_PIN);
  noTone(BEEP_PIN);


  timerActive = false;
  warningTimeReached = false;
  maxTimeReached = false;
  warningToneActive = false;
  warningToneCompleted = false;
  warningBeepsActive = false;
  
  Serial.println("EMERGENCY STOP - Radio sessie beëindigd (NIET GEBRUIKT)");
  
  // NO BEEP SEQUENCE - omgezet naar gewoon uitschakelen
  // emergencyBeepSequence();  // UITGESCHAKELD - GEEN EXTRA BEEPS
}

void emergencyBeepSequence() {
  Serial.println("EMERGENCY BEEP SEQUENCE");
  
  for (int i = 0; i < 15; i++) {
    tone(BEEP_PIN, BEEP_FREQUENCY);  // Luider beep
    delay(100);
    noTone(BEEP_PIN);
    delay(100);
  }
  
  // Final long beep
  tone(BEEP_PIN, BEEP_FREQUENCY);
  delay(1000);
  noTone(BEEP_PIN);
}

void updateStatusLED() {
  if (timerActive) {
    if (warningBeepsActive) {
      // Snel knipperen bij beep (500ms)
      digitalWrite(LED_STATUS_PIN, (millis() / 500) % 2);
    } else if (warningToneActive) {
      // Middelmatig knipperen bij 1sec warning tone (500ms)
      digitalWrite(LED_STATUS_PIN, (millis() / 500) % 2);
    } else if (warningTimeReached) {
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

void setupPWMChannels() {
  // Software PWM setup - geen hardware kanalen nodig
  // Arduino tone() functie handelt alles automatisch af
  // Initialiseer pinnen als outputs
  pinMode(TONE_PIN, OUTPUT);
  pinMode(BEEP_PIN, OUTPUT);
  
  // Zorg ervoor dat alle tonen uit staan bij opstarten
  noTone(TONE_PIN);
  noTone(BEEP_PIN);
  
  Serial.println("Software PWM klaar (tone/noTone functies)");
}

void readTimerPot() {
  // Lees analoog (0-4095 voor ESP32) - ABSOLUTE TIMING seconden
  int potValue = analogRead(TIMER_POT_PIN);
  
  // Converteer naar seconden (30-300 sec)
  float potFraction = (float)potValue / 4095.0;
  totalTimerSeconds = MIN_TIMER_SECONDS + (potFraction * (MAX_TIMER_SECONDS - MIN_TIMER_SECONDS));
}

void updateTimerSettings() {
  // Bereken totale tijd in milliseconden - ABSOLUTE TIMING
  totalTimerMs = (unsigned long)(totalTimerSeconds * 1000.0);
  
  // ABSOLUTE TIMING: Waarschuwings en beep tijden vanaf einde van timer
  warningTimeMs = (unsigned long)((totalTimerSeconds - WARNING_TIME_FROM_END) * 1000.0);
  beepTimeMs = (unsigned long)((totalTimerSeconds - BEEP_TIME_FROM_END) * 1000.0);
  
  // Debug uitvoer - ABSOLUTE TIMING
  Serial.print("ABSOLUTE TIMING Timer instelling: ");
  Serial.print(totalTimerSeconds, 1);
  Serial.print(" sec (Toon bij: ");
  Serial.print(WARNING_TIME_FROM_END);
  Serial.print("s voor einde, Beeps bij: ");
  Serial.print(BEEP_TIME_FROM_END);
  Serial.print("s voor einde, beeps duren: ");
  Serial.print(BEEP_DURATION/1000);
  Serial.println("s)");
}

void welcomeSequence() {
  Serial.println("=== ABSOLUTE TIMING WELKOMST SEQUENCE ===");
  
  // LED test
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(200);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(200);
  }
  
  // TEST toon
  Serial.println("Test 1kHz toon (ABSOLUTE TIMING MODE)...");
  activateWarningTone();
  delay(1000);
  warningToneActive = false;
  noTone(TONE_PIN);
  
  // TEST beeps
  Serial.println("Test snel pulserende beeps (ABSOLUTE TIMING MODE)...");
  for (int i = 0; i < 3; i++) {
    tone(BEEP_PIN, BEEP_FREQUENCY);
    delay(200);
    noTone(BEEP_PIN);
    delay(200);
  }
  
  Serial.println("*** ABSOLUTE TIMING SYSTEEM KLAAR - GEBRUIK NIET VOOR ECHTE UITZENDING ***");
  Serial.println();
}

// NIEUWE FUNCTIES VOOR 7-SEGMENT DISPLAY

void displayDigit(int digit) {
  // Controleer of digit geldig is (0-9)
  if (digit < 0 || digit > 9) {
    // Bij ongeldige waarde: alle segmenten uit
    for (int i = 0; i < 7; i++) {
      digitalWrite(segmentPins[i], LOW);
    }
    return;
  }
  
  // Haal pattern op voor het gevraagde cijfer
  byte pattern = digitPatterns[digit];
  
  // Zet segmenten aan volgens pattern (Common Cathode: HIGH = aan)
  for (int i = 0; i < 7; i++) {
    bool segmentOn = (pattern >> (6-i)) & 0x01;
    digitalWrite(segmentPins[i], segmentOn ? HIGH : LOW);
  }
  
  // Debug output (alleen af en toe)
  static unsigned long lastDisplayDebug = 0;
  if (millis() - lastDisplayDebug > 1000) {  // Elke seconde
    Serial.print("📺 Display toont: ");
    Serial.println(digit);
    lastDisplayDebug = millis();
  }
}

void displayLetter(char letter) {
  // Converteer naar hoofdletters als dat nog niet zo is
  letter = toupper(letter);
  
  // Controleer of letter geldig is (A-Z)
  if (letter < 'A' || letter > 'Z') {
    // Bij ongeldige waarde: alle segmenten uit
    for (int i = 0; i < 7; i++) {
      digitalWrite(segmentPins[i], LOW);
    }
    return;
  }
  
  int letterIndex = letter - 'A';
  
  // Controleer of letter implementeerbaar is
  byte pattern = letterPatterns[letterIndex];
  if (pattern == 0) {
    // Letter niet implementeerbaar - knipper als waarschuwing
    Serial.print("⚠️ Letter ");
    Serial.print(letter);
    Serial.println(" niet implementeerbaar op 7-segment display");
    
    // Knipper alle segmenten 3 keer
    for (int blink = 0; blink < 3; blink++) {
      for (int i = 0; i < 7; i++) {
        digitalWrite(segmentPins[i], HIGH);
      }
      delay(100);
      for (int i = 0; i < 7; i++) {
        digitalWrite(segmentPins[i], LOW);
      }
      delay(100);
    }
    return;
  }
  
  // Zet segmenten aan volgens pattern (Common Cathode: HIGH = aan)
  for (int i = 0; i < 7; i++) {
    bool segmentOn = (pattern >> (6-i)) & 0x01;
    digitalWrite(segmentPins[i], segmentOn ? HIGH : LOW);
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
      
      // Debug output (alleen af en toe)
      if (millis() % 2000 < 10) {  // Elke 2 seconden
        Serial.print("💡 DP ");
        Serial.print(dpState ? "AAN" : "UIT");
        Serial.print(" - PTT Status: ");
        Serial.print(timerActive ? "ACTIEF" : "INACTIEF");
        Serial.println();
      }
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
    
    Serial.print("📊 Timer percentage: ");
    Serial.print((int)percentage);
    Serial.print("% - Display: ");
    Serial.println(currentDigit);
  }
}

void displayStartupTest() {
  Serial.println("🔧 7-SEGMENT DISPLAY OPSTART TEST - PE2PVD Sequence");
  
  // PE2PVD sequence - elk character 0.5 seconden
  String callSign = "PE2PVD";
  
  Serial.println("Toon call sign: " + callSign);
  
  for (int i = 0; i < callSign.length(); i++) {
    char ch = callSign.charAt(i);
    
    if (ch >= '0' && ch <= '9') {
      // Cijfer - gebruik digitPatterns
      int digit = ch - '0';
      displayDigit(digit);
      Serial.print("Toon cijfer: ");
      Serial.println(ch);
    } else if (ch >= 'A' && ch <= 'Z') {
      // Letter - gebruik letterPatterns
      int letterIndex = ch - 'A';
      byte pattern = letterPatterns[letterIndex];
      
      displayLetter(ch);
      
      Serial.print("Toon letter: ");
      Serial.print(ch);
      
      if (pattern == 0) {
        Serial.print(" (NIET IMPLEMENTEERBAAR)");
      } else {
        Serial.print(" (segments: ");
        String segments = "";
        if (pattern & 0b01000000) segments += "A ";
        if (pattern & 0b00100000) segments += "B ";
        if (pattern & 0b00010000) segments += "C ";
        if (pattern & 0b00001000) segments += "D ";
        if (pattern & 0b00000100) segments += "E ";
        if (pattern & 0b00000010) segments += "F ";
        if (pattern & 0b00000001) segments += "G ";
        Serial.print(segments);
        Serial.print(")");
      }
      Serial.println();
    }
    
    delay(500); // 0.5 seconde per character
  }

    // Alle segmenten uit na test
  for (int i = 0; i < 7; i++) {
    digitalWrite(segmentPins[i], LOW);
  }

  // Korte pauze 
  delay(1000);
    
  // Toon 0 als standaard display na test
  displayDigit(0);
  Serial.println("✅ 7-SEGMENT DISPLAY TEST VOLTOOID - Display op 0");
}

// Test specifiek de PE2PVD letters
void testPE2PVDLetters() {
  Serial.println("\n=== TEST PE2PVD LETTERS INDIVIDUEEL ===");
  
  String callSign = "PE2PVD";
  
  for (int i = 0; i < callSign.length(); i++) {
    char ch = callSign.charAt(i);
    
    Serial.print("Test ");
    Serial.print(ch);
    Serial.print(": ");
    
    if (ch >= '0' && ch <= '9') {
      Serial.print("CIJFER - toon ");
      Serial.println(ch);
      displayDigit(ch - '0');
    } else if (ch >= 'A' && ch <= 'Z') {
      displayLetter(ch);
      
      // Uitleg van het patroon
      int letterIndex = ch - 'A';
      byte pattern = letterPatterns[letterIndex];
      if (pattern == 0) {
        Serial.print("LETTER - NIET IMPLEMENTEERBAAR");
      } else {
        Serial.print("LETTER - segments: ");
        // Toon welke segmenten aan staan
        String segments = "";
        if (pattern & 0b01000000) segments += "A ";
        if (pattern & 0b00100000) segments += "B ";
        if (pattern & 0b00010000) segments += "C ";
        if (pattern & 0b00001000) segments += "D ";
        if (pattern & 0b00000100) segments += "E ";
        if (pattern & 0b00000010) segments += "F ";
        if (pattern & 0b00000001) segments += "G ";
        Serial.print(segments);
      }
    }
    
    Serial.println();
    delay(2000); // 2 seconden per karakter om goed te bekijken
  }
  
  // Reset naar 0
  displayDigit(0);
  Serial.println("=== PE2PVD LETTER TEST VOLTOOID ===");
}

// Test functies voor debugging
void selfTest() {
  Serial.println("\n=== ZELF TEST ESP32 ===");
  
  // Test PTT relay
  Serial.println("Test PTT relay...");
  digitalWrite(PTT_RELAY_PIN, HIGH);
  delay(1000);
  digitalWrite(PTT_RELAY_PIN, LOW);
  
  // Test tonen
  Serial.println("Test 1kHz toon...");
  warningToneActive = true;
  delay(2000);
  warningToneActive = false;
  noTone(TONE_PIN);
  
  Serial.println("Test beep PWM...");
  for (int i = 0; i < 5; i++) {
    tone(BEEP_PIN, BEEP_FREQUENCY);
    delay(200);
    noTone(BEEP_PIN);
    delay(200);
  }
  
  // Test LED
  Serial.println("Test LED...");
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(100);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(100);
  }
  
  // Test 7-segment display
  Serial.println("Test 7-segment display...");
  displayStartupTest();
  
  // Test specifiek de PE2PVD letters
  testPE2PVDLetters();
  
  Serial.println("Test voltooid - Systeem operationeel\n");
}

// EEPROM functies voor instellingen (ESP32 NVS)
void saveSettings() {
  // Placeholder voor toekomstige instellingen opslag
  Serial.println("Settings saved to NVS");
}

void loadSettings() {
  // Placeholder voor instellingen laden
  Serial.println("Settings loaded from NVS");
}

// NIEUWE FUNCTIE: PTT WATCHER - Zorgt ervoor dat PTT HIGH blijft tijdens warning fasen
void ensurePTTStaysHigh() {
  // Als timer actief is en PTT per ongeluk LOW zou zijn, maak het weer HIGH
  if (timerActive && !maxTimeReached) {
    if (!digitalRead(PTT_RELAY_PIN)) {
      digitalWrite(PTT_RELAY_PIN, HIGH);
      Serial.println("🚨 PTT WATCHER: PTT was LOW, weer HIGH gezet tijdens timer!");
    }
  }
}