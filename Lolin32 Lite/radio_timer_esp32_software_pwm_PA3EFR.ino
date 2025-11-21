/*
 * Radio Zendtijd Timer - ESP32 Lolin32 Lite
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

// Debounce variabelen
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50;

void setup() {
  // Pin configuratie
  pinMode(PTT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(PTT_RELAY_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  
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
  Serial.println("\n=== Radio Timer ESP32 - Lolin32 Lite (ABSOLUTE TIMING) ===");
  Serial.println("TEST MODE: Potmeter instelbaar: 20-60 SECONDEN timer");
  Serial.println("Timer-15sec = 1kHz toon (1 sec), Timer-6sec = snel beeps (5 sec)");
  Serial.println("*** GEBRUIK NIET VOOR NORMALE UITZENDT ***");
  Serial.println("Klaar voor gebruik...\n");
  
  // Welcome LED sequence
  welcomeSequence();
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
    
    // Einde van timer: PTT UIT
    if (elapsed >= totalTimerMs) {
      timerActive = false;
      digitalWrite(PTT_RELAY_PIN, LOW);  // PTT UIT bij einde van timer
      warningBeepsActive = false;
      noTone(BEEP_PIN);
      digitalWrite(LED_STATUS_PIN, LOW);
      
      Serial.println("⏹️  TIMER EINDE - PTT UIT!");
    }
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
  
  // Converteer naar seconden (20-60 sec)
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