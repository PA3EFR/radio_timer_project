# ESP32 Radio Timer Handleiding

**Project:** Radio Zendtijd Timer met ESP32 Lolin32 Lite  
**Versie:** Software PWM - Absolute Timing  
**Auteur:** MiniMax Agent  
**Datum:** November 2025

---

## 📋 Project Overzicht

Deze ESP32 Lolin32 Lite radio timer helpt amateur radio operators bij het beperken van zendtijd door automatisch de PTT (Push-To-Talk) relay uit te schakelen na een ingestelde periode.

### 🎯 Belangrijke Kenmerken
- **Absolute timing**: Waarschuwingen komen altijd op vaste tijden voor het einde
- **Instelbare timer**: 20-300 seconden (uitbreidbaar)
- **Betrouwbare audio**: Software PWM (Arduino tone() functies)
- **PTT beveiliging**: Relay blijft HIGH tijdens hele waarschuwingsfase

---

## 🔧 Hardware Vereisten

### ESP32 Bord
- **Aanbevolen:** Lolin32 Lite (ESP32)
- **Alternatief:** Andere ESP32 borden

### Benodigde Componenten
- 10kΩ lineaire potentiometer (timer instelling)
- 5V PTT relay module
- Luidspreker of buzzer (500Ω-1000Ω)
- Status LED (optioneel - onboard LED gebruikt)
- Verbindingsdraden

### Pin Configuratie
| Functie | ESP32 Pin | Beschrijving |
|---------|-----------|--------------|
| PTT Switch | GPIO 4 | Input (pull-up) |
| Waarschuwings Toon | GPIO 17 | 1000Hz output |
| Beeps | GPIO 18 | 500Hz output |
| PTT Relay | GPIO 16 | Relay driver |
| Status LED | GPIO 22 | ESP32 onboard LED |
| Timer Pot | GPIO 34 | Analoog input |

---

## ⚙️ Functionaliteit

### Timer Werkingswijze
1. **Normale Operatie**: PTT HIGH, timer telt af
2. **Waarschuwingsfase**: Audio waarschuwingen + PTT blijft HIGH
3. **Einde**: PTT automatisch LOW

### Timing Gedrag
| Tijdsmoment | Actie | Duur | PTT Status |
|-------------|-------|------|------------|
| Timer - 15s | 🔊 1000Hz toon | 1 seconde | HIGH |
| Timer - 6s | ⚡ 500Hz beeps | 5 seconden | HIGH |
| Timer einde | ⏹️ PTT uit | - | LOW |

### Absolute Timing Voorbeelden
| Timer Setting | Waarschuwings Toon | Beeps Start | Beeps Einde | Timer Einde |
|---------------|-------------------|-------------|-------------|-------------|
| 60 seconden | 45s | 54s | 59s | 60s |
| 300 seconden | 285s | 294s | 299s | 300s |

---

## 🚀 Installatie & Setup

### 1. Hardware Montage
```
ESP32 Lolin32 Lite
├── Potmeter (10kΩ) → GPIO 34
├── PTT Switch → GPIO 4 (met pull-up)
├── PTT Relay → GPIO 16
├── Speaker/Buzzer → GPIO 17 & 18
└── Status LED → GPIO 22 (ingebouwd)
```

### 2. Software Installatie
1. Installeer Arduino IDE met ESP32 board support
2. Upload `radio_timer_esp32_software_pwm.ino`
3. Open Serial Monitor (115200 baud)

### 3. Initiële Setup
```
=== Radio Timer ESP32 - Lolin32 Lite (ABSOLUTE TIMING) ===
TEST MODE: Potmeter instelbaar: 20-60 SECONDEN timer
Timer-15sec = 1kHz toon (1 sec), Timer-6sec = snel beeps (5 sec)
*** GEBRUIK NIET VOOR NORMALE UITZENDT ***
Klaar voor gebruik...
```

---

## ⚙️ Configuratie

### Timer Bereik Aanpassen
```cpp
// In radio_timer_esp32_software_pwm.ino
#define MIN_TIMER_SECONDS  20.0     // Minimum tijd
#define MAX_TIMER_SECONDS  300.0    // Maximum tijd (uitbreidbaar)
```

### Waarschuwings Timing
```cpp
#define WARNING_TIME_FROM_END 15   // Waarschuwings toon 15s voor einde
#define BEEP_TIME_FROM_END    6    // Beeps 6s voor einde
#define BEEP_DURATION         5000 // Beeps duren 5 seconden
```

### Audio Instellingen
```cpp
#define TONE_FREQUENCY     1000     // Waarschuwings toon Hz
#define BEEP_FREQUENCY     500      // Beeps Hz
#define WARNING_TONE_DURATION 1000  // Waarschuwings toon duur (ms)
```

---

## 🔍 Bediening

### Normale Gebruik
1. **Stel timer in**: Draai potmeter naar gewenste tijd (20-60s)
2. **Start transmissie**: Druk PTT switch in
3. **Monitor waarschuwingen**: Luister naar audio signalen
4. **Beëindig transmissie**: Laat PTT los voor handmatige stop

### Serial Monitor Output
```
🚀 PTT RELAY ACTIEF (HIGH) - Timer gestart
📡 Radio sessie - Timer: 60.0 SECONDEN
🔊 Waarschuwings toon gestart (1 seconde) - PTT BLIJFT HIGH
🔇 Waarschuwings toon klaar - Wachten op beeps...
⚡ SNEL PULSERENDE BEEPS gestart - PTT BLIJFT HIGH
⚡ 5-SECONDEN SNEL PULSERENDE BEEPS KLAAR - PTT BLIJFT HIGH tot einde!
⏹️ TIMER EINDE - PTT UIT!
```

---

## 🔧 Troubleshooting

### Veel Voorkomende Problemen

**Geen audio output:**
- Controleer speaker/buzzer verbindingen
- Verificeer GPIO pin configuratie
- Test metSerial Monitor output

**PTT relay werkt niet:**
- Controleer relay module verbinding
- Verificeer voeding (5V voor relay)
- Test relay module apart

**Timer timing onjuist:**
- Controleer absolute timing constanten
- Verifieer potmeter calibratie
- Controleer Serial Monitor voor debugging

**Serial Monitor error:**
- Stel baud rate in op 115200
- Controleer ESP32 board selectie
- Reset ESP32 na upload

### Debug Modi
De code bevat uitgebreide Serial output voor troubleshooting:
- Timer configuratie weergave
- Waarschuwings momenten
- PTT status monitoring
- Audio timing controle

---

## ⚠️ Belangrijke Waarschuwingen

### TEST MODE
```
*** GEBRUIK NIET VOOR NORMALE UITZENDT ***
Deze versie is alleen voor testdoeleinden met langere tijden.
```

### Veiligheidsmaatregelen
- **Test eerst**: Gebruik alleen met test apparatuur
- **PTT backup**: Altijd handmatig kunnen ingrijpen
- **Audio niveau**: Houd volume op veilig niveau
- **Emergency stop**: PTT switch altijd beschikbaar

---

## 🔄 Updates & Onderhoud

### Versie Geschiedenis
- **v1.0**: Basis functionaliteit
- **v1.1**: PTT beveiliging toegevoegd
- **v1.2**: Absolute timing geïmplementeerd
- **v1.3**: 5-seconden beeps toegevoegd

### Toekomstige Mogelijkheden
- EEPROM instellingen opslaan
- LCD display voor timer weergave
- Meerdere audio profielen
- Remote configuratie via WiFi

---

## 📞 Support

Voor vragen of problemen:
- Controleer eerst deze handleiding
- Verifieer Serial Monitor output
- Test individuele componenten
- Documenteer error messages

---

**© 2025 MiniMax Agent - ESP32 Radio Timer Project**