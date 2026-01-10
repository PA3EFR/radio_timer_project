# Radio Zendtijd Timer - Gebruikershandleiding

## Overzicht
De Radio Zendtijd Timer is een Arduino Nano project voor het meten en beperken van radiouitzendtijden. Het systeem bewaakt de PTT (Push-to-Talk) functie en geeft audio- en visuele waarschuwingen voordat de ingestelde tijd verloopt.

## Belangrijke Waarschuwing
**⚠️ GEBRUIK NIET VOOR NORMALE AMATEUR RADIO UITZENDING!**  
Deze versie is alleen bedoeld voor testdoeleinden met langere tijden.

## Hardware Vereisten
- Arduino Nano (ATmega328P)
- PTT switch/knop (pin D2)
- Speaker voor audio waarschuwingen (pin D4)
- 7-segment display (pin A1, A2, D3, D8-D12)
- Potmeter voor timer instelling (pin A0)
- PTT relay driver (pin D7)
- Status LED (pin D13 - ingebouwde LED)

## Pin Configuratie
```
PTT Switch:    D2 (INPUT_PULLUP)
Audio Output:  D4 (TONE_PIN) - alle audio signalen
PTT Relay:     D7 (OUTPUT)
Status LED:    D13 (ingebouwde LED)
Timer Pot:     A0 (INPUT)
Display Segments: D3, D8-D12, A1, A2
```

## Timer Instellingen
- **Bereik**: 30-300 seconden (instelbaar via potmeter)
- **Waarschuwings Tijd**: 15 seconden voor einde
- **Beep Tijd**: 6 seconden voor einde
- **Audio Frequenties**:
  - Warning Tone: 1kHz (1 seconde)
  - Warning Beeps: 500Hz (snel pulserend)

## Werking

### 1. Timer Start
- Druk PTT in (active LOW)
- Timer begint met aftelling
- Status LED gaat aan
- 7-segment display toont voortgang (percentage)
- PTT relay wordt geactiveerd

### 2. Audio Waarschuwingen

#### Fase 1: Warning Tone (-15 sec)
- **Duur**: 1 seconde
- **Audio**: 1kHz toon (continu)
- **LED**: Middelmatig knipperen (500ms)

#### Fase 2: Stilte Tussen Fasen
- **Duur**: 9 seconden (tussen tone en beeps)
- **Audio**: Geen
- **LED**: Langzaam knipperen (1000ms)

#### Fase 3: Warning Beeps (-6 sec)
- **Duur**: 5 seconden
- **Audio**: 500Hz snel pulserend (100ms aan/uit)
- **LED**: Snel knipperen (500ms)

#### Fase 4: Timer Einde
- **Actie**: PTT wordt uitgeschakeld
- **Audio**: Alle audio stopt
- **Display**: Reset naar 0

### 3. Timer Stop
- Laat PTT los (manuele stop)
- Timer stopt onmiddellijk
- PTT relay wordt uitgeschakeld
- Display reset naar 0

## 7-Segment Display
- Toont voortgang als percentage (0-9)
- Decimal Point knippert tijdens actieve timer (500ms interval)
- Toont call-sign bij opstarten (PE2PVD test)
- Reset naar 0 bij timer einde

## Audio Test bij Opstarten
Bij het inschakelen wordt een test uitgevoerd:
1. LED knipperen (3x)
2. Warning tone test (1kHz - 500ms)
3. Beep sequence test (6x pulserend 500Hz)

## Potmeter Instelling
- Draai potmeter om timer tijd in te stellen (30-300 sec)
- Instelling wordt elke 500ms bijgewerkt
- Waarde wordt direct in secondes getoond

## Status LED Indicaties
- **Uit**: Timer niet actief
- **Aan**: Timer actief (normale werking)
- **Middelmatig knipperen**: Warning tone fase
- **Langzaam knipperen**: Tussen fase (tone naar beeps)
- **Snel knipperen**: Beep fase

## Technische Details
- **Audio**: Alle audio signalen op één pin (D4)
- **Geheugen**: PROGMEM optimalisatie voor Flash memory
- **Timing**: Vaste millis() tijdstippen (geen accumulatie errors)
- **PTT Beveiliging**: ensurePTTStaysHigh() functie
- **Debouncing**: 50ms debounce voor PTT input
- **Display**: PROGMEM patterns voor cijfers en letters

## Troubleshooting

### Geen Audio
- Controleer speaker aansluiting op D4
- Verifieer pin configuratie
- Check potmeter instelling
- Let op: Arduino tone() functie kan maar 1 toon tegelijk afspelen

### Display Werkt Niet
- Controleer 7-segment display bekabeling
- Verifieer common cathode aansluiting
- Check segment pin configuratie

### Timer Start Niet
- Controleer PTT switch (active LOW)
- Verificeer pull-up weerstand
- Check pin D2 verbinding

### Audio Timing Klopt Niet
- De timing wordt vastgesteld bij start TX
- Potmeter wijzigingen tijdens TX hebben geen effect meer
- Dit is ontwerpgedrag voor consistente timing

## Bestanden
- `radio_timer_nano_final_corrected.ino`: Hoofdprogramma (Arduino IDE)
- Deze handleiding

## Auteur
MiniMax Agent  
Versie: Versimpelde Versie - 2025

---
*Voor technische vragen of problemen, raadpleeg de code comments voor gedetailleerde uitleg.*