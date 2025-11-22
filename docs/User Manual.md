User Manual — Arduino Solar Battery Float Charger (PWM Edition)
Version: 1.3
Panel Support: 25 W–100 W, 12 V nominal
Battery Types: Flooded Lead-Acid (FLA), AGM
Author: StJohn Lennox-Kerr
GitHub: https://github.com/orsty3001/Basic-AGM-Lead-Acid-Battery-Float-Charger


1. Overview
The Arduino Solar Battery Float Charger is a smart, microcontroller-driven charging system designed to keep a 12 V automotive battery fully maintained using only a small solar panel.
It provides three-stage charging, battery temperature compensation, state-of-charge estimation, and an OLED interface for real-time monitoring.
This system is ideal for long-term vehicle storage in locations without AC power. Depending on your vehicle you could leave the battery hooked up if the current draw is low. Recommended to disconnect the battery from the vehicle in storage and leave it connected only to the battery float charger. 
It ensures the battery remains healthy, prevents sulfation, and maintains a full charge safely over long durations.

2. Key Capabilities
Smart Charging (Bulk → Absorb → Float)
    • Automatically manages the full charge cycle
    • Mimics commercial maintainers (NOCO, CTEK, BatteryTender)
Temperature Compensation
    • Uses a 10k NTC thermistor mounted to battery
    • Adjusts charge voltage based on battery temperature
    • Prevents overcharging in heat and undercharging in cold
OLED Live Display
Shows:
    • Charging stage (NIGHT, BULK, ABSORB, FLOAT)
    • Battery voltage
    • Solar panel voltage
    • Battery temperature (°C or °F)
    • Charge output level (PWM %)
    • Battery type (FLA/AGM)
    • Estimated state of charge (SoC %)
Dual-Button Control System
Button 1 — Mode & Menu
Button 2 — Adjust & Confirm
Functions:
    • Toggle FLA/AGM mode
    • Enter calibration menu
    • Adjust ADC scaling/offset
    • Capture custom Bulk & Float voltages
    • Toggle between Celsius and Fahrenheit
    • Exit menu
EEPROM Persistence
All settings survive:
    • Power loss
    • Nighttime shutdown
    • Firmware resets
 Automotive-Safe
    • Schottky reverse-current blocking
    • Automotive fuse protection
    • Low standby drain (nighttime < 2 mA)
    • No risk of overcharge

3. System Architecture
3.1 Major Components
    • Arduino Nano (ATmega328P)
    • 25 W–100 W “12 V nominal” solar panel
    • P-channel MOSFET (high-side PWM)
    • Schottky diode ≥ 5 A
    • SSD1306 OLED display (I²C)
    • 10k NTC thermistor
    • Two momentary buttons
    • Voltage sensing dividers (100k / 6.8k)
    • Automotive fuse (5 A)
3.2 Recommended Use
    • Parked vehicles
    • Boats, tractors, ATVs
    • Seasonal equipment
    • Generator maintenance
    • Remote installations with no AC outlets

4. Charging Modes Explained
The charger operates automatically based on battery voltage, temperature, and solar availability.
NIGHT Mode
    • Solar panel voltage too low (< ~5 V)
    • Charger turns off
    • Battery isolated via diode
    • Arduino enters low-power state

BULK Mode
    • Battery below target Bulk voltage
    • Maximum safe charging current
    • PWM ramps as needed
    • Fastest charge phase

ABSORPTION Mode
    • Battery reached Bulk voltage
    • Charger maintains constant voltage
    • Current tapers as battery tops off
    • Timer (45 min FLA / 60 min AGM) ensures proper absorption

FLOAT Mode
    • Maintains full charge at lower voltage
    • Prevents battery self-discharge
    • Safe for continuous long-term maintenance

Re-Bulk Conditions
If battery drops significantly (e.g. load, cold, parasitic drain):
    • Re-enters Bulk to restore charge

5. Temperature Compensation
Charge voltage adjusts automatically using a 10k NTC sensor:
Battery Type
Compensation
Example
FLA
–18 mV/°C
Hot battery ⇒ lower charge voltage
AGM
–24 mV/°C
Cold battery ⇒ increases charge voltage

Maintains safe chemistry limits in all seasons.

6. OLED Interface
Home Screen Shows:
Stage: FLOAT      AGM
Vbat: 13.58V
Vpv : 15.21V
Temp: 27.4C  Out: 45%
Bulk/Abs: 14.62V
Float:    13.60V   SoC 100%

Items Explained
    • Stage: Current charging state
    • Battery Type: FLA or AGM
    • Vbat: Battery terminal voltage
    • Vpv: Solar panel voltage
    • Temp: Battery temperature (°C or °F)
    • Out: PWM output duty cycle (%)
    • Bulk/Abs: Target voltage (temperature compensated)
    • Float: Target float voltage
    • SoC: Estimated state-of-charge

7. Controls
Quick Actions
    • BTN1 short: Toggle battery type (FLA ↔ AGM)
    • BTN1 long (3s): Enter calibration menu
    • BTN2 short: Adjust settings / small increments
    • BTN2 long: Confirm / save / large increments

8. Calibration Menu
Hold Button 1 for 3 seconds.
Use:
    • BTN1 = next item
    • BTN2 short = small adjust
    • BTN2 long = large adjust / confirm
Menu Items
    1. Scale + / −
    2. Offset + / −
    3. Capture BULK voltage
    4. Capture FLOAT voltage
    5. Temperature Unit (°C/°F)
    6. Exit
How to calibrate
    1. Measure battery voltage with a multimeter
    2. Adjust scale/offset until OLED matches
    3. Capture Bulk & Float if desired
    4. Set temperature units
    5. Exit
All values saved instantly to EEPROM.

9. Installation
Wiring Summary
Solar Panel + ----> Schottky Diode ----> Fuse ----> MOSFET Source ----> Battery +
Solar Panel - --------------------------------------------------------> Battery -

NTC sensor attaches to battery side using thermal tape or epoxy
    • OLED connects via I²C (SDA/SCL)
    • Buttons connect to GND using INPUT_PULLUP
Panel Positioning
    • Best angle: 30–40° tilt south
    • For SUV roof temporary stands work
    • Later panel can mount to carport roof


10. Safety Notes
    • Always fuse the positive lead near the battery
    • Ensure correct polarity (reverse polarity protection diode optional)
    • Protect wiring from chafing and moisture
    • Temperature sensor must stay attached to battery for correct compensation
    • Do not bypass fuse or diode

11. Troubleshooting
Issue
Cause
Fix
OLED blank
No 5 V or wiring issue
Check USB or 5V regulator
Stuck in NIGHT mode
Low sun or incorrect wiring
Check panel and polarity
Incorrect voltage readout
Divider tolerance
Recalibrate via menu
Overly low float voltage
Cold temps
Expected due to temp compensation
Overly high float voltage
Hot temps
Expected—battery protection


12. Technical Specifications


Parameter
Value
Panel Voltage
18–22 V open circuit
Panel Power
25–100 W
Charge Current
Up to panel limit
Bulk Voltage
14.4 V (FLA) / 14.6 V (AGM) @ 25°C
Float Voltage
13.5 / 13.6 V
Temp Coeff
–18 or –24 mV/°C
Output Type
PWM (high-side MOSFET)
ADC Reference
1.1 V internal
EEPROM Retention
>100k writes
Night Drain
< 2 mA



13. Licensing

    • Software: MIT License
    • Hardware: CERN-OHL-P v2 Permissive
    • Documentation: CC BY 4.0

14. Intended Use Example
This design was developed for a 1999 GMC Yukon Denali stored without access to AC power.
A spare 25 W panel maintains the battery on the roof, with the option to upgrade to 100 W when mounted on a future carport.

15. Revision History
v1.3 — Updated with °C/°F selection, improved UI, EEPROM structure, and revised calibration workflow.
v1.2 — Added AGM/FLA profiles and temperature compensation.
v1.1 — Initial OLED and dual-button UX.
v1.0 — Initial PWM charger logic.

