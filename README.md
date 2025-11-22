Arduino Solar Car Battery Maintainer (25W PWM Charger)

This project is an open-source Arduino-based smart solar maintainer designed to keep a 12 V automotive battery (Lead Acid or AGM) fully charged using a small solar panel (25 W recommended).
It is ideal for vehicles stored long-term without access to AC power—such as an SUV parked under a future car-port or in remote locations.

The system performs full three-stage charging (Bulk → Absorb → Float) with temperature compensation and includes a built-in OLED status display, calibration tools, and support for both °C and °F temperature units.

Features

Smart 3-Stage Charging: Bulk, Absorption, and Float—just like commercial maintainers.

PWM-Based Charge Control: Simple, efficient, and reliable for 12 V nominal panels.

Battery Type Selection: Choose FLA or AGM, each with proper voltage targets.

Temperature Compensation: Adjusts charge voltage based on battery temp (NTC sensor).

OLED Display (SSD1306): Real-time readout of:

Battery voltage

Solar panel voltage

Battery temperature (°C/°F option)

Charge stage

PWM output level

State of charge estimate

Dual-Button Menu System:

Button 1: Mode & menu navigation

Button 2: Adjust/confirm settings

On-Device Calibration:

ADC scaling & offset

Capture bulk & float voltages

Select °C / °F

All settings saved to EEPROM

Safe Night Behavior: Charger shuts off and isolates battery when panel voltage drops.

Hardware Overview

The charger uses simple, robust components:

Arduino Nano (ATmega328P)

25 W 12 V solar panel (100 W panel supported if PWM only)

P-channel MOSFET for high-side PWM control

Schottky blocking diode (5 A+)

NTC 10k thermistor for temperature sensing

SSD1306 OLED display

Two momentary pushbuttons

Voltage divider resistors for battery & panel sensing

Fuse and wiring (automotive rated)

A full BOM is included in the project.

Why PWM Instead of MPPT?

For maintaining a healthy car battery with a 25 W panel, PWM is:

Simpler

More reliable outdoors

Nearly as efficient for this use-case

Much easier to implement without a custom buck converter

The design focuses on long-term reliability, low idle power, and easy DIY assembly.

How It Works

At sunrise, the panel voltage rises and the Arduino boots.

Battery voltage and temperature are read via analog inputs.

Depending on the battery level and profile (FLA/AGM), the charger enters:

Bulk: Maximum safe current (PWM full/variable)

Absorb: Hold target voltage for a timed period

Float: Maintain ~13.5–13.6 V to prevent self-discharge

If the panel voltage drops (clouds or night), the charger switches to NIGHT mode and disconnects.

EEPROM stores calibration, battery type, and °C/°F preference.

Intended Use

Long-term storage of cars, trucks, SUVs, boats, tractors, generators, etc.

Situations where AC power isn’t available (driveways, fields, carports).

Prevents sulfation and preserves battery life.

This project was designed for a 1999 GMC Yukon Denali but works with any 12 V lead-acid system.

Licensing

Software: MIT License

Hardware (schematics/BOM): CERN-OHL-P v2

Documentation: CC BY 4.0

Repository Contents

src/ – Arduino sketch

hardware/ – Schematic, wiring diagram, BOM

docs/ – Manual, SVG graphics, calibration notes

LICENSE.md – Combined licensing info

Contributing

Pull requests are welcome—especially improvements to:

Charge algorithms

Temperature modeling

Display UX

Hardware design (relay version, MOSFET upgrades, MPPT experiments)
