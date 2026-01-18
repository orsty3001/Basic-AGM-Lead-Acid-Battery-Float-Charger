Changelog - Jan 18th 2026


Latest Code Revision 1.1v (PWM Solar Float Charger, Nano + SSD1306)
Board Rev.B 

PCB

The silkscreen is wrong for the resistor divider for the battery to PIN A0. 

The top resistor should be 150k ohms and the bottom should be 6.8k ohms if you look at the board with the solar panel plug on top.

If you have a REV. A board. Just put a 150k resistor in the 6.8k spot, and the 6.8k in the 100k right above the 10 amp fuse. 


Added

Dual calibration paths (Battery vs PV):

Introduced separate calibration variables:

cal_scale_bat, cal_offset_bat

cal_scale_pv, cal_offset_pv

Updated voltage conversion to accept per-channel calibration:

dividerVoltage(adc, R1, R2, scale, offset)

Expanded EEPROM layout to support PV calibration:

Added EEPROM slots for PV scale/offset.

Preserved existing storage for battery profile selection, charger profiles, and temperature unit.

Run-screen battery mode switching + CAL menu entry (Button 1):

Short press toggles FLA ⇄ AGM and persists to EEPROM.

Long press enters Calibration Menu.

Calibration menu actions (Button 2):

Short press: fine adjust scale/offset, toggle °C/°F.

Long press: coarse adjust scale/offset, capture Bulk/Float targets, exit CAL menu.

Temperature unit selection persisted in EEPROM:

Added useFahrenheit setting and storage at EE_TEMP_UNIT.

Changed

PV divider ratio updated for higher PV open-circuit voltage:

R1_PV changed from 100k → 150k (with R2_PV = 6.8k) to extend PV measurement range to ~25.3 V with 1.1 V ADC reference.

ADC read stability improvement:

Added an initial “throw-away” analogRead(pin) after mux switching inside readADCavg() to reduce channel-to-channel carryover error.

PV voltage computation separated from battery computation:

Battery measurement uses battery calibration.

PV measurement uses PV calibration.

Code organization hardened against Arduino auto-prototype issues:

Enums placed at top.

Added explicit forward declarations for key functions (including button handlers).

Fixed

Compile issues related to missing functions and mismatched symbol names:

Restored/defined handleButtons_RUN() and button helpers (btn1Down(), btn2Down()).

Eliminated remaining references to old calibration symbols (cal_scale, EE_CAL_SCALE, etc.).

EEPROM initialization logic updated to populate new calibration fields:

On first run (magic mismatch), defaults are written for both BAT and PV calibration slots.

Notes / Implications

EEPROM compatibility:
Because the EEPROM layout changed, boards previously flashed with older firmware may interpret old EEPROM bytes as the new PV calibration floats. If PV readings appear wildly incorrect, perform a “factory reset” by bumping EE_MAGIC_VAL (or add a reset routine) to reinitialize EEPROM defaults.

PV measurement headroom:
With R1_PV=150k / R2_PV=6.8k and 1.1 V reference, PV input clips at approximately 25.36 V; higher PV voltage will saturate the ADC and the display will not increase beyond that (unless calibration values are corrupted).
