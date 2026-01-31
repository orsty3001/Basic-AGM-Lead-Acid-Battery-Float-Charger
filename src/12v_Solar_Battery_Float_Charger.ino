/*
 MIT License

Copyright (c) 2026 StJohn Lennox-Kerr (K4IOK)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 
  25 W Solar → 12 V Lead-Acid Maintainer with OLED, Temp-Comp, Calibration, Dual Buttons
  + Fahrenheit/Celsius display toggle (persistent in EEPROM)

  Board: Arduino Nano/Uno (ATmega328P). Uses INTERNAL 1.1 V ADC reference.
*/

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

/*************** ENUMS FIRST (avoid Arduino auto-prototype issues) ***************/
enum ControlMode { RELAY_MODE, PWM_MODE };
enum Stage { NIGHT, BULK, ABSORB, FLOAT };
enum BatteryMode { MODE_FLA=0, MODE_AGM=1 };
enum UiScreen { RUN_SCREEN, CAL_MENU };
enum CalItem {
  CALI_ADC_SCALE_UP,
  CALI_ADC_SCALE_DOWN,
  CALI_ADC_OFFSET_UP,
  CALI_ADC_OFFSET_DOWN,
  CALI_CAPTURE_BULK,
  CALI_CAPTURE_FLOAT,
  CALI_TEMP_UNIT,   // °C / °F toggle
  CALI_EXIT
};

/*************** FORWARD DECLARATIONS ***************/
const __FlashStringHelper* calItemName(CalItem it);
int  estimateSOC(float vBat, Stage s);
void enterStage(Stage s);
void regulateVoltage(float vBat, float vSet);
void handleButtons_RUN();
void handleButtons_CAL(float vBat);

/*************** OLED ***************/
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/*************** USER CONFIG (WIRING) ***************/
const ControlMode CONTROL_MODE = PWM_MODE;   // set RELAY_MODE if using a relay

// Pins
const uint8_t BAT_V_PIN = A0;
const uint8_t PV_V_PIN  = A1;
const uint8_t NTC_PIN   = A2;
const uint8_t CHG_PIN   = 9;    // D9 supports PWM
const uint8_t BTN1_PIN  = 2;    // Mode/Menu (to GND, INPUT_PULLUP)
const uint8_t BTN2_PIN  = 3;    // Adjust/Confirm (to GND, INPUT_PULLUP)

// Voltage dividers (Ohms)
const float R1_BAT = 100000.0f;
const float R2_BAT =   6800.0f;

// PV divider updated for ~25V full-scale @ 1.1V ref
const float R1_PV  = 150000.0f;
const float R2_PV  =   6800.0f;

// NTC (10k, B≈3950). Divider: 5V -> 10k fixed -> A2 -> 10k NTC -> GND
const float NTC_R_FIXED   = 10000.0f;
const float NTC_BETA      = 3950.0f;
const float NTC_R0        = 10000.0f;
const float NTC_T0_KELVIN = 298.15f;  // 25°C

// Profiles @25°C (before temp comp)
struct Profile {
  float bulk_target;    // Bulk/Absorb voltage
  float absorb_time_s;  // seconds
  float float_target;   // Float voltage
  float temp_slope_v_per_degC; // dV/dT across entire 12V battery (negative)
};
const Profile DEFAULT_FLA = { 14.40f, 45*60, 13.50f, -0.018f };
const Profile DEFAULT_AGM = { 14.60f, 60*60, 13.60f, -0.024f };

// Thresholds
const float REBULK_ENTRY_V  = 12.40f;
const float FLOAT_EXIT_V    = 13.20f;
const float PV_PRESENT_MIN  = 5.0f;
const float PV_MARGIN_ON    = 0.7f;

// ADC / sampling
const float ADC_REF_V = 1.1f;
const uint16_t ADC_SAMPLES = 32;

// PWM tuning
const uint8_t PWM_MIN = 10;
const uint8_t PWM_MAX = 255;
float Kp = 18.0f;
float Ki = 0.045f;

// Timing
const uint32_t CONTROL_PERIOD_MS = 500;

/*************** EEPROM LAYOUT ***************/
const uint8_t EE_MAGIC_ADDR = 0;
const uint8_t EE_MAGIC_VAL  = 0x43;
const int     EE_MODE       = 1;

// Separate BAT and PV calibration
const int EE_CAL_SCALE_BAT  = 2;   // float (4 bytes) 2..5
const int EE_CAL_OFFSET_BAT = 6;   // float (4 bytes) 6..9
const int EE_CAL_SCALE_PV   = 10;  // float (4 bytes) 10..13
const int EE_CAL_OFFSET_PV  = 14;  // float (4 bytes) 14..17

// Profiles start after calibration block
const int EE_FLA_START  = 18;
const int EE_AGM_START  = EE_FLA_START + (int)sizeof(Profile);
const int EE_TEMP_UNIT  = EE_AGM_START + (int)sizeof(Profile); // 0 = °C, 1 = °F

void ewrite(int addr, const void* p, size_t n){
  for(size_t i=0;i<n;i++) EEPROM.update(addr+i, ((const uint8_t*)p)[i]);
}
void eread (int addr, void* p, size_t n){
  for(size_t i=0;i<n;i++) ((uint8_t*)p)[i]=EEPROM.read(addr+i);
}

/*************** STATE ***************/
Stage       stage = NIGHT;
BatteryMode batteryMode = MODE_FLA;
Profile     profileFLA, profileAGM;

float cal_scale_bat  = 1.000f;
float cal_offset_bat = 0.000f;
float cal_scale_pv   = 1.000f;
float cal_offset_pv  = 0.000f;

bool useFahrenheit = false; // persisted in EEPROM

uint32_t lastControl = 0;
uint32_t absorbStart = 0;
float    integrator  = 0.0f;
uint8_t  pwmOut      = 0;

UiScreen ui = RUN_SCREEN;
CalItem  calItem = CALI_ADC_SCALE_UP;

uint32_t btn1DownAt = 0, btn2DownAt = 0;
bool btn1WasDown = false, btn2WasDown = false;

uint16_t rawBat=0, rawPv=0;

/*************** BUTTON HELPERS ***************/
inline bool btn1Down(){ return digitalRead(BTN1_PIN)==LOW; }
inline bool btn2Down(){ return digitalRead(BTN2_PIN)==LOW; }

/*************** UTILS ***************/
uint16_t readADCavg(uint8_t pin){
  analogRead(pin); // throw-away first read after mux switch
  uint32_t sum=0;
  for(uint16_t i=0;i<ADC_SAMPLES;i++) sum += analogRead(pin);
  return (uint16_t)(sum / ADC_SAMPLES);
}

float dividerVoltage(uint16_t adc, float R1, float R2, float scale, float offset){
  // Using 5.0V reference instead of 1.1V to prevent saturation at 17V+
  float v_in = (adc / 1023.0f) * 5.0f; 
  float v_actual = v_in * (R1 + R2) / R2;
  return v_actual * scale + offset;
}

float readNTCdegC(){
  uint16_t adc = readADCavg(NTC_PIN);
  
  if (adc >= 1023) return -99.0f;
  if (adc <= 0)    return 99.0f;

  // Since everything is 5V now, we don't need to switch references anymore
  float Rntc = NTC_R_FIXED * ((float)adc / (1023.0f - (float)adc));
  float invT = (1.0f/NTC_T0_KELVIN) + (1.0f/NTC_BETA)*log(Rntc/NTC_R0);
  float Tk = 1.0f / invT;
  return Tk - 273.15f;
}

void setChargeOutput(uint8_t duty){
  if (CONTROL_MODE == PWM_MODE) analogWrite(CHG_PIN, duty);
  else digitalWrite(CHG_PIN, (duty >= 128) ? HIGH : LOW);
  pwmOut = duty;
}
void chargeOn(){ setChargeOutput(CONTROL_MODE==PWM_MODE ? PWM_MAX : 255); }
void chargeOff(){ setChargeOutput(0); }

void enterStage(Stage s){
  stage = s;
  if (s==ABSORB) absorbStart = millis();
  integrator=0.0f;
}

Profile& currentProfile(){ return (batteryMode==MODE_FLA) ? profileFLA : profileAGM; }

float tempCompensate(float baseV, float tempC, float slopePerDegC){
  float dT = tempC - 25.0f;
  return baseV + slopePerDegC * dT;
}

int estimateSOC(float vBat, Stage s){
  if (s!=NIGHT && vBat>13.0f) return 100;
  struct P{ float v; int soc; };
  const P curve[] = {
    {11.9f,10},{12.0f,20},{12.2f,40},{12.3f,55},{12.4f,70},
    {12.5f,80},{12.6f,90},{12.7f,95},{12.8f,100}
  };
  if (vBat <= curve[0].v) return curve[0].soc;
  for (int i=1;i<(int)(sizeof(curve)/sizeof(curve[0]));++i){
    if (vBat <= curve[i].v){
      float t = (vBat - curve[i-1].v)/(curve[i].v - curve[i-1].v);
      return (int)(curve[i-1].soc + t*(curve[i].soc - curve[i-1].soc) + 0.5f);
    }
  }
  return 100;
}

/*************** UI ***************/
void drawRunScreen(float vBat, float vPv, float tempC, float vBulkT, float vFloatT){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0);
  display.print(F("Stage: "));
  switch(stage){
    case NIGHT:  display.print(F("NIGHT")); break;
    case BULK:   display.print(F("BULK")); break;
    case ABSORB: display.print(F("ABSORB")); break;
    case FLOAT:  display.print(F("FLOAT")); break;
  }
  display.setCursor(80,0);
  display.print( (batteryMode==MODE_FLA) ? F("FLA") : F("AGM") );

  display.setCursor(0,12);
  display.print(F("Vbat: ")); display.print(vBat, 2); display.print(F("V"));
  display.setCursor(0,22);
  display.print(F("Vpv : ")); display.print(vPv, 2); display.print(F("V"));

  display.setCursor(0,32);
  float dispTemp = useFahrenheit ? (tempC * 9.0f/5.0f + 32.0f) : tempC;
  display.print(F("Temp: ")); display.print(dispTemp,1);
  display.print(useFahrenheit ? F("F  ") : F("C  "));
  display.print(F("Out: "));
  if (CONTROL_MODE==PWM_MODE){
    int pct = (int)((pwmOut/255.0f)*100.0f + 0.5f);
    display.print(pct); display.print(F("%"));
  } else {
    display.print( (pwmOut>0) ? F("ON") : F("OFF") );
  }

  display.setCursor(0,42);
  display.print(F("Bulk/Abs: ")); display.print(vBulkT,2); display.print(F("V"));
  display.setCursor(0,52);
  display.print(F("Float: ")); display.print(vFloatT,2); display.print(F("V"));
  display.setCursor(80,52);
  display.print(F("SoC ")); display.print(estimateSOC(vBat, stage)); display.print(F("%"));

  display.display();
}

const __FlashStringHelper* calItemName(CalItem it){
  switch(it){
    case CALI_ADC_SCALE_UP:    return F("Scale +0.2% / +1%");
    case CALI_ADC_SCALE_DOWN:  return F("Scale -0.2% / -1%");
    case CALI_ADC_OFFSET_UP:   return F("Offset +10mV / +50mV");
    case CALI_ADC_OFFSET_DOWN: return F("Offset -10mV / -50mV");
    case CALI_CAPTURE_BULK:    return F("Capture BULK (Btn2 Long)");
    case CALI_CAPTURE_FLOAT:   return F("Capture FLOAT (Btn2 Long)");
    case CALI_TEMP_UNIT:       return F("Temp Units: C/F (Btn2)");
    case CALI_EXIT:            return F("Exit (Btn2 Long)");
  }
  return F("?");
}

void drawCalMenu(float vBat){
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0);  display.print(F("CAL MENU  (Btn1:Next)"));
  display.setCursor(0,12); display.print(calItemName(calItem));

  display.setCursor(0,28);
  display.print(F("Scale: "));  display.print(cal_scale_bat,4);
  display.setCursor(0,38);
  display.print(F("Offset: ")); display.print(cal_offset_bat,3); display.print(F("V"));

  display.setCursor(0,50);
  display.print(F("Vbat: ")); display.print(vBat,2); display.print(F("V"));
  display.setCursor(80,50);
  display.print(F("Unit: ")); display.print(useFahrenheit ? F("F") : F("C"));

  display.display();
}

/*************** BUTTONS ***************/
void handleButtons_RUN(){
  bool b1 = btn1Down();
  bool b2 = btn2Down();
  uint32_t now = millis();

  // --- Button 1 (D2) Logic ---
  if (b1 && !btn1WasDown) btn1DownAt = now;
  if (!b1 && btn1WasDown){
    if ((now - btn1DownAt) < 1500){
      // short press: toggle battery profile
      batteryMode = (batteryMode==MODE_FLA)?MODE_AGM:MODE_FLA;
      EEPROM.update(EE_MODE, (uint8_t)batteryMode);
    }
  }
  if (b1 && btn1WasDown && (now - btn1DownAt) >= 3000){
    // long press: enter CAL
    ui = CAL_MENU;
    calItem = CALI_ADC_SCALE_UP;
    btn1DownAt = now + 100000; 
  }

  // --- Button 2 (D3) Logic (NEW) ---
  if (b2 && !btn2WasDown) btn2DownAt = now;
  if (!b2 && btn2WasDown){
    if ((now - btn2DownAt) < 1500){
      // short press: toggle F/C
      useFahrenheit = !useFahrenheit;
      // Save to EEPROM (using your existing EE_TEMP_UNIT address)
      EEPROM.update(EE_TEMP_UNIT, (uint8_t)(useFahrenheit ? 1 : 0));
    }
  }

  btn1WasDown = b1;
  btn2WasDown = b2;
}

void handleButtons_CAL(float vBat){
  bool b1 = btn1Down();
  bool b2 = btn2Down();
  uint32_t now = millis();

  // BTN1 short = next item
  if (b1 && !btn1WasDown) btn1DownAt = now;
  if (!b1 && btn1WasDown){
    if ((now - btn1DownAt) < 1500){
      calItem = (CalItem)((calItem + 1) % (CALI_EXIT + 1));
    }
  }

  // BTN2 short = small adjust / toggle
  if (b2 && !btn2WasDown) btn2DownAt = now;
  if (!b2 && btn2WasDown){
    if ((now - btn2DownAt) < 1200){
      switch(calItem){
        case CALI_ADC_SCALE_UP:
          cal_scale_bat += 0.002f;
          ewrite(EE_CAL_SCALE_BAT, &cal_scale_bat, sizeof(cal_scale_bat));
          break;

        case CALI_ADC_SCALE_DOWN:
          cal_scale_bat -= 0.002f;
          ewrite(EE_CAL_SCALE_BAT, &cal_scale_bat, sizeof(cal_scale_bat));
          break;

        case CALI_ADC_OFFSET_UP:
          cal_offset_bat += 0.010f;
          ewrite(EE_CAL_OFFSET_BAT, &cal_offset_bat, sizeof(cal_offset_bat));
          break;

        case CALI_ADC_OFFSET_DOWN:
          cal_offset_bat -= 0.010f;
          ewrite(EE_CAL_OFFSET_BAT, &cal_offset_bat, sizeof(cal_offset_bat));
          break;

        case CALI_TEMP_UNIT:
          useFahrenheit = !useFahrenheit;
          EEPROM.update(EE_TEMP_UNIT, (uint8_t)(useFahrenheit ? 1 : 0));
          break;

        default:
          break;
      }
    }
  }

  // BTN2 long = large adjust / confirm
  if (b2 && btn2WasDown && (now - btn2DownAt) >= 1800){
    switch(calItem){
      case CALI_ADC_SCALE_UP:
        cal_scale_bat += 0.010f;
        ewrite(EE_CAL_SCALE_BAT, &cal_scale_bat, sizeof(cal_scale_bat));
        break;

      case CALI_ADC_SCALE_DOWN:
        cal_scale_bat -= 0.010f;
        ewrite(EE_CAL_SCALE_BAT, &cal_scale_bat, sizeof(cal_scale_bat));
        break;

      case CALI_ADC_OFFSET_UP:
        cal_offset_bat += 0.050f;
        ewrite(EE_CAL_OFFSET_BAT, &cal_offset_bat, sizeof(cal_offset_bat));
        break;

      case CALI_ADC_OFFSET_DOWN:
        cal_offset_bat -= 0.050f;
        ewrite(EE_CAL_OFFSET_BAT, &cal_offset_bat, sizeof(cal_offset_bat));
        break;

      case CALI_CAPTURE_BULK: {
        Profile &p = currentProfile();
        p.bulk_target = vBat;
        if (batteryMode==MODE_FLA) ewrite(EE_FLA_START, &profileFLA, sizeof(Profile));
        else                        ewrite(EE_AGM_START, &profileAGM, sizeof(Profile));
        break;
      }

      case CALI_CAPTURE_FLOAT: {
        Profile &p = currentProfile();
        p.float_target = vBat;
        if (batteryMode==MODE_FLA) ewrite(EE_FLA_START, &profileFLA, sizeof(Profile));
        else                        ewrite(EE_AGM_START, &profileAGM, sizeof(Profile));
        break;
      }

      case CALI_TEMP_UNIT:
        useFahrenheit = !useFahrenheit;
        EEPROM.update(EE_TEMP_UNIT, (uint8_t)(useFahrenheit ? 1 : 0));
        break;

      case CALI_EXIT:
        ui = RUN_SCREEN;
        break;
    }

    btn2DownAt = now + 100000; // prevent repeats
  }

  btn1WasDown = b1;
  btn2WasDown = b2;
}

/*************** CONTROL ***************/
void regulateVoltage(float vBat, float vSet){
  if (CONTROL_MODE == RELAY_MODE){
    const float HYST = 0.08f;
    if (vBat < (vSet - HYST)) chargeOn();
    else if (vBat > (vSet + HYST)) chargeOff();
    return;
  }

  float error = vSet - vBat;
  integrator += error * (CONTROL_PERIOD_MS / 1000.0f);

  if (integrator > 20.0f) integrator = 20.0f;
  if (integrator < -2.0f) integrator = -2.0f;

  float u = Kp * error + Ki * integrator;
  int duty = pwmOut + (int)(u * 10.0f);

  if (duty < 0) duty = 0;
  if (duty > PWM_MAX) duty = PWM_MAX;
  if (vBat >= (vSet - 0.03f) && duty < PWM_MIN) duty = PWM_MIN;

  setChargeOutput((uint8_t)duty);
}

/*************** EEPROM INIT ***************/
void loadOrInitEEPROM(){
  if (EEPROM.read(EE_MAGIC_ADDR) != EE_MAGIC_VAL){
    EEPROM.update(EE_MAGIC_ADDR, EE_MAGIC_VAL);

    uint8_t mode = (uint8_t)MODE_FLA;
    EEPROM.update(EE_MODE, mode);

    float scb=1.0f, offb=0.0f, scp=1.0f, offp=0.0f;
    ewrite(EE_CAL_SCALE_BAT,  &scb, sizeof(scb));
    ewrite(EE_CAL_OFFSET_BAT, &offb, sizeof(offb));
    ewrite(EE_CAL_SCALE_PV,   &scp, sizeof(scp));
    ewrite(EE_CAL_OFFSET_PV,  &offp, sizeof(offp));

    ewrite(EE_FLA_START, &DEFAULT_FLA, sizeof(Profile));
    ewrite(EE_AGM_START, &DEFAULT_AGM, sizeof(Profile));

    EEPROM.update(EE_TEMP_UNIT, (uint8_t)0); // default to °C
  }

  uint8_t mode = EEPROM.read(EE_MODE);
  batteryMode = (mode==1)?MODE_AGM:MODE_FLA;

  eread(EE_CAL_SCALE_BAT,  &cal_scale_bat,  sizeof(cal_scale_bat));
  eread(EE_CAL_OFFSET_BAT, &cal_offset_bat, sizeof(cal_offset_bat));
  eread(EE_CAL_SCALE_PV,   &cal_scale_pv,   sizeof(cal_scale_pv));
  eread(EE_CAL_OFFSET_PV,  &cal_offset_pv,  sizeof(cal_offset_pv));

  eread(EE_FLA_START, &profileFLA, sizeof(Profile));
  eread(EE_AGM_START, &profileAGM, sizeof(Profile));

  uint8_t rawUnit = EEPROM.read(EE_TEMP_UNIT);
  if (rawUnit > 1) {
    useFahrenheit = false;
    EEPROM.update(EE_TEMP_UNIT, (uint8_t)0);
  } else {
    useFahrenheit = (rawUnit == 1);
  }
}

/*************** SETUP/LOOP ***************/
void setup(){
  pinMode(CHG_PIN, OUTPUT);
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  chargeOff();

  // Change or remove this line:
  // analogReference(INTERNAL); 
  analogReference(DEFAULT); // Set global reference to 5V
  
  delay(50);

  loadOrInitEEPROM();

  (void)display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay(); display.display();

  Serial.begin(115200);
  Serial.println(F("Solar Maintainer (OLED, TempComp, Calibration, Dual Buttons, C/F Units)"));
  Serial.print(F("Mode: ")); Serial.println( (batteryMode==MODE_FLA)?"FLA":"AGM" );

  Serial.print("cal_scale_bat="); Serial.println(cal_scale_bat, 6);
  Serial.print("cal_offset_bat="); Serial.println(cal_offset_bat, 6);
  Serial.print("cal_scale_pv=");  Serial.println(cal_scale_pv,  6);
  Serial.print("cal_offset_pv="); Serial.println(cal_offset_pv, 6);
}

void loop(){
  uint32_t now = millis();

  // Read inputs
  rawBat = readADCavg(BAT_V_PIN);
  rawPv  = readADCavg(PV_V_PIN);

  float vBat = dividerVoltage(rawBat, R1_BAT, R2_BAT, cal_scale_bat, cal_offset_bat);
  float vPv  = dividerVoltage(rawPv,  R1_PV,  R2_PV,  cal_scale_pv,  cal_offset_pv);
  float tempC = readNTCdegC();

  // Buttons
  if (ui==RUN_SCREEN) handleButtons_RUN();
  else                handleButtons_CAL(vBat);

  // Targets with temp comp
  Profile &p = currentProfile();
  float vBulkT  = tempCompensate(p.bulk_target,  tempC, p.temp_slope_v_per_degC);
  float vFloatT = tempCompensate(p.float_target, tempC, p.temp_slope_v_per_degC);

  // Control cadence
  if (now - lastControl >= CONTROL_PERIOD_MS){
    lastControl = now;

    bool pvPresent = (vPv > PV_PRESENT_MIN) && (vPv > vBat + PV_MARGIN_ON);

    if (!pvPresent){
      if (stage != NIGHT){ enterStage(NIGHT); chargeOff(); }
    } else {
      switch(stage){
        case NIGHT:
          if (vBat < REBULK_ENTRY_V) enterStage(BULK);
          else enterStage(FLOAT);
          break;

        case BULK:
          chargeOn();
          if (vBat >= vBulkT - 0.05f) enterStage(ABSORB);
          break;

        case ABSORB:
          regulateVoltage(vBat, vBulkT);
          if ((millis()-absorbStart) >= (uint32_t)(p.absorb_time_s*1000UL)) enterStage(FLOAT);
          break;

        case FLOAT:
          if (vBat < REBULK_ENTRY_V) { enterStage(BULK); break; }
          regulateVoltage(vBat, vFloatT);
          if (vBat < FLOAT_EXIT_V && CONTROL_MODE==RELAY_MODE) chargeOn();
          break;
      }
    }
  }

  // OLED
  if (ui==RUN_SCREEN) drawRunScreen(vBat, vPv, tempC, vBulkT, vFloatT);
  else               drawCalMenu(vBat);
}
