
#include <EEPROM.h>
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC
#include <Time.h>         //http://www.arduino.cc/playground/Code/Time  
#include <Wire.h>         //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <MAX7219LedMatrix.h>
#include <SoftwareSerial.h>
#include <JQ6500_Serial.h>
#include "font.h"

#define font IMAGES


/*
   9 = matrix DIN
   8 = matrix CS
   7 = matrix CLK

   6 = MP3 RX
   5 = MP3 TX

   2 = i2c clock SDA
   3 = i2c clock SCL

   10 = pwm out

   A0 = V Ref = V in / 7.8

   18 = switch
   16 = key
   14 = key
   15 = key

*/

#define MATRIX_DIN_PIN 9
#define MATRIX_CS_PIN 8
#define MATRIX_CLK_PIN 7
#define NUM_MATRIX 4

// 2 and 3 are i2c for clock
#define CLOCK_SDA_PIN 2
#define CLOCK_SCL_PIN 3

#define MP3_TX_PIN 5
#define MP3_RX_PIN 6
//#define SND_PIN 6

#define PWM_OUT_PIN 10
#define PWM_MAX 32767

#define KEY_MODE_PIN 16
#define KEY_PLUS_PIN 14
#define KEY_MINUS_PIN 15
#define KEY_ALARM_PIN 18

//#define LED_PIN 17
#define ALARM_PIN 17

// EEPROM addresses
#define EA_DIM 0
#define EA_VOLUME 1
#define EA_TRACK 2
#define EA_LIGHT_MINS 3
#define EA_ALARM_MINS 4

#define EA_ALARM_BASE 8

#define MODE_NONE 0
#define MODE_SEC 1
#define MODE_ALARM1 2
#define MODE_ALARM2 3
#define MODE_ALARM3 4
#define MODE_ALARM4 5
#define MODE_ALARM5 6
#define MODE_ALARM6 7
#define MODE_ALARM7 8
#define MODE_DATE 9
#define MODE_YEAR 10

#define MODE_DIM 20
#define MODE_PWM 21
#define MODE_TEMP 22
#define MODE_VREF 23
#define MODE_VOL 24
#define MODE_TRACK 25
#define MODE_LIGHT_MIN 26
#define MODE_ALARM_MIN 27
// MODE RADIO

#define KEY_MODE 1
//#define KEY_SET 2
#define KEY_PLUS 4
#define KEY_MINUS 8

#define NUM_ALARMS 7

#define NUMBER_NO_ZERO(x) (font[((x)?(x+0x10):0)])
#define NUMBER_FONT(x) (font[((x+0x10))])
#define NARROW_NUMBER_NO_ZERO(x) (font[((x)?(x)+0x60:0)])
#define NARROW_NUMBER_FONT(x) (font[((x+0x60))])
#define CHAR_FONT(x) (font[((x-0x20))])

struct alarm_t {
  byte flags; // 0 = off
  byte Hour;
  byte Minute;
  byte Second;
};

static byte lastsec = -1, lastkeys, tenthofsecond = 0;
static byte mode = 0, alarmon = 0, modefirst;
static unsigned short alarmsecs, settingcount, pushkeycount, modecount;
static char setting = 0;

static short pwm = 0, pwmTarget = 0; // 0-100
static short vref = 0;
static alarm_t alarms[NUM_ALARMS];
static tmElements_t t;
static char intensity = 0;
static char volume, track, countTracks = 5;
static byte alarm_mins, light_mins;
static byte alarm_enabled;

static byte fb[8][NUM_MATRIX];
#ifdef MATRIX_DIN_PIN
MAX7219LedMatrix matrix(MATRIX_DIN_PIN, MATRIX_CS_PIN, MATRIX_CLK_PIN, NUM_MATRIX, (byte *)fb);
#endif

// Create the mp3 module object,
//   Arduino Pin 5 is connected to TX of the JQ6500
//   Arduino Pin 6 is connected to one end of a  1k resistor,
//     the other end of the 1k resistor is connected to RX of the JQ6500
//   If your Arduino is 3v3 powered, you can omit the 1k series resistor
#ifdef MP3_TX_PIN
JQ6500_Serial mp3(MP3_TX_PIN, MP3_RX_PIN);
#endif

static void adjust(short increment);

static uint8_t dec2bcd(uint8_t n)
{
  return n + 6 * (n / 10);
}

static uint8_t __attribute__ ((noinline)) bcd2dec(uint8_t n)
{
  return n - 6 * (n >> 4);
}

void setup() {
  char buffer[80];

  Serial.begin(9600);
  Serial.println(F("DS3231 Real Time Clock"));
  Serial.println(F("Version 20160305ss"));

  //sprintf(buffer, "%02x %02x %02x %02x \n", RTC.readRTC(ALM2_MINUTES), RTC.readRTC(ALM2_HOURS), RTC.readRTC(2), RTC.readRTC(3));
  //Serial.println(buffer);
  setSyncProvider(RTC.get);   // the function to get the time from the RTC

  intensity = EEPROM.read(EA_DIM) & 0x0f;
  readAlarms();

#ifdef MATRIX_DIN_PIN
  matrix.setup();
  fb[0][3] = B00111100;
  fb[1][3] = B01000010;
  fb[2][3] = B10100101;
  fb[3][3] = B10000001;
  fb[4][3] = B10100101;
  fb[5][3] = B10011001;
  fb[6][3] = B01000010;
  fb[7][3] = B00111100;
  matrix.show();
  matrix.maxAll(0x0a, intensity & 0x0f);
#endif

  pinMode(KEY_MODE_PIN, INPUT_PULLUP);
  pinMode(KEY_PLUS_PIN, INPUT_PULLUP);
  pinMode(KEY_MINUS_PIN, INPUT_PULLUP);
  pinMode(KEY_ALARM_PIN, INPUT_PULLUP);

  pinMode(PWM_OUT_PIN,   OUTPUT);
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);

#ifdef LED_PIN
  pinMode(LED_PIN,   OUTPUT);
#endif
#ifdef SND_PIN
  pinMode(SND_PIN,   INPUT);
  attachInterrupt(digitalPinToInterrupt(snd), wakeUpNow, HIGH); // use interrupt 0 (pin 2) and run function
#endif
  setupPWM16();

  volume = EEPROM.read(EA_VOLUME) & 0x1f;
  track = EEPROM.read(EA_TRACK) & 0x03;
#ifdef MP3_TX_PIN
  pinMode(MP3_TX_PIN, INPUT);
  pinMode(MP3_RX_PIN, OUTPUT);
  mp3.begin(9600);
  mp3.reset();
  mp3.setVolume(volume);
  mp3.setLoopMode(MP3_LOOP_NONE);
  // countTracks = mp.getCount... Does not work for me
#endif
  light_mins = EEPROM.read(EA_LIGHT_MINS) % 100;
  alarm_mins = EEPROM.read(EA_ALARM_MINS) % 100;

#if 0
  // disable comparator
  ACSR |= _BV(ADC );
  // Disable the ADC by setting the ADEN bit (bit 7)  of the
  // ADCSRA register to zero.
  ADCSRA = ADCSRA & B01111111;
#endif

#if 0
  t.Hour = 23;
  RTC.writeRTC(RTC_HOURS, dec2bcd(t.Hour));
  t.Minute = 04;
  RTC.writeRTC(RTC_MINUTES, dec2bcd(t.Minute));
  t.Second = 0;
  RTC.writeRTC(RTC_SECONDS, dec2bcd(t.Second));
  alarm.Hour = 7;
  RTC.writeRTC(ALM2_HOURS, dec2bcd(alarm.Hour));
  alarm.Minute = 5;
  RTC.writeRTC(ALM2_MINUTES, dec2bcd(alarm.Minute));
  t.Day = 6;
  RTC.writeRTC(RTC_DATE, dec2bcd(t.Day));
  t.Month = 2;
  RTC.writeRTC(RTC_MONTH, dec2bcd(t.Month));
#endif

}


#ifdef SND_PIN
volatile byte snd_detected;

void wakeUpNow()        // here the interrupt is handled after wakeup
{
  snd_detected = 1;
}
#endif

/* 16-bit version of analogWrite(). Works only on pins 9 and 10. */
static void analogWrite16(uint8_t pin, uint16_t val)
{
  switch (pin) {
    //case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
  //analogWrite(pin, val);
}

void setupPWM16() {
  // Stop the timer while we muck with it

  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);

  // Set the timer to mode 14...
  //
  // Mode  WGM13  WGM12  WGM11  WGM10  Timer/Counter Mode of Operation  TOP   Update of OCR1x at TOV1  Flag Set on
  //              CTC1   PWM11  PWM10
  // ----  -----  -----  -----  -----  -------------------------------  ----  -----------------------  -----------
  // 14    1      1      1      0      Fast PWM                         ICR1  BOTTOM                   TOP

  // Set output on Channel A and B to...
  //
  // COM1z1  COM1z0  Description
  // ------  ------  -----------------------------------------------------------
  // 1       0       Clear OC1A/OC1B on Compare Match (Set output to low level).

  TCCR1A =
    //(1 << COM1A1) | (0 << COM1A0) |   // COM1A1, COM1A0 = 1, 0
    (1 << COM1B1) | (0 << COM1B0) |
    (1 << WGM11) | (0 << WGM10);      // WGM11, WGM10 = 1, 0

  // Set TOP to...
  //
  // fclk_I/O = 16000000
  // N        = 1
  // TOP      = 799
  //
  // fOCnxPWM = fclk_I/O / (N * (1 + TOP))
  // fOCnxPWM = 16000000 / (1 * (1 + 799))
  // fOCnxPWM = 16000000 / 800
  // fOCnxPWM = 20000

  ICR1 = PWM_MAX;

  // Ensure the first slope is complete

  TCNT1 = 0;

  // Ensure Channel A and B start at zero / off

  //OCR1A = 0;
  OCR1B = 0;

  // We don't need no stinkin interrupts

  TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);

  // Ensure the Channel A and B pins are configured for output
  //DDRB |= (1 << DDB1);
  //DDRB |= (1 << DDB2);

  //DDRB |= (1 << DDB5);
  DDRB |= (1 << DDB6); // Arduino pin 10

  // Start the timer...
  //
  // CS12  CS11  CS10  Description
  // ----  ----  ----  ------------------------
  // 0     0     1     clkI/O/1 (No prescaling)

  TCCR1B =
    (0 << ICNC1) | (0 << ICES1) |
    (1 << WGM13) | (1 << WGM12) |              // WGM13, WGM12 = 1, 1
    (0 << CS12) | (0 << CS11) | (1 << CS10);
}



void loop() {
  byte keys = 0;
  byte pushed, halfsec;
  short i;

#ifdef LED_PIN
  digitalWrite(LED_PIN, HIGH);
#endif

  if (!digitalRead(KEY_MODE_PIN)) keys |= KEY_MODE;
  if (!digitalRead(KEY_PLUS_PIN)) keys |= KEY_PLUS;
  if (!digitalRead(KEY_MINUS_PIN)) keys |= KEY_MINUS;
  alarm_enabled = !digitalRead(KEY_ALARM_PIN);

  RTC.read(t);
  t.Year = RTC.readRTC(RTC_YEAR);

  vref += analogRead(A0);
  vref /= 2;

  pushed = keys & ~lastkeys;

  if (!setting && (keys & (KEY_MODE | KEY_PLUS | KEY_MINUS))) {
    pushkeycount++;
    if (pushkeycount > 10 && (keys & (KEY_PLUS | KEY_MINUS))) {
      setting = 1;
      settingcount = 30;
      pushkeycount = 0;
    }
    if (pushkeycount > 10 && (keys & KEY_MODE)) {
      mode = MODE_DIM;
      pushkeycount = 0;
    }
    if (mode <= MODE_SEC && !setting && (pushed & KEY_MINUS)) {
      pwmTarget = 0;
    }
    if (mode <= MODE_SEC && !setting && (pushed & KEY_PLUS)) {
      pwmTarget = 100;
    }
  } else {
    pushkeycount = 0;
  }

  if (pushed & KEY_MODE) {
    if (!setting) {
      mode++;
      if (mode >= MODE_ALARM1 && mode <= MODE_ALARM7) {
        int emptyAlarm;
        for ( emptyAlarm = 0; emptyAlarm < NUM_ALARMS; emptyAlarm++) {
          if (!alarms[emptyAlarm].flags) {
            break;
          }
        }
        while (mode <= MODE_ALARM7 && !alarms[mode - MODE_ALARM1].flags && (mode - MODE_ALARM1) != emptyAlarm) {
          mode++; // jump over empty alarms, except the first one
        }
      }
      setting = 0;
      modecount = 120;
      modefirst = 1;
    } else {
      byte settingMax = 3;
      switch (mode) {
        case MODE_SEC:
          // TODO Alarms
          settingMax = 3;
          break;
        case MODE_NONE:
        case MODE_DATE:
          settingMax = 2;
          break;
        case MODE_YEAR:
        case MODE_DIM:
        case MODE_TRACK:
        case MODE_VOL:
        case MODE_LIGHT_MIN:
        case MODE_ALARM_MIN:
          settingMax = 1;
          break;
        case MODE_TEMP:
        case MODE_VREF:
          settingMax = 0;
          break;
      }
      setting++;
      if (setting > settingMax) {
        setting = 0;
      }
    }
  }

  if (setting) {
    if (pushed) {
      settingcount = 30;
      if (pushed & KEY_MINUS) {
        adjust(1);
      }
      if (pushed & KEY_PLUS) {
        adjust(-1);
      }
    }
  }

  if (alarm_enabled) {
    short year = t.Year + 1970;
    byte wday = dayOfWeek2(year, t.Month, t.Day);
    for (byte alarmNumber = 0; alarmNumber < NUM_ALARMS; alarmNumber++) {
      byte flags = alarms[alarmNumber].flags;
      int secs = t.Hour * 3600 + t.Minute * 60 + t.Second;
      if (flags && (flags == 8 || flags == wday || (wday == 0 && flags == 7))) {
        int asecs = alarms[alarmNumber].Hour * 3600 + alarms[alarmNumber].Minute * 60 + alarms[alarmNumber].Second;
        if (asecs == secs) {
          alarm_on();
        }
        // FIXME invalid at midnight!
        if (secs + light_mins * 60 > asecs && secs < asecs) {
          pwmTarget = 100 - (asecs - secs) * (int)100 / (light_mins * 60);
        }
      }
    }
    if (alarmon) {
      pwmTarget = 100;
      //analogWrite(ALARM_PIN, tenthofsecond * 16);
      digitalWrite(ALARM_PIN, (t.Second & 1) ? HIGH : LOW);

      if (pushed) {
        alarm_off();
      }

      if (t.Second != lastsec) {
        if (alarmon) {
          alarmsecs --;
          if (alarmsecs <= 0) {
            alarm_off();
          }
        }
      }
    }
  } else {
    if (alarmon) {
      alarm_off();
    }
  }

  if (t.Second != lastsec) {
    if (setting) {
      if (settingcount > 0) {
        settingcount--;
      } else {
        setting = 0;
      }
    }
    tenthofsecond = 0;
    if (mode > MODE_SEC) {
      modecount--;
      if (!modecount) {
        mode = MODE_NONE;
      }
    }
  }


  halfsec = tenthofsecond == 5;
  if (t.Second != lastsec || pushed || (setting && halfsec)) {
    if (!halfsec || pushed) {
      show(setting, 0);
    } else {
      show(setting, 1);
    }
  }

  while (pwm != pwmTarget) {
    if (pwm < pwmTarget) {
      pwm++;
    }
    if (pwm > pwmTarget) {
      pwm--;
    }

    {
      long pwm2 = pwm % 100; // 0-100
      if (pwm2) {
        if (pwm2 > 50) {
          pwm2 += (pwm2 - 50) * 600;
        }
        pwm2 = pwm2 + 20;
      }
      if (pwm >= 100) {
        pwm2 = PWM_MAX;
      }

      // vref = 1024 * (vin / 7.8) / 5
      // ~500 = 19.5V
      // ~305 = 12V
      // vin = vref * 7.8 * 5 / 1024
      //
#define VREF12 280
#if 0
      if (vref > VREF12) {
        pwm2 = pwm2 * VREF12 / vref;
      }
#endif

      analogWrite16(PWM_OUT_PIN, pwm2);
      //analogWrite(PWM_OUT_PIN, pwm2);
    }
    if (!setting) delay(10);
  }

  lastsec = t.Second;
  lastkeys = keys;
#ifdef LED_PIN
  digitalWrite(LED_PIN, LOW);
#endif
  tenthofsecond++;
  if (tenthofsecond > 15) {
    lastsec = -1;
  }

  delay(100);
}

static void alarm_on() {
#ifdef MP3_TX_PIN
  mp3.setLoopMode(MP3_LOOP_ONE);
  mp3.playFileByIndexNumber(track);
#endif
  alarmon = 1;
  alarmsecs = light_mins * 60;
  pwmTarget = 100;
}

static void alarm_off() {
#ifdef MP3_TX_PIN
  mp3.setLoopMode(MP3_LOOP_NONE);
  mp3.pause();
#endif
  alarmon = 0;
  pwmTarget = 0;
}


static void show(byte setting, byte blink) {
  memset(fb, 0, sizeof fb);

  switch (mode) {
    case MODE_NONE:
      if (setting != 1 || blink) {
        drawChar(-1, NUMBER_NO_ZERO(t.Hour / 10));
        drawChar(7, NUMBER_FONT(t.Hour % 10));
      }
      if (setting != 2 || blink) {
        drawChar(17, NUMBER_FONT(t.Minute / 10));
        drawChar(25, NUMBER_FONT(t.Minute % 10));
      }
      fb[1][1] |= 0x01;
      fb[2][1] |= 0x01;
      fb[5][1] |= 0x01;
      fb[6][1] |= 0x01;
      fb[1][2] |= 0x80;
      fb[2][2] |= 0x80;
      fb[5][2] |= 0x80;
      fb[6][2] |= 0x80;
      if (alarm_enabled) {
        fb[7][1] |= 0x01;
        fb[7][2] |= 0x80;
      }
      break;
    case MODE_SEC:
      //show = 6;
      //dots[3] = 1;
      // 1111.222 2...3333 .4444..5 555.6666
      if (setting != 1 || blink) {
        drawChar(0 - 2, NARROW_NUMBER_NO_ZERO(t.Hour / 10));
        drawChar(5 - 2, NARROW_NUMBER_FONT(t.Hour % 10));
      }
      if (setting != 2 || blink) {
        drawChar(12 - 2, NARROW_NUMBER_FONT(t.Minute / 10));
        drawChar(17 - 2, NARROW_NUMBER_FONT(t.Minute % 10));
      }
      if (setting != 3 || blink) {
        drawChar(23 - 2, NARROW_NUMBER_FONT(t.Second / 10));
        drawChar(28 - 2, NARROW_NUMBER_FONT(t.Second % 10));
      }
      // colon between hours and minutes
      fb[1][1] |= 0x20;
      fb[2][1] |= 0x20;
      fb[5][1] |= 0x20;
      fb[6][1] |= 0x20;
      if (alarm_enabled) {
        fb[7][1] |= 0x20;
      }
      break;
    case MODE_ALARM1:
    case MODE_ALARM2:
    case MODE_ALARM3:
    case MODE_ALARM4:
    case MODE_ALARM5:
    case MODE_ALARM6:
    case MODE_ALARM7:
      {
        byte alarmNumber = mode - MODE_ALARM1;
        if (!alarms[alarmNumber].flags || setting == 1 || modefirst) {
          modefirst = 0;
          // No alarm, or setting mode
          drawChar(0, CHAR_FONT('A'));
          if (blink) {
            modefirst = 0;
            //drawChar(8, NARROW_NUMBER_FONT(alarmNumber));
            switch (alarms[alarmNumber].flags) {
              case 0:
                break;
              case 1:
              case 2:
              case 3:
              case 4:
              case 5:
              case 6:
              case 7:
                drawChar(8, CHAR_FONT('D'));
                drawChar(16, CHAR_FONT('A'));
                drawChar(24, CHAR_FONT('Y'));
                break;
              case 8:
                //drawChar(16, CHAR_FONT('A'));
                break;
            }
          } else {
            switch (alarms[alarmNumber].flags) {
              case 0:
                drawChar(8, CHAR_FONT('O'));
                drawChar(16, CHAR_FONT('F'));
                drawChar(24, CHAR_FONT('F'));
                break;
              case 1:
              case 2:
              case 3:
              case 4:
              case 5:
              case 6:
              case 7:
                drawChar(24, NUMBER_FONT(alarms[alarmNumber].flags));
                break;
              case 8:
                drawChar(8, CHAR_FONT('A'));
                drawChar(16, CHAR_FONT('L'));
                drawChar(24, CHAR_FONT('L'));
                break;
            }
          }
        } else {
          if (setting != 2 || blink) {
            drawChar(0, NUMBER_NO_ZERO(alarms[alarmNumber].Hour / 10));
            drawChar(8, NUMBER_FONT(alarms[alarmNumber].Hour % 10));
          }
          if (setting != 3 || blink) {
            drawChar(16, NUMBER_FONT(alarms[alarmNumber].Minute / 10));
            drawChar(24, NUMBER_FONT(alarms[alarmNumber].Minute % 10));
            //dots[5] = 1;
          }
        }
      }
      break;
    case MODE_DATE:
      if (modefirst) {
        modefirst = 0;
        drawChar(0, CHAR_FONT('D'));
        drawChar(8, CHAR_FONT('A'));
        drawChar(16, CHAR_FONT('T'));
        drawChar(24, CHAR_FONT('E'));
      } else {
        if (setting != 1 || blink) {
          drawChar(-1, NUMBER_NO_ZERO(t.Day / 10));
          drawChar(6, NUMBER_FONT(t.Day % 10));
        }
        if (setting != 2 || blink) {
          drawChar(15, NUMBER_NO_ZERO(t.Month / 10));
          drawChar(22, NUMBER_FONT(t.Month % 10));
        }
        fb[7][1] |= 0x03;
        fb[6][1] |= 0x03;
        fb[7][3] |= 0x03;
        fb[6][3] |= 0x03;
      }
      break;
    case MODE_YEAR:
      {
        short year = t.Year + 1970;
        if (modefirst) {
          modefirst = 0;
          drawChar(0, CHAR_FONT('Y'));
          drawChar(8, CHAR_FONT('E'));
          drawChar(16, CHAR_FONT('A'));
          drawChar(24, CHAR_FONT('R'));
        } else {
          if (setting != 1 || blink) {
            drawChar(0, NUMBER_FONT(year / 1000));
            drawChar(8, NUMBER_FONT((year / 100) % 10));
            drawChar(16, NUMBER_FONT((year / 10) % 10));
            drawChar(24, NUMBER_FONT(year % 10));
          }
        }
      }
      break;
    case MODE_TEMP:
      {
        short temp = 0;
        byte minus = 0;
        temp = RTC.temperature();
        minus = 0;
        temp = temp * 10 / 4;
        if (temp < 0) {
          minus = 1;
          temp = -temp;
        }
        //chars[4] = minus ? MINUS : 0;
        drawChar(25, CHAR_FONT('C'));
        drawChar(0, NUMBER_NO_ZERO((temp / 100) % 10));
        drawChar(7, NUMBER_FONT((temp / 10) % 10));
        drawChar(17, NUMBER_FONT(temp % 10));
        fb[6][2] |= 0x80;
        fb[7][2] |= 0x80;
        fb[6][1] |= 0x01;
        fb[7][1] |= 0x01;
      }
      break;
    case MODE_VREF:
      {
        drawChar(0, NUMBER_FONT((vref / 1000) % 10));
        drawChar(8, NUMBER_FONT((vref / 100) % 10));
        drawChar(16, NUMBER_FONT((vref / 10) % 10));
        drawChar(24, NUMBER_FONT(vref % 10));
      }
      break;
    case MODE_DIM:
      drawChar(0, CHAR_FONT('D'));
      drawChar(8, CHAR_FONT('I'));
      if (setting != 1 || blink) {
        drawChar(16, NUMBER_NO_ZERO(intensity / 10));
        drawChar(24, NUMBER_FONT(intensity % 10));
      }
      break;
    case MODE_PWM:
      drawChar(0, CHAR_FONT('P'));
      if (setting != 1 || blink) {
        drawChar(8, NUMBER_FONT((pwm / 100) % 10));
        drawChar(16, NUMBER_FONT((pwm / 10) % 10));
        drawChar(24, NUMBER_FONT(pwm % 10));
      }
      break;
    case MODE_VOL:
      drawChar(0, CHAR_FONT('V'));
      drawChar(8, CHAR_FONT('O'));
      if (setting != 1 || blink) {
        drawChar(16, NUMBER_FONT((volume / 10) % 10));
        drawChar(24, NUMBER_FONT(volume % 10));
      }
      break;
    case MODE_TRACK:
      drawChar(0, CHAR_FONT('T'));
      drawChar(8, CHAR_FONT('R'));
      if (setting != 1 || blink) {
        drawChar(16, NUMBER_FONT((track / 10) % 10));
        drawChar(24, NUMBER_FONT(track % 10));
      }
      break;
    case MODE_LIGHT_MIN:
      drawChar(0, CHAR_FONT('L'));
      drawChar(8, CHAR_FONT('M'));
      if (setting != 1 || blink) {
        drawChar(16, NUMBER_FONT((light_mins / 10) % 10));
        drawChar(24, NUMBER_FONT(light_mins % 10));
      }
      break;
    case MODE_ALARM_MIN:
      drawChar(0, CHAR_FONT('A'));
      drawChar(8, CHAR_FONT('M'));
      if (setting != 1 || blink) {
        drawChar(16, NUMBER_FONT((alarm_mins / 10) % 10));
        drawChar(24, NUMBER_FONT(alarm_mins % 10));
      }
      break;
    default:
      mode = MODE_NONE;
  }

#ifdef SND_PIN
  if (snd_detected) {
    memset(fb, 0, sizeof fb);
    snd_detected = 0;
    // attachInterrupt(0, wakeUpNow, LOW); // us6e interrupt 0 (pin 2) and run function wakeUpNow when pin 2 gets LOW
    fb[0][2] = B00111100;                     //  + - - - - - - -
    fb[1][2] = B01000010;                     //  - + - - - - - -
    fb[2][2] = B10100101;                     //  - - + - - - - -
    fb[3][2] = B10000001;                     //  - - - + - - - -
    fb[4][2] = B10011001;                     //  - - - - + - - -
    fb[5][2] = B10100101;                     //  - - - - - + - -
    fb[6][2] = B01000010;                     //  - - - - - - + -
    fb[7][2] = B00111100;                     //  - - - - - - - +
  }
#endif

#ifdef MATRIX_DIN_PIN
  matrix.show();
#endif

  //    snprintf( buffer, sizeof(buffer), "%02d:%02d:%02d", t.Hour, t.Minute, t.Second);
  //    snprintf(buffer2, sizeof(buffer2), "%d.%d. %d", t.Day, t.Month, t.Year-30+2000);

  //    Serial.write(buffer);
  //    Serial.write("\n");

}

static void drawChar(short pos, byte * ch) {
  short i = pos / 8;
  byte off = pos & 7;
  if (pos < 0) {
    i = -1;
  }
  for (byte j = 0; j < 8; j++) {
    byte b = ch[j];
    if (i >= 0) {
      fb[j][i] |= b >> off;
    }
    if (i < 3 && off) {
      fb[j][i + 1] |= b << (8 - off);
    }
  }
}

static void readAlarms() {
  for (byte i = 0; i < sizeof(alarms); i++) {
    ((uint8_t*)alarms)[i] = EEPROM.read(EA_ALARM_BASE + i);
  }
  for (byte i = 0; i < NUM_ALARMS; i++) {
    if (alarms[i].flags > 8 || alarms[i].Hour > 23 || alarms[i].Minute > 59 || alarms[i].Second > 59) {
      alarms[i].flags = alarms[i].Hour = alarms[i].Minute = alarms[i].Second = 0;
      short alarmBase = EA_ALARM_BASE + 4 * i;
      EEPROM.write(alarmBase + 0, 0);
      EEPROM.write(alarmBase + 1, 0);
      EEPROM.write(alarmBase + 2, 0);
      EEPROM.write(alarmBase + 3, 0);
    }
  }
}

static void adjust(short increment) {
  switch (mode) {
    case MODE_NONE:
    case MODE_SEC:
      if (setting == 1) {
        t.Hour += increment;
        t.Hour %= 24;
        RTC.writeRTC(RTC_HOURS, dec2bcd(t.Hour));
      }
      if (setting == 2) {
        t.Minute += increment;
        t.Minute %= 60;
        RTC.writeRTC(RTC_MINUTES, dec2bcd(t.Minute));
      }
      if (setting == 3) {
        t.Second += increment;
        t.Second %= 60;
        RTC.writeRTC(RTC_SECONDS, dec2bcd(t.Second));
      }
      break;
    case MODE_ALARM1:
    case MODE_ALARM2:
    case MODE_ALARM3:
    case MODE_ALARM4:
    case MODE_ALARM5:
    case MODE_ALARM6:
    case MODE_ALARM7:
      {
        byte alarmNumber = mode - MODE_ALARM1;
        short alarmBase = EA_ALARM_BASE + 4 * (alarmNumber);
        if (setting == 1) {
          alarms[alarmNumber].flags += increment;
          alarms[alarmNumber].flags = alarms[alarmNumber].flags % 9;
          EEPROM.write(alarmBase + 0, alarms[alarmNumber].flags);
        }
        if (setting == 2) {
          alarms[alarmNumber].Hour += increment;
          alarms[alarmNumber].Hour = alarms[alarmNumber].Hour % 24;
          EEPROM.write(alarmBase + 1, alarms[alarmNumber].Hour);
        }
        if (setting == 3) {
          alarms[alarmNumber].Minute += increment;
          alarms[alarmNumber].Minute = alarms[alarmNumber].Minute % 60;
          EEPROM.write(alarmBase + 2, alarms[alarmNumber].Minute);
          EEPROM.write(alarmBase + 3, 0);
        }
        break;
      }
    case MODE_DATE:
      if (setting == 1) {
        t.Day += increment;
        if (t.Day > 31) t.Day = 1;
        if (t.Day < 1) t.Day = 31;
        RTC.writeRTC(RTC_DATE, dec2bcd(t.Day));
      }
      if (setting == 2) {
        t.Month += increment;
        if (t.Month > 12) t.Month = 1;
        if (t.Month < 1) t.Month = 12;
        RTC.writeRTC(RTC_MONTH, dec2bcd(t.Month));
      }
      break;
    case MODE_YEAR:
      if (setting == 1) {
        //t.Year = RTC.readRTC(RTC_YEAR);
        t.Year += increment;
        RTC.writeRTC(RTC_YEAR, t.Year);
      }
      break;
    case MODE_DIM:
      if (setting == 1) {
        intensity += increment;
        if (intensity > 15) intensity = 15;
        if (intensity < 0) intensity = 0;
#ifdef MATRIX_DIN_PIN
        matrix.maxAll(0x0a, intensity & 0x0f);
#endif
        EEPROM.write(EA_DIM, intensity);
      }
      break;
    case MODE_PWM:
      pwmTarget += increment;
      break;
    case MODE_VOL:
      volume += increment;
      if (volume < 0) volume = 0;
      if (volume > 30) volume = 30;
      EEPROM.write(EA_VOLUME, volume);
#ifdef MP3_TX_PIN
      mp3.setVolume(volume);
#endif
      break;
    case MODE_TRACK:
      track += increment;
      if (track < 1) track = 1;
      if (track > countTracks) track = countTracks;
      EEPROM.write(EA_TRACK, track);
#ifdef MP3_TX_PIN
      mp3.playFileByIndexNumber(track);
#endif
      break;
    case MODE_LIGHT_MIN:
      light_mins += increment;
      light_mins = light_mins % 100;
      EEPROM.write(EA_LIGHT_MINS, light_mins);
      break;
    case MODE_ALARM_MIN:
      alarm_mins += increment;
      alarm_mins = alarm_mins % 100;
      EEPROM.write(EA_ALARM_MINS, alarm_mins);
      break;
  }
}

//    FILE: dayOfWeek.ino
//  AUTHOR: Rob Tillaart
// VERSION: 2013-09-01
// PURPOSE: experimental day of week code (hardly tested)
// Released to the public domain

#define LEAP_YEAR(Y)     ( (Y>0) && !(Y%4) && ( (Y%100) || !(Y%400) ))     // from time-lib

int dayOfWeek2(uint16_t year, uint8_t month, uint8_t day)
{
  uint16_t months[] = {
    0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365
  };   // days until 1st of month

  uint32_t days = year * 365;        // days until year
  for (uint16_t i = 4; i < year; i += 4) if (LEAP_YEAR(i) ) days++;     // adjust leap years, test only multiple of 4 of course

  days += months[month - 1] + day;  // add the days of this year
  if ((month > 2) && LEAP_YEAR(year)) days++;  // adjust 1 if this year is a leap year, but only after febr

  return days % 7;   // remove all multiples of 7
}
