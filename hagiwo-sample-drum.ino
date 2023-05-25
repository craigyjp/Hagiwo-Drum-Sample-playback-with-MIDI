#define ENCODER_OPTIMIZE_INTERRUPTS  //rotary encoder
#include "sample.h"                  //sample file
#include <Encoder.h>                 //rotary encoder
#include <EEPROM.h>
#include <MIDI.h>

#define TRIGGER D2
#define FILTER_SW D4
#define ENCB D10
#define ENCA D1
#define AUDIO D5
#define MIDI_CHANNEL 1

Encoder myEnc(ENCA, ENCB);  //rotary encoder

float i;  //sample play progress
float freq = 1;
float midifreq = 0;  //sample frequency
bool trig1, old_trig1, done_trig1;
int sound_out;       //sound out PWM rate
byte sample_no = 1;  //select sample number
static long encPrevious = 0;
boolean encCW = true;
long timer = 0;         //timer count for eeprom write
bool eeprom_write = 0;  //0=no write,1=write

//-------------------------timer interrupt for sound----------------------------------
hw_timer_t *timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t ledstat = 0;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux0);  // enter critical range
  if (done_trig1 == 1) {               //when trigger in
    i = i + freq;
    if (i >= 28800) {  //when sample playd all ,28800 = 48KHz sampling * 0.6sec
      i = 0;
      done_trig1 = 0;
    }
  }
  sound_out = (((pgm_read_byte(&(smpl[sample_no][(int)i * 2]))) | (pgm_read_byte(&(smpl[sample_no][(int)i * 2 + 1]))) << 8) >> 6);  //16bit to 10bit
  ledcWrite(1, sound_out + 511);                                                                                                    //PWM output
  portEXIT_CRITICAL_ISR(&timerMux0);                                                                                                // exit critical range
}

MIDI_CREATE_INSTANCE(HardwareSerial, Serial0, MIDI);


void setup() {
  Serial.begin(9600);
  EEPROM.begin(1);           //1byte memory space
  EEPROM.get(0, sample_no);  //callback saved sample number
  sample_no++;               //countermeasure rotary encoder error
  if (sample_no >= 48) {     //countermeasure rotary encoder error
    sample_no = 0;
  }

  pinMode(TRIGGER, INPUT);      //trigger in
  pinMode(ENCA, INPUT_PULLUP);  //rotary encoder
  pinMode(ENCB, INPUT_PULLUP);  //rotary encoder
  pinMode(AUDIO, OUTPUT);       //sound_out PWM
  pinMode(FILTER_SW, OUTPUT);   //turns off the filter
  digitalWrite(FILTER_SW, LOW);
  timer = millis();  //for eeprom write
  analogReadResolution(10);

  ledcSetup(1, 39000, 10);  //PWM frequency and resolution
  ledcAttachPin(AUDIO, 1);  //(LED_PIN, LEDC_CHANNEL_0);//timer ch1 , apply D5 output

  timer0 = timerBegin(0, 1666, true);            // timer0, 12.5ns*1666 = 20.83usec(48kHz), count-up
  timerAttachInterrupt(timer0, &onTimer, true);  // edge-triggered
  timerAlarmWrite(timer0, 1, true);              // 1*20.83usec = 20.83usec, auto-reload
  timerAlarmEnable(timer0);                      // enable timer0

  MIDI.begin(MIDI_CHANNEL);
  delay(300);
}

void eeprom_update() {
  EEPROM.put(0, sample_no);
  EEPROM.commit();
}

void loop() {

  // ------------------------ MIDI-----------------------------------
  //MIDI.read(MIDI_CHANNEL);
  int type, noteMsg, velocity, channel, d1, d2;
  if (MIDI.read(MIDI_CHANNEL)) {
    byte type = MIDI.getType();
    switch (type) {

      case midi::NoteOn:
        d1 = MIDI.getData1();
        d2 = MIDI.getData2();
        switch (d1) {
          case 36:  // 36 for bottom C Bass
            if (d2 != 0) {
              done_trig1 = 1;
              i = 0;
            }
            break;
        }
        break;

      case midi::ControlChange:
        d1 = MIDI.getData1();
        d2 = MIDI.getData2();
        switch (d1) {
          case 9:
            midifreq = d2 * 0.016;
            freq = midifreq;
            break;
          case 10:
            if (d2 > 63) {
              digitalWrite(FILTER_SW, HIGH);
            } else {
              digitalWrite(FILTER_SW, LOW);
            }
            break;
        }
        break;

      case midi::ProgramChange:
        d1 = MIDI.getData1();
        if (d1 >= 0 || d1 <= 47) {
          sample_no = d1;
        }
        done_trig1 = 1;  //1 shot play when sample changed
        i = 0;
        timer = millis();
        eeprom_write = 1;  //eeprom update flug on
        break;
    }
  }

  // From 0 to 127
  //-------------------------trigger----------------------------------
  old_trig1 = trig1;
  trig1 = digitalRead(TRIGGER);
  if (trig1 == 1 && old_trig1 == 0) {  //detect trigger signal low to high , before sample play was done
    done_trig1 = 1;
    i = 0;
  }

  //-------------------------pitch setting----------------------------------
  //freq = analogRead(A3) * 0.002 + midifreq;

  //-------------------------sample change----------------------------------

  long encRead = myEnc.read();
  if ((encCW && encRead > encPrevious + 3) || (!encCW && encRead < encPrevious - 3)) {
    encPrevious = encRead;
    sample_no = sample_no + 1;
    if (sample_no >= 48) {
      sample_no = 0;
    }
    done_trig1 = 1;  //1 shot play when sample changed
    i = 0;
    timer = millis();
    eeprom_write = 1;  //eeprom update flug on

  } else if ((encCW && encRead < encPrevious - 3) || (!encCW && encRead > encPrevious + 3)) {
    encPrevious = encRead;
    sample_no = sample_no - 1;
    if (sample_no < 0 || sample_no > 200) {  //>200 is overflow countermeasure
      sample_no = 47;
    }
    done_trig1 = 1;  //1 shot play when sample changed
    i = 0;
    timer = millis();
    eeprom_write = 1;  //eeprom update flug on
  }

  //-------------------------save to eeprom----------------------------------
  if (timer + 5000 <= millis() && eeprom_write == 1) {  //Memorized 5 seconds after sample number change
    eeprom_write = 0;
    eeprom_update();
  }
}
