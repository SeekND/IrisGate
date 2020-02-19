
/**
   DIY RF Laptimer by Andrey Voroshkov (bshep)
   SPI driver based on fs_skyrf_58g-main.c by Simon Chambers
   fast ADC reading code is by "jmknapp" from Arduino forum
   fast port I/O code from http://masteringarduino.blogspot.com.by/2013/10/fastest-and-smallest-digitalread-and.html
   IRIS code by Carlos Costa (SeekNDFPV @ youtube)

  The MIT License (MIT)

  Copyright (c) 2016 by Andrey Voroshkov (bshep)

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
*/

// --------------------------------------- USER SETUP -----------------------------------------

uint16_t rssiThreshold = 254 ; // (CC) play around with this value to find your best RSSI threshold. (between 190 to 260 seem to be acceptable values). The higher the value the closer the quad needs to get. But if it's too high it wont detect it.
int servomin = 1000; // (CC) if you want the servo operation reversed just set the values inverted eg. min=2000 max=1000
int servomax = 2000; // (CC) adjust both min/max if the servo is twitching when operating. 1000 and 2000 are normally maximum servo values.

// --------------------------------------- END OF USER SETUP -----------------------------------------


#include <SPI.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <Servo.h>

Servo myservo;

// ---------- PIN DEFINITION

#define servosignal 5 // servo connection
#define spiDataPin 10
#define slaveSelectPin 11
#define spiClockPin 12
#define rssiPinA 3  //analog


//----- RSSI --------------------------------------
#define RSSI_READS 5 // 5 should give about 10 000 readings per second
#define FILTER_ITERATIONS 5
uint16_t rssiArr[FILTER_ITERATIONS + 1];
uint16_t rssi;


#define RSSI_MAX 1024
#define RSSI_MIN 0
#define MAGIC_THRESHOLD_REDUCE_CONSTANT 2
#define THRESHOLD_ARRAY_SIZE  100
uint16_t rssiThresholdArray[THRESHOLD_ARRAY_SIZE];


#define DEFAULT_RSSI_MONITOR_DELAY_CYCLES 1000 //each 100ms, if cycle takes 100us
#define MIN_RSSI_MONITOR_DELAY_CYCLES 10 //each 1ms, if cycle takes 100us, to prevent loss of communication
uint16_t rssiMonitorDelayCycles = DEFAULT_RSSI_MONITOR_DELAY_CYCLES;


//----- other globals------------------------------
uint32_t now = 0;
uint32_t lastMillis = 0;
uint8_t allowEdgeGeneration = 0;
uint8_t channelIndex = 0;     // (CC) This starts the module in channel 1 ( yes, 0 means channel 1, 1 means channel 2 ... 7 means channel 8 .. you get the picture ;)  )
uint8_t bandIndex = 0;        // (CC) This starts the module in Raceband. Check Channels.h file if you want to use another band instead.
uint8_t isSoundEnabled = 1;
uint8_t rssiMonitor = 1;
uint16_t rssiMonitorDelayExpiration = 0;
uint16_t frequency = 0;




// ----------------------------------------------------------------------------




void setup() {

  Serial.begin(9600);

  setupSPIpins();
  frequency = setModuleChannel(channelIndex, bandIndex);
  initFastADC();
  allowEdgeGeneration = 0;

  myservo.attach(servosignal);
  myservo.writeMicroseconds(servomin); // resets the servo to 0

  Serial.println("Finished setup");

}
// ----------------------------------------------------------------------------
void loop() {

  freqAnal();
  if (rssi >= rssiThreshold )
  {
    myservo.writeMicroseconds(servomax);
    delay(5000);
    myservo.writeMicroseconds(servomin);
    delay(1000);
  }
}



// ------------------ FAST READ/WRITE ------------------


#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define digitalToggle(P) *(portOfPin(P))^=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))



// -----------------------------CHANNELS--------------------------


#define MAX_BAND 7

// Channels' MHz Values. Just for reference. Not used in code.
//
const uint16_t channelFreqTable[] PROGMEM = {
  //     // Channel 1 - 8
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917, // Raceband
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // Band F / Airwave
  5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621, // Band D / 5.3
  5180, 5200, 5220, 5240, 5745, 5765, 5785, 5805, // connex
  5825, 5845, 5845, 5845, 5845, 5845, 5845, 5845  // even more connex, last 6 unused!!!
};


const uint16_t channelFreqTableseq[] PROGMEM = {
  //     // Channel 1 - 8
  /*5645, 5658, 5665, 5685, 5695, 5705 ,5725, 5732,
    5733, 5740, 5745, 5752, 5760, 5765, 5769, 5771,
    5780, 5785, 5790, 5800, 5805, 5806, 5809, 5820,
    5825, 5828, 5840, 5843, 5845, 5847, 5860, 5865,
    5866, 5880, 5880, 5885, 5905, 5917, 5925, 5945,*/

  5605, 5610, 5615, 5620, 5625, 5630, 5635, 5640,
  5645, 5650, 5665, 5670, 5675, 5680, 5685, 5690,
  5695, 5700, 5705, 5710, 5715, 5720, 5725, 5730,
  5735, 5740, 5745, 5750, 5765, 5770, 5775, 5780,
  5785, 5790, 5795, 5800, 5805, 5810, 5815, 5820,
  5825, 5830, 5835, 5840, 5845, 5850, 5865, 5870,
  5875, 5880, 5885, 5890, 5895, 5900, 5905, 5910,
  5915, 5920, 5925, 5930, 5935, 5940, 5945, 5950,

};



//----- ADC IMPROVEMENTS --------------------------------
// to increase frequency of ADC readings
// defines for setting and clearing register bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void initFastADC() {
  // set ADC prescaler to 16 to speedup ADC readings
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
}




// ---------------------- PINS ------------------------------

// Pin definitions



// ------------------ RX5808SPI -------------------------------

// rx5808 module needs >30ms to tune.
#define MIN_TUNE_TIME 30

void setupSPIpins() {
  // SPI pins for RX control
  pinMode (slaveSelectPin, OUTPUT);
  pinMode (spiDataPin, OUTPUT);
  pinMode (spiClockPin, OUTPUT);
}

void SERIAL_SENDBIT1() {
  digitalLow(spiClockPin);
  delayMicroseconds(1);

  digitalHigh(spiDataPin);
  delayMicroseconds(1);
  digitalHigh(spiClockPin);
  delayMicroseconds(1);

  digitalLow(spiClockPin);
  delayMicroseconds(1);
}

void SERIAL_SENDBIT0() {
  digitalLow(spiClockPin);
  delayMicroseconds(1);

  digitalLow(spiDataPin);
  delayMicroseconds(1);
  digitalHigh(spiClockPin);
  delayMicroseconds(1);

  digitalLow(spiClockPin);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_LOW() {
  delayMicroseconds(1);
  digitalLow(slaveSelectPin);
  delayMicroseconds(1);
}

void SERIAL_ENABLE_HIGH() {
  delayMicroseconds(1);
  digitalHigh(slaveSelectPin);
  delayMicroseconds(1);
}

uint16_t setModuleFrequency(uint16_t frequency) {
  uint8_t i;
  uint16_t channelData;

  channelData = frequency - 479;
  channelData /= 2;
  i = channelData % 32;
  channelData /= 32;
  channelData = (channelData << 7) + i;

  // bit bang out 25 bits of data
  // Order: A0-3, !R/W, D0-D19
  // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
  cli();
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);
  SERIAL_ENABLE_LOW();

  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT1();

  SERIAL_SENDBIT0();

  // remaining zeros
  for (i = 20; i > 0; i--) {
    SERIAL_SENDBIT0();
  }

  // Clock the data in
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);
  SERIAL_ENABLE_LOW();

  // Second is the channel data from the lookup table
  // 20 bytes of register data are sent, but the MSB 4 bits are zeros
  // register address = 0x1, write, data0-15=channelData data15-19=0x0
  SERIAL_ENABLE_HIGH();
  SERIAL_ENABLE_LOW();

  // Register 0x1
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();

  // Write to register
  SERIAL_SENDBIT1();

  // D0-D15
  //   note: loop runs backwards as more efficent on AVR
  for (i = 16; i > 0; i--) {
    // Is bit high or low?
    if (channelData & 0x1) {
      SERIAL_SENDBIT1();
    }
    else {
      SERIAL_SENDBIT0();
    }
    // Shift bits along to check the next one
    channelData >>= 1;
  }

  // Remaining D16-D19
  for (i = 4; i > 0; i--) {
    SERIAL_SENDBIT0();
  }

  // Finished clocking data in
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);

  digitalLow(slaveSelectPin);
  digitalLow(spiClockPin);
  digitalLow(spiDataPin);
  sei();

  delay(MIN_TUNE_TIME);

  return frequency;
}


uint16_t setModulefrequency(uint16_t frequency) {
  uint8_t iseq;
  uint16_t channelDataseq;

  channelDataseq = frequency - 479;
  channelDataseq /= 2;
  iseq = channelDataseq % 32;
  channelDataseq /= 32;
  channelDataseq = (channelDataseq << 7) + iseq;

  // bit bang out 25 bits of data
  // Order: A0-3, !R/W, D0-D19
  // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
  cli();
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);
  SERIAL_ENABLE_LOW();

  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT1();

  SERIAL_SENDBIT0();

  // remaining zeros
  for (iseq = 20; iseq > 0; iseq--) {
    SERIAL_SENDBIT0();
  }

  // Clock the data in
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);
  SERIAL_ENABLE_LOW();

  // Second is the channel data from the lookup table
  // 20 bytes of register data are sent, but the MSB 4 bits are zeros
  // register address = 0x1, write, data0-15=channelData data15-19=0x0
  SERIAL_ENABLE_HIGH();
  SERIAL_ENABLE_LOW();

  // Register 0x1
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();

  // Write to register
  SERIAL_SENDBIT1();

  // D0-D15
  //   note: loop runs backwards as more efficent on AVR
  for (iseq = 16; iseq > 0; iseq--) {
    // Is bit high or low?
    if (channelDataseq & 0x1) {
      SERIAL_SENDBIT1();
    }
    else {
      SERIAL_SENDBIT0();
    }
    // Shift bits along to check the next one
    channelDataseq >>= 1;
  }

  // Remaining D16-D19
  for (iseq = 4; iseq > 0; iseq--) {
    SERIAL_SENDBIT0();
  }

  // Finished clocking data in
  SERIAL_ENABLE_HIGH();
  delayMicroseconds(1);

  digitalLow(slaveSelectPin);
  digitalLow(spiClockPin);
  digitalLow(spiDataPin);
  sei();

  delay(MIN_TUNE_TIME);

  return frequency;
}



uint16_t setModuleChannel(uint8_t channel, uint8_t band) {

  uint16_t frequency = pgm_read_word_near(channelFreqTable + channel + (8 * band));
  return setModuleFrequency(frequency);
}

uint16_t setModuleChannelseq(uint8_t channel, uint8_t band) {

  uint16_t frequency = pgm_read_word_near(channelFreqTableseq + channel + (8 * band));
  return setModuleFrequency(frequency);
}


uint16_t readRSSI() {
  int rssiA = 0;

  for (uint8_t i = 0; i < RSSI_READS; i++) {
    rssiA += analogRead(rssiPinA);
  }

  rssiA = rssiA / RSSI_READS; // average of RSSI_READS readings
  return rssiA;
}

uint16_t getFilteredRSSI() {
  rssiArr[0] = readRSSI();

  // several-pass filter (need several passes because of integer artithmetics)
  // it reduces possible max value by 1 with each iteration.
  // e.g. if max rssi is 300, then after 5 filter stages it won't be greater than 295
  for (uint8_t i = 1; i <= FILTER_ITERATIONS; i++) {
    rssiArr[i] = (rssiArr[i - 1] + rssiArr[i]) >> 1;
  }

  return rssiArr[FILTER_ITERATIONS];
}


// ------------------- RSSI MONITOR AND RISING EDGE ------------------------------

uint16_t setRssiMonitorDelay(uint16_t delay) {
  return delay < MIN_RSSI_MONITOR_DELAY_CYCLES ? MIN_RSSI_MONITOR_DELAY_CYCLES : delay;
}

uint16_t getMedian(uint16_t a[], uint16_t size) {
  return a[size / 2];
}

void gen_rising_edge(int pin) {
  digitalHigh(pin); //this will open mosfet and pull the RasPi pin to GND
  delayMicroseconds(10);
  digitalLow(pin); // this will close mosfet and pull the RasPi pin to 3v3 -> Rising Edge
}

// ------------------FREQ LOOP -----------------------------


void freqAnal()
{
  for (int checkingRSSI = 1; checkingRSSI < 100 ; checkingRSSI = checkingRSSI + 1)
  {
    rssi = getFilteredRSSI();
  }

  if ( channelIndex == 7 )
  {
    if (bandIndex < 7)
    {
      bandIndex++;
    }
    else
    {
      bandIndex = 0;
    }
  }


  if (channelIndex < 7)
  {
    channelIndex++;
  }
  else
  {
    channelIndex = 0;
  }

  frequency = setModuleChannelseq(channelIndex, bandIndex);

}
