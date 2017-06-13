/*
  GCC Solar
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Temp. thresholds
float maxOffT = 80; // Aus. Max Buffer temp. erreicht.
float maxOnT = 75; // Wieder ein nach max. Buffer temp. erreicht
float minOnT = 65; // Ein.. Min. solar temp.
float minOffT = 60; // Wieder aus nach min. Solar temp. erreicht
float diffOnT = 8; // Ein wenn Temp. Diff größer
float diffOffT = 6; // Wieder aus, wenn Temp. Diff. kleinergleich

boolean maxOffReached = false;
boolean minOnReached = false;
boolean diffOnReached = false;

// Resistance used in the measurement circuit
float R_meas = 2190;

// Temp. sensor resistance at 25°C
int R_25 = 2000;

// Solar pump relay
int pumpPin = 2;

// Sensor pins
int bufferPin = 0;
int solarPin = 1;

boolean solarControl(float tempBuffer, float tempSolar) {

  float tempDiff = tempSolar - tempBuffer;

  if (!maxOffReached) {
    maxOffReached = tempBuffer >= maxOffT;
  } else {
    maxOffReached = tempBuffer > maxOnT;
  }

  if (!minOnReached) {
    minOnReached = tempSolar >= minOnT;
  } else {
    minOnReached = tempSolar > minOffT;
  }

  if (!diffOnReached) {
    diffOnReached = tempDiff >= diffOnT;
  } else {
    diffOnReached = tempDiff > diffOffT;
  }

  /*Serial.print(" ");
    Serial.print("maxOffReached=");
    Serial.print(maxOffReached);
    Serial.print(" ");
    Serial.print("minOnReached=");
    Serial.print(minOnReached);
    Serial.print(" ");
    Serial.print("diffOnReached=");
    Serial.print(diffOnReached);
    Serial.print(" ");*/

  return !maxOffReached && minOnReached && diffOnReached;
}

void setup() {
  Serial.begin(115200);

  lcd.begin(20, 4);

  // initialize the digital pin as an output.
  pinMode(pumpPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(pumpPin, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  //controlTest();
}

void loop() {

  float tempBuffer = readTempPoly(bufferPin);
  float tempSolar = readTempPoly(solarPin);

  bool pumpOn = false;
  pumpOn = solarControl(tempBuffer, tempSolar);

  setPump(pumpOn);

  displayStatus(tempBuffer, tempSolar, pumpOn);

  Serial.print(tempBuffer);
  Serial.print(";");
  Serial.print(tempSolar);
  Serial.print(";");
  Serial.print(pumpOn);
  Serial.println();

  delay(1000);
}


void displayStatus(float tempBuffer, float tempSolar, bool pumpOn) {
  //lcd.clear();
  lcd.setCursor ( 0, 0 );
  lcd.print ("GCC Pumpe     ");
  lcd.print (pumpOn ? "an " : "aus");

  lcd.setCursor ( 0, 1 );
  lcd.print("    Solar    ");

  if (tempSolar < 10 && tempSolar >= 0) {
    lcd.print ("  ");
  } else if (tempSolar < 100 && tempSolar >= 10) {
    lcd.print (" ");
  }

  lcd.print(tempSolar, 1);
  lcd.write(byte(0xDF));
  lcd.print ("C");
  //lcd.print ("     "); // Clear remaining chars

  lcd.setCursor ( 0, 2 );
  lcd.print ("    Puffer    ");
  lcd.print (tempBuffer, 1);
  lcd.write(byte(0xDF));
  lcd.print ("C");
  //lcd.print ("     "); // Clear remaining chars

  lcd.setCursor ( 0, 3 );
  lcd.print ("    Delta     ");

  float tempDiff = tempSolar - tempBuffer;

  if (tempDiff < 10 && tempDiff >= 0) {
    lcd.print (" ");
  }

  lcd.print (tempDiff, 1);
  lcd.write(byte(0xDF));
  lcd.print ("C");
  //lcd.print ("     "); // Clear remaining chars
}

/*
  Read KTY10 sensor (2kOhm bei 25C) SECOND ORDER POLYNOMIAL based on
  http://pdf.datasheetcatalog.com/datasheet/infineon/1-kt.pdf
*/
float readTempPoly(unsigned int port) {
  float adc = 0;
  ADCSRA = 0x00;
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADMUX = 0x00;
  ADMUX = (1 << REFS0);
  ADMUX |= port;

  for (int i = 0; i <= 63; i++)
  {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    adc += (ADCL + ADCH * 256);
  }

  adc /= 64; // get loop avg

  float vcc = readVcc();
  float U_sens = adc * vcc / 1023;

  float I_sens = (vcc - U_sens) / R_meas;
  float R_sens = U_sens / I_sens;

  return calcTempPoly(R_sens);
}

float calcTempPoly(float R_T) {
  double alpha = 0.00788;
  double beta = 0.00001937;

  double k_T = R_T / R_25;

  double denominator = sqrt(pow(alpha, 2) - 4 * beta + 4 * beta * k_T) - alpha;
  double T = (25 + (denominator / (2 * beta)));

  /*Serial.print(R_T);
    Serial.print("  ");
    Serial.print(k_T, 4);
    Serial.print("  ");
    Serial.println(T, 4);*/

  return T;
}

/*
  Read KTY10 sensor (2kOhm bei 25C) LINEAR WAY based on
  http://playground.arduino.cc/Deutsch/KtyTemperatureExtDe
  http://www.sprut.de/electronic/pic/projekte/thermo/thermo.htm#kal
*/
float readTemp(unsigned int port) {
  float temp = 0;
  ADCSRA = 0x00;
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADMUX = 0x00;
  ADMUX = (1 << REFS0);
  ADMUX |= port;

  for (int i = 0; i <= 63; i++)
  {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    temp += (ADCL + ADCH * 256);
  }

  // Consts, see the sheet
  temp /= 64; // get loop avg
  temp /= 1.74; // ADC / K
  temp -= 227; // Temp. offset

  return (temp);
}


float readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  float result = (high << 8) | low;

  result = 1125.300 / result; // Calculate Vcc (in mV); 1125.300 = 1.1*1023
  return result; // Vcc in Volts
}

void setPump(bool on) {
  digitalWrite(pumpPin, on ? HIGH : LOW);
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}


struct ControlTestType {
  float tempBuffer;
  float tempSolar;
  bool shouldPumpOn;
};

bool controlTest() {
  bool failed = true;

  struct ControlTestType samples[] = {
    {25, 35, false}, // Anfangs Test
    {65, 100, true}, // Min ein
    {95, 110, false}, // Buffer max
    {50, 64, true}, // Min (noch) ein
    {50, 60, false}, // Min wieder aus
    {95, 105, false}, // Buffer max aus
    {92, 105, false},
    {90, 105, true}, // Buffer max wieder an
    {70, 72, false}, // Delta nicht erreicht
    {70, 78, true}, // Delta erreicht
    {70, 75, true},
    {70, 74, false} // Delta wieder aus
  };

  for (int i = 0; i < sizeof(samples) / sizeof(ControlTestType); i++)
  {
    Serial.print("Testing: Buffer:");
    Serial.print(samples[i].tempBuffer);
    Serial.print(" Solar:");
    Serial.print(samples[i].tempSolar);

    bool pumpOn = solarControl(samples[i].tempBuffer, samples[i].tempSolar);
    bool should = samples[i].shouldPumpOn;

    if (pumpOn != should) {
      Serial.print(" Test failed:");
      Serial.print(" Should:");
      Serial.print(should);
      Serial.print(" Is:");
      Serial.println(pumpOn);
    } else {
      Serial.println();
    }
  }

  return !failed;
}

