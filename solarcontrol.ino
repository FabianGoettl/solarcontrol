/*
  GCC Solar
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>

#include <MemoryFree.h>

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Temp. thresholds
const float maxSolarOffT = 110; // Off. max. solar temp. is reached
const float maxSolarOnT = 105; // Off->On after max. solar temp. was reached
const float maxPufferOffT = 77; // Off. max. buffer temp. is reached
const float maxPufferOnT = 75; // Off->On after max. puffer temp. is reached
const float minSolarOnT = 50; // On. min. solar temp. is reached
const float minSolarOffT = 45; // Off->On after min. solar temp. was reached
const float diffOnT = 8; // On when temp. diff. is larger
const float diffOffT = 6; // On->Off, when temp. diff. is smaller

// Resistance used in the measurement circuit
const float R_meas = 2190;

// Temp. sensor resistance at 25°C
const int R_25 = 2000;

// Temp. sensor coefficients
const double alpha = 0.00788;
const double beta = 0.00001937;
const float alpha_pow2 = 0.000062;
const float beta_mul4 = 0.000078;
const float beta_mul2 = 0.000039;

// Solar pump relay
const int pumpPin = 2;

// Sensor pins
const int bufferPin = 0;
const int solarPin = 1;

const float VCC = 5.0;

// Control state variables
boolean maxSolarOffReached = false;
boolean maxPufferOffReached = false;
boolean minOnReached = false;
boolean diffOnReached = false;

// Display reset
const int displayResetSeconds = 60;
int displayResetLast = 0;

void setup() {
  wdt_disable();
  Serial.begin(115200);

  lcd.begin(20, 4);

  // initialize the digital pins as an output.
  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  //controlTest();

  wdt_enable(WDTO_8S);
}

boolean solarControl(float tempBuffer, float tempSolar) {

  float tempDiff = tempSolar - tempBuffer;

  if (!maxSolarOffReached) {
    maxSolarOffReached = tempSolar >= maxSolarOffT;
  } else {
    maxSolarOffReached = tempSolar > maxSolarOnT;
  }

  if (!maxPufferOffReached) {
    maxPufferOffReached = tempBuffer >= maxPufferOffT;
  } else {
    maxPufferOffReached = tempBuffer > maxPufferOnT;
  }

  if (!minOnReached) {
    minOnReached = tempSolar >= minSolarOnT;
  } else {
    minOnReached = tempSolar > minSolarOffT;
  }

  if (!diffOnReached) {
    diffOnReached = tempDiff >= diffOnT;
  } else {
    diffOnReached = tempDiff > diffOffT;
  }

  Serial.print(F(" "));
  Serial.print(F("maxSolarOffReached="));
  Serial.print(maxSolarOffReached);
  Serial.print(F(" "));
  Serial.print(F("maxPufferOffReached="));
  Serial.print(maxPufferOffReached);
  Serial.print(F(" "));
  Serial.print(F("minOnReached="));
  Serial.print(minOnReached);
  Serial.print(F(" "));
  Serial.print(F("diffOnReached="));
  Serial.print(diffOnReached);
  Serial.print(F(" "));

  return !maxSolarOffReached && !maxPufferOffReached && minOnReached && diffOnReached;
}

void loop() {

  Serial.print(F("*START*"));

  Serial.print(F("*TEMPS*"));
  float tempBuffer = readTemp(bufferPin);
  float tempSolar = readTemp(solarPin);

  bool pumpOn = false;
  Serial.print(F("*CONTROL*"));
  pumpOn = solarControl(tempBuffer, tempSolar);

  Serial.print(F("*SETPUMP*"));
  setPump(pumpOn);

  Serial.print(F("*DISPLAY*"));
  displayStatus(tempBuffer, tempSolar, pumpOn);

  Serial.print(F("*STATUS*"));
  Serial.print(F(" tempBuffer="));
  Serial.print(tempBuffer);
  Serial.print(F(" tempSolar="));
  Serial.print(tempSolar);
  Serial.print(F(" pumpOn="));
  Serial.print(pumpOn);

  Serial.print(F("freeMemory()="));
  Serial.println(freeMemory());

  Serial.print(F("*END*"));
  Serial.println();

  wdt_delay(1000);
}


void displayStatus(float tempBuffer, float tempSolar, bool pumpOn) {
  
  // Display reset after x seconds
  if(displayResetLast < displayResetSeconds){
    displayResetLast++;
  } else {
    lcd.clear();
    displayResetLast = 0;
  }
  
  lcd.setCursor ( 0, 0 );
  lcd.print (F("GCC Pumpe     "));
  lcd.print (pumpOn ? F("an ") : F("aus"));

  lcd.setCursor ( 4, 1 );
  lcd.print(F("Solar    "));

  if (tempSolar < 10 && tempSolar >= 0) {
    lcd.print (F("  "));
  } else if (tempSolar < 100 && tempSolar >= 10) {
    lcd.print (F(" "));
  }

  lcd.print(tempSolar, 1);
  lcd.write(byte(0xDF));
  lcd.print (F("C"));

  lcd.setCursor ( 4, 2 );
  lcd.print (F("Puffer    "));
  lcd.print (tempBuffer, 1);
  lcd.write(byte(0xDF));
  lcd.print (F("C"));

  lcd.setCursor ( 4, 3 );
  lcd.print (F("Delta     "));

  float tempDiff = tempSolar - tempBuffer;

  if (tempDiff < 10 && tempDiff >= 0) {
    lcd.print (F(" "));
  }

  lcd.print (tempDiff, 1);
  lcd.write(byte(0xDF));
  lcd.print (F("C"));
}

/*
  Read KTY10 sensor (2kOhm at 25°C) by second order polynomial based on
  http://pdf.datasheetcatalog.com/datasheet/infineon/1-kt.pdf
*/
float readTemp(unsigned int port) {
  unsigned int adc = 0;

  for (int i = 0; i < 4; i++)
  {
      adc += analogRead(port);
  }

  Serial.print(F(" adc:"));
  Serial.print(adc);

  //float adcAvg = adc / 4.0; // get loop avg
  float adcAvg = adc >> 2;

  float U_sens = adcAvg * VCC / 1023.0;

  Serial.print(F(" U_sens:"));
  Serial.print(U_sens);

  float I_sens = (VCC - U_sens) / R_meas;

  Serial.print(F("  I_sens:"));
  Serial.print(I_sens, 4);

  float R_sens = U_sens / I_sens;

  Serial.print(F("  R_sens:"));
  Serial.println(R_sens, 4);


  return calcTempPoly(R_sens);
}

float calcTempPoly(float R_T) {

  float k_T = R_T / R_25;

  Serial.print(F("  k_T:"));
  Serial.print(k_T, 4);

  //double denominator = sqrt(pow(alpha, 2) - 4 * beta + 4 * beta * k_T) - alpha;

  // With optimization
  double denominator = sqrt(alpha_pow2 + beta_mul4 * (k_T - 1)) - alpha;

  Serial.print(F(" denominator:"));
  Serial.print(denominator);

  float T = (25.0 + (denominator / beta_mul2));

  Serial.print(F("  T:"));
  Serial.println(T, 4);

  return T;
}

void setPump(bool on) {
  digitalWrite(pumpPin, on ? HIGH : LOW);
  //digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

void wdt_delay(unsigned long msec) {
  wdt_reset();

  while (msec > 1000) {
    wdt_reset();
    delay(1000);
    msec -= 1000;
  }
  delay(msec);
  wdt_reset();
}


/*struct ControlTestType {
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
  }*/
