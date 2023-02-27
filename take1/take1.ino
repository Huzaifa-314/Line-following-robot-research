#include <QTRSensors.h>

// This example is designed for use with six analog QTR sensors. These
// reflectance sensors should be connected to analog pins A0 to A5. The
// sensors' emitter control pin (CTRL or LEDON) can optionally be connected to
// digital pin 2, or you can leave it disconnected and remove the call to
// setEmitterPin().
//
// The main loop of the example reads the raw sensor values (uncalibrated). You
// can test this by taping a piece of 3/4" black electrical tape to a piece of
// white paper and sliding the sensor across it. It prints the sensor values to
// the serial monitor as numbers from 0 (maximum reflectance) to 1023 (minimum
// reflectance).

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);

  Serial.begin(74880);
}


void loop()
{
  int position;
  int setpoint = 10738;
  int sum=0;
  // read raw sensor values
  qtr.read(sensorValues);
  for (uint8_t i = 0,j = 1; i < SensorCount;j++, i++)
  {
    sum+=sensorValues[i]*j;
  }
  position = setpoint-sum;
  
//  for (uint8_t i = 0; i < SensorCount; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print('\t');
//  }
  Serial.println(position);
}
