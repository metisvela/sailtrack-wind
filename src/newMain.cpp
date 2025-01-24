/*
wind compass object()
{
	for better calculating some datas using wind direction
	0 degree will be where the wind is coming from
}

true wind calculator() to implement on the core
{
	vectorial differcane of the boat and wind
}

wind speed function()
{
	check how fast the turbine turns (done)
	can check the avr speed (not used / to be decided)
	have to check if the boat is moving or not
}

wind direction function()
{
	check the direction of the pointer (done)
	can check the avr direction (not used / to be decided)
	have to check if the boat is moving or not
}

VMG function() to implement on the core
{
	Still not super sure on how this would work(are vmg and the tüyler the same thing)
	have to check the max degree the boat can sail
	vmg = true wind speed * cos(boat direction to the wind)
}

gust checker() (not used / to be decided)
{
	if there is a big differance of wind speed and direction from the normal ones
	to check if there is a gust coming or not
}

lay-line calculator() to implement on the core
{
	it is a line begining from the mark. is to show the earliest possible tacking line to go straight to the mark
	Have to get the coordinates of the mark and the boat
}

when to tack() to implement on the core
{
	combination of VMG and lay-line. Shows when you should tack
}

*/

#include <Arduino.h>
#include <Wire.h>

// AS5600 I2C address
#define AS5600_ADDR 0x36

// Registers for angle data
#define AS5600_RAW_ANGLE_HIGH 0x0C
#define AS5600_RAW_ANGLE_LOW 0x0D

void setup() {
  Wire.begin();  // Initialize I2C communication
  Serial.begin(115200); // Initialize serial communication
  while (!Serial);
  Serial.println("AS5600 Magnetic Encoder Test");
}

uint16_t readRawAngle() { //tested working
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_HIGH); // Request the high byte of the raw angle
  Wire.endTransmission(false);       // Restart I2C (do not release the bus)

  Wire.requestFrom(AS5600_ADDR, 2);  // Request 2 bytes (high and low)

  if (Wire.available() == 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    return (highByte << 8) | lowByte; // Combine high and low bytes
  } else {
    Serial.println("Error: No data received from AS5600");
    return 0; // Return 0 if no data is received
  }


}

long windDir() //tested working
{
    uint16_t rawAngle = readRawAngle();
    long windDirN = map(rawAngle,0,4095,0,360);
    return windDirN;
}

int counterAvWindDir;
long AvWindDir() //Not tested but working
{
    long totalDir;
    long AvWindDirN;
    if(counterAvWindDir < 10)
    {
        totalDir = totalDir+windDir();
        counterAvWindDir++;
        return 31;
    }
    else{
        AvWindDirN = totalDir/counterAvWindDir; 
        counterAvWindDir = 0;
        return 69;
    }
    return 0;

}

unsigned long lastTime = 0; //global varibles for rpm calculation
float lastAngle = 0;

float calculateRPM(float currentAngle) { //not tested
    unsigned long currentTime = millis();

    float timeDiff = (currentTime - lastTime) / 1000.0; // Time difference in seconds
    float angleDiff = currentAngle - lastAngle;

    if (angleDiff < 0) {
        angleDiff += 360.0; 
    }

    float rpm = (angleDiff / 360.0) / timeDiff * 60.0; // Convert to RPM

    lastAngle = currentAngle;
    lastTime = currentTime;

    return rpm;
}

void loop() {
  uint16_t rawAngle = readRawAngle();
  float rpm = calculateRPM(windDir());

  Serial.print("Raw Angle: ");
  Serial.println(rawAngle);
  Serial.print("Angle: ");
  Serial.println(windDir());
  Serial.print("AV Angle: ");
  Serial.println(AvWindDir());
  Serial.print("rpm: ");
  Serial.println(rpm);


  delay(100); // Wait for 100ms
}
