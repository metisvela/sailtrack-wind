//SailTrack Wind, part of SailTrack project
//rev. A 30-01-2021
//https://github.com/metis-vela-unipd
//Mètis Vela UNIPD - Stefano Pieretti
//Ultrasonic anemometer with HC-SR04 and similar sensors, based on trigger/echo
//time measurements based on clock cycle counter. Overhead ignored as calculation

//smoothin variables
const int numReadings = 20;
long readings1[numReadings];      // reading buffer
int readIndex = 0;              // index of the current reading
long total1 = 0;                  // running total
long average1 = 0;                // average
long readings2[numReadings];      // reading buffer
long total2 = 0;                  // running total
long average2 = 0;                // average

//CPU clock in MHz
int cpu_freq = 0;

//pin mapping
int sensor1Trigger = 2;
int sensor2Trigger = 3;
int sensor1Echo = 4;
int sensor2Echo = 16;
int currentEchoPin; //current pin to read from isr

//isr variable
volatile unsigned long cpuTimeRising = 0;
volatile unsigned long cpuTimeFalling = 0;
volatile unsigned long cpuTimePlaceholder = 0;


//other variables
unsigned long elapsedCpuTime = 0;
float velocity=0;


void IRAM_ATTR isrCHANGE() {
  //gets cpu timing when echo pin changes logic state

  cpuTimePlaceholder = ESP.getCycleCount(); //get cpu time before evaluating if statement

  //assign cpu time to the right variable for high and low cases
  if (digitalRead(currentEchoPin) == 1) {
    cpuTimeRising = cpuTimePlaceholder;
  }
  else {
    cpuTimeFalling = cpuTimePlaceholder;
  }
}




void setup() {

  //set input and output
  pinMode(sensor1Trigger, OUTPUT);
  pinMode(sensor2Trigger, OUTPUT);
  pinMode(sensor1Echo, INPUT);
  pinMode(sensor2Echo, INPUT);

  //enable serial communication
  Serial.begin(250000);

  //get cpu frequency
  cpu_freq = ESP.getCpuFreqMHz();

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings1[thisReading] = 0;
  }
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings2[thisReading] = 0;
  }

}

void loop() {
  //read ToF and smooth the values
  // subtract the last reading:
  total1 = total1 - readings1[readIndex];
  total2 = total2 - readings2[readIndex];
  // read from the sensor:
  readings1[readIndex] =   (ToF(sensor1Trigger, sensor2Trigger, sensor1Echo));
  readings2[readIndex] =   (ToF(sensor1Trigger, sensor2Trigger, sensor2Echo));
  // add the reading to the total:
  total1 = total1 + readings1[readIndex];
  total2 = total2 + readings2[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average1 = total1 / numReadings;
  average2 = total2 / numReadings;

  velocity=(0.15*1e12*((1.0/average1)-(1.0/average2)));

  // send it to the computer as ASCII digits
  
  Serial.println(velocity,5);




}



long ToF (int trigger1, int trigger2, int echo1) {
  //function: measure time of fligh from one sensor (label "1") to another (label "2")
  // units: nanoseconds (1e-9s)

  //enable interrupts for ToF calculation on echo1 pin and set current echo pin
  currentEchoPin = echo1;
  attachInterrupt(echo1, isrCHANGE, CHANGE);

  //trigger both sensors
  digitalWrite(trigger1, LOW); //reset trigger logic level before start
  digitalWrite(trigger2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigger1, HIGH); //send a 10uS trigger HIGH pulse
  digitalWrite(trigger2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger1, LOW);
  digitalWrite(trigger2, LOW);

  //wait while ISRs collect timing data, adjust delay to at least double of expected ToF
  delay(20);

  //disable interrupts
  detachInterrupt(echo1);

  //do some math
  elapsedCpuTime = (cpuTimeFalling - cpuTimeRising) * (1000.0 / cpu_freq); //elapsed time in ns



  return elapsedCpuTime;
}
