/*
  RGB Color Sensor Demonstration
  ReadColours.ino
  Read RGB values from Color Sensor
  Must use calibration values from Color Sensor Calibration Sketch
*/
 
// Define color sensor pins
 
#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define sensorOut 2
 
// Calibration Values
// Get these from Calibration Sketch
const uint8_t redMin = 29; // Red minimum value
const uint8_t redMax = 191; // Red maximum value
const uint8_t greenMin = 33; // Green minimum value
const uint8_t greenMax = 250; // Green maximum value
const uint8_t blueMin = 25; // Blue minimum value
const uint8_t blueMax = 216; // Blue maximum value
 
// Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;
 
// Variables for final Color values
uint8_t redValue;
uint8_t greenValue;
uint8_t blueValue;
 
void setup() {
 
  // Set S0 - S3 as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // Set Sensor output as input
  pinMode(sensorOut, INPUT);
  
  // Set Frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
  // Setup Serial Monitor
  Serial.begin(115200);
}
 
void loop() {
  
  // Read Red value
  redPW = getRedPW();
  
  // Map to value from 0-255
  redValue = map(redPW, redMin,redMax,255,0);
  
  // Delay to stabilize sensor
  delay(200);
  
  // Read Green value
  greenPW = getGreenPW();
  
  // Map to value from 0-255
  greenValue = map(greenPW, greenMin,greenMax,255,0);
  
  // Delay to stabilize sensor
  delay(200);
  
  // Read Blue value
  bluePW = getBluePW();
  
  // Map to value from 0-255
  blueValue = map(bluePW, blueMin,blueMax,255,0);
  
  // Delay to stabilize sensor
  delay(200);

  predictColour(redValue, greenValue, blueValue);
  
}

// Send RGB Values in CSV format over Serial to PC Program running Colour Classifier
void predictColour(int redVal, int greenVal, int blueVal){
  
  String predictedColour;

  Serial.print(redVal); Serial.print(","); Serial.print(greenVal); Serial.print(","); Serial.println(blueVal);
}
 
// Function to read Red Pulse Widths
int getRedPW() {
 
  // Set sensor to read Red only
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  // Define integer to represent Pulse Width
  int PW;
  int avg = 0;
  const int avgPoints = 25;
  
  for(int i = 0; i < avgPoints; ++i){
    
    // Read the output Pulse Width
    PW = pulseIn(sensorOut, LOW);
    
    if(PW > redMax){
      PW = redMax;
    }
    if(PW < redMin){
      PW = redMin;
    }

    avg += PW;
  }
  
  return (int)(avg/avgPoints);
}
 
// Function to read Green Pulse Widths
int getGreenPW() {
 
  // Set sensor to read Green only
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);

  int PW;
  int avg = 0;
  const int avgPoints = 25;
  
  for(int i = 0; i < avgPoints; ++i){
    // Read the output Pulse Width
    PW = pulseIn(sensorOut, LOW);
    if(PW > greenMax){
      PW = greenMax;
    }
    if(PW < greenMin){
      PW = greenMin;
    }
    avg += PW;
  }
  
  return (int)(avg/avgPoints); 
}
 
// Function to read Blue Pulse Widths
int getBluePW() {
 
  // Set sensor to read Blue only
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  // Define integer to represent Pulse Width
  int PW;
  int avg = 0;
  const int avgPoints = 25;
  
  for(int i = 0; i < avgPoints; ++i){
    
    // Read the output Pulse Width
    PW = pulseIn(sensorOut, LOW);
    if(PW > blueMax){
      PW = blueMax;
    }
    if(PW < blueMin){
      PW = blueMin;
    }

    avg += PW;
  }
  
  return (int)(avg/avgPoints);
}
