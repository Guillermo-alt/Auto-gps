
#include <BlynkSimpleSerialBLE.h>
#include <SoftwareSerial.h>
#include <DFRobot_QMC5883.h>

#define MOTOR_A_EN_PIN 5//controlar velocidad
#define MOTOR_B_EN_PIN 9

#define MOTOR_A_IN_1_PIN 7//pines motores
#define MOTOR_A_IN_2_PIN 8
#define MOTOR_B_IN_1_PIN 12
#define MOTOR_B_IN_2_PIN 4

#define MOTOR_A_OFFSET 20
#define MOTOR_B_OFFSET 0

bool enabled = false;
char auth[] = "NYMkkC7RWyQJvGC3KvLZgLb6Zgrb0thR";//token
SoftwareSerial bluetoothSerial(3, 2);// tx rx
DFRobot_QMC5883 compass;

struct GeoLoc {//structura para la geolocalizacion
  float lat;
  float lon;
};

void setup() {

  setupCompass();
  pinMode(MOTOR_A_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT);
  
  Serial.begin(4800);
 /* bluetoothSerial.begin(9600);//inicualizo puerto de blue
  Blynk.begin(bluetoothSerial, auth);//inicualizo puerto de blue*/
 delay(100);

}

void loop() {
test_norte();
}
void pruebaMot(){
           analogWrite(MOTOR_A_EN_PIN, 80);
       digitalWrite (MOTOR_A_IN_1_PIN, HIGH);
  digitalWrite (MOTOR_A_IN_2_PIN, LOW); 
  delay(4000);
  // Motor no gira
  digitalWrite (MOTOR_A_IN_1_PIN, LOW); 
  delay(500);
  // Motor gira en sentido inverso
  digitalWrite (MOTOR_A_IN_2_PIN, HIGH);
  delay(4000);
  // Motor no gira
  digitalWrite (MOTOR_A_IN_2_PIN, LOW); 
  delay(5000);
  }

void setSpeed(int speed)
{
  setSpeedMotorA(speed);
  setSpeedMotorB(speed);
}
void setSpeedMotorA(int speed) {
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
   analogWrite(MOTOR_A_EN_PIN, speed + MOTOR_A_OFFSET );
}

void setSpeedMotorB(int speed) {
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
 analogWrite(MOTOR_B_EN_PIN, speed + MOTOR_B_OFFSET);
}

void stop() {
  // now turn off motors
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);  
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}


float geoHeading() {
  
 Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.23 + (26.0 / 60.0)) / (180 / PI);
  heading += declinationAngle;
  heading -= COMPASS_OFFSET;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0){
    heading += 2 * PI;
  }

  if (heading > 2 * PI){
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 
  while (headingDegrees < -180) headingDegrees += 360;
  while (headingDegrees >  180) headingDegrees -= 360;
  
  return headingDegrees;
}

void setupCompass(){
  while (!compass.begin())
  {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }

    if(compass.isHMC()){
        Serial.println("Initialize HMC5883");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
    }
   else if(compass.isQMC()){
        Serial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
   }
}
void test_norte(){
  Serial.println(geoHeading());
  int ang =geoHeading();
  while(!(ang < 5 && ang > -5)){
    ang =geoHeading();
      Serial.println(ang);
      pruebaMot();
    }

  stop();
  }
  void giro(){
  setSpeedMotorA(60);
    }

void testDriveNorth() {
  float heading = geoHeading();
  int testDist = 10;
  delay(100);
    Serial.println("angulo = ");
  Serial.println(heading);
  
  while(!(heading < 5 && heading > -5)) {
    drive(testDist, heading);
    heading = geoHeading();
     Serial.println("angulo = ");
    Serial.println(heading);
    delay(500);
  }
  stop();
}
void drive(int distance, float turn) {
  int fullSpeed = 230;
  int stopSpeed = 0;

  // drive to location
  int s = fullSpeed;
  if ( distance < 8 ) {
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 8.0f;
    s = stopSpeed + wouldBeSpeed;
  }
  
  int autoThrottle = constrain(s, stopSpeed, fullSpeed);
  autoThrottle = 230;

  float t = turn;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;
  
  Serial.print("turn: ");
  Serial.println(t);
  Serial.print("original: ");
  Serial.println(turn);
  
  float t_modifier = (180.0 - abs(t)) / 180.0;
  float autoSteerA = 1;
  float autoSteerB = 1;

  if (t < 0) {
    autoSteerB = t_modifier;
  } else if (t > 0){
    autoSteerA = t_modifier;
  }

  Serial.print("steerA: "); Serial.println(autoSteerA);
  Serial.print("steerB: "); Serial.println(autoSteerB);

  int speedA = (int) (((float) autoThrottle) * autoSteerA);
  int speedB = (int) (((float) autoThrottle) * autoSteerB);

    Serial.print("velocidadA: "); Serial.println(speedA);
  Serial.print("velocidadB: "); Serial.println(speedB);
  
  setSpeedMotorA(speedA);
  setSpeedMotorB(speedB);
   
}

//enciende motores /// ahora me comunico con blynk
BLYNK_WRITE(V0) {
  enabled = !enabled;
  if(enabled){
    testDriveNorth();
   
    }else{
      stop();
      }
    
}
