
#include <BlynkSimpleSerialBLE.h>
#include <SoftwareSerial.h>
#include <DFRobot_QMC5883.h>
#include <TinyGPS.h>
#include <Wire.h>

#define BLYNK_USE_DIRECT_CONNECT

#define GPS_TX_PIN 6
#define GPS_RX_PIN 3

#define BLUETOOTH_TX_PIN 10
#define BLUETOOTH_RX_PIN 11

#define MOTOR_A_EN_PIN 5
#define MOTOR_B_EN_PIN 9
#define MOTOR_A_IN_1_PIN 7
#define MOTOR_A_IN_2_PIN 8
#define MOTOR_B_IN_1_PIN 12
#define MOTOR_B_IN_2_PIN 4

#define MOTOR_A_CAL 15
#define MOTOR_B_CAL 0

#define COMPASS_OFFSET 0.0f//calibrar
#define DECLINATION  4.23 //declinacion mexico

#define GPS_STREAM_TIMEOUT 18
#define GPS_WAYPOINT_TIMEOUT 45

struct GeoLoc {
  float lat;
  float lon;
};

char auth[] = "NYMkkC7RWyQJvGC3KvLZgLb6Zgrb0thR";

SoftwareSerial nss(GPS_TX_PIN, 255);  
TinyGPS gps;

SoftwareSerial bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);


DFRobot_QMC5883 compass;

bool enabled = false;


void setup() {
  // put your setup code here, to run once:
  setupCompass();
  pinMode(MOTOR_A_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(4800);
  nss.begin(9600);
  bluetoothSerial.begin(9600);
  Blynk.begin(bluetoothSerial, auth);
 
}

void setSpeed(int speed)
{
  setSpeedMotorA(speed);
  setSpeedMotorB(speed);
}
void setSpeedMotorA(int speed) {
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
   analogWrite(MOTOR_A_EN_PIN, speed + MOTOR_A_CAL);// calibrar
}

void setSpeedMotorB(int speed) {
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
 analogWrite(MOTOR_B_EN_PIN, speed + MOTOR_B_CAL);//calibrar 
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
  float declinationAngle = DECLINATION;
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
+
  
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

void drive(int distance, float turn) { 
  int fullSpeed = 100;
  int stopSpeed = 0;

  // drive to location
  int s = fullSpeed;
  if ( distance < 8 ) {// si la distancia es menor a 8
    int wouldBeSpeed = s - stopSpeed;
    wouldBeSpeed *= distance / 8.0f;
    s = stopSpeed + wouldBeSpeed;
  }
  
  int autoThrottle = constrain(s, stopSpeed, fullSpeed);//acelerador erestringe s entre 0 y 200 
  autoThrottle = 100;// revisar imprimir

  float t = turn;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;
  
  Serial.print("turn: ");
  Serial.println(t);
  Serial.print("original: ");
  Serial.println(turn);
  
  float t_modifier = (180.0 - abs(t)) / 180.0;
  float autoSteerA = 1;//incializa valores.
  float autoSteerB = 1;

  if (t < 0) {//velocidad para cada llanta dependiendo los grados
    autoSteerB = t_modifier;
  } else if (t > 0){
    autoSteerA = t_modifier;
  }

  Serial.print("steerA: "); Serial.println(autoSteerA);
  Serial.print("steerB: "); Serial.println(autoSteerB);

  int speedA = (int) (((float) autoThrottle) * autoSteerA);
  int speedB = (int) (((float) autoThrottle) * autoSteerB);
  
 setSpeedMotorA(speedA);
 setSpeedMotorB(speedB);//imprimir velocidades
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



GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < 1000) {
    if (feedgps())
      newdata = true;
  }
  if (newdata) {
    return gpsdump(gps);
  }

  GeoLoc coolerLoc;
  coolerLoc.lat = 0.0;
  coolerLoc.lon = 0.0;
  
  return coolerLoc;
}

// Get and process GPS data
GeoLoc gpsdump(TinyGPS &gps) {
  float flat, flon;
  unsigned long age;
  
  gps.f_get_position(&flat, &flon, &age);

  GeoLoc coolerLoc;
  coolerLoc.lat = flat;
  coolerLoc.lon = flon;
Serial.print("Obteniendo local");
  Serial.print(coolerLoc.lat, 7); Serial.print(", "); Serial.println(coolerLoc.lon, 7);

  return coolerLoc;
}

// Feed data as it becomes available 
bool feedgps() {
  while (nss.available()) {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float geoBearing(struct GeoLoc &a, struct GeoLoc &b) {
  float y = sin(b.lon-a.lon) * cos(b.lat);
  float x = cos(a.lat)*sin(b.lat) - sin(a.lat)*cos(b.lat)*cos(b.lon-a.lon);
  return atan2(y, x) * RADTODEG;
}


float geoDistance(struct GeoLoc &a, struct GeoLoc &b) {
  const float R = 6371000; // km
  float p1 = a.lat * DEGTORAD;
  float p2 = b.lat * DEGTORAD;
  float dp = (b.lat-a.lat) * DEGTORAD;
  float dl = (b.lon-a.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

void driveTo(struct GeoLoc &loc, int timeout) {
  nss.listen();
  GeoLoc coolerLoc = checkGPS();
  bluetoothSerial.listen();

  if (coolerLoc.lat != 0 && coolerLoc.lon != 0 && enabled) {
    float d = 0;
    //Start move loop here
    do {
      nss.listen();
      coolerLoc = checkGPS();
      bluetoothSerial.listen();
      
      d = geoDistance(coolerLoc, loc);
      float t = geoBearing(coolerLoc, loc) - geoHeading();
      
      Serial.print("Distance: ");
      Serial.println(geoDistance(coolerLoc, loc));
    
      Serial.print("Bearing: ");
      Serial.println(geoBearing(coolerLoc, loc));

      Serial.print("heading: ");
      Serial.println(geoHeading());
      
      drive(d, t);
      timeout -= 1;
    } while (d > 3.0 && enabled && timeout>0);

    stop();
  }
}

BLYNK_WRITE(V0) {

  enabled=true;
  if(enabled){

 testDriveNorth();
   
    }else{
      stop();
      }
    
}

BLYNK_WRITE(V1) {
  enabled = !enabled;
  
  //Stop the wheels
  stop();
}

// GPS Streaming Hook
BLYNK_WRITE(V2) {
  GpsParam gps(param);
  
  Serial.println("Received remote GPS: ");
  
  // Print 7 decimal places for Lat
  Serial.print(gps.getLat(), 7); Serial.print(", "); Serial.println(gps.getLon(), 7);

  GeoLoc phoneLoc;
  phoneLoc.lat = gps.getLat();
  phoneLoc.lon = gps.getLon();

  driveTo(phoneLoc, GPS_STREAM_TIMEOUT);
}

// Terminal Hook


void loop() {
 Blynk.run();

}
