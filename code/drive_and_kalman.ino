#include <Wire.h>
#include <VL53L0X.h>
#include <Arduino.h>
#include <stdlib.h> 
#include <float.h>
#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>
#include <BasicLinearAlgebra.h>
#include <Servo.h>

#include "debug.h"
#include "file.h"
#include "server.h"


#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18
#define    FULL_SPEED  .114112 //m/s

#define SDA_PORT 14
#define SCL_PORT 12

double velocityL = 0; 
double velocityR = 0; 
VL53L0X sensor;
VL53L0X sensor2;
const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;
// WiFi AP parameters
char ap_ssid[13];
char* ap_password = "";
// WiFi STA parameters
char* sta_ssid = 
  "...";
char* sta_password = 
  "...";
char* mDNS_name = "paperbot";
String html;
String css;

// METHODS USED

//MPU TEST

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  
  Wire.write(Data);
  Wire.endTransmission();
}

void drive(int left, int right) {
  DEBUG("Driving");
  servo_left.write(left);
  servo_right.write(right);
}

void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
  velocityL = 0; 
  velocityR = 0; 
}

void forward() {
  DEBUG("forward");
  drive(0, 180);
  velocityL = FULL_SPEED; 
  velocityR = FULL_SPEED; 
}

void backward() {
  DEBUG("backward");
  drive(180, 0);
  velocityL = -FULL_SPEED; 
  velocityR = -FULL_SPEED;
}

void left() {
  DEBUG("left");
  drive(180, 180);
  velocityL = -FULL_SPEED;
  velocityR = FULL_SPEED; 
}

void right() {
  DEBUG("right");
  drive(0, 0);
  velocityL = FULL_SPEED; 
  velocityR = -FULL_SPEED;
}
// INITIALIZATIONS

unsigned long timer =0; 
long timeSlice = 0;
double theta = 0; 
double thetaDot = 0; 
double x = 0; 
double y = 0; 
double theta2 = 0; 
double W = 1000; // 550 millimeters
double L = 1000;  // 850 millimeters

BLA::Matrix<4,4> P; 
//double P[4][4]; // This is the uncertainty matrix for our iteration
BLA::Matrix<4,4> Q; 
BLA::Matrix<4,4> A; 
BLA::Matrix<4,2> B; 
BLA::Matrix<4> xVector; //x,y,thetadot theta
BLA::Matrix<2> u; 
BLA::Matrix<4,4> S; 
BLA::Matrix<4,4> R; 
BLA::Matrix<4,4> K; 
//double Q[4][4]; // THIS is the noise covariance matrix
//double A[4][4]; // This is the state transition identity matrix
//double B[4][2]; // This is 


void setup() {
  // put your setup code here, to run once:
// MPU
  // Arduino initializations
  Wire.begin(SDA_PORT,SCL_PORT);
  Serial.begin(115200);

  // Configure accelerometer
  I2CwriteByte (MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
  // Configure gyroscope 
  I2CwriteByte (MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS); 

  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

  
  // Dual Sensor

  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  delay(500);
  Wire.begin(SDA_PORT,SCL_PORT);

  Serial.begin (115200);

  digitalWrite(D3, HIGH);
  delay(150);
  Serial.println("00");
  
  sensor.init(true);
  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);

  digitalWrite(D4, HIGH);
  delay(150);
  sensor2.init(true);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");

  Serial.println("addresses set");
  
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

  delay(3000);

  //paperbot 

      setupPins();

    sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        LED_ON;
        delay(500);
        LED_OFF;
        delay(500);
    }
    LED_ON;
    //setupSTA(sta_ssid, sta_password);
    setupAP(ap_ssid, ap_password);
    LED_OFF;

    setupFile();
    html = loadFile("/controls.html");
    css = loadFile("/style.css");
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);

    setupHTTP();
    setupWS(webSocketEvent);
    //setupMDNS(mDNS_name);

    stop();

  timer = millis(); 

  P.Fill(0); 
  Q.Fill(0); 
  A.Fill(0); 
  B.Fill(0); 
  xVector.Fill(0); 
  u.Fill(0); 
  S.Fill(0); 
  R.Fill(0); 
  K.Fill(0); 
  
  xVector(0) = L-(1.07*sensor2.readRangeSingleMillimeters()-46.5); //x 
  xVector(1) = .969*sensor.readRangeSingleMillimeters()-32.7; // y
  xVector(2) = 0; // thetadot
  xVector(3) = 1;  


  Serial.print(xVector(0));
  Serial.print("\t"); 
  Serial.print(xVector(1));
  Serial.print("\t"); 
  Serial.print(xVector(2));
  Serial.print("\t"); 
  Serial.println(xVector(3)); 

  
  P(0,0) = .1; 
  P(1,1) = .1; 
  P(2,2) = .1; 
  P(3,3) = .1; 

  
  // SET INITIAL STATES: THIS IS THE INITIALIZATION 
  x = L-(1.07*sensor2.readRangeSingleMillimeters()-46.5); // SENSOR TWO IS THE FRONT SENSOR
  y = .969*sensor.readRangeSingleMillimeters()-32.7; // SENSOR ONE IS THE BACK SENSOR
  theta2 = 0; 
  thetaDot = 0; 

}

long int cpt=0;

void loop() {
  //1.  paperbot
   wsLoop();
   httpLoop();
   //2.  THIS IS THE CODE FOR MPU TEST

  uint8_t Buf[14]; 
  timeSlice = millis()-timer;
  timer = millis();  
  //Serial.print(timeSlice); 
  //Serial.print("\n");     
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
  
  // Convertir registros acelerometro
  int16_t ax = -(Buf[0] << 8 | Buf[1]);
  int16_t ay = -(Buf[2] << 8 | Buf[3]);
  int16_t az = Buf[4] << 8 | Buf[5];

  // Convertir registros giroscopio
  int16_t gx = -(Buf[8] << 8 | Buf[9]);
  int16_t gy = -(Buf[10] << 8 | Buf[11]);
  int16_t gz = Buf[12] << 8 | Buf[13];
  timeSlice = (int16_t)timeSlice; 
//  Serial.print(gx, DEC);
//  Serial.print("\t");
//  Serial.print(gy, DEC);
//  Serial.print("\t");
//  Serial.print(gz, DEC);
//  Serial.print("\t");

  
  
  Serial.print(xVector(0));
  Serial.print("\t"); 
  Serial.print(xVector(1));
  Serial.print("\t"); 
  Serial.print(xVector(2));
  Serial.print("\t"); 
  Serial.println(xVector(3));

  
   
  //x' = (A*x) + (B*u) =
  // DYNAMICS UPDATE
// Set the A Matrix
  A(0,0) = timeSlice/1000 +1; 
  A(1,1) = timeSlice/1000 +1; 
  A(2,2) = timeSlice/1000 + 1; 
  A(3,3) = timeSlice/1000 + 1; 

  
  x+=timeSlice*((velocityL+velocityR)/2)*cos(theta2); // Millimeters
  y+= timeSlice*((velocityL+velocityR)/2)*sin(theta2); // Millimeters
  theta2 += timeSlice*((velocityL-velocityR)/.508)/1000; // radians
  thetaDot = (velocityL-velocityR)/.508; 

  B(0,0) = timeSlice * cos(xVector(3))/2;
  B(0,1) = timeSlice* cos(xVector(3))/2; 
  B(1,0) = timeSlice * sin(xVector(3))/2; 
  B(1,1) = timeSlice * sin(xVector(3))/2; 
  B(2,0) = velocityL/.508; 
  B(2,1) = -velocityR/.508; 
  B(3,0) = timeSlice/1000 *(-1/.508); 
  B(3,1) = timeSlice/1000 * (1/.508); 

  R(0,0) = 1; 
  R(1,1) = 1;
  R(2,2) = 1; 
  R(3,3) = 1; 

  Q(0,0) = 1; 
  Q(1,1) = 1;
  Q(2,2) = 1;
  Q(3,3) = 1; 
   

  u(0) = velocityL;
  u(1) = velocityR;  
  
  
  xVector = A*xVector + B*u; 

  

  //P = (A*P*A') + Q = P + Q
  // UNCERTAINTY UPDATE
  //Q = diag{var x, var y, var dtheta/dt, var theta}
  //P = P + Q; 
 // ADD P AND Q


  P = (A * P * ~A) +Q; 
  
  
  //  NEED TO Characterize the Noise from dynamics update


  
  // H function is Minimum of the four walls
  BLA::Matrix<4,4>  H; 
  H.Fill(0); 
  BLA::Matrix<4> mu;
  mu.Fill(0);  
  // Using the values of x,y and theta2 we compute our expected state using 
  // COMPUTE EXPECTED READING FOR SIDE SENSOR MIN

  theta += (((8-gz)*timeSlice)/1000)*3.14*.0974025/180;  
  char send3[400];
  sprintf(send3, "theta = %d", theta); 
  //wsSend(0, send3); 
  
  double frontVals[4];
  double sideVals[4];
  
  double sideSensorMin = DBL_MAX;  
  double frontSensorMin = DBL_MAX;
  int frontWallPicked = -1;
  int sideWallPicked = -1;   
  //COMPUTE EXPECTED FRONT SENSOR VALUE
  frontVals[0] = (L-xVector(0))/cos(xVector(3)); 
  frontVals[1] = (W-xVector(1))/sin(xVector(3)); 
  frontVals[2] = xVector(0)/cos(xVector(3)+3.14); 
  frontVals[3] = xVector(1)/sin(xVector(3)+3.14); 
  
  for(int i=0; i<4; i++){
    if (frontVals[i]<frontSensorMin && frontVals[i]>0){
      frontSensorMin = frontVals[i];
      frontWallPicked =i;  
    }
  }
  
  // COMPUTE EXPECTED SIDE SENSOR VALUE
  sideVals[0] = (L-xVector(0))/sin(xVector(3)); 
  sideVals[1] = (W-xVector(1))/cos(xVector(3)+3.14); 
  sideVals[2] = xVector(0)/sin(xVector(3)+3.14); 
  sideVals[3] = xVector(1)/cos(xVector(3)); 

   for(int i=0; i<4; i++){
    if (sideVals[i]<sideSensorMin && sideVals[i]>0){
      sideSensorMin = sideVals[i];
      sideWallPicked = i;  
    }
  }

  
  //Approximate H function based on which wall was picked
  // This is the approximation for the front sensor
   switch(frontWallPicked){
    case 0: // WALL 1 SELECTED
      H(0,0) = -(1/cos(xVector(3))); // x
      H(0,1) = 0;  // y 
      H(0,2) = 0; // theta dot
      H(0,3) = ((L-xVector(0))*sin(xVector(3)))/(cos(xVector(3))*cos(xVector(3))); // theta
      mu(0) = (L-xVector(0))/cos(xVector(3)) + (xVector(0)/cos(xVector(3))) - ((L-xVector(0))*sin(xVector(3))*xVector(3))/(cos(xVector(3))*cos(xVector(3))); // Take out constants and adjust mean
      // STILL NEED TO ADJUST THE MEAN
      break; 
    case 1: // WALL 2 SELECTED
      H(0,0) = 0; // x
      H(0,1) = -1/sin(xVector(3));  // y
      H(0,2) = 0; // theta dot
      H(0,3) = (-(W-xVector(1))*cos(xVector(3)))/(sin(xVector(3))*sin(xVector(3)));   // theta
      mu(0) = (W-xVector(1))/sin(xVector(3)) + ((W-xVector(1))*cos(xVector(3))*xVector(3))/(sin(xVector(3))*sin(xVector(3))) + xVector(1)/(sin(xVector(3))); 
      break; 
    case 2: // WALL 3 SELECTED
      H(0,0) = 1/(cos(xVector(3) + 3.14)); // x
      H(0,1) = 0; // y 
      H(0,2) = 0; // theta dot 
      H(0,3) = (xVector(0)*sin(xVector(3) + 3.14))/(cos(xVector(3) + 3.14)*cos(xVector(3) + 3.14)); //theta
      mu(0) = xVector(0)/cos(xVector(3)+3.14) - xVector(0)*xVector(3)*sin(xVector(3)+3.14)/(cos(xVector(3)+3.14)*cos(xVector(3)+3.14)) - xVector(0)/(cos(xVector(3)+3.14));  
      break;
    case 3: // WALL 4 SELECTED
      H(0,0) = 0; //x
      H(0,1) = 1/(sin(xVector(3) + 3.14)); // y 
      H(0,2) = 0; //theta dot
      H(0,3) = (-xVector(1)*cos(xVector(3) + 3.14))/(sin(xVector(3) + 3.14)*sin(xVector(3)+3.14));  // theta
      mu(0) = xVector(1)/(sin(xVector(3) + 3.14)) + (xVector(1)*cos(xVector(3)+3.14)*xVector(3))/(sin(xVector(3)+3.14)*sin(xVector(3)+3.14)) - xVector(1)/(sin(xVector(3)+3.14)); 
      break; 
    default:
      wsSend(0,"ERROR");
      break;  
   }
   switch(sideWallPicked){
    case 0:
      H(1,0) = -1/sin(xVector(3));  // x 
      H(1,1) = 0 ;   // y 
      H(1,2) = 0;    // theta dot 
      H(1,3) = (-(L-xVector(0))*cos(xVector(3)))/(sin(xVector(3))*sin(xVector(3)));     // theta
      mu(1) = (L-xVector(0))/sin(xVector(3)) + (L-xVector(0))*xVector(3)*cos(xVector(3))/(sin(xVector(3))*sin(xVector(3))) + xVector(0)/sin(xVector(3)); 
      break;
    case 1:
      H(1,0) = 0; // x 
      H(1,1) = -1/(cos(xVector(3) + 3.14)); // y
      H(1,2) = 0; // theta dot
      H(1,3) = ((W-xVector(1))*sin(xVector(3) + 3.14))/(cos(xVector(3)+3.14)*cos(xVector(3)+3.14)); //theta
      mu(1) = (W-xVector(1))/cos(xVector(3)+3.14) -(xVector(3)*(W-xVector(1))*sin(xVector(3)+3.14))/(cos(xVector(3) + 3.14) * cos(xVector(3)+3.14)) + xVector(1)/(cos(xVector(3) + 3.14)); 
      break; 
    case 2:
      H(1,0) = 1/(sin(xVector(3) + 3.14)); //x
      H(1,1) = 0; // y 
      H(1,2) = 0; //theta dot
      H(1,3) = (-xVector(0)*cos(xVector(3) + 3.14))/(sin(xVector(3)+3.14)*sin(xVector(3)+3.14)); //theta
      mu(1) = xVector(0)/sin(xVector(3)+3.14) + (xVector(3)*xVector(0)*cos(xVector(3) + 3.14))/(sin(xVector(3) + 3.14)*sin(xVector(3)+3.14)) - xVector(0)/sin(xVector(3) + 3.14) ; 
      break; 
    case 3:
      H(1,0) = 0; //x
      H(1,1) = 1/(cos(xVector(3))); // y 
      H(1,2) = 0; //theta dot
      H(1,3) = (xVector(1)*sin(xVector(3)))/(cos(xVector(3))*cos(xVector(3)));   // theta
      mu(1) = xVector(1)/cos(xVector(3)) - (xVector(1)*sin(xVector(3))*xVector(3))/(cos(xVector(3))*cos(xVector(3)))-xVector(1)/(cos(xVector(3))); 
      break;
    default:
      wsSend(0,"ERROR"); 
      break;
   }
  // For Gyroscope, we measure angular velocity
  H(2,0) = 0; //x 
  H(2,1) = 0; //y 
  H(2,2) = 1;  // theta Dot
  H(2,3) = 0; 
  mu(2) = 0; 

  // For the angle measured
  H(3,0) = 0; 
  H(3,1)= 0;
  H(3,2) = 0; 
  H(3,3) = 1;
  mu (3) = 0;     

  BLA::Matrix<4,4> H_transpose = ~H; 


  //COMPUTE S MATRIX
  S = (H * P* H_transpose) + R;

  int res = 0;
  BLA::Matrix<4,4> S_inv = S.Inverse(&res); // RETURNS NONZERO if S  is not invertible 
  if(res!=0){
    Serial.print("Non invertible S matrix"); 
  }
  //KALMAN GAIN
  K = P*H_transpose*S_inv; 

  BLA::Matrix<4,1> m;  // Measurement Vector
  m.Fill(0); 
   
  m(0) =  1.07*(double)sensor2.readRangeSingleMillimeters()-46.5; 
  m(1) = .969*(double)sensor.readRangeSingleMillimeters()-32.7; 
  
  m(2) =  (8-gz)*3.14*.0974025/180;  
  m(3)=  theta; 
Serial.print(theta); 
  BLA::Matrix<4,1> y = m - (H*xVector); 
  xVector = xVector+(K*y); 
  BLA::Matrix<4,4> I = {1, 0, 0, 0,

                   0, 1, 0 , 0,

                   0, 0, 1, 0,
                   0, 0 , 0 , 1};
                   
  P = (I-(K*H))*P; 
  char send1[400];
  char send2[400]; 
  char send4[400]; 
  
//  double xOut = mu(0)+xVector(0); 
//  double yOut = mu(1)+xVector(1);
  sprintf (send1, "x: %lf, y: %lf, thetaDot: %lf, theta: %lf", xVector(0),xVector(1),xVector(2),xVector(3));  
  wsSend(0, send1); 
  sprintf(send2, "P(0,0): %lf \t , P(1,1): %lf \t P(2,2): %lf \t, P(3,3): %lf ", P(0,0), P(1,1), P(2,2), P(3,3)); 
  wsSend(0,send2); 
  //sprintf(send4, "Predicted states are: x: %lf, y: %lf, z: %lf" , x,y,theta2);
  
  //sprintf (send1, "front expected sensor reading: %lf  side expected sensor val: %lf",frontSensorMin, sideSensorMin);
  //sprintf (send2, "front actual sensor: %d  side actual sensor: %d", frontActual, sideActual); 
  //sprintf(send1, "x: %lf   y: %lf   theta2: %lf ",x,y,  theta2); 
  //wsSend(0, send1); 
  //wsSend(0, send2); 
//  char send1[200];
//  char send2[200];   
//  sprintf(send1, "\rHeading:\t %s Radians  \t %s  + Degrees  \t", heading, headingDegrees ); 
//  sprintf(send2, "Magnetometer readings: \tMx: %s  \tMy: %s  \tMz: %s \t", mx, my, mz  );   
//  wsSend(0, send1);
//  wsSend(0, send2); 

  
  // End of line
 

  //3. This is the code for the dual sensor test
//
//    Serial.print(" front Actual (mm): ");
//  Serial.print(frontActual);
//  Serial.print("\t"); 
//  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
//  
//  Serial.print(" Side Actual(mm): ");
//  Serial.println(sideActual);
//  if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }
//
//  Serial.print("Front Sensor Min: "); 
//  Serial.print(frontSensorMin); 
//  Serial.print("\t");
//  Serial.print("Side Sensor Min: ");  
//  Serial.println(sideSensorMin);
 delay(100); 
  
}




//SETUP 
void setupPins() {
    // setup Serial, LEDs and Motors
    Serial.begin(115200);
    DEBUG("Started serial.");

    pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
    LED_OFF;                     //Turn off LED
    DEBUG("Setup LED pin.");

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    DEBUG("Setup motor pins");
}


void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            break;
        case WStype_CONNECTED: 
        {
            // IPAddress ip = webSocket.remoteIP(id);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
            DEBUG("Web socket connected, id = ", id);

            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid);
            break;
        }
        case WStype_BIN:
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);

            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);

        case WStype_TEXT:
            DEBUG("On connection #", id)
            DEBUG("  got text: ", (char *)payload);

            if (payload[0] == '#') {
                if(payload[1] == 'C') {
                  LED_ON;
                  wsSend(id, "Hello world!");
                }
                else if(payload[1] == 'F')  
                  forward();
                else if(payload[1] == 'B') 
                  backward();
                else if(payload[1] == 'L') 
                  left();
                else if(payload[1] == 'R') 
                  right();
                else if(payload[1] == 'U') {
                  if(payload[2] == 'L') 
                    servo_left_ctr -= 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr += 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') 
                    servo_left_ctr += 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr -= 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else 
                  stop();
            }

            break;
    }
}
