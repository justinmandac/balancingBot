#include <Wire.h>
#include <math.h>
#include <ITG3200.h>

#define PWM_WIDTH 128

#define spiclk  53    // connect to ADXL CLK
#define spimiso 51  // connect to ADXL DO
#define spimosi 49  // connect to ADXL DI
#define spics   47    // connect to ADXL CS

#define ACC_SENS_2G   0.00390625//
#define S_RATE_DIV 0x63 //100Hz sampling rate

#define M_PI 3.14159265359
#define P_FILTER_COEFF .98

ITG3200 gyro = ITG3200();
float  gyr_x,gyr_y,gyr_z;
float  pitch     = 0;
float  gyr_pitch = 0;
float  acc_pitch = 0;
float  delta     = 0;
long   timer     = 0;
int    write_pitch=0;

//ACCELEROMETER
float  acc_x,acc_y,acc_z; //in g
byte   raw_acc[8];  // raw accelerometer data storage
byte   spiread; //SPI buffer

double ang_tar = 5.0; //target "stable" angle
double ang_diff; //difference between actual angle and target
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing");
  Wire.begin();
  init_adxl();
  gyro.init(ITG3200_ADDR_AD0_LOW); 
  gyro.setSampleRateDiv(S_RATE_DIV);
  gyro.setFilterBW(BW005_SR1);
  gyro.zeroCalibrate(2500, 2);  
  pinMode(8, OUTPUT);     
  pinMode(9, OUTPUT);     
  pinMode(10, OUTPUT);     
  pinMode(11, OUTPUT);  
}

void loop()  {
  delta = ((float)micros() - timer)/10000000.000f; //get delta
  timer = micros();
  read_xyz();
  gyro.readGyro(&gyr_x,&gyr_y,&gyr_z);
  gyr_pitch+=gyr_x*delta;
  acc_pitch = 4068*(atan(acc_y/(sqrt((acc_x*acc_x)+(acc_z*acc_z)))))/71;
  
  pitch = fmod(P_FILTER_COEFF*(gyr_pitch*10) + (1.0f-P_FILTER_COEFF)*acc_pitch,360); //Complementary filter
  
  if(!isnan(pitch)) {
    Serial.println(pitch); 
     
      //moveBoth(255, false,0);
    if(pitch > -20 && pitch < 20) {
      
       stopBoth();
     
    }  
    else if (pitch <= -20) {
      moveBoth(PWM_WIDTH, true);
    }
    else if ( pitch >= 20) {
      moveBoth(PWM_WIDTH, false);
    }
  }
}

/*
  _dir: true - fwd, false - rev
*/
void moveBoth(int _speed, boolean _dir) {
  long count; 
  
  if(_dir) {
    digitalWrite(8,  LOW);
    digitalWrite(11, LOW);
  }
  else {
    digitalWrite(8,  HIGH);
    digitalWrite(11, HIGH);  
  }
  
  //for(count = 0; count < _duration; count++) {
    analogWrite(9, _speed);
    analogWrite(10,_speed);
  //}
}

void stopBoth() {
  long count; 
  
  //for(count = 0; count < _duration; count++) {
    analogWrite(9,0);
    analogWrite(10,0);
  //}
}
/*SPI FUNCTIONS*/
void spi_out(byte spidat){
  byte bitnum=8;
 
    spiread=0;
    // start spi bit bang
    while(bitnum>0){
       
      pinMode(spiclk,OUTPUT);    // SPI CLK =0
      if((spidat & 0x80)!=0)
        pinMode(spimosi,INPUT);  // MOSI = 1 if MSB =1
        else
        pinMode(spimosi,OUTPUT);  // else MOSI = 0
      
      spidat=spidat<<1; 
      pinMode(spiclk,INPUT);  // SPI CLK = 1
      
      // read spi data
      spiread=spiread<<1;
      
      if(digitalRead(spimiso)==HIGH) spiread |= 0x01; // shift in a 1 if MISO is 1
 
      pinMode(spimosi,INPUT);  // reset MOSI to 1
      bitnum--; 
    }
}
/*  Initialize ADXL345 */
 
void  init_adxl(void){
  delay(250);
  pinMode(spics,OUTPUT);  // CS=0  
  
  //Write to register 0x31, DATA FORMAT
  spi_out(0x31);
  // uncomment your desired range
 // spi_out(0x0B); //full resolution, +/- 16g range
  //spi_out(0x0A); //full resolution, +/- 8g range
  //spi_out(0x09); //full resolution, +/- 4g range
   spi_out(0x08); //full resolution, +/- 2g range
  
  pinMode(spics,INPUT);  //CS HIGH
  
  delay(1);
    pinMode(spics,OUTPUT);  // CS=0   
  
  // Write to register 0x2d, POWER_CTL
  spi_out(0x2d);
  //set to measure mode
  spi_out(0x08);
  pinMode(spics,INPUT);  //CS HIGH
  
  delay(1);
}
void read_xyz(void){
  int i;
    pinMode(spics,OUTPUT);  // CS=0   
    
  //Set start address to 0x32
  //D7= 1 for read and D6=1 for sequential read
  spi_out(0xF2);
  // dump xyz content to array
  for(i=0;i<6;i++){
    spi_out(0x00);
    raw_acc[i]=spiread;
  }  
  // merge to convert to 16 bits
  acc_x=((int)(raw_acc[1]<<8)|(int)raw_acc[0])*ACC_SENS_2G;
  acc_y=((int)(raw_acc[3]<<8)|(int)raw_acc[2])*ACC_SENS_2G;
  acc_z=((int)(raw_acc[5]<<8)|(int)raw_acc[4])*ACC_SENS_2G;
  
  pinMode(spics,INPUT);  //CS HIGH
}
