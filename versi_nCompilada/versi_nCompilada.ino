//Conexión de pines Módulo SD:
//3v3= 3.3 voltios
// Cs= pin 6
// Mosi= pin 11
// Clk= pin 13
// Miso= pin 12
// GND= tierra

#include <SD.h>
#include <SPI.h>
#include <LoRa.h>
#include <MPU6050.h>
#include <MPU9250.h>
MPU6050 mpu;
MPU9250 IMU(Wire,0x69);
File logFile; //MÓDULO SD
int status;
// Timers
unsigned long timer = 0;
float timeStep = 0.01;
// Pitch, Roll and Yaw values
float pitch;
float roll;
float yaw;
//Sensor de corriente
float Sensibilidad=0.1;

const int csPin = 53;          // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 19;
int contador=0;  //Contador para identificación de trama

float ;

//Tacómetro 

boolean est = false;
int entrada = A1; //Pin entrada del sensor
int entrada2; 
unsigned long reloj1; //Variables de tiempo para medir las RPM
unsigned long reloj2 = 0;
unsigned long tiempo1;//Variables de tiempo para imprimir las RPM
unsigned long tiempo2=0;
float RPM;
int i; //Variable para solo medir las RPM una vez por flanco

void setup() {
  
  //Inicio LoRa
  Serial.begin(9600);
  LoRa.setPins(csPin, resetPin, irqPin);
  while (!Serial);
  Serial.println("LoRa Sender");

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  //Inicio acelerometros
  status = IMU.begin();
    if (status < 0) {
  Serial.println("IMU initialization unsuccessful");
  Serial.println("Check IMU wiring or try cycling power");
  Serial.print("Status: ");
  Serial.println(status);
    while(1) {}
  }
    {

    Serial.println("Initialize MPU6050");

    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
      delay(500);
    }
  }

  //Inicio modulo SD
  Serial.print("Iniciando SD ...");
  if (!SD.begin(6))
  {
    Serial.println("Error al iniciar");
    return;
  }
  Serial.println("Iniciado correctamente");
  
}


void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.println();
  mpu.calibrateGyro();
  mpu.setThreshold(2);
}

void loop() {

 //Modulo SD, abre el archivo en la SD
 logFile = SD.open("datalog.txt", FILE_WRITE);

 //Tacometro
    reloj1 = micros();
    entrada2 = analogRead(entrada);
    if(entrada2>200 && i <1){
    tiempo1 = micros();
    RPM = (float)30000000/(tiempo1-tiempo2);
    i = 1;
    }
    else if(entrada2<200 && i == 1){
    tiempo2 = micros();
    RPM = (float)30000000/(tiempo2-tiempo1);
    i = 0;
    }

  //Sensor corriente
  float I=get_corriente(200);//obtenemos la corriente promedio de 500 muestras 
  Serial.print("Corriente: ");
  Serial.println(I,3); 
  delay(100);

 //Acelerómetro
  Serial.print("Sending packet: ");
  // display the data
  Serial.print(" Xnormblue=");  
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print(" Ynormblue= ");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print(" Znormblue= ");
  Serial.println(IMU.getAccelZ_mss(),6);

  
  timer = millis();
  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();
  Vector norm = mpu.readNormalizeGyro();
  

  Serial.print(" XnormPurple = ");
  Serial.print(normAccel.XAxis,6);
  Serial.print(" YnormPurple = ");
  Serial.print(normAccel.YAxis,6);
  Serial.print(" ZnormPurple = ");
  Serial.println(normAccel.ZAxis,6);


    // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  int roll = -(atan2(normAccel.XAxis, normAccel.ZAxis)*180.0)/M_PI;
  int pitch = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  // Output
  
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.println(roll);
  
  
 // delay(100);
  

  // send packet
  //String pitch=String(pitch);
  //String roll=String(roll);
  LoRa.beginPacket();
  //sensor delantero
  LoRa.print(String(contador));
  LoRa.print("*");
  LoRa.print(String(normAccel.XAxis,6));
  LoRa.print("*");
  LoRa.print(String(normAccel.YAxis,6));
  LoRa.print("*");
  LoRa.print(String(normAccel.ZAxis,6));
  LoRa.print("*");
  //sensor trasero
  LoRa.print(String(IMU.getAccelX_mss(),6));
  LoRa.print("*");
  LoRa.print(String(IMU.getAccelY_mss(),6));
  LoRa.print("*");
  LoRa.print(String(IMU.getAccelZ_mss(),6));
  LoRa.print("*");
  //gioscopio
  LoRa.print(String(pitch));
  LoRa.print("*");
  LoRa.print (String(roll));
  LoRa.print("*");
  //Sensor de corriente
  LoRa.print(String(I,3));
  LoRa.print("*");
  //Tacometro (no imprime siempre)
if(reloj1-reloj2 >2000000 ){
  LoRa.print(RPM);
   reloj2 = reloj1;
      }
  LoRa.println(";");
 
  LoRa.endPacket();

//Escribir los datos en la SD
  if (logFile) { 
      logFile.print(String(contador)); 
      logFile.print("*");
      logFile.print(String(normAccel.XAxis,6));
      logFile.print("*");
      logFile.print(String(normAccel.YAxis,6));
      logFile.print("*");
      logFile.print(String(normAccel.ZAxis,6));
      logFile.print("*");
      //sensor trasero
      logFile.print(String(IMU.getAccelX_mss(),6));
      logFile.print("*");
      logFile.print(String(IMU.getAccelY_mss(),6));
      logFile.print("*");
      logFile.print(String(IMU.getAccelZ_mss(),6));
      logFile.print("*");
      //gioscopio
      logFile.print(String(pitch));
      logFile.print("*");
      logFile.print(String(roll));
      logFile.print("*");
      //Sensor de corriente 
      logFile.print(String(I,3));
      logFile.print("*");
      //Tacometro (no imprime siempre)
      if(reloj1-reloj2 >2000000 ){
       logFile.print(RPM);
           reloj2 = reloj1;
              }
      logFile.println(";");
      logFile.close();
  
  } 
  else {
    Serial.println("Error al abrir el archivo");
  }
  contador++;
  delay(100);
}

float get_corriente(int n_muestras)
{
  float voltajeSensor;
  float corriente=0;
  for(int i=0;i<n_muestras;i++)
  {
    voltajeSensor = analogRead(A0) * (5.0 / 1023.0);////lectura del sensor
    corriente=corriente+(voltajeSensor-2.5)/Sensibilidad; //Ecuación  para obtener la corriente
  }
  
  corriente=corriente/n_muestras;
  return(corriente);
}
