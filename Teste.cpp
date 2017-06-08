// Baixar:
//Biblioteca PID: http://arduino-pid-library.googlecode.com/files/PID_FrontEnd_v03.zip
//Biblioteca MPU: http://diyhacking.com/projects/MPU6050.zip
//Biblioteca I2Cdev: http://diyhacking.com/projects/I2Cdev.zip
//Biblioteca Servo: https://playground.arduino.cc/uploads/ComponentLib/SoftwareServo.zip
#include "I2Cdev.h"
#include <PID_v1.h>
#include <Servo.h>
#include <Ultrasonic.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
 
//Biblioteca usada para escrever na MPU.
//Disponível em https://www.arduino.cc/en/reference/wire
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
 
// Definição da biblioteca MPU da MPU
#define OUTPUT_READABLE_YAWPITCHROLL
 
//Variaveis do drone
#define QUANT_MOTOR   4 // Definiço de quantos motores serao testados
// Valores de pulso de PWM
#define LIGA      30  //Mínimo necessário para ligar os ESCS
#define RODA      32  //Mínimo necessário para os motores começarem a rodar    
#define VOO       110 //Mínimo necessário para o drone levantar voo
#define MAX_PWM   180 //Maior valor de saída PWM possível.
#define MAX_PWM_USADO 150 //Maior valor base (escrito em todos os motores) que vamos usar

//Altura Desejada
#define ALT_DES 5
 
//Define que motores são responsáveis por que angulos
#define ROLL_MOTOR_UP1 0 
#define ROLL_MOTOR_UP2 1
#define ROLL_MOTOR_DOWN1 2
#define ROLL_MOTOR_DOWN2 3
 
#define PITCH_MOTOR_UP1 1
#define PITCH_MOTOR_UP2 2
#define PITCH_MOTOR_DOWN1 3
#define PITCH_MOTOR_DOWN2 0
 
#define PINO_TRIGGER 4
#define PINO_ECHO 7

//Objeto para sensor de giroscópios
MPU6050 mpu;

//Inicializa o sensor ultrassonico.
Ultrasonic ultrasonic(PINO_TRIGGER, PINO_ECHO);
 
//Motores do drone
Servo motores[QUANT_MOTOR];         // Cria um vetor de objetos servo, com tamanho QUANT_MOTOR
int pinosPwm[] = {3, 5, 6, 9, 10, 11};              // Vetor com os pinos que possuem funçao pwm
double Velocidade_Base;                //Velocidade de giro comum a todos os motores.
int Velocidade_Giro[QUANT_MOTOR];            
 
//angulos de YawPitchwRoll
double AYaw, APitch, ARoll;

//Altura do Drone em centimetros
double Altura;
 
//LED usada para testes
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
 
//Variaveis da MPU
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q;         // [w, x, y, z]       quaternion container
VectorInt16 aa;       // [x, y, z]          accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]          gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]          world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]          gravity vector
float euler[3];       // [psi, theta, phi]  Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
 
 
// ================================================================
// ===            INTERRUPT DETECTION ROUTINE             ===
// ================================================================
 
volatile bool mpuInterrupt = false;   // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
 
//Fim das variaveis da MPU6050
 
//Declaração do PID
 
//Variaveis com as quais vamos conectar
double Equilibrio_Roll, Incremento_Roll;
double Equilibrio_Pitch, Incremento_Pitch;
double Equilibrio_Altura;
 
//Valores iniciais para os parametros da PID
double Kp_Roll=2, Ki_Roll=5, Kd_Roll=1;
double Kp_Pitch=2, Ki_Pitch=5, Kd_Pitch=1;
double Kp_Altura=2, Ki_Altura=5, Kd_Altura=1; 

//Declara PIDs  
PID PID_Roll(&ARoll, &Incremento_Roll, &Equilibrio_Roll, Kp_Roll, Ki_Roll, Kd_Roll, DIRECT);
PID PID_Pitch(&APitch, &Incremento_Pitch, &Equilibrio_Pitch, Kp_Pitch, Ki_Pitch, Kd_Pitch, DIRECT);
PID PID_Altura(&Altura, &Velocidade_Base, &Equilibrio_Altura, Kp_Altura, Ki_Altura, Kd_Altura, DIRECT);
 
//Função de atualização dos valores de YAW, PITCH e ROll
void LeMPU();

//Pega media de cinco valores do sensor ultrassonico e escreve, em cm, na variável altura.
void LeUltrassonico();
 
// ================================================================
// ===                    INITIAL SETUP                     ===
// ================================================================
 
void setup() {
  //Definições e serial da MPU
    
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
    
  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
 
  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.
 
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
 
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 
  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());              // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
 
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
 
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(118);
  mpu.setYGyroOffset(-10);
  mpu.setZGyroOffset(80);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
 
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
 
      // enable Arduino interrupt detection
      Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
 
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;
 
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
    
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
    
  //Fim das definições da MPU
    
  //Seta PID como automatica
  PID_Pitch.SetMode(AUTOMATIC);
  PID_Roll.SetMode(AUTOMATIC);
  PID_Altura.SetMode(AUTOMATIC);
 
  //Seta janela de saída dos PIDs entre 0 e MAX_PWM (cuidado para que MAX_PWM - MAX_PWM_USADO < MAX_PWM_USADO)
  PID_Altura.SetOutputLimits(0, MAX_PWM_USADO);
  PID_Pitch.SetOutputLimits(MAX_PWM_USADO-MAX_PWM, MAX_PWM - MAX_PWM_USADO);
  PID_Roll.SetOutputLimits(MAX_PWM_USADO-MAX_PWM, MAX_PWM - MAX_PWM_USADO);
    
  //Salva leituras verticais da MPU como valores desejaveis
  while (!mpuInterrupt && fifoCount < packetSize); // Espera o primeiro valor estar Disponível
  LeMPU();//Le primeiro valor
    
  Equilibrio_Pitch = APitch; // Salva angulos
  Equilibrio_Roll = ARoll;
  Equilibrio_Altura = ALT_DES; //Altura desejada    
    
  //Inicia os Motores
  for(int i=0; i<QUANT_MOTOR; i++)
    motores[i].attach(pinosPwm[i]); // Anexa os motores com os pinos pwm. ATENÇAO: OS PINOS ESTAO EM ORDEM CRESCENTE
 
    
  for(int i=0; i<QUANT_MOTOR; i++)
    motores[i].write(LIGA);   //Liga os ESCs
     
  delay(2000); // Tempo entre ligar o ESC e começar a usar.
 
  for(int i=0; i<QUANT_MOTOR; i++)
    motores[i].write(RODA);   //Começa a rodar os motores
   
  delay(2000); //DEixa os moteres começarem a rodar
    
  for(int i=0; i<QUANT_MOTOR; i++)
    motores[i].write(VOO);   //Coloca os motores em velocidade de voo;
    
  for(int i=0; i<QUANT_MOTOR; i++)
    Velocidade_Giro[i] = VOO;
   
}
 
 
 
// ================================================================
// ===                  MAIN PROGRAM LOOP                   ===
// ================================================================
 
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
 
  // wait for MPU interrupt or extra packet(s) available
    //Atualiza saída do PID
    PID_Roll.Compute();
    PID_Pitch.Compute();
    PID_Altura.Compute();
  
  //Atualiza velocidade comum em todos os motores;
    for(int i=0; i<QUANT_MOTOR; i++)
      Velocidade_Giro[i] = (int) Velocidade_Base;

    //Atualiza velocidade dos motores de cada angulo
    Velocidade_Giro[ROLL_MOTOR_UP1] += (int) Incremento_Roll;
    Velocidade_Giro[ROLL_MOTOR_UP2] +=  (int) Incremento_Roll;
    Velocidade_Giro[ROLL_MOTOR_DOWN1] -= (int) Incremento_Roll;
    Velocidade_Giro[ROLL_MOTOR_DOWN2] -= (int) Incremento_Roll;
    
    Velocidade_Giro[PITCH_MOTOR_UP1] += (int) Incremento_Pitch;
    Velocidade_Giro[PITCH_MOTOR_UP2] += (int) Incremento_Pitch;
    Velocidade_Giro[PITCH_MOTOR_DOWN1] -= (int) Incremento_Pitch;
    Velocidade_Giro[PITCH_MOTOR_DOWN2] -= (int) Incremento_Pitch;
    
    //Esccreve velocidades nos respectivos motores
    for(int i = 0; i < QUANT_MOTOR; i++)
      motores[i].write(Velocidade_Giro[i]);
     
  dbg();  

  //Atualiza valores dos angulos de YAW, PITCH e Roll, se esses estiverem disponiveis.
  if(!mpuInterrupt && fifoCount < packetSize)
    LeMPU();

  //Atuliza altura.
  LeUltrassonico();
    
}

void dbg(){
  for(int i = 0; i < QUANT_MOTOR; i++){
    Serial.print("Velocidade motor ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(Velocidade_Giro[i]);
  }
}
 
//Le e imprime valores da MPU (copiado do exemplo da biblioteca)
void LeMPU(){
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
 
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
 
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
 
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
     
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
 
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          Serial.print("ypr\t");
          Serial.print(ypr[0] * 180/M_PI);
          Serial.print("\t");
          Serial.print(ypr[1] * 180/M_PI);
          Serial.print("\t");
          Serial.println(ypr[2] * 180/M_PI);
      #endif
 
      //salva angulos;
      AYaw = ypr[0];
      ARoll = ypr[1];
      APitch = ypr[2];
     
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
  }
 
}

//Pega media de cinco valores do sensor ultrassonico e escreve, em cm, na variável altura.
void LeUltrassonico(){
  //Le as informacoes do sensor em cm
  double cmMsec;
  double cmMsec_tot = 0;

  for(int i = 0; i < 5; i++){
    long microsec = ultrasonic.timing();
    cmMsec = ultrasonic.convert(microsec, Ultrasonic::CM);
    cmMsec_tot += cmMsec;
  }

  Altura = cmMsec_tot/5;

  //Exibe informacoes no serial monitor
  Serial.print("Distancia em cm: ");
  Serial.println(Altura);

  delay(100); //muito baixo?
}
