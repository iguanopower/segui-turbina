//declaracion de librerías
#include <Servo.h>
#include <TapatioElectronics.h>
#include <IRremote.h>

//definicion de pines
//IR
#define SensorIR            2

//Turbina
#define turbina             3
Servo fan;

//Rango estándar de señal ESC en microsegundos
#define ESC_MIN_US          1000
#define ESC_MAX_US          2000

//botón físico (no se usa)
#define boton               4

//puente H
#define standby             9

#define PWM_motor_izq       5
#define motor_izq_dirA      8
#define motor_izq_dirB      7

#define PWM_motor_der       6
#define motor_der_dirA      11
#define motor_der_dirB      10

//led interno
#define led       LED_BUILTIN

#define sensors 16

#define octo    A1
#define tetra   A2
#define bi      A3
#define uni     A4
#define output  A0

//declaración de arreglo de sensores
SensorBar16 sb(octo, tetra, bi, uni, output);

//Potencia de la turbina (0 a 100%)
int TURBINA_POTENCIA = 70;
int turbina_actual_pct = 0;

bool sensores_activos = false;
bool robot_activo = false;

//---------------------CONTROL DE TURBINA (ESC)---------------------

void armarTurbina()
{
  fan.writeMicroseconds(ESC_MIN_US);
  turbina_actual_pct = 0;
  delay(1500); // Tiempo para que el ESC arme y emita su tono de listo
}

void stopTurbina()
{
  fan.writeMicroseconds(ESC_MIN_US);
  turbina_actual_pct = 0;
}

void setTurbina(int target_pct)
{
  if(target_pct < 0) target_pct = 0;
  if(target_pct > 100) target_pct = 100;

  // Rampa suave para evitar caídas de voltaje en la batería
  while(turbina_actual_pct < target_pct)
  {
    turbina_actual_pct += 5;
    if(turbina_actual_pct > target_pct) turbina_actual_pct = target_pct;

    int us = map(turbina_actual_pct, 0, 100, ESC_MIN_US, ESC_MAX_US);
    fan.writeMicroseconds(us);
    delay(20);
  }

  while(turbina_actual_pct > target_pct)
  {
    turbina_actual_pct -= 10;
    if(turbina_actual_pct < target_pct) turbina_actual_pct = target_pct;

    int us = map(turbina_actual_pct, 0, 100, ESC_MIN_US, ESC_MAX_US);
    fan.writeMicroseconds(us);
    delay(10);
  }
}

//Motor Izquierdo

void setMotorIzq(int speed)
{
  if(speed >= 0)
  {
    digitalWrite(motor_izq_dirA, LOW);
    digitalWrite(motor_izq_dirB, HIGH);
  }
  else
  {
    digitalWrite(motor_izq_dirA, HIGH);
    digitalWrite(motor_izq_dirB, LOW);

    speed = -speed;
  }

  if(speed > 255) speed = 255;  

  analogWrite(PWM_motor_izq, speed);
}

//Motor Derecho

void setMotorDer(int speed)
{
  if(speed >= 0)
  {
    digitalWrite(motor_der_dirA, LOW);
    digitalWrite(motor_der_dirB, HIGH);
  }
  else
  {
    digitalWrite(motor_der_dirA, HIGH);
    digitalWrite(motor_der_dirB, LOW);

    speed = -speed;
  }

  if(speed > 255) speed = 255;

  analogWrite(PWM_motor_der, speed);
}

//Configuración de velocidad en los motores
void setMotors(int izq, int der)
{
  digitalWrite(standby, HIGH);

  setMotorDer(der);
  setMotorIzq(izq);
}

//Detener
void stopMotors()
{
  analogWrite(PWM_motor_der, 0);
  analogWrite(PWM_motor_izq, 0);

  digitalWrite(motor_der_dirA, LOW);
  digitalWrite(motor_der_dirB, LOW);

  digitalWrite(motor_izq_dirA, LOW);
  digitalWrite(motor_izq_dirB, LOW);
}


//frenado
void Brake(bool izq, bool der, int brake)
{
  digitalWrite(standby, HIGH);

  if(izq)
  {
    digitalWrite(motor_izq_dirA, HIGH);
    digitalWrite(motor_izq_dirB, HIGH);
  
    analogWrite(PWM_motor_izq, brake);
  }
  if(der)
  {
    digitalWrite(motor_der_dirA, HIGH);
    digitalWrite(motor_der_dirB, HIGH);

    analogWrite(PWM_motor_der, brake);
  }
}

//---------------------FASE ACTIVA---------------------

//modo espera al prender el robot
void esperaCalibracion()
{
  sensores_activos = false;

  while(!sensores_activos)
  {
    digitalWrite(led, HIGH);
    delay(500);

    digitalWrite(led, LOW);
    delay(500);

    if(IrReceiver.decode())
    {
      if(IrReceiver.decodedIRData.decodedRawData == 0xBA45FF00)
      {
        sensores_activos = true;
      }

      IrReceiver.resume();
    }
  }
}

//modo espera para arrancar

void esperarArranque()
{
  robot_activo = false;

  while(!robot_activo)
  {
    digitalWrite(led, HIGH);
    delay(500);

    digitalWrite(led, LOW);
    delay(500);

    if(IrReceiver.decode())
    {
      if(IrReceiver.decodedIRData.decodedRawData == 0xB946FF00)
      {
        robot_activo = true;
      }

      IrReceiver.resume();
    }
  }
}

//secuencia de arranque
void startup()
{
  stopMotors();
  //stopTurbina();

  //botón 1
  esperaCalibracion();

  sb.calibrate();

  //botón 2
  esperarArranque();

  digitalWrite(led, LOW);
  delay(300);

  // Encender turbina y dar impulso inicial a motores
  //setTurbina(TURBINA_POTENCIA);
  setMotors(150, 150);
  delay(50);
}

//---------------------CONTROL DE PD---------------------

uint16_t position = 0;

int proportional = 0;
int derivative = 0;
int power_difference = 0;

int last_proportional = 0;

//Ganancias PD en acción
float KP = 0;
float KD = 0;

int SPEED = 0;

float KP_reg = 0.35;
float KD_reg = 2.5;

//Velocidad
int MAX_SPEED = 145;

//Ganancias en extremos
float KP_turn = 78;
float KD_turn = 1;

int MIN_SPEED = 70;

//Frenos
float frenos_inv = 200;
float frenos = 150;

////---------------------ARRANQUE---------------------

void setup() 
{
  //Setup Puente H TB6612
  pinMode(standby, OUTPUT);

  pinMode(motor_der_dirA, OUTPUT);
  pinMode(motor_der_dirB, OUTPUT);
  pinMode(PWM_motor_der, OUTPUT);

  pinMode(motor_izq_dirA, OUTPUT);
  pinMode(motor_izq_dirB, OUTPUT);
  pinMode(PWM_motor_izq, OUTPUT);

  digitalWrite(standby, LOW);
  stopMotors();

  //Setup Botón físico
  pinMode(boton, INPUT);

  //Setup LED
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  //Setup IR KY-022
  IrReceiver.begin(SensorIR, DISABLE_LED_FEEDBACK);

  //Setup turbina (ESC)
  //fan.attach(turbina, ESC_MIN_US, ESC_MAX_US);
  //armarTurbina();

  startup();
}

void loop() 
{
  //Botón 3 -- Reset
  if(IrReceiver.decode())
  {
    if(IrReceiver.decodedIRData.decodedRawData == 0XB847FF00)
    {
      //Detener el robot y la turbina
      stopMotors();

      sensores_activos = false;
      robot_activo = false;

      //Indicador de reset
      for(int i = 0; i < 6; i++)
      {
        digitalWrite(led, HIGH);
        delay(50);

        digitalWrite(led, LOW);
        delay(50);
      }

      //stopTurbina();
      
      IrReceiver.resume();

      //Regreso al inicio
      startup();

      last_proportional = 0;
      
      return;
    }

    IrReceiver.resume();
  }


  if(!robot_activo)
  {
    stopMotors();
    //stopTurbina();
    digitalWrite(led, LOW);

    return;
  }

  //Leyendo la línea
  position = sb.getPosition();

  //Error proporcional
  proportional = ((int)position) - 850;

  //Derivada
  derivative = proportional - last_proportional;
  last_proportional = proportional;

  if(position < 600 || position > 1100)
  {
    KP = KP_turn;     //Estas ganancias son utilizadas al estar en los extremos del sensor
    KD = KD_turn;
    SPEED = MIN_SPEED;
  }
  else
  {
    KP = KP_reg;      //Estas se usan mientras estamos en recta
    KD = KD_reg;
    SPEED = MAX_SPEED;
  }

  //Control PD
  power_difference = (proportional * KP)+(derivative * KD);

  //Limitar la correción
  if(power_difference > SPEED)
  {
    power_difference = SPEED; 
  }
  else if(power_difference < -SPEED)
  {
    power_difference = -SPEED;
  }

  //Control de motores
  if(power_difference < 0)
  {
    setMotors(SPEED, SPEED + power_difference);
  }
  else
  {
    setMotors(SPEED - power_difference, SPEED);
  }

  //Correción del extremo izquierdo
  if(position <= 300)
  {
    setMotors(frenos, -frenos_inv); 
  }

  //Correción del extremo derecho
  if(position >= 1400)
  {
    setMotors(-frenos_inv, frenos);
  }
}