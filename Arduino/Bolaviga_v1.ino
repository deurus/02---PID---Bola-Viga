/***************************************************************
* Desarrollado por Garikoitz Martínez [garikoitz.info] [06/2022]
* https://garikoitz.info/blog/?p=1301
***************************************************************/
/***************************************************************
* Librerías
***************************************************************/
#include <Wire.h>
#include <PID_v1.h>
#include "Adafruit_VL53L0X.h"
#include <Servo.h>
/***************************************************************
* Variables
***************************************************************/
Servo myservo;
const int Interruptor = 12;
unsigned long previousMillis = 0; 
int Ts = 50, pos = 0;
unsigned long contador = 0; 
//Control PID
boolean modo = false;                       //false=auto true=manual
float tmp=0.0, tmp_ant=0.0, OP=0, angulo=0.0, angulo_ant=0.0, media=0.0;
double SetpointP, InputP, OutputP;
/***************************************************************
* Sintonías calculadas (Relé --> Ku=0.99 y Tu=4.6 segundos)
***************************************************************/
//double Kc=0.59, Ki=0.26, Kd=0.34;   //Z-N classic PID
//double Kc=0.69, Ki=0.38, Kd=0.48;   //Z-N Pessen Integral Rule 
//double Kc=0.33, Ki=0.14, Kd=0.49;   //Z-N some overshoot
//double Kc=0.20, Ki=0.09, Kd=0.30;   //Z-N no overshoot
//double Kc=0.33, Ki=0.07, Kd=0.15;    //Farrington
//double Kc=0.49, Ki=0.11, Kd=0.57;    //Farrington
//double Kc=0.53, Ki=0.12, Kd=0.49;    //McAvoy and Johnson
//double Kc=0.25, Ki=0.07, Kd=0.28;    //Atkinson and Davey (20% overshoot)
//double Kc=0.99, Ki=0.43, Kd=0.57;    //Pettit and Carr (Underdamped)
//double Kc=0.66, Ki=0.14, Kd=0.50;    //Pettit and Carr (Critically damped)
//double Kc=0.49, Ki=0.07, Kd=0.38;    //Pettit and Carr (Overdamped)
//double Kc=0.44, Ki=0.16, Kd=0.38;    //Tinham (Less than quarter decay ratio response)
//double Kc=0.74, Ki=0.26, Kd=0.34;    //Corripio (Quarter decay ratio)
//double Kc=0.62, Ki=0.27, Kd=0.24;    //ABB (P+D tuning rule)
//double Kc=0.45, Ki=0.04, Kd=0.33;    //Tyreus and Luyben
//double Kc=0.33, Ki=0.14, Kd=0.19;    //Yu (Some overshoot)
//double Kc=0.20, Ki=0.09, Kd=0.11;    //Yu (No overshoot)
//double Kc=0.74, Ki=0.26, Kd=0.34;    //Smith
//double Kc=0.99, Ki=0.34, Kd=0.45;    //Alfaro Ruiz (Quarter decay ratio)
//double Kc=1.65, Ki=0.57, Kd=0.76;    //Alfaro Ruiz (Quarter decay ratio)
//double Kc=0.39, Ki=0.09, Kd=0.29;    //Lloyd (Non self-regulating processes)
//double Kc=0.59, Ki=0.26, Kd=0.33;    //NI Labview (Quarter decay ratio)
//double Kc=0.25, Ki=0.11, Kd=0.14;    //NI Labview (Some overshoot)
//double Kc=0.15, Ki=0.06, Kd=0.08;    //NI Labview (Little overshoot)
//---------------------------------------------------------------
double Kc=0.45, Ki=0.1, Kd=0.35;      //Tyreus and Luyben mod
//---------------------------------------------------------------
PID myPID(&InputP, &OutputP, &SetpointP, Kc, Ki, Kd,P_ON_E, REVERSE);//PI-D
//PID myPID(&InputP, &OutputP, &SetpointP, Kc, Ki, Kd,P_ON_M, REVERSE);  //I-PD
//
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
/***************************************************************
* SETUP
***************************************************************/
void setup() 
{ 
  Wire.begin();
  if (!lox.begin()) {
    Serial.println(F("Error al conectar al sensor VL53L0X"));
    while(1);
  }
  lox.startRangeContinuous();
  myservo.attach(3); // asignamos el pin al servo.
  Serial.begin(9600);
  myPID.SetOutputLimits(0, 100);
  SetpointP = 70;
}
/***************************************************************
* BUCLE PRINCIPAL
***************************************************************/
void loop() 
{
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    //media =  measure.RangeMilliMeter;
    media = Rolling_avg(measure.RangeMilliMeter);
    media = media * 0.1; //Pasamos a cm
  }
  if (millis() - previousMillis > Ts) //Bucle de control cada Ts milisegundos
  {
    previousMillis = millis();
    contador +=1;
    if (digitalRead(12) == HIGH){
      modo=true;                              //Modo Manual
      myPID.SetMode(MANUAL);
    }else{
      modo=false;                             //Modo Automático
      myPID.SetMode(AUTOMATIC);
    }
      InputP = mapf(media, 2, 26, 0, 100);    //PV en cm a porcentaje
      //InputP = media;
    /***************************************************************
    * Modo MANUAL - Potenciómetro ---> Ángulo Servo
    ***************************************************************/
    if (modo==true){
      //----------------------
      //   Método del Relé
      //----------------------
      if (1){
         if (InputP <60){
          pos=950;                       //arriba -> bola hacia la izq
          myservo.writeMicroseconds(pos);  
         }else if (InputP >60){     
          pos=1475;                       //abajo -> bola hacia la dcha
          myservo.writeMicroseconds(pos);  
         }
         OP = mapf(pos, 850, 1575, 0, 100);
      }
      //----------------------
      //     Potenciómetro 
      //----------------------
      if (0){
        SetpointP=0;
        tmp = analogRead(A1);
        tmp = mapf(tmp, 1023.0, 0.0, 850, 1575);        // escalamos potenciometro al servo
        if(abs(tmp-tmp_ant)>1){
          angulo = tmp;
        }
        tmp_ant=angulo;
        OP = mapf(tmp,850, 1575, 0, 100);
        if (angulo>=850 || angulo<=1575) myservo.writeMicroseconds(angulo);
      }   
      /***************************************************************
      * DEBUG PUERTO SERIE (Para Arduino COM Plotter)
      ***************************************************************/
      Serial.print("#");              //Char inicio
      Serial.print(0);                //SP
      Serial.write(" ");              //Char separador
      Serial.print(InputP,0);         //PV InputP
      Serial.write(" ");              //Char separador
      Serial.print(OP,0);             //OP
      Serial.println();
      }
      /***************************************************************
      * Modo AUTOMÁTICO - PID
      ***************************************************************/
      if (modo==false){
    if (0){  //SetPoint mediante Potenciómetro
      tmp = analogRead(A1);
      tmp = mapf(tmp, 0.0, 1023.0, 0, 100);
      if(abs(tmp-tmp_ant)>4){
        SetpointP = tmp;
      }
      tmp_ant=SetpointP;
    }
    if (1){
      // SetpointP automatizado
      if (contador < 1000) SetpointP = 70;  
      if (contador >= 1000 && contador < 2000) SetpointP = 60; 
      if (contador >= 2000 && contador < 3000) SetpointP = 50; 
      if (contador >= 3000 && contador < 4000) SetpointP = 60; 
      if (contador > 4000) contador = 0;
    }
        
        angulo = mapf(OutputP, 0, 100, 850, 1575);
    
        if (abs(SetpointP-InputP)>5){   //Banda muerta del 5%
          myPID.Compute();
          myservo.writeMicroseconds(angulo);
          angulo_ant = angulo;  
        }else{
          myservo.writeMicroseconds(angulo_ant); 
        }
    
        /***************************************************************
        * DEBUG PUERTO SERIE (Para Arduino COM Plotter)
        ***************************************************************/
        Serial.print("#");                //Char inicio
        Serial.print(SetpointP,0);        //SP
        Serial.write(" ");                //Char separador
        Serial.print(InputP,0);           //PV
        Serial.write(" ");                //Char separador
        Serial.print(OutputP,0);          //OP
        Serial.println();  
      }
  }//millis            
}//Loop
/***************************************************************
* FUNCIONES
***************************************************************/
//Función MAP adaptada a punto flotante.
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//High-pass Filter
    //A low α will be very slow to rapid input changes and take many samples into account. 
    //A high α will be fast, but average over fewer samples. You can look at α as kind of a cutoff frequency in a low-pass filter.
    //OP = HPS(tmp,0.1,EMA_s_pot);
    //OP = tmp-EMA_s_pot;
double HPS(double sensorValue, float EMA_a, int EMA_S) {
    return (EMA_a*sensorValue) + ((1-EMA_a)*EMA_S);
}
//Función Promedio
float Rolling_avg(float value) {
  const byte nvalues = 10;             // Moving average window size
  static byte current = 0;            // Index for current value
  static byte cvalues = 0;            // Count of values read (<= nvalues)
  static float sum = 0;               // Rolling sum
  static float values[nvalues];
  sum += value;
  if (cvalues == nvalues)
    sum -= values[current];
  values[current] = value;          // Replace the oldest with the latest
  if (++current >= nvalues)
    current = 0;
  if (cvalues < nvalues)
    cvalues += 1;
  return sum/cvalues;
}
