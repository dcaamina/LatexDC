//Bibliotecas
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Filter.h"  //Filtro
#include "MegunoLink.h"
#include "MedianFilterLib.h"  //Filtro de Mediana
#include "MeanFilterLib.h"  //Filtro de Media
#include <TimerOne.h> //Libreria PWM
#include "Adafruit_Si7021.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> //sensor THP
#include <Adafruit_ADS1015.h> //Convertidor analogico digital
#include <Custom_PID_Tunel.h> //Libreria modificada por Cristian Yapura

Adafruit_Si7021 sensor = Adafruit_Si7021();//DHT21
Adafruit_BME280 bme; // //BME280
Adafruit_ADS1115 ads;//ADS115

//Variables utilizadas en los filtros
MeanFilter<float> meanFilter(20);
MedianFilter<float> medianFilter(40);

//inicializacion de variables
#define Fallaext 5
float t1 = 0 , h1 = 0, p2 = 0; //variables utilizadas en THP
float presionSF2 = 0, presionSF21, ADCFilterM = 0, ADCFilterM1; //Variab
float velocidadA = 0, velocidadB = 0; //////Variables velocidad
float adc0 = 0, offsetp2 = 0, offsetdf = 0, offsetdf1 = 2.592;
float entrada1[30], entrada2[30];
float Inref1, In = 0;
float VelRef = 0, VelRef1 = 0; //PID
int entrada[7], Inref, len, inc = 0, inc1 = 1, contserie = 0, contador = 0;
long tiemporead = 0, vtiempoant, tiempoautomatico; 
long time1, time2, time3, tiemporetardo;
double output = 0; //PID
boolean paro, BOT = 0, BOT2 = 0, step1 = 0, Estado = 0, Errorvar = 0;
boolean pulsador, Estado1 = 0, Errorvar1 = 0, Estadoant = 0, Errorvarant = 0;
boolean EnableAi1 = 0, RUNSTOP = 0, Control = 0, FallaExterna = 0;
boolean Resetfalla = 0, Encendido;
bool ControlAutomatico = 0, cambio = 0, cambio1 = 0, terminoautoma = 0;
String data; //Guardo los datos del buffer para utilizar en funcion princ.

///Constantes para calculo de densidad//
float a0 = 0.00000158123, a1 = -0.000000029331, a2 = 0.00000000011043;
float b0 = 0.000005707, b1 = -0.00000002051;
float c0 = 0.00019898, c1 = -0.000002376;
float d = 0.0000000000183, e = -0.00000000765;
float A = 0.000012378847, B = -0.019121316;
float C = 33.93711047, D = -6343.1645, alfa = 1.00062;
float beta = 0.0000000314, gamma = 0.00000056;
float T1; //temperatura en absoluto
float fpt, psv, xv, Z, den;

//PID
PID pid(0.6846, 0.4183, 0);
float Error1 = 0;
long tiempo = 0;
int pw = 0;
float Cte = 56.88;

void setup() {
  Serial.begin(115200);
  if (!sensor.begin()) {//SHT21
    Serial.println("Problema sensor - Si7021");
    while (true);
  }
  if (!bme.begin(0x76)) {//BME280
    Serial.println("Problema sensor - BME280");
    while (1);
  }
  ads.setGain(GAIN_ONE);// +/- 4.096V 1bit=2mV   0.125mV
  ads.begin(); //ADS1115
  /////Inicializo Entradas/Salidas
  pinMode(2, INPUT_PULLUP);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(Fallaext, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  ///Inicializo pwm//
  Timer1.initialize(90);
}

void loop() {
  THP();
  difPresion();
  vel_tiempo();
  if (terminoautoma == 1 &(millis() -tiempoautomatico)>=3500){
    cambio_automatico();
  }
  PIDS();
  entradas();
  salidas();
  pulsador = digitalRead(8);
  imprimir_datos();
}


void serialEvent() {
  if (Serial.available() > 0) { // Lectura del puerto mientras sigue abierto
    data = Serial.readStringUntil('\n'); //lectura hasta \n
    int n, n1 = 0, n2 = 0; // Variables para algoritmo de lectura

    for (int i = 0; i <= data.length(); i++) { // Lectura total
      if (data.substring(i, i + 1) == ",") {  // Lectura hasta ","
        if (n1 == 0) {
          entrada[n1] = data.substring(0, i).toInt();
          n1 = n1 + 1;
          n = i + 1;
        }
        else {
          entrada[n1] = data.substring(n, i).toInt();
          n1 = n1 + 1;
          n = i + 1;    // Posicion de la letra final leida + 1
        }
      }
    }
    n = 0;
    ///////////////////STRING A FLOAT////////////
    for (int i = 0; i <= data.length(); i++) { // Lectura
      if (data.substring(i, i + 1) == ",") {  // Lectura hasta ","
        if (n2 == 0) {
          entrada2[n2] = data.substring(0, i).toFloat();
          n2 = n2 + 1;
          n = i + 1;
        }
        else {
          entrada2[n2] = data.substring(n, i).toFloat();
          n2 = n2 + 1;
          n = i + 1;   // Posicion de la letra final leida + 1
        }
      }
    }
    if (entrada[6] != 0) {
      int j = 0;
      for (int i = 7; i <= data.length(); i++) {
        entrada1[j] = entrada2[i];
        j++;
      }
      tiempoautomatico = millis();
      cambio = 1;
      terminoautoma = 1;
      len = entrada[6];
      ControlAutomatico = 1;
    } else {
      terminoautoma = 0;
      cambio = 0;
      ControlAutomatico = 0;
    }

  }
}

/*        LECTURA THP, ESTIMACION DE VELOCIDAD             */    
void THP() {
  t1 = sensor.readTemperature(); //SHT21
  T1 = t1 + 273.15; //T abs
  h1 = sensor.readHumidity(); //SHT21
  p2 = bme.readPressure(); //BME280
}

void difPresion() {
  adc0 = ads.readADC_SingleEnded(0) * 4.096 / 32768; //ADS1115
  presionSF2 = (adc0 - offsetdf1 - offsetp2) * 1000; //Presion
  ADCFilterM = medianFilter.AddValue(presionSF2);
}

void calculo_offset() {
  if (contador < 35) { //contador para descartar primeros valores
    offsetdf = meanFilter.AddValue(adc0 - offsetdf1);
    contador = contador + 1;
  }
  else {
    if (contador == 35) {
      offsetp2 = offsetdf;
      contador = contador + 1;
    }
  }
}

void vel_tiempo() {
  /////////Calculo densidad////////////////////
  fpt = alfa + beta * p2 + gamma * pow(17, 2);
  psv = 1 * exp(A * (pow(T1, 2)) + B * T1 + C + D / T1);
  xv = (h1 / 100) * fpt * psv / p2;
  Z = 1 - (p2 / T1) * (a0+a1*t1+a2*pow(t1, 2) + (b0 + b1 * t1) * 
  xv + (c0+c1*t1)*pow(xv, 2)) +(d+e*pow(xv, 2))*pow((p2/T1), 2);
  den = 0.00348374 * p2 * (1 - 0.378 * xv) / (Z * T1); //densidad
  //////////Calculo velocidad////////////////
  if (ADCFilterM < 0) {
    ADCFilterM1 = 0;
  } else ADCFilterM1 = ADCFilterM;
  if (presionSF2 < 0) {
    presionSF21 = 0;
  } else presionSF21 = presionSF2;
  velocidadA = sqrt((2 * (abs(presionSF21))) / den);  //sin filtro
  velocidadB = sqrt((2 * (abs(ADCFilterM1))) / den);//con filtro
  tiempo = millis();
}


/*        AUTOFUNCIoN             */    
void cambio_automatico() {

  if (cambio1 == 0) {
    entrada[3] = 1; //Encendido = 1;   Habilito Ai1
    if ((millis() - tiempoautomatico) >= 4000) {
      entrada[0] = 1;
    } //RUNSTOP = 1;    Doy Marcha
    if ((millis() - tiempoautomatico) >= 9000) {
      entrada[2] = 1;
      cambio1 = 1;
      inc = 0;
      inc1 = 1;
    }
  }
  else {

    if  (inc1 <= len - 1) {
      if (cambio == 1) {
        Inref1 = entrada1[inc];
        vtiempoant = millis();
        cambio = 0;
      }
      if ((millis() - vtiempoant) >= entrada1[inc1] * 1000) {
        inc = inc + 2;
        inc1 = inc1 + 2;
        cambio = 1;
        tiemporetardo = millis();
      }
    } else {
      entrada[0] = 0; //RUNSTOP = 0;   Apago Motor
      entrada[2] = 0;
      Control = 0;
      if ((millis() - tiemporetardo) > 3000) {
        entrada[3] = 0; //Encendido = 0;  Deshabilito Ai1
        terminoautoma = 0;
        cambio1 = 0;
        cambio = 0;
        ControlAutomatico = 0;
      }
    }
  }
}

/*        ENTRADAS Y SALIDAS             */    
void entradas() {
  ////FISICAS///
  if ((millis() - tiemporead) > 300) {
    Estado1 = !digitalRead(2);
    Errorvar1 = !digitalRead(7);
    tiemporead = millis();
    if (Estado1 == 1 && Estadoant == 1) {
      Estado = 1;
    }
    if (Estado1 == 0 && Estadoant == 0) {
      Estado = 0;
    }
    if (Errorvar1 == 1 && Errorvarant == 1) {
      Errorvar = 1;
    }
    if (Errorvar1 == 0 && Errorvarant == 0) {
      Errorvar = 0;
    }
    Estadoant = Estado1;
    Errorvarant = Errorvar1;
  }
  if (Errorvar == 1) {
    entrada[0] = 0; //RUNSTOP = 0;  Apago Motor
    entrada[3] = 0; //Encendido = 0;  Deshabilito Ai1
    entrada[2] = 0;
    terminoautoma = 0;
    cambio = 0;
    ControlAutomatico = 0;
  }
  /////Serial/////
  RUNSTOP = entrada[0];     //Variable Marcha- Parada
  Inref = entrada[1];       //velref o frec aprox.
  Control = entrada[2];     //Control activado/desacivado
  Encendido = entrada[3];   //ON - OFF . Habilitacion Ai1
  Resetfalla = entrada[4];  //Reset falla
  FallaExterna = entrada[Fallaext]; //Falla externa
}

void salidas() {
  digitalWrite(3,RUNSTOP);  //senal de on off
  digitalWrite(4,Encendido);   //Habilito control por Ai1
  digitalWrite(Fallaext,FallaExterna); //senal que ocurrio una falla
  digitalWrite(6,Resetfalla); //senal para reset de fallas
}


/*        ESCALONES             */    

void escalon() {
  if (tiempo >= 20000 & tiempo <= 40000 & pw <= 250) {
    pw = pw + 2;
  }
  if (tiempo >= 40000 & tiempo <= 50000) { //13+10hz
    pw = 250;
  }
  if (tiempo >= 50000) { //13+20hz
    pw = 500;
  }
  if (tiempo >= 60000) { //13+24hz
    pw = 600;
  }
  if (tiempo >= 70000) { //13+22hz
    pw = 550;
  }
  if (tiempo >= 80000) { //13+18hz
    pw = 450;
  }
  if (tiempo >= 90000) { //13+14hz
    pw = 350;
  }
  if (tiempo >= 100000) { //13hz
    pw = 0;
  }
  Timer1.pwm(9, pw);
}

void escalon2() {
  pw = analogRead(A0);
  Timer1.pwm(9, pw);
}

void escalon22() {
  pw = analogRead(A0);
  pw = map(pw, 0, 1023, 50, 170);
  Timer1.pwm(9, pw);
}

void escalon3() {
  if (tiempo >= 20000 & tiempo <= 33000 ) {
    //pw=220;     //// supongo 17,33 Hz
    pw = 235;   //// supongo 17,33 Hz
  }
  if (tiempo >= 33000 & tiempo <= 45000) { //
    //pw=270;    /////Supongo 19.14 HZ
    pw = 392;  /////Supongo 25 HZ
  }
  if (tiempo >= 45000 & tiempo <= 55000) { //
    //pw=220;  ////Supongo 17.33 Hz
    pw = 235; ////Supongo 17.33 Hz
  }
  if (tiempo >= 55000) { //
    pw = 0;
  }
  Timer1.pwm(9, pw);
}

void escalon4() {
  if (tiempo >= 20000 & tiempo < 35000 ) {
    //pw=325;     //// supongo 21.05Hz
    pw = 193;   //// supongo 21.05Hz
  }
  if (tiempo >= 35000 & tiempo < 55000) { //
    //pw=595;    /////Supongo 32.73 HZ
    pw = 314;   //// supongo 21.05Hz
  }
  if (tiempo >= 55000 & tiempo < 70000) { //
    //pw=325;  ////Supongo 21.05 Hz
    pw = 434;   //// supongo 21.05Hz
  }
  if (tiempo >= 70000 & tiempo < 85000) { //
    //pw=325;  ////Supongo 21.05 Hz
    pw = 531;   //// supongo 21.05Hz
  }
  if (tiempo >= 85000 & tiempo < 105000) { //
    //pw=325;  ////Supongo 21.05 Hz
    pw = 434;   //// supongo 21.05Hz
  }
  if (tiempo >= 105000 & tiempo < 120000) { //
    //pw=325;  ////Supongo 21.05 Hz
    pw = 314;   //// supongo 21.05Hz
  }
  if (tiempo >= 120000 & tiempo < 140000) { //
    //pw=325;  ////Supongo 21.05 Hz
    pw = 193;   //// supongo 21.05Hz
  }
  if (tiempo >= 140000) { //
    pw = 0;
  }
  Timer1.pwm(9, pw);
}

void escalonserial() {
  In = entrada[2];
  //Timer1.pwm(9, pw);
}

void escalonautoma() {
}


/*        PID             */    
void PIDS() {
  if (terminoautoma == 0) {
    In = Inref;
  } else In = Inref1;
  if (Control == 1) {
    if (terminoautoma == 1) {
      VelRef = In;
    } else {
      In = map(In, 0, 32767, 9059, 32767);
      VelRef = In / 1927.47;
    }
    if (VelRef > 17.5) {
      VelRef = 17;
    }
    if (VelRef < 4.7) {
      VelRef = 4.7;
    }
  }
  else if (Control == 0) {
    VelRef1 = map(In, 0, 32767, 9175, 32767); ///9175
    VelRef1 = VelRef1 / 655.3;
    In = map(In, 0, 32767, 7300, 32767); ///9175
    VelRef = In / 655.34;
    if (VelRef >= 50) {
      VelRef = 50;
    }
    if (VelRef <= 11.14) {
      VelRef = 11.14;
    }
    step1 = 0;
  } else VelRef = 0;
  Error1 = Cte * (VelRef - velocidadB);
  if (Control == 1) {
    if (step1 == 0) {
      pid.Initialize(output);
      step1 = 1;
    }
    output = pid.Update(Error1, 55); //Calcula PID c/Error
  }
  else {
    output = VelRef * 19.8; //Out determinada por la frec_aprox
    step1 = 0;
  }
  pw = output;
  Timer1.pwm(9, pw);
}


/*       IMPRIMIR DATOS             */    
void imprimir_datos() {
  Serial.print(velocidadB, 3); Serial.print(";");//Vel del Aire con FMediana
  if (Control == 1) {
    Serial.print(VelRef); Serial.print(";");//vel_ref o frec_aprox
  }
  else {
    Serial.print(VelRef1); Serial.print(";");
  }
  Serial.print(ADCFilterM, 3); Serial.print(";"); //DdPresion -FMediana
  Serial.print(pw); Serial.print(";");       //PWM -- Variable de control
  Serial.print(tiempo); Serial.print(";");
  Serial.print(t1, 3); Serial.print(";"); //temperatura
  Serial.print(h1, 3); Serial.print(";"); //humedad
  Serial.print(p2, 3); Serial.print(";"); //presion
  Serial.print(den, 4); Serial.print(";");    //Densidad
  Serial.print(Control); Serial.print(";");  //Control activado
  Serial.print(Error1); Serial.print(";");//Error:Dif entre v_aire y v_ref
  Serial.print(Estado); Serial.print(";"); //Estado del variador (ON/OFF)
  Serial.print(Errorvar); Serial.print(";");  //error en variador
  Serial.print(ControlAutomatico); Serial.println(";");  //vel automatica ON
  digitalWrite(13, Control);
}
