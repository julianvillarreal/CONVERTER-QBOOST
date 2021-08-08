#include <TimerThree.h>
#include <TimerFour.h>
#include <PWM.h>

// Declaration of variables
const int PWM = 11, VoutADC =A1, VrefADC =A3, VIinADC =A0, VinADC =A2,  Led =A4;
float Kp = 0.7371, Ki = 6.168, Kd = 0.02202, Tm = 0.005, R = 200;
float Kp1 = 0.01177, Ki1 = 0.04564, Kd1 = 0.00076;
float  E = 0, E_anterior = 0, E_anterior_anterior = 0,U = 0, U_anterior = 0, Vout = 0, Vref = 0, Iin = 0, Vin = 0;
float Power = 0, Vo = 0, Duty_Copy = 0;
long F_Inversor = 60000;
int Inicio = 1;

// Declaration of Volatile Variables to share information on SRI functions
volatile unsigned long UCopy = 0;
volatile double VIinCopy = 0;
volatile double VoutCopy = 0;
volatile double VinCopy = 0;
volatile unsigned long F_InversorCopy = 0;

void setup() {
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,OUTPUT);
  pinMode(13,OUTPUT);


  Timer3.initialize(5000); // Set Timer 3 for an interrupt every 5ms
  Timer3.attachInterrupt(Control); // Execute the Control function every 5ms

  Timer4.initialize(4000000); // Set Timer 4 for an interruption every 4s
  Timer4.attachInterrupt(F_Inversor_F); // Execute the function F_Inversor_F every 4s

  Timer5_Initialize(); // Initialize Timer 5
  //Timer5_SetFrequency(150000); // Set PWM pin-9 and pin-10 to 150 kHz using Timer 2

  arduino_FastPWM(); // Set the PWM frequency of pins 11 and 12 to 62.5kHz using Timer 1

  Serial.begin(250000); // Initialize Serial communication


}

void loop() {
  // Declaration of auxiliary variables
  unsigned int Duty;
  float VIin = 0;
  float Vi = 0;
  float Pi = 0;
  float IinAMP =0;
  long Frecuencia_Inversor = 0;

  noInterrupts(); // Turn off interrupts to use shared variables
  Duty = UCopy;
  VIin = VIinCopy;
  Vo = VoutCopy;
  Vi = VinCopy;
  Frecuencia_Inversor  = F_InversorCopy;
  interrupts(); // Turn on interruptions

  Duty_Copy = Duty; // Copies the duty cycle of the DC-DC converter
  analogWrite(PWM,Duty); // Adjusts the duty cycle of the DC-DC converter

  // Limits the switching frequency value of the inverter
  if (Frecuencia_Inversor > 150000){
  Frecuencia_Inversor = 150000;
  }
  if (Frecuencia_Inversor < 60000){
  Frecuencia_Inversor = 60000;
  }

  Timer5_SetFrequency(Frecuencia_Inversor); // Sets the switching frequency of the inverter

  // Adjusts the duty cycle of the inverter switching signal
  pwmWrite(44, 127);
  pwmWrite(45, 135);

  // Estimate the current and power of input and output of the converter
  IinAMP = ((VIin*4.9618)-12.864);
  float Iout  = (0.91*(Vi*11.855*IinAMP))/(Vo*79.759);
  Power = 0.91*(Vi*11.855*IinAMP);
  Pi = Vi*11.855*IinAMP ;

  // Transmits by serial the values of Duty Cycle, Power input and output
  Serial.print("Duty = ");
  Serial.print(Duty);
  Serial.print("  Pi[W]= ");
  Serial.print(Pi);
  Serial.print("  Po[W]= ");
  Serial.print(Power);
  Serial.println();

}

//********Frequency setting of PWM pins 11-12 at 62.5kHz********
void arduino_FastPWM(){
  // Selects a PWM frequency of 62,500 Hz on the PWM pins associated with the Timer1
  #if defined(__AVR_ATmega328P__)
    analogWrite(9,127); // let Arduino do PWM timer and pin initialization
  #elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    analogWrite(11,100); // // let Arduino do PWM timer and pin initialization
  #else
    *** wrong board ***
  #endif
    // Set fast-PWM at highest possible frequency
    TCCR1A = _BV(COM1A1) | _BV(WGM10);
    TCCR1B = _BV(CS10) | _BV(WGM12);
}

//********ADC measurement********
  float SenseADC(int Pin){
    // Averages 10 values
    int v1  = analogRead(Pin);
    int v2  = analogRead(Pin);
    int v3  = analogRead(Pin);
    int v4  = analogRead(Pin);
    int v5  = analogRead(Pin);
    int v6  = analogRead(Pin);
    int v7  = analogRead(Pin);
    int v8  = analogRead(Pin);
    int v9  = analogRead(Pin);
    int v10 = analogRead(Pin);
    int v11 = analogRead(Pin);
    int v12 = analogRead(Pin);
    int v13 = analogRead(Pin);
    int v14 = analogRead(Pin);
    int v15 = analogRead(Pin);
    float Vsense =(v1+v2+v3+v4+v5+v6+v7+v8+v9+v10+v11+v12+v13+v14+v15)/15;
    float ADCout = (Vsense*5.01/1023);
  return ADCout;
  }

//********DC-DC Converter Control - Interrupt 1********
void Control(void){
  // Measurement of external variables
  Vout  = SenseADC(VoutADC);
  Vref  = SenseADC(VrefADC);
  Iin   = (analogRead(VIinADC)*5.08/1023);
  Vin   = (analogRead(VinADC)*5.08/1023);
  // Calculate and measure the error
  E_anterior_anterior = E_anterior;
  E_anterior = E ;
  U_anterior = U;
  E = ((Vref - Vout)*255)/5;
  if (E>0 && E <=2){
  E =0;
  }
  if (E>=-1 && E <0){
  E =0;
  }

  // Calculate the duty cycle value using the difference equation
  U =  U_anterior + (Kp*1 + Ki*Tm*1 + Kd*1)*E - Kp*E_anterior*1 - Kd*1*E_anterior_anterior;

  // Limits the duty cycle value of the converter
  if (U >= 158 && E > 0){
    E = E_anterior;
    U = U_anterior;
    digitalWrite(Led,HIGH);
  }else {
    digitalWrite(Led,LOW);
  }
  if (U <= 50 && E < 0){
    E = E_anterior;
    U = U_anterior;
  }

  // copy the variables of the function to variables of volatile type for external use
  UCopy = U;
  VIinCopy = Iin;
  VoutCopy = Vout;
  VinCopy = Vin;

  }

//********Inverter Frequency - Interrupt 2********
void F_Inversor_F(void){

  float V_out = Vo*79.759;

  // Evaluates Power, Output Voltage and Duty of converter DC-DC
  if (Power <= 205 && V_out < 207 && V_out > 195 && Duty_Copy < 156 ){
   F_Inversor = F_Inversor + 2000; // Increase the frequency of the inverter by 2kHz
  }
  if (Duty_Copy >= 156 || V_out < 198){
  F_Inversor = F_Inversor - 2000; // Decrease the frequency of the inverter by 2kHz
  }

  // Limits the value of the inverter frequency
  if (F_Inversor > 152000){
  F_Inversor = 150000;
  }
  if (F_Inversor < 58000){
  F_Inversor = 60000;
  }
  // Copies the frequency value to a volatile type variable for external use
  F_InversorCopy = F_Inversor;

}
