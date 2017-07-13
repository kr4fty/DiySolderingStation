/*
 * Estacion de soldado hecho sobre un ATMEGA328p utilizando un Lapiz compatible
 * con la estacion HAKKO 907 (con ESD)
 *
 * Autor:
 *       Tapia Favio ( technicc(at)gmail.com )
*/

#include <Arduino.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <ST7558.h>
#include <TimerOne.h>
#define  LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>
#include <EncoderPCI.h>
#include <EEPROM.h>
#include "config.h"

double   setPoint,
         input,
         output,
         aggKp=50,
         aggKi=0.5,
         aggKd=0.01,
         consKp=10,
         consKi=0.5,
         consKd=0.001;
volatile bool     zc;
volatile uint16_t dimtime;
volatile uint8_t  cont;

unsigned long nextCheck, nextCheckLed, timePush, timeChange;
uint16_t oldAdc, adc;
uint16_t newPos, oldPos;
uint16_t oldSetPoint;
uint8_t  uNt, dNt, cNt, uOt, dOt, cOt;
bool     enabled=false, oldEnabled, firstTime=true, BoN;
bool     ledSate;

#ifdef DEBUG
  unsigned long start, stop;
#endif

union {
  uint16_t setPoint_ee;
  uint8_t sp_uint8[2];
}_eeprom;

PID      myPID(&input, &output, &setPoint, consKp, consKi, consKd, DIRECT);
ST7558   lcd(LCDRESETPIN);
Encoder  myEnc(INA, INB);

void setup() {
  // put your setup code here, to run once:
  pinMode(TcPin,           INPUT);
  pinMode(zeroCrossPin,    INPUT);
  pinMode(acPwmPin,        OUTPUT);
  pinMode(LCDBACKLIGHTPIN, OUTPUT);
  pinMode(BUTTON,          INPUT);
  pinMode(LEDPIN,          OUTPUT);

  myPID.SetSampleTime(50);
  myPID.SetOutputLimits(0, STEPS);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  enableInterrupt(zeroCrossPin, zeroCrossInterrupt, FALLING);
  Timer1.initialize(freqStep);
  Timer1.attachInterrupt(dim_check, freqStep);

  lcd.init();
  lcd.cp437(true);
  lcd.setContrast(75);
  digitalWrite(LCDBACKLIGHTPIN, HIGH);
  printBase();

/*
 * Recupero la ultima temperatura seteada. La misma se guarda en la EEPROM
*/
  _eeprom.sp_uint8[0]= EEPROM.read(0);
  _eeprom.sp_uint8[1]= EEPROM.read(1);
  if(_eeprom.setPoint_ee>500)
    _eeprom.setPoint_ee = 300;

  myEnc.write(_eeprom.setPoint_ee);

  #ifdef DEBUG
    Serial.begin(9600);
  #endif
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t      gap, newTemp=0;
  bool          tuningState=false;       // 0: conservative, 1: agresive
  bool          button;

/* Inicio de seleccion de proceso
 *
 *   si es la primera que se ejecuta o cancelo la tarea previa, entonces:
 *      - mueva el encoder para seleciona la temperatura deseada
 *      - y con un pulso corto se comienza el proceso de calentar la punta
 *      - un pulso largo no tiene efecto alguno aca
 *   si el sistema ya estaba funcionando y con una temperatura setada, entonces
 *      - un pulso corto PAUSA/DETIENE el calentamiento de la punta
 *      - con solo mover el encoder, el setPoint se setea automaticamente a la
 *        posicion
 *   y si el sistema ya estaba seteado la temperatura pero pausado/detenido:
 *      - un pulso corto REANUDA el proceso
 *      - un pulso largo CANCELA el proceso
 */

  newPos = myEnc.read();
  button = digitalRead(BUTTON);
  if(!button){
    timePush=millis();
    while(!button)
      button = digitalRead(BUTTON);
    if(millis()<(timePush+200) || firstTime){
      if(firstTime){        // Solo se ejecuta cuando arranca o cuando se
        firstTime = false;  // cancela el proceso actual

        _eeprom.setPoint_ee = newPos;
        EEPROM.write(0, _eeprom.sp_uint8[0]);
        EEPROM.write(1, _eeprom.sp_uint8[1]);

        setPoint = newPos;
        enabled = true;
        nextCheck = millis()+500;
        nextCheckLed = nextCheck;

        #ifdef DEBUG
          Serial.println("Iniciando");
        #endif
      }
      else
        enabled=not enabled;
    }
    else if(millis()>=(timePush+200)) {
      if(!enabled){ // Cancelo el proceso actual
        firstTime = true;
        oldPos--;  // Para evitar que quede el texto en blanco sobre negro
                   // Como la condicion para actualizar el display es que
        dOt--;     // cambien los vlores actuales en comparacion a los guardados
        cOt--;     // entonces fuerzo a que sean distintos. De este modo voy a
        uOt--;     // mostrar el estado de la temperatura como del setPoint
        printBase();
      }
    }
  }
  else{
    if(newPos!=oldPos){
      _eeprom.setPoint_ee = newPos;
      EEPROM.write(0, _eeprom.sp_uint8[0]);
      EEPROM.write(1, _eeprom.sp_uint8[1]);

      setPoint = newPos;
    }
  }// fin seleccion de proceso

  // control PID
  adc = analogRead(TcPin);
  adc = (float)((X*oldAdc + adc)/(X+1));      // Filtro digital pasa bajos
  oldAdc = (uint16_t)adc;

  if(adc<338) // es para hacer un estimativo a temperaturas menores de 150ºc
    input = map(oldAdc, 172, 338, 0, 150);  // entre 0 y 150 ºc
  else        // a partir de 150ºc el ptc tiene mayor precision en la medicion
    input = map(oldAdc, 338, 585, 150, 350); // entre 150 y 350 ºc
  if(input<0)
    input = 0;

  newTemp = (uint16_t)input;

/*
 *  Descompongo newTemp en sus digitos, centena-decena-unidad, para lograr una
 *  mayor velocidad en la actualizacion del display. Resulta mas rapido imprimir
 *  el digito que cambio en lugar de todo el numero.
 *   ejemplo:
 *           si newTemp = 359
 *                           => cNT=3, dNt=5 y uNt=9
*/
  cNt = (uint8_t)(newTemp/100);               // centena de newTemp
  dNt = (uint8_t)((uint16_t)newTemp%100)/10;  // decena  de newTemp
  uNt = (uint8_t)((uint16_t)newTemp%10);      // unidad  de newTemp

/*
 * Una vez que se seleciono el setPoint y se dio arranque al proceso, el PID
 * se encargara de mantener la temperatura lo mas estable posible. Solo se
 * ejecuta si el proceso NO esta pausado/detenido.
*/
  if(enabled){
    gap = abs(setPoint-input);
    if(gap<5){
      if(tuningState!=0){
        myPID.SetTunings(consKp, consKi, consKd);
        tuningState = 0;
      }
    }
    else{
      if(tuningState!=1){
        myPID.SetTunings(aggKp, aggKi, aggKd);
        tuningState = 1;
      }
    }
    myPID.Compute();
  }
  else
    output = 0;
  // fin control PID

/*
 * El setPoint esta seteado sin importar so se esta ejecutando o esta detenido,
 * por lo que llegado a un determinado margen de temperaturas, el LED comenzara
 * a parpadear, mas o menos rapido, depdendiendo de que tan cerca este de la
 * temperatura fijada.
*/
  if(!firstTime){
    // si esta dentro del rango (±15), hago parpadear el led
    if(newTemp<=setPoint-15){
      digitalWrite(LEDPIN, LOW);
      timeChange = 0;
    }
    else{
      if(abs(setPoint-newTemp)<=5)
        timeChange = 150;
      else{
         if(abs(setPoint-newTemp)<15)
          timeChange = 600;
        else{
          digitalWrite(LEDPIN, HIGH);
          timeChange = 0;
        }
      }
      if(millis()>=nextCheckLed && timeChange>0){
        ledSate = not ledSate;
        digitalWrite(LEDPIN, ledSate);

        nextCheckLed = millis()+timeChange;
      }
    }
    if(millis()>nextCheck){
      nextCheck += 500;
      // Muestro el setPoint parpadeando si no modifico la pos del encoder
      BoN = not BoN;
    }
  }

  #ifdef DEBUG
   start = millis();
  #endif

  updateDisplay();

  #ifdef DEBUG
    stop = millis();
    Serial.println((stop-start));
  #endif
}
// Solo se imprime la primera vez o cuando se ha cancelado el proceso
void printBase(){
  lcd.clearDisplay();
  lcd.setTextColor(BLACK, WHITE);
  lcd.setCursor(0, 0);
  lcd.setTextSize(2);
  lcd.print(F("T: "));

  lcd.setCursor(72, 20);
  lcd.print((char)248);
  lcd.print(F("C"));

  lcd.display();
}
/*
 * Se buscaba imprimir solo el caracter que se modifico, logrando de esta manera
 * acelerar el refresco de la pantalla.
 * Si en cambio se borra e imprime en cada ciclo de loop(), el mismo tomaba unos
 * 500ms debido a que el lcd es i²c lo que de buenas a primeras ya es un
 * protocolo muy lento en comparacion a SPI, por ejemplo.
*/
void updateDisplay()  // se muestra en el display
{
  if(firstTime && !enabled)
    BoN = true;
  lcd.setCursor(2*6*2, 0);
  lcd.setTextSize(2);
  // si el proceso esta seteado y se movio el setPoint a un nuevo valor
  if(newPos!=oldPos || enabled){
    lcd.setTextColor(BoN, not BoN);
    if(newPos<100)
      lcd.print(F(" "));
    lcd.print(newPos);
    oldPos = newPos;
  }
  else if(!firstTime && oldEnabled && !enabled){
    lcd.setTextColor(BLACK, WHITE);
    if(newPos<100)
      lcd.print(F(" "));
    lcd.print(newPos);
    lcd.print(F("  "));
  }

  // el proceso esta seteado pero pausado
  lcd.setCursor(5*6*2, 0);
  if(!firstTime && !enabled){
    if(BoN){
      lcd.print((char)222);   // '|'
      lcd.print((char)222);   // '|'
    }
    else{
      lcd.setTextColor(BLACK, WHITE);
      lcd.print(F("  "));
    }
  }
  else if(!firstTime && enabled && oldEnabled){
    lcd.setTextColor(BLACK, WHITE);
    lcd.print(F(" "));
    lcd.print((char)175);
  }

  lcd.display();

  // Si cambio la temperatura entonces muestro el nuevo valor
  if( (cNt!=cOt) || (dNt!=dOt) || (uNt!=uOt) ){
    lcd.setTextSize(4);
    lcd.setTextColor(BLACK, WHITE);

    if(cOt!=cNt){
      lcd.setCursor(0*6*4, 20);
      if(cNt==0)
        lcd.print(F(" "));
      else
        lcd.print(cNt);
      cOt = cNt;
    }
    if(dOt!=dNt){
      lcd.setCursor(1*6*4, 20);
      if(cNt==0 && dNt==0)
        lcd.print(F(" "));
      else
        lcd.print(dNt);
      dOt = dNt;
    }
    if(uOt!=uNt){
      lcd.setCursor(2*6*4, 20);
      lcd.print(uNt);
      uOt = uNt;
    }
    lcd.display();
  }

  // Siempre muestro el setPoint y el estado solo si el proceso activo
  if( ((oldEnabled!=enabled)||(oldSetPoint!=setPoint)) && !firstTime){
    lcd.setTextSize(1);
    lcd.setCursor(0, 55);
    lcd.print((uint16_t)setPoint);
    lcd.setTextColor(BLACK, WHITE);
    lcd.print(F(" "));
    lcd.print(enabled?F("Encendido"):F("Detenido!"));
    lcd.display();

    oldEnabled=enabled;
    oldSetPoint=setPoint;
  }
  #ifdef DEBUG
    lcd.setTextSize(1);
    lcd.setCursor(13*6, 55);
    lcd.print(oldAdc);
    lcd.display();
  #endif
}

// ISR's
/*
 * Encargada de detectar cuando la onda senoidal CRUZA POR CERO
 * Recordar que un TRIAC una vez que se activa, la misma conduce hasta que entre
 * T1 y T2 haya 0Vac, osea, la onda senoidal cuando cruza por cero lo desactiva
 * automaticamente. Por lo que en cada cruce se debera activar pasado un tiempo
 * de espera "dimtime".
*/
void zeroCrossInterrupt()
{
  int dimming;

  digitalWrite(acPwmPin, LOW);
  if(output>=0){
    dimming = STEPS - output; // Invierto por que una vez activado el triac no
                              // se desactiva hasta el proximo cruce por cero
    if(dimming==0){
      zc = false;
      digitalWrite(acPwmPin, HIGH);
    }
    else if(0<dimming<STEPS){
      zc = true;
      dimtime = (freqStep*dimming);
    }
    else if(dimming==STEPS){
      zc = false;
    }
  }
}
/*
 * Encargada de encender el triac tras la espera de tiempo "dimtime", simulando
 * ser el control por PWM de continua, en este caso es un control de fase.
 * Solo actua si "dimtime" esta entre 0<dimtime<255. Los casos particulares son
 * cuando es 0:siempre encendido, 255: siempre apagado.
*/
void dim_check() {
  if(zc == true){
    if(cont>=dimtime) {
      digitalWrite(acPwmPin, HIGH); // turn on light
      cont=0;                       // reset time step counter
      zc = false;                   // reset zero cross detection
    }
    else
      cont++;                       // increment time step counter
  }
}
