#define   zeroCrossPin     2 // INT0
#define   TcPin           A0 // PTC
#define   acPwmPin         4 // Control AC

#define   LEDPIN           5 // LED

//        LCDSCKPIN       A5 // I2C SCLK
//        LCDDINPIN       A4 // I2C DATA
#define   LCDRESETPIN     A3 // LCD reset
#define   LCDBACKLIGHTPIN A2 // LCD backlight pin

#define   INA              8 // Encoder A pin
#define   INB              7 // Encoder B pin
#define   BUTTON           6 // button

/* AC Phace control
 * (50Hz)-> 1/50=20ms -> 1 cycle,  1/2 cycle: 10ms
 * 10ms=10000us
 *
 * 127 steps/cycle:
 *  (10000uS)/127 steps = 78 uS/Steps
 * 255 steps/cycle:
 * (10000uS)/255 steps = 39 uS/Steps
*/
#define   STEPS           255
#define   freqStep        39

/* Filtro digital
 * valor = (X*ValorAnt + valor)/(X+1);
 * X: varilble utilizada por el flitro
 *        X=2^n - 1, 'n' cuanto mas grande mas inmune al ruido
*/
#define   X               31

//#define DEBUG 1
