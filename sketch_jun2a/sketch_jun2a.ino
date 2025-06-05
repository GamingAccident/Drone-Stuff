#include <ESP32Servo.h>
int buttonPin = 2;
int buttonState = 0;
int loopNumber = 1;
int motorPower = 1000;
//int prevButtonState = 0;

Servo servoMotor[4];            // Create an object for each servo
int servoPin[4] = {9,0,0,0}; // ESP32 pins to be used, starting from top right motor clockwise
int motorInput[4]= {0};

int startingDelay = 3000; // Time before loop starts (ms)

int motorUpdateSpeed = 4000;       // Minimum time (μs) between motor updates. 250 times a second, doesnt coincide much with pingSpeed
unsigned long lastMotorUpdate;

const byte        interruptPin = 4;              // Assign the interrupt pin
volatile uint64_t StartValue;                     // First interrupt value
volatile uint64_t PeriodCount;                    // period in counts of 0.000001 of a second
float             Freg;                           // frequency     
char              str[21];                        // for printing uint64_t values
 
hw_timer_t * timer = NULL;                        // pointer to a variable of type hw_timer_t 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // synchs between maon cose and interrupt?

// Digital Event Interrupt
// Enters on falling edge in this example
//=======================================
void IRAM_ATTR handleInterrupt()  {
  portENTER_CRITICAL_ISR(&mux);
      uint64_t TempVal= timerRead(timer);         // value of timer at interrupt
      PeriodCount= TempVal - StartValue;          // period count between rising edges in 0.000001 of a second
      StartValue = TempVal;                       // puts latest reading as start for next calculation
  portEXIT_CRITICAL_ISR(&mux);
}


// Converts unit64_t to char for printing
// Serial.println(uintToStr( num, str ));
//================================================
char * uintToStr( const uint64_t num, char *str ) {
  uint8_t i = 0;
  uint64_t n = num;
  do
    i++;
  while ( n /= 10 );
  
  str[i] = '\0';
  n = num;
 
  do
    str[--i] = ( n % 10 ) + '0';
  while ( n /= 10 );

  return str;
}

void setup() {

  pinMode(buttonPin, INPUT_PULLDOWN);

  Serial.begin(115200);

  // Initialise Motors
  servoMotor[0].attach(servoPin[0],1000,2000);  // Attaches the servos on each ESP32 pin
  servoMotor[0].write(90); // Provides a "neutral" pulse. The ESC won't start without this.

  pinMode(interruptPin, INPUT_PULLUP);                                            // sets pin high
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING); // attaches pin to interrupt on Falling Edge
  timer = timerBegin(1000000);                                                    // this returns a pointer to the hw_timer_t global variable
                                                                                  // 0 = first timer
                                                                                  // 80 is prescaler so 80MHZ divided by 80 = 1MHZ signal ie 0.000001 of a second
                                                                                  // true - counts up
  timerStart(timer);
  
  delay(startingDelay);
}

void loop() {
  motorPower = 1000;
  servoMotor[0].write(motorPower);
  buttonState = digitalRead(buttonPin);

  Serial.print("Loop: ");  Serial.print(loopNumber);  Serial.println("\t");

  if ( buttonState == HIGH ) {

    for (int i=0; i<10; i++) {
      motorPower += 100;
      servoMotor[0].write(motorPower);
      portENTER_CRITICAL(&mux);
      Freg = 1000000.00/PeriodCount;                       // PeriodCount in 0.000001 of a second
      portEXIT_CRITICAL(&mux);
      Serial.println(Freg,2);
      delay(100);
    }
  }
  delay(startingDelay);
  
  loopNumber++;
}