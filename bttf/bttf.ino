volatile int number = 0;                // Testnumber, print it when it changes value,
                                        // used in loop and both interrupt routines
int oldnumber = number;

volatile boolean halfleft = false;      // Used in both interrupt routines
volatile boolean halfright = false;

const int  buttonPin = 4;    // the pin that the pushbutton is attached to
const int ledPin = 13;       // the pin that the LED is attached to
const int pressurePin = A0;
int pressure;

int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button
int run = 1;

int r_motor_n = 11;  //PWM control Motor -
int r_motor_p = 10;  //PWM control Motor +

int lastEnc = 0;
int rate = 0;

unsigned long time1, time2, timetaken;

int limit = 150; //This is the number of ticks the motor will turn, +/- 1 tick accuracy

int startingNumber = 0;

void setup(){
  Serial.begin(9600);
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);                // Turn on internal pullup resistor
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);                // Turn on internal pullup resistor
  attachInterrupt(0, isr_2, FALLING);   // Call isr_2 when digital pin 2 goes LOW
  attachInterrupt(1, isr_3, FALLING);   // Call isr_3 when digital pin 3 goes LOW
 pinMode(r_motor_n, OUTPUT);  //Set control pins to be outputs
  pinMode(r_motor_p, OUTPUT);
  digitalWrite(r_motor_n, LOW);  //set both motors off for start-up
  digitalWrite(r_motor_p, LOW);
  // initialize the button pin as a input:
  pinMode(buttonPin, INPUT);
  // initialize the LED as an output:
  pinMode(ledPin, OUTPUT);
  // initialize serial communication:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  
//  //Timer stuff
//cli();
//TCCR1A = 0;
//TCCR1B = 0;
//TIMSK1 |= (1 << TOIE1);
//TCNT1 = 0x0FDC;
//TCCR1B |= (1 << CS12);
//sei();

//  //END TIMER SETUP
//  
  
  Serial.println("Start");

}

//ISR(TIMER1_OVF_vect)
//{
//Serial.println("Running timer ISR");
//rate = number - lastEnc;
//if(rate > 0) Serial.println(rate);
//lastEnc = number;
//Serial.println("..........");
//digitalWrite(ledPin, !digitalRead(ledPin));
//}

//ISR(TIMER1_COMPA_vect) {//Interrupt at freq of 2 Hz to measure tension rate
//  rate = number - lastEnc;
//  if(rate > 0){
// //   Serial.println("-----------");
//    //Serial.println("Rate is: ");
//    //Serial.println(rate);
//  }
//  lastEnc = number;
//  Serial.println(number);
//  Serial.println(lastEnc);
//  //Serial.println("-----------");  
//  
//}



void loop(){
//digitalWrite(r_motor_p, HIGH);  //Set motor direction, 1 low, 2 high
//    analogWrite(r_motor_n, LOW);
  
  pressure = analogRead(pressurePin);
 // Serial.println(pressure);
  //delay(250);
  
  buttonState = digitalRead(buttonPin);
  if(((buttonState == HIGH) || (pressure >= 700)) && run){
    startingNumber = number; 
    oldnumber = number;
    Serial.println("starting number:");
    Serial.println(startingNumber);
    digitalWrite(r_motor_n, HIGH);  //Set motor direction, 1 low, 2 high
    analogWrite(r_motor_p, LOW);
    while(number <= startingNumber+limit){
      
      time1 = millis();
      
      if(number != oldnumber){              // Change in value ?
        Serial.println(number);             // Yes, print it (or whatever)
        oldnumber = number;
      
      time2 = millis();
      
      timetaken = time2 - time1;
      Serial.println("Time taken: ");
      Serial.println(timetaken);
      if(timetaken > limit){
         Serial.println("Time taken: ");
         Serial.println(timetaken);
         Serial.println("Stopping");
         digitalWrite(r_motor_n, LOW);  //Set motor direction, 1 low, 2 high
         digitalWrite(r_motor_p, LOW);
         run = 0;
      }
      
      }
    }
    Serial.println("Stopping");
    digitalWrite(r_motor_n, LOW);  //Set motor direction, 1 low, 2 high
    digitalWrite(r_motor_p, LOW);
    run = 0;
  }
  
  pressure = analogRead(pressurePin); 
  buttonState = digitalRead(buttonPin);
  if(buttonState == HIGH && !run){
    startingNumber = number; 
    Serial.println("Reverse");
    //number = number - 2;
    digitalWrite(r_motor_p, HIGH);  //Set motor direction, 1 low, 2 high
    analogWrite(r_motor_n, LOW);
    while(number >= startingNumber-limit){
       time1 = millis();
      if(number != oldnumber){              // Change in value ?
        Serial.println(number);             // Yes, print it (or whatever)
        oldnumber = number;
        
        time2 = millis();
      
      timetaken = time2 - time1;
      //Serial.println("Time taken: ");
      //Serial.println(timetaken);
      if(timetaken >= limit){
         Serial.println("Time taken: ");
         Serial.println(timetaken);
         Serial.println("Stopping");
         digitalWrite(r_motor_n, LOW);  //Set motor direction, 1 low, 2 high
         digitalWrite(r_motor_p, LOW);
         run = 0;
       }  
        
      }
      
    }
    Serial.println("Stopping");
    digitalWrite(r_motor_n, LOW);  //Set motor direction, 1 low, 2 high
    digitalWrite(r_motor_p, LOW);
    run = 1;
  }
  
}

void isr_2(){                                              // Pin2 went LOW
  delay(1);                                                // Debounce time
  if(digitalRead(2) == LOW){                               // Pin2 still LOW ?
    if(digitalRead(3) == HIGH && halfright == false){      // -->
      halfright = true;                                    // One half click clockwise
    }  
    if(digitalRead(3) == LOW && halfleft == true){         // <--
      halfleft = false;                                    // One whole click counter-
      number--;                                            // clockwise
    }
  }
}
void isr_3(){                                             // Pin3 went LOW
  delay(1);                                               // Debounce time
  if(digitalRead(3) == LOW){                              // Pin3 still LOW ?
    if(digitalRead(2) == HIGH && halfleft == false){      // <--
      halfleft = true;                                    // One half  click counter-
    }                                                     // clockwise
    if(digitalRead(2) == LOW && halfright == true){       // -->
      halfright = false;                                  // One whole click clockwise
      number++;
    }
  }
}
