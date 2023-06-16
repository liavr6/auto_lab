//Encoder Example


// Define Pins

#define ENCODER_PINA 2

#define ENCODER_PINB 3

const int M0_PHASE = 4;
const int M0_PWM = 5;
const int ENP = 6;

// encoder variables
  unsigned long prev =0;

volatile int encoderCounts = 0;
float revs = 0 ;
const int gear = 30; // changeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define Kp 2
#define Ki 2
#define Kd 0

int pulse_per_r = 12;
int rev_prev = 0;
// Encoder ISR functions - Interupt Service Routine
float intg = 0;
float prv_err = 0;


void encoderA();

void encoderB();


void setup() {

  // initialize serial communication at 115200 bits per second:

  Serial.begin (115200);

 

  // initialize encoder, attache ISR functions

  pinMode(ENCODER_PINA, INPUT);

  pinMode(ENCODER_PINB, INPUT);

   // Attached interrupt to encoder pins

  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderA, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), encoderB, CHANGE);


  Serial.print("Encoder_Value");

}


void loop() {

  // print encoder position
    int POTENTIOMETER = analogRead(A0);
  //int PMW = map(abs(POTENTIOMETER - 512), 0, 512, 0, 255);
 // float target = normalize_value(POTENTIOMETER, 0, 1023, -10, 10);
//  float normalize_value(int value, int old_min, int old_max, int new_min, int new_max)

  float target = (float)(((float)(POTENTIOMETER - 0) / (1023 - 0)) * (10 - (-10)) + (-10));


  unsigned long curr = millis();
  unsigned long delta = curr - prev;
  if(delta<10)
    return;
  //float counts = encoderCounts;
  //encoderCounts = 0;
  prev = curr;
  float dt_s = (float)delta / 1000;
  float revs = (float)encoderCounts / (gear*pulse_per_r); 
  float d_rev = revs - rev_prev;
  rev_prev = revs;

  float vel = (float)(d_rev * 60) / dt_s; 

  float f_vel =  (0.9) * prev_velocity + 0.1 * vel ;
  prev_velocity = filt_velocity;
  
  // PID controller
  float err = target - revs;
  intg += err * ((float)delta/1000);
  float drv = (err - prv_err) / ((float)delta/1000);
  prv_err = err;
  float u = Kp * err + Ki * intg + Kd * drv;

  // Adjust motor velocity
  int PWM = min(abs(u), 255);

  
  if (u >= 0)
  {
    analogWrite(M0_PWM, PWM);
    analogWrite(M0_PHASE, 0);
  }
  else 
  {
    analogWrite(M0_PWM, 0);
    analogWrite(M0_PHASE, PWM);
  }
  Serial.print(target);
  Serial.print(", ");
  Serial.print(revs);
  Serial.print(", ");
  Serial.print(err);
  Serial.print("\n");
   }



// EncoderA ISR

void encoderA() {

  // look for a low-to-high on channel B

  if (digitalRead(ENCODER_PINA) == HIGH) {

    // check channel A to see which way encoder is turning

    digitalRead(ENCODER_PINB) ? encoderCounts++ : encoderCounts--;    

  }else{

    // check channel A to see which way encoder is turning

    digitalRead(ENCODER_PINB) ? encoderCounts-- : encoderCounts++;

  }

} // End EncoderA ISR


// EncoderB ISR

void encoderB() {

  // look for a low-to-high on channel B

  if (digitalRead(ENCODER_PINB) == HIGH) {

    // check channel A to see which way encoder is turning

    digitalRead(ENCODER_PINA) ? encoderCounts-- : encoderCounts++;    

  }else{

    // check channel A to see which way encoder is turning

    digitalRead(ENCODER_PINA) ? encoderCounts++ : encoderCounts--;

  }

} // End EncoderB ISR

