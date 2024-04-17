 /* 
Code created by the Voltweavers for the initial design of the tabletop research electrospinner. 
Team: Franco Raimondi, Logan Guilfoil, Lauren Kadlec, Kiefer Frank, Paityn Krout, Oscar Yang
Last edited on: 4/1/2024
*/

//-------------------------------------------------------------------------------------
//
//    Takes inputs from matlab in the form of serial
//    a. needle target speed (RPM)
//
//    b. oscilator width (steps)
//
//    c. oscilator speed (Steps per second)
//
//    d. Drum Target Speed (RPM)
//
//
//
//
//
//
//-------------------------------------------------------------------------------------











#include <Tone.h>     // library used for pulse frequency modulation on multiple pins

// create a class of Tone objects and define them for each motor
Tone needMotor;       
Tone oscMotor;
Tone drumMotor;

// pin assignment and relevant variable initialization for the syringe pump, oscillator, and drum motors
// stepper motors used have 200 steps/revolution
// speeds with Tone.h are Hz for the pulse frequency modulation
int needPin = 4;      
int needDir = 5;      
float needSpeed = 200;   // 166.66 = 50RPM
int oscPin = 7;
int oscDir = 8;
float oscSpeed = 0; 
int drumPin = 13;  // wired to the enable on the H bridge
int drumIn1 = 11; 
int drumIn2 = 12;    
float drumSpeed = 0; // duty cycle (1 through 0)
float maxDrumSpeed = 0.22;
float p = 0.00001;
int needErrors=0;
int drumErrors=0;
int red=40;
int yellow=42;
int blue=44;
int green=46;

// pin assignment and relevant variable initialization for the tachometers at the syringe pump and drum collector
const byte tachoNeedle = 2; 
volatile byte countNeedle = LOW;
const byte tachoDrum = 3;    
volatile byte countDrum = LOW;

// pin assignment for the limit switches
// front and back are for the syringe pump, left and right are for the oscillator
int needLimitRight = 15;  
int needLimitLeft = 14;  
int oscLimitFront = 16;   
int oscLimitBack = 17; 

// Temporary and target speeds for the motors
float drumTarget = 0; //measured in RPM, for closed loop control
float drumTargetNew = 0;
//float needTarget = 0; //measured in RPM, for closed loop control
float oscWidth = 0;
float oscTime = 0;

//Variables indicating modes that the electrospinner may be in
bool resetting= true;
bool homing=true;

//Box open Stop parameters
int openPin = 9;


// define functions to call when interrupts are triggered for the tachometers
void incrementNeed() {
  countNeedle++;
  }
void incrementDrum() {
  countDrum++;
  }

  
void setup()
{  
    Serial.begin(9600);   // begin serial communication
    sei();                // enable interrupts

    // declare motor direction pins as outputs and initialize to low
    pinMode(needDir, OUTPUT);
    digitalWrite(needDir, LOW);
    pinMode(oscDir, OUTPUT);
    digitalWrite(oscDir, LOW);
    
    pinMode(drumIn1, OUTPUT);
    pinMode(drumIn2, OUTPUT);
    digitalWrite(drumIn1, LOW);
    digitalWrite(drumIn2, HIGH);

    // declare limit switches as inputs
    pinMode(needLimitRight, INPUT);
    pinMode(needLimitLeft, INPUT);
    pinMode(oscLimitFront, INPUT);
    pinMode(oscLimitBack, INPUT);

    // declare LED indicator pins as outputs
    pinMode(green, OUTPUT); // green = syringe pump forward
    pinMode(blue, OUTPUT); // blue = syringe pump backward
    pinMode(yellow, OUTPUT); // yellow = syringe pump stop
    pinMode(red, OUTPUT); // red = all motors stop
    
    // initialize the pulse frequency modulation waves
    needMotor.begin(needPin);
    oscMotor.begin(oscPin);
    pinMode(drumPin, OUTPUT);

    // declare tachoNeedle as an input, and link it to an interupt
    pinMode(tachoNeedle, INPUT);
    attachInterrupt(digitalPinToInterrupt(tachoNeedle), incrementNeed, RISING);
    // declare tachoDrum as an input, and link it to an interupt
    pinMode(tachoDrum, INPUT);
    attachInterrupt(digitalPinToInterrupt(tachoDrum), incrementDrum, RISING);

    //initialize limit switch that stops system if you open the box
    //pinMode(openPin, INPUT);                  
    // set lights
    digitalWrite(red,HIGH); digitalWrite(yellow,HIGH); digitalWrite(blue,HIGH); digitalWrite(green,HIGH);
    //-----------------------------------------------------------------------------------------
}


void loop()
{
    // take note of the time for rpm calculations with tachometers
    long int t1 = millis();


  //Read Serial Data if it has been sent

 if (Serial.available()){          //if arduino has received data
  char myChar = Serial.read();    //save the data as a variable called "myChar"
  if (myChar == 'a'){             //if mychar is an "a"
    needSpeed = Serial.parseFloat();  //read the remaining data as a float, and save it as needle target speed in Hz
    if (needSpeed>0){
      digitalWrite(needDir, HIGH);
      resetting=false;
      // set lights to forward
      digitalWrite(red,LOW); digitalWrite(yellow,LOW); digitalWrite(blue,LOW); digitalWrite(green,HIGH);
    }
    else if (needSpeed<0){
      digitalWrite(needDir, LOW);
      resetting=true;
      // set lights to backward
      digitalWrite(red,LOW); digitalWrite(yellow,LOW); digitalWrite(blue, HIGH); digitalWrite(green,LOW);
    }
    else{
      digitalWrite(red,HIGH); digitalWrite(yellow,LOW); digitalWrite(blue,LOW); digitalWrite(green,LOW);
    }
    needSpeed=abs(needSpeed);
     }
     myChar = Serial.read();
   if (myChar == 'b'){
    oscWidth = Serial.parseFloat();
   }
   myChar = Serial.read();
   if (myChar == 'c'){
    oscSpeed = Serial.parseFloat();
    //time from center in that the oscillator goes in milliseconds
    oscTime = oscWidth*1000/oscSpeed;
   }
   myChar = Serial.read();
   if (myChar == 'd'){
    drumTargetNew = Serial.parseFloat();
    if (drumTarget==0&&drumTargetNew>0){
      drumSpeed=0.15;
    }
    if (drumTargetNew==0){
      drumSpeed=0;
    }
    drumTarget=drumTargetNew;
   }
   Serial.write(42);
  }





  
  
  for(int i=0;i<5000;i++){

    //stop the system if the box comes open
    
    if (digitalRead(openPin) == LOW){
      needMotor.stop();
      oscMotor.stop();   
      analogWrite(drumPin, 0);
      while (digitalRead(openPin) == LOW){
          if (Serial.available()){          //if arduino has received data
  char myChar = Serial.read();    //save the data as a variable called "myChar"
  if (myChar == 'a'){             //if mychar is an "a"
    needSpeed = Serial.parseFloat();  //read the remaining data as a float, and save it as needle target speed in Hz
    if (needSpeed>0){
      digitalWrite(needDir, HIGH);
      resetting=false;
      // set lights to forward
      digitalWrite(red,LOW); digitalWrite(yellow,LOW); digitalWrite(blue,LOW); digitalWrite(green,HIGH);
    }
    else if (needSpeed<0){
      digitalWrite(needDir, LOW);
      resetting=true;
      // set lights to backward
      digitalWrite(red,LOW); digitalWrite(yellow,LOW); digitalWrite(blue,HIGH); digitalWrite(green,LOW);
    }
    else{
      digitalWrite(red,HIGH); digitalWrite(yellow,LOW); digitalWrite(blue,LOW); digitalWrite(green,LOW);
    }
    needSpeed=abs(needSpeed);
     }
     myChar = Serial.read();
   if (myChar == 'b'){
    oscWidth = Serial.parseFloat();
   }
   myChar = Serial.read();
   if (myChar == 'c'){
    oscSpeed = Serial.parseFloat();
    oscTime = oscWidth*1000/oscSpeed;
   }
   myChar = Serial.read();
   if (myChar == 'd'){
    drumTargetNew = Serial.parseFloat();
    if (drumTarget==0&&drumTargetNew>0){
      drumSpeed=0.15;
    }
    if (drumTargetNew==0){
      drumSpeed=0;
    }
    drumTarget=drumTargetNew;
   }
   needErrors=0;
   drumErrors=0;
   Serial.write(42);
  }
      }
    }




    
    // enable the pulse frequency modulation at the variable speeds
    if(needSpeed==0){
      needMotor.stop();
    }
    else{
    needMotor.play(needSpeed);
    }
    if(oscSpeed==0){
      oscMotor.stop();
    }
    else{
    oscMotor.play(oscSpeed);
    }

    
    // create PWM wave of varying duty cycle for the drum collector
    analogWrite(drumPin, round(drumSpeed*255));

    // if a limit switch is activated, toggle the direction of the syringe pump motor
    if (digitalRead(needLimitRight) == HIGH){
      digitalWrite(needDir, LOW);
      oscSpeed=0;
      drumSpeed=0;
      resetting=true;
      digitalWrite(red,LOW); digitalWrite(yellow,LOW); digitalWrite(blue,HIGH); digitalWrite(green,LOW);
      }
    if (digitalRead(needLimitLeft) == HIGH){
      if (resetting){
        needSpeed=0;
        //indicate with yellow light syringe pump is ready
        digitalWrite(red,LOW); digitalWrite(yellow,HIGH); digitalWrite(blue,LOW); digitalWrite(green,LOW);
      }
      }
      
    // set indicator lights for appropriate directions of syringe pump
    //if (digitalRead(needDir) == HIGH) {digitalWrite(26,HIGH); digitalWrite(28,LOW); digitalWrite(30,LOW); digitalWrite(32,LOW);}
    //if (digitalRead(needDir) == LOW) {digitalWrite(26,LOW); digitalWrite(28,HIGH); digitalWrite(30,LOW); digitalWrite(32,LOW);}

    // if a limit switch is activated, toggle the direction of the oscillator motor
    if (digitalRead(oscLimitBack) == HIGH){
      digitalWrite(oscDir, HIGH);
      //delay(10); //10 milisecond delay
      }
    if (digitalRead(oscLimitFront) == HIGH){
      digitalWrite(oscDir, LOW);
      //delay(10); //10 milisecond delay
    }
    // if statement to stop the SYRINGE PUMP MOTOR
    // accepts serial input from MATLAB
    // 0hz, 'pause syringe pump', or digital E-stop is sent from MATLAB 
    // needMotor.stop();
    // digitalWrite(26,LOW); digitalWrite(28,LOW); digitalWrite(30,HIGH); digitalWrite(32,LOW);

    // if statement to stop ALL MOTORS
    // accepts serial input from MATLAB
    // 'stop all motors' or digital E-stop is sent from MATLAB
    // needMotor.stop(); oscMotor.stop(); drumSpeed = 0.00;
    // digitalWrite(26,LOW); digitalWrite(28,LOW); digitalWrite(30,LOW); digitalWrite(32,HIGH); 


  }
  //------------------------------------------------------------------------------------------
  
    // take the second time for rpm calculations with tachometers
    long int t2 = millis();

    
    float tmil = t2-t1;
    // create and cast as floats: counted values from the tachometers
    float countNeedf = float(countNeedle);
    float countDrumf = float(countDrum);
    // calculate the revolutions taken by each tachometer (20 slots per tachometer wheel)
    float revNeed = countNeedf/20;
    float revDrum = countDrumf/4;
    // cast the time difference from miliseconds to seconds
    float tsec = tmil/1000;
    // calculate 
    float RPMNeed = (revNeed/tsec)*60;
    float RPMDrum = (revDrum/tsec)*60;
    countNeedle = 0;
    countDrum = 0;

    //Adjust drum speed to be closer to target
    
  //At start up, go slow
  /*if (drumTarget==0){
    drumSpeed=0;
  }
  else
  {*/
    //Proportional Control System. P was determined experimentally
    drumSpeed=drumSpeed+p*(drumTarget-RPMDrum);
    if (drumSpeed<0){
    drumSpeed=0;
    }
    if(drumSpeed!=0){
      if(RPMDrum==0){
        drumErrors=drumErrors+1;
        if (drumErrors>=3){
          drumSpeed=0;
          digitalWrite(red,HIGH);
        }
      }
      else{
        drumErrors=0;
      }
    }
    if (drumSpeed>maxDrumSpeed){
      drumSpeed=maxDrumSpeed;
    }
 // }

    //Throw error if RPM is less than half of what is expected for three consecutive beats
    if (RPMNeed < needSpeed*60/200*0.5){
        needErrors=needErrors+1;
        if (needErrors>=3){
          needSpeed=0;
          digitalWrite(red,HIGH); digitalWrite(yellow,LOW); digitalWrite(blue,LOW); digitalWrite(green,LOW); 
        }
    }
    else{
      needErrors=0;
    }
    
 /*   Serial.print("Syringe Pump RPM = "); Serial.println(RPMNeed);
    Serial.print("Drum RPM = "); Serial.println(RPMDrum);
    Serial.print("Drum cycle = "); Serial.println(drumSpeed);
    Serial.println(" "); 
    */
    //open stop

    








    
}
