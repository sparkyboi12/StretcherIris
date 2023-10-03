//Stretcher Command with Command Line inverface
float Version = 0.1;

#include <AccelStepper.h>

//#define debug // uncomment to display debug data
#define displayPosition // uncomment to display position as the stretcher moves

//#define showParsedData

const int stepPin = 7;
const int dirPin = 6;
const int zeroPinA = 9;
const int irqPin = 2;
const float microsteps = 800; // number of microsteps set on motor driver/rotation
//const float screwPitch = 2; // mm/rotation
const float gear_ratio = 10; // change this to the correct gear ratio. 
const float baudrate = 115200;

double currentPosition = 0;// absolute position of the tracker

volatile bool irq_state = 0;

#define debug

int MOVE = -6000;
unsigned int SPD = 1000;  
unsigned int ACCEL = 1600;


const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char CommandfromPC[numChars] = {0};
float unit = 0;
//float floatFromPC = 0.0;


boolean newData = false;

void recvWithStartEndMarkers();
void parseData(); 
void stepperOne(float cmd);

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(baudrate);
  pinMode(irqPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(irqPin), interruptflag, FALLING);

}

void loop() {
  irq_state = 0;
  recvWithStartEndMarkers();
  if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the " " with \0
        parseData();
        #ifdef showParsedData();
        showParsedData();
        #endif
        newData = false;
    }

  if (strcmp(CommandfromPC, "zero") == 0){ // eventually will find the zero position
    CommandfromPC[0] = 0;
  }

  else if (strcmp(CommandfromPC, "setZero") == 0){ // sets the zero to current position
    stepper.setCurrentPosition(0);
    Serial.print("Current position is 0.00 mm");
    CommandfromPC[0] = 0;
  }

  else if (strcmp(CommandfromPC, "goto") == 0){ 
    Serial.print("goto: ");
    Serial.println(unit);    
    stepperOne(unit);
    Serial.print("Position: ");
    Serial.println(convert_steps_to_radius(stepper.currentPosition()));
    CommandfromPC[0] = 0; 
  }

  else if (strcmp(CommandfromPC, "move") == 0){
    double temp_pos = unit + convert_steps_to_radius(stepper.currentPosition());
    Serial.print("move: ");
    Serial.println(unit);
    stepperOne(temp_pos);
    Serial.print("Position: ");
    Serial.println(convert_steps_to_radius(stepper.currentPosition()));
    CommandfromPC[0] = 0;
  }

  else if (strcmp(CommandfromPC, "position") == 0){
      Serial.print(convert_steps_to_radius(stepper.currentPosition()));
      Serial.println(" mm");
      CommandfromPC[0] = 0;
  }

  else if (strcmp(CommandfromPC, "speed") == 0){
      //SPD = unit*(1/screwPitch)*microsteps; // need to write a new speed function
      Serial.print("New speed = ");
      Serial.println(SPD);

    CommandfromPC[0] = 0;
  }

  else if (strcmp(CommandfromPC, "test")==0){

    CommandfromPC[0] = 0;


  }

  else if (strcmp(CommandfromPC, "help") == 0){
    
    Serial.println("\n");
    Serial.print("Membrane Stretcher Code Version: "); Serial.println(Version);
    Serial.print("\n");
    Serial.println("List of Commands:");
    Serial.println("  goto x: Goes to specified position. Replace 'x' with position required"); 
    Serial.println("  zero: Runs the Zeroing function so that the focal plane is zero");
    Serial.println("  setZero: Sets the current position as zero, useful for fine tuning zero");
    Serial.println("  position: displays current position");
    Serial.println("  speed x: changes the speed of the motors in mm/sec");
    Serial.println("  move x: moves by specified distance.");

    Serial.println("\nIn order to execute a command, type 'X' followed by intended command");
    Serial.println("In order to see the live stepper position as it moves, uncomment '#define displayPosition'.");


    CommandfromPC[0] = 0;


  }
  
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = 'X';
    char endMarker = '\n';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars," ");      // get the first part - the string
    strcpy(CommandfromPC, strtokIndx); // copy it to CommandfromPC

    strtokIndx = strtok(NULL, " ");
    unit = atof(strtokIndx);     // convert this part to a float

}

  void showParsedData() {
    Serial.print("Command ");
    Serial.println(CommandfromPC);
    Serial.print("unit ");
    Serial.println(unit);

}

void stepperOne(float cmd) {
  stepper.setMaxSpeed(SPD);
  stepper.setSpeed(SPD);
  stepper.setAcceleration(ACCEL);

  //stepper.setCurrentPosition(0); //looses zero position everytime command is run
  //long steps = -cmd/(2/microsteps);
  long steps = convert_radius_to_steps(cmd);

#ifdef debug
  Serial.print( "steps = ");
  Serial.println(steps);
#endif

  stepper.moveTo(steps);

  while (stepper.distanceToGo() != 0) {
      if(irq_state == 1){
        irq_state = 0;
        break;
      }
    stepper.setSpeed(SPD);
    stepper.runSpeedToPosition();

    #ifdef displayPosition
    //Serial.println(convert_steps_to_radius(stepper.currentPosition())); //tells how much the steppers moved by
    Serial.println(convert_steps_to_radius(stepper.currentPosition()));
    #endif
  } 
}

/*int Zero(){ // currently unused fuction to set the zero of the device. 
  stepper.setMaxSpeed(3000);
  stepper.setSpeed(2500);
  stepper.setAcceleration(ACCEL);
   stepper.moveTo(30/(screwPitch/microsteps)); // adjust to not use the screw pitch cause # reasons

  //while ((digitalRead(8) == LOW) && (stepper.distanceToGo() != 0)) {
   // while ((digitalRead(zeroPinA) == 0) && (stepper.distanceToGo() != 0)) {
      while ((stepper.distanceToGo() != 0)) {
    stepper.setSpeed(SPD);
    stepper.runSpeedToPosition();
  } 

  stepper.setCurrentPosition(0);

}*/

void interruptflag(){
  irq_state = 1;
  delay(10);
}

// converts intended radius change into the steps needed to move that far. 
int convert_radius_to_steps(float radius_change){
  float angle = 8.02568*(radius_change) - 6.71107; // conver radius change to an angle;
  int steps = (int)((angle/((360/microsteps)*gear_ratio)+0.5f));

  return steps;
}

// converts steps to the radius change to display how far the device has moved. 
float convert_steps_to_radius(int steps){
  float radius = 0.1246*(steps*(360/microsteps)*gear_ratio)+0.8362;
  return radius;
}
