
#include "math.h"

const int blackGate=50;
const int activeGate=80;//85
const int whiteGate=92;
//delay time 
const int delT=50;
const int count1=5;
int j;

//variable to know when timer finished. This timer is used to distinguish turns from Ts
long int timer1=0;
long int duration1=35;

//this timer is used to give the robot some time to reallign after a turn. After a turn is made another "turn" cannot be made or a T cannot be found for 2 sec
long int timer2;
long int duration2=1500;

//this timer is to differentiate a "T" cross-road from end of maze
long int timer3;
long int duration3=80;


//PathStatus keeps record of the road and MoveStatus tells us if the robot goes straigh, turn or finished
char PathStatus[31]={'S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','S','\0'};
int pathCounter=0;
char MoveStatus='S';
char oldMoveStatus='S';


//pwm to control speed
int frwSpeed=90;//95
int minSpeed=frwSpeed-20;//60
int maxSpeed=frwSpeed+30;
int adjustSpeed=0;
int leftSpeed=0;
int rightSpeed=0;
int rightWheelOffset=14;//16

//pwm to control speed
int trnSpeed=90;//95

//IR straigh line proportional and derivative error
float ErrorP=0.1;
float ErrorPabs=0.1;
float ErrorPast=0.1;
float ErrorI=0.01;
float ErrorD=0.1;

//PID constants
float Kp=0.48;// 0.45, 0.4,  0.7, 0.5, 0.5,0.5, 0.7,  0.5 , 0.7, 0.3,0.5, 0.7, 0.7, 0.3, 0.3
float Ki=0;//0,0
float Kd=0.20;//0.16, 0.3, 0.16,0.2,0.14,0.26 ,0.12,0.22, 0.1,0.08,0.18,0.08,0.12,0.08

//Define motor control pins
const int leftForward = 10;
const int leftBackward = 9;
const int rightForward = 5;
const int rightBackward = 6;

//Define max and min value for each sensor
int maxIR[5]={880,760,870,910,955};
int minIR[5]={300,300,300,300,300};


//WHITE 15: 937.00 | 16: 928.00 | 17: 935.00 | 18: 938.00 | 19: 944.00 | 
//black 15: 381.00 | 16: 406.00 | 17: 391.00 | 18: 410.00 | 19: 464.00 | 


//Define Variable array for sensors
float IR1=0,IR2=0,IR3=0,IR4=0,IR5=0;
float IRs[5]={IR1,IR2,IR3,IR4,IR5};

//Define variable array for analog pins
int Anlgs[5]={15,16,17,18,19};


void setup()
{
  Serial.begin(9600);
  Serial.println("TRSensor example");
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  pinMode(leftForward , OUTPUT);
  pinMode(leftBackward , OUTPUT);
  pinMode(rightForward , OUTPUT);
  pinMode(rightBackward , OUTPUT);

  //there cannot be a turn not less than two sec after the robot starts or after it takes a turn
  timer2=millis();

}


void loop()
{

  while(MoveStatus!='E'){
    readIR();
     Serial.print("After read: ");
    checkPath();
    Serial.println(PathStatus);
  }
  while(MoveStatus=='E'){
    stayStill();
  }
  
}


void checkPath(){


    //End of maze
   if( IRs[4]<=activeGate  && IRs[3]<=blackGate  && IRs[2]<=blackGate && IRs[1]<=blackGate  && IRs[0]<=activeGate && ((millis()-timer2)>duration2)){
    stayStill();
    timer3=millis();
    oldMoveStatus=MoveStatus;
    while(millis()-timer3<duration3){
      Serial.println(millis()-timer3);
      readIR();    
      if(IRs[4]>blackGate  || IRs[3]>blackGate  || IRs[2]>blackGate || IRs[1]>blackGate  || IRs[0]>blackGate){
        stayStill();
        MoveStatus='T';
        updatePathStatus('T');
        Serial.print(" TE ");
        turnLeft();
        timer2=millis();
        break;
      }
      else{

          MoveStatus='E';

      } 
    } 
    if(MoveStatus=='E'){
        stayStill();
        updatePathStatus('E');
        Serial.print(" End of maze ");
        timer2=millis();
    }
   }

  //T-section-1
  // else if( IRs[4]<=activeGate  && IRs[3]<=blackGate  && IRs[2]<=blackGate && IRs[1]<=blackGate  && IRs[0]<=activeGate && ((millis()-timer2)>duration2) && MoveStatus!='E'){
  //  stayStill();
   // updatePathStatus('T');
  //  MoveStatus='T';
  //  Serial.print(" TT ");
 //   turnLeft();
    //turnLeft();
 //   timer2=millis();
  // }


    //Dead End
  else if( IRs[4]>=whiteGate  && IRs[3]>=whiteGate  && IRs[2]>=whiteGate && IRs[1]>=whiteGate  && IRs[0]>=whiteGate && ((millis()-timer2)>duration2)){
    stayStill();
    oldMoveStatus=MoveStatus;
    Serial.println(oldMoveStatus);
    MoveStatus='D';
    Serial.println(MoveStatus);
    updatePathStatus('D');
    turnAround();
    Serial.print(" Dead end ");
    timer2=millis();

   }

 //Right 90o  turn
   else if(IRs[4]<=blackGate && IRs[3]<=blackGate && IRs[2]<=activeGate && IRs[0]>activeGate &&  ((millis()-timer2)>duration2) && MoveStatus!='E'){
    timer1=millis();
    oldMoveStatus=MoveStatus;
    while(millis()-timer1<duration1){
      Serial.println(millis()-timer1);
      readIR();
      if(IRs[0]<=blackGate){
        stayStill();
        MoveStatus='T';
        updatePathStatus('T');
        Serial.print(" TR ");
        turnLeft();
        timer2=millis();
        break;
     }
     else{

          MoveStatus='R';

      }  
    }
    if(MoveStatus=='R'){
        //turnRight();
        updatePathStatus('R');
        timer2=millis();
        Serial.print(" Turn Right! ");
    }
   }

//Left 90o  turn
   else if(IRs[0]<=blackGate && IRs[1]<=blackGate && IRs[2]<=activeGate && IRs[4]>activeGate && ((millis()-timer2)>duration2) && MoveStatus!='E'){
    timer1=millis();
    oldMoveStatus=MoveStatus;      
    while(millis()-timer1<duration1){
      Serial.println(millis()-timer1);
      readIR();
      if(IRs[4]<=blackGate){
        stayStill();
        MoveStatus='T';
        updatePathStatus('T');
        Serial.print(" TL ");
        turnLeft();
        timer2=millis();
        break;
     }
     else{
          MoveStatus='L';
      } 
    }
    if(MoveStatus=='L'){
      turnLeft();
      updatePathStatus('L');
      timer2=millis();
      Serial.print(" Turn Left! ");
    }
   }
 
    
   else if ( MoveStatus!='E'){
    oldMoveStatus=MoveStatus;
    MoveStatus='S';
    updatePathStatus('S');
    movePID();
    Serial.print(" Move! ");
   }
 
 }


void updatePathStatus(char input){
  if(oldMoveStatus!=MoveStatus){
    PathStatus[pathCounter]=input;
    pathCounter++;
  }
}


 
void movePID(){
     ErrorP=IRs[3]+(1.25*IRs[4])-IRs[1]-(1.25*IRs[0]);
     
     //ErrorPabs=abs(ErrorP);
     
     if(j<=count1){
      ErrorI=ErrorI+ErrorP;
      j++;
     }
     else{
      ErrorI=ErrorP/2;
      j=0;
     }

    //Just so i remember next time i write a PID the derivative is just the difference between the current and past value.
    //I got confused with the signs but the Derivative term grows negatively or positivelly and adds and subtracts from the positive on negative values
    //if we simple have the equations that I use here.
     ErrorD=ErrorP-ErrorPast;
     //ErrorD=ErrorPabs-ErrorPast;
     //adjustSpeed=(Kp*ErrorPabs+Kd*ErrorD)*(ErrorP/ErrorPabs);
     adjustSpeed=Kp*ErrorP+Ki*ErrorI+Kd*ErrorD;
      //adjustSpeed=Kp*ErrorP;
     leftSpeed=frwSpeed-adjustSpeed;
     rightSpeed=frwSpeed+adjustSpeed;
     //ErrorPast=ErrorPabs;
     ErrorPast=ErrorP;
     if(leftSpeed<=minSpeed){leftSpeed=minSpeed;}
     if(rightSpeed<=minSpeed){rightSpeed=minSpeed;}
    
     if(leftSpeed>=maxSpeed){leftSpeed=maxSpeed;}
     if(rightSpeed>=maxSpeed){rightSpeed=maxSpeed;}
     
     
     
     analogWrite(leftForward , leftSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward , rightSpeed+rightWheelOffset);
     analogWrite(rightBackward , LOW);
    
  }



void turnAround(){
  Serial.print(" Turning around ");
  stayStill();
  delay(200);
  for (int k=0;k<=6000;k++){
  //Go a bit backward
   analogWrite(leftForward , LOW );
   analogWrite(leftBackward , trnSpeed);
   analogWrite(rightForward ,  LOW);
   analogWrite(rightBackward ,trnSpeed+rightWheelOffset);
  }
   stayStill();
  delay(200);
  while(IRs[0]>blackGate){
   analogWrite(leftForward , LOW);
   analogWrite(leftBackward , trnSpeed);
   analogWrite(rightForward , trnSpeed+rightWheelOffset);
   analogWrite(rightBackward , LOW);
   readIR();
  }
    stayStill();
  delay(100);
  while(IRs[0]<=activeGate){
     analogWrite(leftForward , LOW);
     analogWrite(leftBackward , trnSpeed);
     analogWrite(rightForward , trnSpeed+rightWheelOffset);
     analogWrite(rightBackward , LOW);
     readIR();
  }
  stayStill();
  delay(100);
  while(IRs[2]>activeGate){
   analogWrite(leftForward , LOW);
   analogWrite(leftBackward , trnSpeed);
   analogWrite(rightForward , trnSpeed+rightWheelOffset);
   analogWrite(rightBackward , LOW);
   readIR();
  }
  stayStill();
  delay(100);
  while(IRs[2]>blackGate){
   analogWrite(leftForward , LOW);
   analogWrite(leftBackward , trnSpeed);
   analogWrite(rightForward , trnSpeed+rightWheelOffset);
   analogWrite(rightBackward , LOW);
   readIR();
  }
  stayStill();
  delay(100);
  Serial.print(" Turned around ");
}

void turnLeft(){

  for (int k=0;k<=6000;k++){
    //Go a bit forward
     analogWrite(leftForward ,  trnSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward ,  trnSpeed+rightWheelOffset);
     analogWrite(rightBackward , LOW);
  }

  stayStill();
  delay(100);

  while(IRs[4]>blackGate){
     analogWrite(leftForward , LOW);
     analogWrite(leftBackward , trnSpeed);
     analogWrite(rightForward , trnSpeed+rightWheelOffset);
     analogWrite(rightBackward , LOW);
     readIR();
  }

  stayStill();
  delay(100);

  while(IRs[4]<=activeGate){
     analogWrite(leftForward , LOW);
     analogWrite(leftBackward , trnSpeed);
     analogWrite(rightForward , trnSpeed+rightWheelOffset);
     analogWrite(rightBackward , LOW);
     readIR();
  }

  stayStill();
  delay(100);

  while(IRs[2]>activeGate){
     analogWrite(leftForward , LOW);
     analogWrite(leftBackward , trnSpeed);
     analogWrite(rightForward , trnSpeed+rightWheelOffset);
     analogWrite(rightBackward , LOW);
     readIR();
  }

  stayStill();
  delay(100);

  while(IRs[2]>blackGate){
     analogWrite(leftForward , LOW);
     analogWrite(leftBackward , trnSpeed);
     analogWrite(rightForward , trnSpeed+rightWheelOffset);
     analogWrite(rightBackward , LOW);
     readIR();
  }

  stayStill();
  delay(100);

}







void turnRight(){

  //Go a bit forward
  for (int k=0;k<=6000;k++){
     analogWrite(leftForward ,  trnSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward ,  trnSpeed+rightWheelOffset);
     analogWrite(rightBackward , LOW);
  }
  stayStill();
  while(IRs[0]>blackGate){
     analogWrite(leftForward , trnSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward , LOW);
     analogWrite(rightBackward , trnSpeed+rightWheelOffset);
     readIR();
  }
  stayStill();
 
  while(IRs[0]<=activeGate){
     analogWrite(leftForward , trnSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward , LOW);
     analogWrite(rightBackward , trnSpeed+rightWheelOffset);
     readIR();
  }
  stayStill();

  while(IRs[2]>blackGate){
     analogWrite(leftForward , trnSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward , LOW);
     analogWrite(rightBackward , trnSpeed+rightWheelOffset);
     readIR();
  }

   stayStill();


}

void stayStill(){
     analogWrite(leftForward , LOW);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward , LOW);
     analogWrite(rightBackward , LOW);
}

void readIR(){

  for(int i=0;i<5;i++){
    IRs[i]=analogRead(Anlgs[i]);
    
    IRs[i]=100*(IRs[i]-minIR[i])/(maxIR[i]-minIR[i]);
  
    if(IRs[i]<=0){
     IRs[i]=0.01;
    }
    if(IRs[i]>=100){
     IRs[i]=99.9;
   }
  /*
   Serial.print(Anlgs[i]);
   Serial.print(": ");
   Serial.print(IRs[i]);
   Serial.print(" | ");
  */
 }

 
// Serial.println();
 /*
 Serial.print(" ErrorP: ");
 Serial.print(ErrorP);
 Serial.print("  |  ErrorD: ");
 Serial.print(ErrorD);
 Serial.print("  |  ErrorI: ");
 Serial.print(ErrorI);
 Serial.print("  |  AdjustSpeed: ");
 Serial.print(adjustSpeed);
 Serial.print("  |  LeftSpeed: ");
 Serial.print(leftSpeed);
 Serial.print("  |  RightSpeed: ");
 Serial.print(rightSpeed);
 */

 //Serial.print("AdjustSpeed: ");
 //Serial.print(adjustSpeed);
// Serial.println();
//delay(200);


}
