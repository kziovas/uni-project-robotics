
#include "math.h"

const int blackGate=50;
const int activeGate=80;
//delay time 
const int delT=30;
const int count1=5;
int j;

//PathStatus keeps record of the road and MoveStatus tells us if the robot goes straigh, turn or finished
char PathStatus[4]={'S','S','S','\0'};
char MoveStatus='S';


//pwm to control speed
int frwSpeed=95;//100
int minSpeed=60;
int maxSpeed=frwSpeed+50;
int adjustSpeed=0;
int leftSpeed=0;
int rightSpeed=0;
int rightWheelOffset=16;

//pwm to control speed
int trnSpeed=90;//115

//IR straigh line proportional and derivative error
float ErrorP=0.1;
float ErrorPabs=0.1;
float ErrorPast=0.1;
float ErrorI=0.01;
float ErrorD=0.1;

//PID constants
float Kp=0.45;//0.45, 0.7, 0.5, 0.5,0.5, 0.7,  0.5 , 0.7, 0.3,0.5, 0.7, 0.7, 0.3, 0.3
float Ki=0;//0,0
float Kd=0.16;//0.16, 0.3, 0.16,0.2,0.14,0.26 ,0.12,0.22, 0.1,0.08,0.18,0.08,0.12,0.08

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

}


void loop()
{
   readIR();
   checkPath();
   chooseMove();
   executeMove();
 
  Serial.println();
  Serial.print("PathStatus: ");
  Serial.print(PathStatus[0]);
  Serial.print(PathStatus[1]);
  Serial.print(PathStatus[2]);
  Serial.print(" | ");
  Serial.print("MoveStatus: ");
  Serial.print(MoveStatus);
  
}
void movePID(){
     ErrorP=IRs[3]+(1.25*IRs[4])-IRs[1]-(1.25*IRs[0]);
     
     ErrorPabs=abs(ErrorP);
     
     if(j<=count1){
      ErrorI=ErrorI+ErrorP;
      j++;
     }
     else{
      ErrorI=ErrorP/2;
      j=0;
     }
     
     ErrorD=ErrorPabs-ErrorPast;
     //adjustSpeed=(Kp*ErrorPabs+Kd*ErrorD)*(ErrorP/ErrorPabs);
     adjustSpeed=Kp*ErrorP+Ki*ErrorI+Kd*ErrorD;
      //adjustSpeed=Kp*ErrorP;
     leftSpeed=frwSpeed-adjustSpeed;
     rightSpeed=frwSpeed+adjustSpeed;
     ErrorPast=ErrorPabs;

     if(leftSpeed<=minSpeed){leftSpeed=minSpeed;}
     if(rightSpeed<=minSpeed){rightSpeed=minSpeed;}
    
     if(leftSpeed>=maxSpeed){leftSpeed=maxSpeed;}
     if(rightSpeed>=maxSpeed){rightSpeed=maxSpeed;}
     
     
     
     analogWrite(leftForward , leftSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward , rightSpeed+rightWheelOffset);
     analogWrite(rightBackward , LOW);
    
  }


void executeMove(){
  if( MoveStatus=='S'){
    movePID();
       
    }

  else if( MoveStatus=='L'){
    stayStill();
    turnLeft();
       
    }

  else if( MoveStatus=='R'){
    stayStill();
    turnRight();
       
    }

 else if( MoveStatus=='T'){
    stayStill();
    turnRight();
       
    }

  else{
    stayStill();
    }
  }


void chooseMove(){
  
  //Go Straight
  if(strcmp(PathStatus, "SSS") == 0){
      MoveStatus='S';
    }

   else if(strcmp(PathStatus, "SSL")==0){
    MoveStatus='L';
    
    }

  else if(PathStatus[2]=='R'){
    MoveStatus='R';
    
    }

   else if(strcmp(PathStatus, "SST")==0){
    MoveStatus='T';
    
    }

   else if(strcmp(PathStatus, "SSD")==0){
    MoveStatus='D';
    
    }
  }


void checkPath(){

  //Straight line
 if(IRs[4]>blackGate  && IRs[0]>blackGate && (IRs[1]>blackGate || IRs[3]>blackGate)){
    updatePathStatus('S');
    }
  
  //Left 90o turn
  else if(IRs[0]<=blackGate && IRs[1]<=blackGate && IRs[4]>activeGate){
   updatePathStatus('L');
   }

  //Right 90o  turn
  else if(IRs[4]<=blackGate && IRs[3]<=blackGate  && IRs[0]>activeGate){
   updatePathStatus('R');
    }

  //T-section
  else if(IRs[1]<blackGate && IRs[2]<blackGate && IRs[3]<blackGate && (IRs[4]<blackGate || IRs[0]<blackGate)){
    updatePathStatus('T');
    }

  //Dead End
  else if(IRs[0]>blackGate && IRs[1]>blackGate && IRs[2]>blackGate && IRs[3]>blackGate && IRs[4]>blackGate ){
    updatePathStatus('D');
    }

   else{
    stayStill();
    }

  
  }

void updatePathStatus(char input){
  
  PathStatus[0]=PathStatus[1];
  PathStatus[1]=PathStatus[2];
  PathStatus[2]=input;
  
  }

void moveStraight(){
     analogWrite(leftForward , frwSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward , frwSpeed);
     analogWrite(rightBackward , LOW);
}

void turnLeft(){
  while(IRs[1]<=activeGate){
     analogWrite(leftForward , LOW);
     analogWrite(leftBackward , trnSpeed);
     analogWrite(rightForward , trnSpeed);
     analogWrite(rightBackward , LOW);
     readIR();
  }   
  stayStill();
}

void turnRight(){

  for (int k=0;k<=5000;k++){
     analogWrite(leftForward ,  trnSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward ,  trnSpeed);
     analogWrite(rightBackward , LOW);
  }

  while(IRs[0]>blackGate){
     analogWrite(leftForward , trnSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward , LOW);
     analogWrite(rightBackward , trnSpeed);
     readIR();
  }


  while(IRs[0]<=activeGate){
     analogWrite(leftForward , trnSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward , LOW);
     analogWrite(rightBackward , trnSpeed);
     readIR();
  }


  while(IRs[2]>blackGate){
     analogWrite(leftForward , trnSpeed);
     analogWrite(leftBackward , LOW);
     analogWrite(rightForward , LOW);
     analogWrite(rightBackward , trnSpeed);
     readIR();
  }

  stayStill();
  delay(3000);
  for(int p=0;p<=1000;p++){
    for (int k=0;k<=1000;k++){
      movePID();
    }
  }
  delay(5000);
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

  /*
 Serial.println();
 
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
