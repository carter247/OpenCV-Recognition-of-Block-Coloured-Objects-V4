
  
#include <Servo.h>    // Servo header file
Servo joint1;
Servo joint2;
Servo joint3;
Servo joint4;     // Sets up each variable to its respective joint
Servo joint5;
Servo joint6;
  
  int j1 = 170;
  int j2 = 90;
  int j3 = 150;
  int j4 = 30;
  int j5 = 80;
  int j6 = 70;
  int j1New;
  int j2New;
  int j3New;
  int j4New;
  int j5New;
  int j6New;
  int inputAngle;
  int width = 0;
  int height = 0;

void setup() {

  joint1.attach(11);    // Pin 11 controls Joint 1
  joint2.attach(10);    // Pin 10 controls Joint 2
  joint3.attach(9);     // Pin 9 controls Joint 3
  joint4.attach(6);     // Pin 6 controls Joint 4
  joint5.attach(5);     // Pin 5 controls Joint 5
  joint6.attach(3);     // Pin 3 controls Joint 6

  Serial.begin(9600);

  joint1.write(j1);
  joint2.write(j2);
  joint3.write(j3);
  joint4.write(j4);
  joint5.write(j5);
  joint6.write(j6);
  
  delay(5000);
  
  homePosition(j1, j2, j3, j4, inputAngle);
  
}

void homePosition(int &j1, int &j2, int &j3, int &j4, int &inputAngle){

  Serial.println("Returning to home position...");
  Serial.println("");

  delay(1000);
  inputAngle = 90;
  j1 = commandJoint1(j1, inputAngle);
  delay(250);
  inputAngle = 130;
  j2 = commandJoint2(j2, inputAngle);
  delay(250);
  inputAngle = 95;
  j3 = commandJoint3(j3, inputAngle);
  delay(250);
  inputAngle = 90;
  j4 = commandJoint4(j4, inputAngle);
  delay(250);
  
}   // This is the home position during Actuation

void sleepPosition(int &j1, int &j2, int &j3, int &j4, int &inputAngle){

  Serial.println("Returning to sleep position...");
  Serial.println("");

  delay(1000);
  inputAngle = 30;
  j4 = commandJoint4(j4, inputAngle);
  delay(250);
  inputAngle = 150;
  j3 = commandJoint3(j3, inputAngle);
  delay(250);
  inputAngle = 160;
  j1 = commandJoint1(j1, inputAngle);
  delay(250);
  inputAngle = 90;
  j2 = commandJoint2(j2, inputAngle);
  delay(250);
  
}   // This is the home position during Actuation

int commandJoint1(int &x1, int &x2){

    int x3 = 0;       // Joint angle 1 displacement
    int x4;       // Joint angle 1 iterator
    
    if(x1 != x2){
        
        x3 = x2 - x1;

        x4 = floor(x3 / 5);
    
    if(x4 < 0){
        
        x4 = x4 * -1;
        
        for(int i = 0; i < x4; i++){
            
            x1 -= 5;
            joint1.write(x1);
            delay(250);
        }
    }
    else{
        
        for(int i = 0; i < x4; i++){
            
            x1 += 5;
            joint1.write(x1);
            delay(250);
        }
    }
    }
    if(x1 == x2){
        
        x2 = x2;
    }
    
    return x2;
}

int commandJoint2(int &x1, int &x2){

    int x3 = 0;       // Joint angle 1 displacement
    int x4;       // Joint angle 1 iterator
    
    if(x1 != x2){
        
        x3 = x2 - x1;

        x4 = floor(x3 / 5);
    
    if(x4 < 0){
        
        x4 = x4 * -1;
        
        for(int i = 0; i < x4; i++){
            
            x1 -= 5;
            joint2.write(x1);
            delay(250);
        }
    }
    else{
        
        for(int i = 0; i < x4; i++){
            
            x1 += 5;
            joint2.write(x1);
            delay(250);
        }
    }
    }
    if(x1 == x2){
        
        x2 = x2;
    }
    
    return x2;
}

int commandJoint3(int &x1, int &x2){

    int x3;       // Joint angle 1 displacement
    int x4;       // Joint angle 1 iterator
    
    if(x1 != x2){
        
        x3 = x2 - x1;

        x4 = floor(x3 / 5);
    
    if(x4 < 0){
        
        x4 = x4 * -1;
        
        for(int i = 0; i < x4; i++){
            
            x1 -= 5;
            joint3.write(x1);
            delay(250);
        }
    }
    else{
        
        for(int i = 0; i < x4; i++){
            
            x1 += 5;
            joint3.write(x1);
            delay(250);
        }
    }
    }
    if(x1 == x2){
        
        x2 = x2;
    }
    
    return x2;
}

int commandJoint4(int &x1, int &x2){

    int x3;       // Joint angle 1 displacement
    int x4;       // Joint angle 1 iterator
    
    if(x1 != x2){
        
        x3 = x2 - x1;

        x4 = floor(x3 / 5);
    
    if(x4 < 0){
        
        x4 = x4 * -1;
        
        for(int i = 0; i < x4; i++){
            
            x1 -= 5;
            joint4.write(x1);
            delay(250);
        }
    }
    else{
        
        for(int i = 0; i < x4; i++){
            
            x1 += 5;
            joint4.write(x1);
            delay(250);
        }
    }
    }
    if(x1 == x2){
        
        x2 = x2;
    }
    
    return x2;
}

int commandJoint5(int &x1, int &x2){

    int x3;       // Joint angle 1 displacement
    int x4;       // Joint angle 1 iterator
    
    if(x1 != x2){
        
        x3 = x2 - x1;

        x4 = floor(x3 / 5);
    
    if(x4 < 0){
        
        x4 = x4 * -1;
        
        for(int i = 0; i < x4; i++){
            
            x1 -= 5;
            joint5.write(x1);
            delay(250);
        }
    }
    else{
        
        for(int i = 0; i < x4; i++){
            
            x1 += 5;
            joint5.write(x1);
            delay(250);
        }
    }
    }
    if(x1 == x2){
        
        x2 = x2;
    }
    
    return x2;
}

int commandJoint6(int &x1, int &x2){

    int x3;       // Joint angle 1 displacement
    int x4;       // Joint angle 1 iterator
    
    if(x1 != x2){
        
        x3 = x2 - x1;

        x4 = floor(x3 / 5);
    
    if(x4 < 0){
        
        x4 = x4 * -1;
        
        for(int i = 0; i < x4; i++){
            
            x1 -= 5;
            joint6.write(x1);
            delay(250);
        }
    }
    else{
        
        for(int i = 0; i < x4; i++){
            
            x1 += 5;
            joint6.write(x1);
            delay(250);
        }
    }
    }
    if(x1 == x2){
        
        x2 = x2;
    }
    
    return x2;
}

void checkConnection(){

  int iter = 0;
  int iter2 = 0;
  int dataBit;

  while(iter == 0){

    if(iter2 == 0){

      Serial.println("Press Enter to Establish Connection");
      Serial.println("");
    }

    iter2++;
    delay(500);

    if(iter2 == 10){

      iter2 = 0;
    }

    while(Serial.available() > 0){
      
      dataBit = Serial.read();

      if(dataBit == 10){
        
        Serial.println("Connection Established");
        Serial.println("");
        homePosition(j1, j2, j3, j4, inputAngle);
        textHomePosition(j1, j2, j3, j4, j5, j6);
        Serial.println("To send the joint angles follow:");
        Serial.println("");
        Serial.println("Click Connection > Send Text File > 'Search jointangles.txt' > Open");
        Serial.println("");
        Serial.println("Or to send the robot to sleep, follow:");
        Serial.println("");
        Serial.println("Click Connection > Send Text File > 'Search sleep.txt' > Open");
        Serial.println("");
        iter = 1;
        return;
      }
    }
  }
}

void textHomePosition(int &j1, int &j2, int &j3, int &j4, int &j5, int &j6){

     Serial.println("Robot in home position");
     Serial.print("Joint 1: ");
     Serial.print(j1);
     Serial.println(" Degrees");
     Serial.print("Joint 2: ");
     Serial.print(j2);
     Serial.println(" Degrees");
     Serial.print("Joint 3: ");
     Serial.print(j3);
     Serial.println(" Degrees");
     Serial.print("Joint 4: ");
     Serial.print(j4);
     Serial.println(" Degrees");
     Serial.print("Joint 5: ");
     Serial.print(j5);
     Serial.println(" Degrees");
     Serial.print("Joint 6: ");
     Serial.print(j6);
     Serial.println(" Degrees");
     Serial.println("");
}

void readIncomingData(int &j1New, int &j2New, int &j3New, int &j4new, int &j5New, int &j6New, int &width, int &height){

  String result = "";
  int incoming = 0;
  int iter = 1;

  while(iter <= 8){
  
  while(Serial.available() > 0){

    incoming = Serial.read();
    
    if(isDigit(incoming)){
      
      result += (char)incoming;
    }

    if(!isDigit(incoming) && iter == 8){

      height = result.toInt();
      result = "";
      return;
    }

    if(!isDigit(incoming) && iter == 7){

      width = result.toInt();
      result = "";
      iter = 8;
    }

    if(!isDigit(incoming) && iter == 6){

      j6New = result.toInt();
      Serial.print("Joint 6: ");
      Serial.print(j6New);
      Serial.println(" Degrees");
      result = "";
      Serial.println(result);
      iter = 7;
    }

    if(!isDigit(incoming) && iter == 5){

      j5New = result.toInt();
      Serial.print("Joint 5: ");
      Serial.print(j5New);
      Serial.println(" Degrees");
      result = "";
      iter = 6;
    }

    if(!isDigit(incoming) && iter == 4){

      j4New = result.toInt();
      Serial.print("Joint 4: ");
      Serial.print(j4New);
      Serial.println(" Degrees");
      result = "";
      iter = 5;
    }

    if(!isDigit(incoming) && iter == 3){

      j3New = result.toInt();
      Serial.print("Joint 3: ");
      Serial.print(j3New);
      Serial.println(" Degrees");
      result = "";
      iter = 4;
    }

    if(!isDigit(incoming) && iter == 2){

      j2New = result.toInt();
      Serial.print("Joint 2: ");
      Serial.print(j2New);
      Serial.println(" Degrees");
      result = "";
      iter = 3;
    }

    if(!isDigit(incoming) && iter == 1){

      Serial.println("Data Received:");
      j1New = result.toInt();
      Serial.print("Joint 1: ");
      Serial.print(j1New);
      Serial.println(" Degrees");
      result = "";
      iter = 2;
    } 
  }
  }
}

void actuateJoints(int &j1New, int &j2New, int &j3New, int &j4New, int &j1, int &j2, int &j3, int &j4){

  Serial.println("Joints Actuated:");

  if(j4New != j4){

    j4 = commandJoint4(j4, j4New);
    Serial.print("Joint 4: ");
    Serial.print(j4);
    Serial.println(" Degrees");
   }

  if(j3New != j3){

    j3 = commandJoint3(j3, j3New);
    Serial.print("Joint 3: ");
    Serial.print(j3);
    Serial.println(" Degrees");
   }

  if(j1New != j1){

    j1 = commandJoint1(j1, j1New);
    Serial.print("Joint 1: ");
    Serial.print(j1);
    Serial.println(" Degrees");
   }

  if(j2New != j2){

    j2 = commandJoint2(j2, j2New);
    Serial.print("Joint 2: ");
    Serial.print(j2);
    Serial.println(" Degrees");
   }
   
   Serial.println("");

   if(j1 == 170 && j2 == 90 && j3 == 150 && j4 == 30 && j5 == 80 && j6 == 70){

    delay(20000);
   }
}

void grabObject(int &inputAngle){

  Serial.println("Grabbing Object...");
  Serial.println("");
  inputAngle = 140;
  j6 = commandJoint6(j6, inputAngle);
  dropPosition(j1, j2, j3, j4, j6, inputAngle);
  homePosition(j1, j2, j3, j4, inputAngle);
  textHomePosition(j1, j2, j3, j4, j5, j6);
}

void dropPosition(int &j1, int &j2, int &j3, int &j4, int &j6, int &inputAngle){

  Serial.println("Dropping Off Object...");
  Serial.println("");
  delay(1000);
  inputAngle = 130;
  j2 = commandJoint2(j2, inputAngle);
  delay(250);
  inputAngle = 90;
  j1 = commandJoint1(j1, inputAngle);
  delay(250);
  inputAngle = 95;
  j3 = commandJoint3(j3, inputAngle);
  delay(250);
  inputAngle = 90;
  j4 = commandJoint4(j4, inputAngle);
  delay(250);
  inputAngle = 100;
  j2 = commandJoint2(j2, inputAngle);
  delay(250);
  inputAngle = 70;
  j6 = commandJoint6(j6, inputAngle);
  delay(250);
  inputAngle = 45;
  j4 = commandJoint4(j4, inputAngle);
  delay(250);

}

void loop(){

  checkConnection();
  readIncomingData(j1New, j2New, j3New, j4New, j5New, j6New, width, height);
  actuateJoints(j1New, j2New, j3New, j4New, j1, j2, j3, j4);
  grabObject(inputAngle);  
  
}

 
