/*
 *  Vacuum control
 *  Value  Vacuum Generator  Vacuum Breaker
 *      0                 0               0
 *      1                 0               1
 *      2                 1               0
 *      3                 1               1
 */

#define VACUUM_GENERATOR     12
#define VACUUM_BREAKER        8
#define CONVRYOR_CONTROL      4
#define PNEUMATIC_CYLINDER    7
#define PRESSURE_TRANSMITTER A5

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(100);
  pinMode(VACUUM_GENERATOR, OUTPUT);
  pinMode(VACUUM_BREAKER, OUTPUT);
  pinMode(CONVRYOR_CONTROL, OUTPUT);
  pinMode(PNEUMATIC_CYLINDER, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*if(Serial.available()){
    char command_type = Serial.read();
    if(command_type == 'v'){
      if(Serial.available()){
        int command = Serial.read() - '0';
        switch(command){
          case 0:
            digitalWrite(VACUUM_GENERATOR, LOW);
            delay(1);
            digitalWrite(VACUUM_BREAKER, LOW);
            break;
          case 1:
            digitalWrite(VACUUM_GENERATOR, LOW);
            delay(1);
            digitalWrite(VACUUM_BREAKER, HIGH);
            break;
          case 2:
            digitalWrite(VACUUM_GENERATOR, HIGH);
            delay(1);
            digitalWrite(VACUUM_BREAKER, LOW);
            break;
          case 3:
            digitalWrite(VACUUM_GENERATOR, HIGH);
            delay(1);
            digitalWrite(VACUUM_BREAKER, HIGH);
            break;
        }
      }
    }
    else if(command_type == 'c'){
      digitalWrite(CONVRYOR_CONTROL, HIGH);
      delay(100);
      digitalWrite(CONVRYOR_CONTROL, LOW);
    }
    else if(command_type == 'p'){
      if(Serial.available()){
        int command = Serial.read() - '0';
        switch(command){
          case 0:
            digitalWrite(PNEUMATIC_CYLINDER, LOW);
            break;
          case 1:
            digitalWrite(PNEUMATIC_CYLINDER, HIGH);
            break;
        }
      }
    }
  }*/
  if(Serial.available()){
    String command_str = Serial.readString();
    char command_type = command_str[0];
    if(command_type=='p'){
      int command = command_str[1]-'0';
      switch(command){
        case 0:
          digitalWrite(PNEUMATIC_CYLINDER, LOW);
          break;
        case 1:
          digitalWrite(PNEUMATIC_CYLINDER, HIGH);
          break;
      }
    }
    else if(command_type=='v'){ 
      int command = command_str[1]-'0';
      switch(command){
        case 0:
          digitalWrite(VACUUM_GENERATOR, LOW);
          delay(1);
          digitalWrite(VACUUM_BREAKER, LOW);
          break;
        case 1:
          digitalWrite(VACUUM_GENERATOR, LOW);
          delay(1);
          digitalWrite(VACUUM_BREAKER, HIGH);
          break;
        case 2:
          digitalWrite(VACUUM_GENERATOR, HIGH);
          delay(1);
          digitalWrite(VACUUM_BREAKER, LOW);
          break;
        case 3:
          digitalWrite(VACUUM_GENERATOR, HIGH);
          delay(1);
          digitalWrite(VACUUM_BREAKER, HIGH);
          break;
      }
    }
    else if(command_type=='c'){
      digitalWrite(CONVRYOR_CONTROL, HIGH);
      delay(100);
      digitalWrite(CONVRYOR_CONTROL, LOW);
    }
    else if(command_type='s'){
      int tmp=0, times=10;
      for(int i=0; i<times; ++i){
        tmp += analogRead(PRESSURE_TRANSMITTER);
        delay(1);
      } Serial.println(tmp/times);
    }
  }
  delay(10);
}
