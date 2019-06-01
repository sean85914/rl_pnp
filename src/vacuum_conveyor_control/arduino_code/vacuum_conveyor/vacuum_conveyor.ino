/*
 *  Vacuum control
 *  Value  Vacuum Generator  Vacuum Breaker
 *      0                 0               0
 *      1                 0               1
 *      2                 1               0
 *      3                 1               1
 */

#define VACUUM_GENERATOR 12
#define VACUUM_BREAKER    8
#define CONVRYOR_CONTROL  4

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(VACUUM_GENERATOR, OUTPUT);
  pinMode(VACUUM_BREAKER, OUTPUT);
  pinMode(CONVRYOR_CONTROL, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    if(Serial.read() == 'v'){ // Vacuum
      int value = Serial.readString().toInt();
      switch(value){
        case 0:
          digitalWrite(VACUUM_GENERATOR, LOW);
          digitalWrite(VACUUM_BREAKER, LOW);
          //Serial.println("v: 0");
          break;
        case 1:
          digitalWrite(VACUUM_GENERATOR, LOW);
          digitalWrite(VACUUM_BREAKER, HIGH);
          //Serial.println("v: 1");
          break;
        case 2:
          digitalWrite(VACUUM_GENERATOR, HIGH);
          digitalWrite(VACUUM_BREAKER, LOW);
          //Serial.println("v: 2");
          break;
        case 3:
          digitalWrite(VACUUM_GENERATOR, HIGH);
          digitalWrite(VACUUM_BREAKER, HIGH);
          //Serial.println("v: 3");
          break;
      }
    }
    else { // Conveyor
      int value = Serial.readString().toInt();
      if(value==1){
        //Serial.println("c: 1");
        digitalWrite(CONVRYOR_CONTROL, HIGH);
        delay(100);
        digitalWrite(CONVRYOR_CONTROL, LOW);
      }
    }
  }
  //delay(1);
}
