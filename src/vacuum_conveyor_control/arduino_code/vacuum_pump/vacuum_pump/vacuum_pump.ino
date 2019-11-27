#define VALVE   4
#define SENSOR A1

void setup() {
  // put your setup code here, to run once:
  pinMode(VALVE, OUTPUT);
  pinMode(SENSOR, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(Serial.available()){
    char msg;
    msg = Serial.read();
    if(msg=='o'){
      digitalWrite(4, HIGH);
      delay(1);
    } else if(msg=='c'){
      digitalWrite(4, LOW);
      delay(1);
    } else if(msg=='s'){
      int sensor_value = 0;
      for(int i=0; i<20; ++i){
        sensor_value += analogRead(SENSOR);
        delay(1);
      }
      Serial.println(sensor_value/20);
    }
  }
  delay(10);
}
