#define MOT_DX_STEP 19
#define MOT_DX_DIR 18
#define MOT_SX_STEP 17
#define MOT_SX_DIR 16

void setup_motors(){
  pinMode(MOT_DX_STEP, OUTPUT);
  pinMode(MOT_DX_DIR, OUTPUT);
  pinMode(MOT_SX_STEP, OUTPUT);
  pinMode(MOT_SX_DIR, OUTPUT);

  Serial.println("Motors up");
}

/*
void test_motors(){
  while(!Serial) delay(10);

  int m = 0;
  while(true){
    digitalWrite(mot_pins[m][0], LOW);
    digitalWrite(mot_pins[m][1], HIGH);
    
    for(int i = 0; i < 255; i++){
      Serial.print("PWM: ");
      Serial.println(i);
      analogWrite(mot_pins[m][2], i);
      delay(100);
    }
    delay(1000);
    digitalWrite(mot_pins[m][0], LOW);
    digitalWrite(mot_pins[m][1], LOW);
    m = 1-m;
  }
}

void test_motors_nodeadzone(){
  int m = 0;
  while(true){
    for(int i = 0; i < 100; i++){
      Serial.print("Speed: ");
      Serial.println(i);
      move(m, i);
      delay(100);
    }
    delay(1000);
    move(m, 0);
    m = 1-m;
  }
}

void test_motors_rpm(){
  while(!Serial) delay(10);

  unsigned long tstart = millis();
  while(millis() - tstart < 200) {
  move(0, 20);
  }
  move(0,0);
}

void test_motors_diff(){
  move(MOT_SX, 50*MOT_SX_MULT);
  move(MOT_DX, 50*MOT_DX_MULT);
}


void test_motors_torque(int motor, int dir){
  while(!Serial){;}
  
  while(1){
    delay(1000);
    Serial.println("Setup, waiting 8 secs");
    delay(8000);
    
  for(int i = 0; i <= 255; i+= 15){
    digitalWrite(mot_pins[motor][1], dir > 0 ? LOW : HIGH);
    digitalWrite(mot_pins[motor][0], dir > 0 ? HIGH : LOW);
    Serial.print("PWM: ");
    Serial.println(i);

    analogWrite(mot_pins[motor][2], i);
    delay(8000);
    
    digitalWrite(mot_pins[motor][0], LOW);
    digitalWrite(mot_pins[motor][1], LOW);
    delay(2000);
    
    }
  }
}
*/