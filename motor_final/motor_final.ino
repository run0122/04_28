const int motor_A1 = 9;
const int motor_A2 = 8;
const int motor_B1 = 10;
const int motor_B2 = 11;
const int IR_R = A1;
const int IR_M = A3;
const int IR_L = A5;
int IR_L_data;
int IR_M_data;
int IR_R_data;

const int trigPin = 7;
const int echoPin = 6;

float duration, distance;

void setup() {
  pinMode(motor_A1, OUTPUT);
  pinMode(motor_A2, OUTPUT);
  pinMode(motor_B1, OUTPUT);
  pinMode(motor_B2, OUTPUT);
  pinMode(IR_L, INPUT);
  pinMode(IR_M, INPUT);
  pinMode(IR_R, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // Serial.begin(9600);
}


void loop() {
  //IR 센서 값을 읽어 출력해주는 코드
  // IR_L_data = analogRead(IR_L);
  // IR_M_data = analogRead(IR_M);
  // IR_R_data = analogRead(IR_R);
  IR_L_data = digitalRead(IR_L);
  IR_M_data = digitalRead(IR_M);
  IR_R_data = digitalRead(IR_R);

  // Serial.println((String)IR_L_data +"-"+IR_M_data+"-"+IR_R_data);

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration * .0343) / 2;

  if (distance < 15) {
    turn();
  } else {
    if (IR_L_data == 0 and IR_M_data == 1 and IR_R_data == 0) {
      forward();
    } else if (IR_L_data == 1 and IR_M_data == 0 and IR_R_data == 0) {
      left();
    } else if (IR_L_data == 0 and IR_M_data == 0 and IR_R_data == 1) {
      right();
    } else if (IR_L_data == 1 and IR_M_data == 1 and IR_R_data == 0) {
      smooth_left();
    } else if (IR_L_data == 0 and IR_M_data == 1 and IR_R_data == 1) {
      smooth_right();
    } else if (IR_L_data == 1 and IR_R_data == 1) {
      stop();
    }
  }
}

void smooth_right() {
  analogWrite(motor_A1, 255);
  digitalWrite(motor_A2, LOW);
  analogWrite(motor_B1, 220);
  digitalWrite(motor_B2, LOW);
}

void smooth_left() {
  analogWrite(motor_A1, 220);
  digitalWrite(motor_A2, LOW);
  analogWrite(motor_B1, 255);
  digitalWrite(motor_B2, LOW);
}

void right() {
  //우
  digitalWrite(motor_A1, HIGH);
  digitalWrite(motor_A2, LOW);
  digitalWrite(motor_B1, LOW);
  digitalWrite(motor_B2, LOW);
}

void left() {
  //좌
  digitalWrite(motor_A1, LOW);
  digitalWrite(motor_A2, LOW);
  digitalWrite(motor_B1, HIGH);
  digitalWrite(motor_B2, LOW);
}

void forward() {
  digitalWrite(motor_A1, HIGH);
  digitalWrite(motor_A2, LOW);
  digitalWrite(motor_B1, HIGH);
  digitalWrite(motor_B2, LOW);
}

void stop() {
  digitalWrite(motor_A1, LOW);
  digitalWrite(motor_A2, LOW);
  digitalWrite(motor_B1, LOW);
  digitalWrite(motor_B2, LOW);
}

void turn() {
  millifirstleft(500);
  milliforward(1200);
  milliright(500);
  milliforward(1500);
  milliright(500);
  milliforward(700);
  millilastforward();
}

void millifirstleft(unsigned long x) {
  unsigned long current_time = millis();
  unsigned long interval = x;
  while (millis() - current_time < interval) {
    hardleft();
  }
}

void milliright(unsigned long x) {
  unsigned long current_time = millis();
  unsigned long interval = x;
  while (millis() - current_time < interval) {
    hardright();
  }
}
void millileft(unsigned long x) {
  unsigned long current_time = millis();
  unsigned long interval = x;
  while (millis() - current_time < interval) {
    hardleft();
    if (IR_L_data == 1 or IR_M_data == 1 or IR_R_data == 1) {
      return;
    }
  }
}
void milliforward(unsigned long x) {
  unsigned long current_time = millis();
  unsigned long interval = x;
  while (millis() - current_time < interval) {
    hardforward();
  }
}
void millilastforward() {
  while (IR_L_data == 0 and IR_L_data == 0 and IR_L_data == 0) {
    forward();
    if (IR_L_data == 1 or IR_M_data == 1 or IR_R_data == 1) {
      stop();
      break;
    }
  }
  millilastleft(200);
}
void millilastleft(unsigned long x) {
  unsigned long current_time = millis();
  unsigned long interval = x;
  while (millis() - current_time < interval) {
    hardleft();
  }
  left();
  if (IR_L_data == 1 or IR_M_data == 1 or IR_R_data == 1) {
    return;
  }
}

void hardright() {
  //우
  digitalWrite(motor_A1, HIGH);
  digitalWrite(motor_A2, LOW);
  digitalWrite(motor_B1, LOW);
  digitalWrite(motor_B2, HIGH);
}

void hardleft() {
  //좌
  digitalWrite(motor_A1, LOW);
  digitalWrite(motor_A2, HIGH);
  digitalWrite(motor_B1, HIGH);
  digitalWrite(motor_B2, LOW);
}

void hardforward() {
  digitalWrite(motor_A1, HIGH);
  digitalWrite(motor_A2, LOW);
  digitalWrite(motor_B1, HIGH);
  digitalWrite(motor_B2, LOW);
}