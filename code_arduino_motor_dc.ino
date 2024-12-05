// Deklarasi pin untuk L298N
const int enA = 9;   // Enable pin untuk kontrol kecepatan PWM
const int in1 = 8;   // Input 1 untuk arah motor
const int in2 = 7;   // Input 2 untuk arah motor

// Deklarasi pin untuk sensor enkoder
const int encoderA = 2;  // Pin interrupt untuk encoder
const int encoderB = 3;  // Pin kedua encoder untuk arah rotasi

// Variabel encoder
volatile long encoderCount = 0;  // Hitungan pulsa encoder
volatile bool encoderDirection = true;  // Arah rotasi
int encoderMultiplier = 1;  // Untuk memastikan RPM selalu positif

// Variabel PID
double set_speed = 300;        // Kecepatan target (RPM)
double kp = 1.0, ki = 0.5, kd = 0.1; // Parameter PID
double pv_speed = 0;           // Kecepatan aktual motor
double e_speed = 0, e_speed_pre = 0, e_speed_sum = 0; // Error PID
double pwm_pulse = 0;          // Nilai PWM

// Timer
unsigned long prev_time = 0;  
const unsigned long interval = 100; // Interval PID (ms)

// Status motor
bool motor_running = false;  
bool motor_forward = true;  // Variabel arah motor

void setup() {
  // Inisialisasi pin L298N
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Inisialisasi pin encoder
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  
  // Attach interrupt untuk encoder
  attachInterrupt(digitalPinToInterrupt(encoderA), countEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB), checkDirection, CHANGE);
  
  Serial.begin(115200);  // Komunikasi serial pada 115200 baud
  Serial.println("Motor Control System Initialized");
}

void loop() {
  // Proses input serial
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    parseInput(input);
  }
  
  if (motor_running) {
    unsigned long current_time = millis();
    if (current_time - prev_time >= interval) {
      prev_time = current_time;
      
      // Hitung RPM (disesuaikan dengan karakteristik encoder)
      // Gunakan abs() untuk memastikan RPM selalu positif
      pv_speed = abs((encoderCount * 60.0) / 11 * encoderMultiplier);
      encoderCount = 0; 
      
      // Hitung error dan PID
      e_speed = set_speed - pv_speed;
      e_speed_sum += e_speed; 
      double d_speed = e_speed - e_speed_pre; 
      e_speed_pre = e_speed;
      
      // Hitung PWM
      pwm_pulse = (kp * e_speed) + (ki * e_speed_sum) + (kd * d_speed);
      pwm_pulse = constrain(pwm_pulse, 0, 255);
      
      // Kontrol motor dengan L298N
      if (pwm_pulse > 0) {
        if (motor_forward) {
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          encoderMultiplier = 1;  // Maju positif
        } else {
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          encoderMultiplier = 1;  // Mundur tetap positif
        }
        analogWrite(enA, pwm_pulse);
      } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(enA, 0);
      }
      
      // Kirim data untuk plot
      Serial.print(pv_speed); 
      Serial.print(",");
      Serial.print(e_speed);   
      Serial.print(",");
      Serial.println(pwm_pulse);
    }
  } else {
    // Matikan motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }
}

// Fungsi interrupt untuk menghitung encoder
void countEncoder() {
  // Hitung pulsa encoder
  encoderCount++;
}

// Fungsi untuk menentukan arah rotasi
void checkDirection() {
  encoderDirection = digitalRead(encoderB) == HIGH;
}

// Fungsi untuk memproses data serial
void parseInput(String input) {
  input.trim();
  
  if (input.equals("STOP")) {
    motor_running = false;
    Serial.println("Motor Stopped");
  } else if (input.equals("START")) {
    motor_running = true;
    Serial.println("Motor Running");
  } else if (input.equals("FORWARD")) {
    motor_forward = true;
    Serial.println("Motor Direction: Forward");
  } else if (input.equals("REVERSE")) {
    motor_forward = false;
    Serial.println("Motor Direction: Reverse");
  } else {
    // Parsing parameter PID dan setpoint
    int firstComma = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1);
    int thirdComma = input.indexOf(',', secondComma + 1);
    
    if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
      kp = input.substring(0, firstComma).toFloat();
      ki = input.substring(firstComma + 1, secondComma).toFloat();
      kd = input.substring(secondComma + 1, thirdComma).toFloat();
      set_speed = input.substring(thirdComma + 1).toFloat();
      
      Serial.print("PID Parameters Updated - ");
      Serial.print("Kp: "); Serial.print(kp);
      Serial.print(", Ki: "); Serial.print(ki);
      Serial.print(", Kd: "); Serial.print(kd);
      Serial.print(", Set Speed: "); Serial.println(set_speed);
    }
  }
}