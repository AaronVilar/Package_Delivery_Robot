#include <Arduino.h>
#include <i2c_driver.h>
#include <i2c_driver_wire.h>
#include <FreqMeasureMulti.h>
#define MEASURE_TYPE FREQMEASUREMULTI_FALLING

void receive_event(int how_many);
void send_event();
int read_data;

int PWM_pin_S = 1;
int DIR_pin_S = 2;
int PWM_pin_F = 3;
int DIR_pin_F = 4;

bool DIR_control_R = false;
bool DIR_control_L = true;

int encoder_pin_S = 22;
int encoder_pin_F = 6;

unsigned long sampleTimer1, sampleTimer2, moveTimer1, moveTimer2;
unsigned long movePeriod = 1000;
unsigned long stopPeriod = 1000;
unsigned long samplePeriod = 25;

float motor_set_F , motor_set_S = 0; // must be at least 304 (pwm) to overcome static friction
float motor_rpm_F, motor_rpm_S = 0;
word output_rpm_L, output_rpm_R = 0;
float rpm_gain = 3.413; // average (8.78) pwm per motor_rpm at "operating speed"
float count_per_wheel_rev = 897.96; // 74.83 * 12
float motor_rpm;
float error;
float max_speed = 1200;
int pwm_reso = 4095;
float max_acceleration = 3000; // motor_rpm/s^2
float set_point_accel;
float prev_error_F, prev_error_S = 0;

float sum_F = 0, sum_S = 0;
int count_F = 0, count_S = 0;
float control_out_F = 0, control_out_S = 0;
float set_point_L, set_point_R = 0;
float set_point = 0;
int set_point_int = 0;
int sign_bit_L, sign_bit_R = 0;
int sign_hist_L, sign_hist_R = 0;

bool began_moving_L, began_moving_R = false;

FreqMeasureMulti encoder_F;
FreqMeasureMulti encoder_S;

String outputData;

int counts2rpm(int counts) {
  motor_rpm = (100 * counts) / count_per_wheel_rev;
  return motor_rpm; // motor rpm * 100
}

float PID_controller_F(float set_point, float motor_rpm) {
  float error_P = set_point - motor_rpm;
  float error_D = error_P - prev_error_F;
  prev_error_F = error_P;
  float Kp = 0.7;
  float Kd = 0.1;
  float control_out = (Kp * error_P) - (Kd * error_D);
  return control_out;
}

float PID_controller_S(float set_point, float motor_rpm) {
  float error_P = set_point - motor_rpm;
  float error_D = error_P - prev_error_S;
  prev_error_S = error_P;
  float Kp = 0.7;
  float Kd = 0.1;
  float control_out = (Kp * error_P) - (Kd * error_D);
  return control_out;
}

void setup() {

  Wire1.begin(8);
  Wire1.onReceive(receive_event);
  Wire1.onRequest(send_event);
  Serial.begin(115200); delay(10);

  analogWriteFrequency(PWM_pin_S, 36621);
  analogWriteFrequency(PWM_pin_F, 36621);
  analogWriteResolution(12);
  
  encoder_F.begin(encoder_pin_F);
  encoder_S.begin(encoder_pin_S);
  
  pinMode(PWM_pin_F, OUTPUT);
  pinMode(DIR_pin_F, OUTPUT);
  digitalWrite(DIR_pin_F, DIR_control_L);

  pinMode(PWM_pin_S, OUTPUT);
  pinMode(DIR_pin_S, OUTPUT);
  digitalWrite(DIR_pin_S, DIR_control_R);

  sampleTimer1 = millis();
  moveTimer1 = sampleTimer1;

  delay(100);
  
  set_point_L = 0;
  set_point_R = 0;

}

void loop() {
  if (encoder_F.available()) {
    began_moving_L = true;
    sum_F = sum_F + encoder_F.read();
    count_F = count_F + 1;
  } else {
    began_moving_L = false;
  }
  
  if (encoder_S.available()) {
    began_moving_R = true;
    sum_S = sum_S + encoder_S.read();
    count_S = count_S + 1;
  } else {
    began_moving_R = false;
  }

  sampleTimer2 = millis() - sampleTimer1;
  if (sampleTimer2 >= samplePeriod) {
    
    if (set_point_L > 0 && not began_moving_L) {
      Serial.println("begin left wheel");
      analogWrite(PWM_pin_S, 200 * rpm_gain);
    } else if (set_point_L == 0 || count_F == 0) {
      began_moving_L = false;
      analogWrite(PWM_pin_F, 0);
    }
    if (set_point_R > 0 && not began_moving_R) {
      Serial.println("begin right wheel");
      analogWrite(PWM_pin_S, 200 * rpm_gain);
    } else if (set_point_R == 0  || count_S == 0) {
      began_moving_R = false;
      analogWrite(PWM_pin_S, 0);
    }
    
    if (set_point_L > (max_speed - 200)) {
      set_point_L = max_speed - 200;
    }
    
    if (set_point_R > (max_speed - 200)) {
      set_point_R = max_speed - 200;
    }
    if (began_moving_L) {
      Serial.print (set_point_L);
      Serial.print(" ");
      motor_rpm = encoder_F.countToFrequency(sum_F / count_F);
      Serial.print(motor_rpm);
      output_rpm_L = motor_rpm * 10;
      control_out_F = PID_controller_F(set_point_L, motor_rpm);
      
      motor_set_F += control_out_F;
      if (motor_set_F > max_speed) {
        motor_set_F = max_speed;
      }
      else if (motor_set_F < 0) {
        motor_set_F = 0;
      }

      if (sign_bit_L != sign_hist_L) {
        sign_hist_L = sign_bit_L;
        DIR_control_L = not DIR_control_L;
        digitalWrite(DIR_pin_F, DIR_control_L);
      }
      analogWrite(PWM_pin_F, motor_set_F * rpm_gain);
      // analogWrite(PWM_pin_F, 4095);
      
      sum_F = 0;
      count_F = 0;
    }

    if (began_moving_R) {
      Serial.print(" ");
      Serial.print(set_point_R);
      Serial.print(" ");
      motor_rpm = encoder_S.countToFrequency(sum_S / count_S);
      Serial.println(motor_rpm);
      output_rpm_R = motor_rpm * 10;
      control_out_S = PID_controller_S(set_point_R, motor_rpm);
      
      motor_set_S += control_out_S;
      if (motor_set_S > max_speed) {
        motor_set_S = max_speed;
      }
      else if (motor_set_S < 0) {
        motor_set_S = 0;
      }

      if (sign_bit_R != sign_hist_R) {
        sign_hist_R = sign_bit_R;
        DIR_control_R = not DIR_control_R;
        digitalWrite(DIR_pin_S, DIR_control_R);
      }
      Serial.println(motor_set_S);
      analogWrite(PWM_pin_S, motor_set_S * rpm_gain);
      // analogWrite(PWM_pin_S, 4095);
      
      sum_S = 0;
      count_S = 0;
    }
    sampleTimer1 = millis();
  }  
}

void receive_event(int how_many) {

  
  int control_byte = Wire1.read();
  if (control_byte == 16) {
  }
  else {
    int set_L_int;
    int set_R_int;
    int data_length = Wire1.read();
    
    if (control_byte == 10 && data_length == 4 ) {
      sign_bit_L = Wire1.read();
      sign_bit_R = Wire1.read();
      
      set_L_int = Wire1.read();
      set_R_int = Wire1.read();
    }
    else if (control_byte == 10 && data_length == 6) {
      sign_bit_L = Wire1.read();
      sign_bit_R = Wire1.read();
      
      set_L_int = Wire1.read();
      set_L_int = set_L_int << 8;
      set_L_int += Wire1.read();
  
      set_R_int = Wire1.read();
      set_R_int = set_R_int << 8;
      set_R_int += Wire1.read();
    }
    Serial.println(String(sign_bit_L) + " " + String(sign_bit_R) + " " + String(set_L_int) + " " + String(set_R_int));
    set_point_L = float(set_L_int);
    set_point_R = float(set_R_int);
  }
}

void send_event() {
  Serial.println(String(output_rpm_L) + " " + String(output_rpm_R));

  byte data_lower_L = lowByte(output_rpm_L);
  byte data_upper_L = highByte(output_rpm_L);
  byte data_lower_R = lowByte(output_rpm_R);
  byte data_upper_R = highByte(output_rpm_R);
  // Serial.println(String(data_upper) + " " + String(data_lower));
  
  Wire1.write(data_upper_L);
  Wire1.write(data_lower_L);
  Wire1.write(data_upper_R);
  Wire1.write(data_lower_R);
  
}
