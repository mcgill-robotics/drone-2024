#include <micro_ros_arduino.h>
#include <Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#define MOTOR1_PIN1 52
#define MOTOR1_PIN2 53
#define EN_MOTOR1 8
#define SERVO1_PIN 2

#define MOTOR2_PIN1 50
#define MOTOR2_PIN2 51
#define EN_MOTOR2 9
#define SERVO2_PIN 3

#define MOTOR3_PIN1 48
#define MOTOR3_PIN2 49
#define EN_MOTOR3 10
#define SERVO3_PIN 4

#define MOTOR4_PIN1 46
#define MOTOR4_PIN2 47
#define EN_MOTOR4 11
#define SERVO4_PIN 5

#define MOTOR5_PIN1 44
#define MOTOR5_PIN2 45
#define EN_MOTOR5 12
#define SERVO5_PIN 6

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

class AirdropMechanism {
  public:
    AirdropMechanism() {}

    AirdropMechanism(int motor_pin1, int motor_pin2, int en_motor, int servo_pin, bool opens_clock_wise):
      motor_pin1(motor_pin1), motor_pin2(motor_pin2), en_motor(en_motor), opens_clock_wise(opens_clock_wise) {
      // motor speed pin
      pinMode(en_motor, OUTPUT);
      // motor_pin1 and 2 are used for the orientation of the spining of the motor
      pinMode(motor_pin1, OUTPUT);
      pinMode(motor_pin2, OUTPUT);
      // servo motor
      servo.attach(servo_pin);
      disarm();
    }

    void arm() {
      if (this->opens_clock_wise) {
        servo.write(20);
      } else {
        servo.write(160);
      }
    }

    void disarm() {
      servo.write(90);
    }

    void perform_airdrop(int drop_speed, int drop_time) {
      // bring bottle up a little
      analogWrite(en_motor, 100);
      digitalWrite(motor_pin1, LOW);
      digitalWrite(motor_pin2, HIGH);
      // WAIT
      delay(1000);
      // retract locking pin
      analogWrite(en_motor, 0);
      disarm();
      // WAIT
      delay(1000);      
      // drop bottle
      digitalWrite(motor_pin1, HIGH);
      digitalWrite(motor_pin2, LOW);
      delay(100);
      analogWrite(en_motor, drop_speed);

      delay(drop_time);

      // stop motor
      analogWrite(en_motor, 0);
      delay(1000);
    }

  private:
    int motor_pin1, motor_pin2;
    int en_motor;
    bool opens_clock_wise;
    Servo servo;
};

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


rcl_subscription_t subscriber_arm;
rcl_subscription_t subscriber_drop;
std_msgs__msg__Int32 msg_arm;
std_msgs__msg__Int32 msg_drop;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


AirdropMechanism airdrops[5];

void drop_subscription_callback(const void * msgin)
{
  static int drop_speed = 220; // Modify this to change drop speed. NOTE: THE MAXIMUM ALLOWED BY ARDUINO IS 255, BUT THIS WILL LIKELY BE ABOVE THE MAX VOLTAGE OF THE MOTOR. CHECK OUTPUT VOLTAGE BEFORE GIVING IT TO THE MOTOR
  static int drop_time = 5000; // Modify this to change the length of the drop
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int input = msg->data;
  if (input < 1 || input > 5) return;
  AirdropMechanism requested_airdrop = airdrops[input - 1];
  requested_airdrop.perform_airdrop(drop_speed, drop_time);
}

void arming_subscription_callback(const void* msgin) {
  for (int i = 0; i < 5; i++) {
    airdrops[i].arm();
  }
}


void setup() {
  set_microros_transports();
  airdrops[0] = {MOTOR1_PIN1, MOTOR1_PIN2, EN_MOTOR1, SERVO1_PIN, false};
  airdrops[1] = {MOTOR2_PIN1, MOTOR2_PIN2, EN_MOTOR2, SERVO2_PIN, true};
  airdrops[2] = {MOTOR3_PIN1, MOTOR3_PIN2, EN_MOTOR3, SERVO3_PIN, false};
  airdrops[3] = {MOTOR4_PIN1, MOTOR4_PIN2, EN_MOTOR4, SERVO4_PIN, true};
  airdrops[4] = {MOTOR5_PIN1, MOTOR5_PIN2, EN_MOTOR5, SERVO5_PIN, false};

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
            &subscriber_drop,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "drop_bottle"));

  RCCHECK(rclc_subscription_init_default(
            &subscriber_arm,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "arm_drop"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_drop, &msg_drop, &drop_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_arm, &msg_arm, &arming_subscription_callback, ON_NEW_DATA));
}

void loop() {
  rclc_executor_spin(&executor);
}
