/*******************************************************************************
  Copyright 2016 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
#define DEVICE_NAME "1" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
#define DEVICE_NAME ""
#endif

#define BAUDRATE  57600
#define DXL_ID_LEFT    1
#define DXL_ID_RIGHT   2

char imu_frame_id[30];
char mag_frame_id[30];

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "lino_velocities.h"
#include "geometry_msgs/Twist.h"
#include "Kinematics.h"

#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
#define MAX_RPM 100             // motor's maximum RPM 55
#define WHEEL_DIAMETER 0.066       // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.135  // distance between left and right wheels 0.800
#define FR_WHEELS_DISTANCE 0.125   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN

#define COMMAND_RATE 20
#define ODOM_RATE 20
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    0.5   //hz
#define IO_RATE   10   //hz
#define LOWBAT_NOTIFY_RATE   1   //hz

#define BDPIN_BAT_PWR_ADC A0
#define led_pin 14
#define buzzer_pin 13

#define STRING_BUF_NUM 64
String cmd[STRING_BUF_NUM];

uint8_t get_id[16];
uint8_t scan_cnt = 0;
uint8_t ping_cnt = 0;

bool isAvailableID(uint8_t id);
void split(String data, char separator, String* temp);
void printInst();

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;
unsigned long g_prev_command_time = 0;
static uint32_t tTime[10];
float voltages;
float voltages_last;
String dataVoltage = "";
String dataVoltage_last = "";

int currentLeftWheelRPM;
int currentRightWheelRPM;
std_msgs::Int32 rpmLeft;
std_msgs::Int32 rpmRight;
std_msgs::String analog;
std_msgs::String input;
std_msgs::Int16MultiArray inputAN;
std_msgs::Bool sw_ok;
std_msgs::Float32 voltage_msg;

uint8_t dxl_id[2] = {DXL_ID_LEFT, DXL_ID_RIGHT};
uint8_t dxl_cnt = 2;

int led_state = 0;
bool FIND_MOTOR[2] = {false, false};

void commandCallback(const geometry_msgs::Twist& cmd_msg);
void messageCb_output( const std_msgs::String& cmd_msg);
void moveBase();


void waitForSerialLink(bool isConnected);



ros::NodeHandle nh;

geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<std_msgs::String> output_sub("output", messageCb_output );

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
ros::Publisher rpmLeft_pub("rpmLeft", &rpmLeft);
ros::Publisher rpmRight_pub("rpmRight", &rpmRight);
ros::Publisher input_pub("input", &input);
ros::Publisher inputAN_pub("inputAN", &inputAN);
ros::Publisher pub_voltage("voltage", &voltage_msg);

unsigned long prev_update_time_m1 = 0;
unsigned long prev_update_time_m2 = 0;

unsigned long pulseRPM1 = 0;
unsigned long pulseRPM2 = 0;

unsigned long prev_update_time_M1 = 0;
unsigned long prev_update_time_M2 = 0;

unsigned long prev_encoder_ticks_M1 = 0;
unsigned long prev_encoder_ticks_M2 = 0;
int counts_per_rev_ = 15;

int stChang = 0;

DynamixelWorkbench dxl_wb;
void setLeftRPM(int rpm) {
  bool result = false;
  int32_t get_data = 0;
  result = dxl_wb.itemRead(DXL_ID_LEFT, "Present_Velocity", &get_data);
  if (result == true)
  {
    int32_t speed = get_data;
    double rpm1 = (double) speed * 0.229;
    //Serial.print("PresVel Left: ");
    //Serial.print(rpm1);
    //Serial.println("");
    currentLeftWheelRPM = (int) rpm1;
    int32_t velocity = rpm * 4.45; //(100 / 22.45) = 4.45
    dxl_wb.goalVelocity(DXL_ID_LEFT, velocity);
  }
}
void setRightRPM(int rpm) {
  bool result = false;
  int32_t get_data = 0;
  result = dxl_wb.itemRead(DXL_ID_RIGHT, "Present_Velocity", &get_data);
  if (result == true) {
    int32_t speed = get_data;
    double rpm1 = (double) speed * 0.229;
    //Serial.print("PresVel Right: ");
    //Serial.print(rpm1);
    //Serial.println("");
    currentRightWheelRPM = (int) rpm1;
    int32_t velocity = rpm * 4.45; //(100 / 22.45)=4.45
    dxl_wb.goalVelocity(DXL_ID_RIGHT, - velocity);
  }
}
void messageCb_output( const std_msgs::String& cmd_msg) {
  // a10,b16,c17,d18,e19,f20,g21,h22
  String data = String(cmd_msg.data);
  //10,15,16,17,18,19,20,21,22;
  //10h
  data.trim();
  data.toUpperCase();
  char* Str3 = (char*)data.c_str();
  int pin = data.toInt();
  int logic = (Str3[2] == 'H' ? HIGH : LOW);
  digitalWrite(pin, logic);
  /*
    if (data.equals("A")) {
    digitalWrite(10, HIGH);
    } else if (data.equals("a")) {
    digitalWrite(10, LOW);
    } else if (data.equals("B")) {
    digitalWrite(16, HIGH);
    } else if (data.equals("b")) {
    digitalWrite(16, LOW);
    } else if (data.equals("C")) {
    digitalWrite(17, HIGH);
    } else if (data.equals("c")) {
    digitalWrite(17, LOW);
    } else if (data.equals("D")) {
    digitalWrite(18, HIGH);
    } else if (data.equals("d")) {
    digitalWrite(18, LOW);
    } else if (data.equals("E")) {
    digitalWrite(19, HIGH);
    } else if (data.equals("e")) {
    digitalWrite(19, LOW);
    } else if (data.equals("F")) {
    digitalWrite(20, HIGH);
    } else if (data.equals("f")) {
    digitalWrite(20, LOW);
    } else if (data.equals("G")) {
    digitalWrite(21, HIGH);
    } else if (data.equals("g")) {
    digitalWrite(21, LOW);
    } else if (data.equals("H")) {
    digitalWrite(22, HIGH);
    } else if (data.equals("h")) {
    digitalWrite(22, LOW);
    }
  */
}
void setup()
{
  delay(3000);
  pinMode(led_pin, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  tone(buzzer_pin, 659, 200);
  delay(200);
  tone(buzzer_pin, 880, 100);


  dxl_wb.begin(DEVICE_NAME, BAUDRATE);

  for (int cnt = 0; cnt < dxl_cnt; cnt++)
  {
    FIND_MOTOR[cnt] = dxl_wb.ping(dxl_id[cnt]);
    dxl_wb.jointMode(dxl_id[cnt]);

  }
  if (!(FIND_MOTOR[0] && FIND_MOTOR[1])) {
    Serial.begin(115200);
    while (1) {
      set_idMotor();
    }

  }
  dxl_wb.wheelMode(DXL_ID_LEFT);
  dxl_wb.wheelMode(DXL_ID_RIGHT);



  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.subscribe(output_sub);

  // nh.advertise(raw_vel_pub);
  // nh.advertise(raw_vel_pub);

  /*nh.advertise(raw_IMUpub);
    nh.advertise(raw_IMUdata_pub);*/
#ifdef IMU_ON
  nh.advertise(imu_pub);
  nh.advertise(raw_mag_data_pub);
#endif

  // nh.advertise(rpmLeft_pub);
  // nh.advertise(rpmRight_pub);
  //nh.advertise(input_pub);
  nh.advertise(inputAN_pub);
  nh.advertise(pub_voltage);
  // while(!Serial); // If this line is activated, you need to open Serial Terminal.

  sprintf(imu_frame_id, "imu_link");
  sprintf(mag_frame_id, "imu_link");


}

/*******************************************************************************
  ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

void loop()
{
  //DEBUG_SERIAL.println("ROS Start.....");
  Kinematics::velocities current_vel;
  static unsigned long prev_control_time = 0;
  static unsigned long prev_odom_time = 0;
  static unsigned long prev_infor_time = 0;
  uint32_t t = millis();
  if ((t - g_prev_command_time) >= 3000) {
    stopBase();
  }
  //this block drives the robot based on defined rate
  if ((t - tTime[0]) >= (1000 / COMMAND_RATE)) {
    moveBase();
    //sprintf (buffer, "Encoder FrontLeft  : %i Encoder FrontRight  : %i ", currentLeftWheelRPM, currentRightWheelRPM);
    //nh.loginfo(buffer);
    rpmLeft.data = currentLeftWheelRPM;
    rpmRight.data = currentRightWheelRPM;
    //  rpmLeft_pub.publish(&rpmLeft);
    //  rpmRight_pub.publish(&rpmRight);

    tTime[0] = t;

  }

  if ((t - tTime[1]) >= (1000 / ODOM_RATE)) {

    int current_rpm1 = currentLeftWheelRPM; //rightWheel.getRPM();
    int current_rpm2 = currentRightWheelRPM; //leftWheel.getRPM();
    int current_rpm3 = 0;
    int current_rpm4 = 0;


    current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
    //current_vel = kinematics.getVelocities(50, 50, 0, 0);

    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;
    //ros::Time stamp_now = nh.now();
    //raw_vel_msg.header.stamp = stamp_now;
    //publish raw_vel_msg

    // raw_vel_pub.publish(&raw_vel_msg);
    tTime[1] = t;
    digitalWrite(led_pin, (led_state) ? HIGH : LOW);
    led_state = !led_state ;
    led_state = led_state & (FIND_MOTOR[0] && FIND_MOTOR[1]);
  }

  if ((t - tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY)) {
    publishBatteryStateMsg();
    //publishSensorStateMsg();
    //publishDriveInformation();
    tTime[2] = t;
  }

  if ((t - tTime[3]) >= (1000 / IO_RATE)) {
    publishIOMsg();
    tTime[3] = t;
  }

  if ((t - tTime[4]) >= (1000 / LOWBAT_NOTIFY_RATE)) {
    voltages = getPowerInVoltage();
    if (voltages <= 7.5) {
      tone(buzzer_pin, 1760, 500);
    } else {
      noTone(buzzer_pin);
    }
    tTime[4] = t;
  }
  nh.spinOnce();
  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}

void stopBase()
{

  setRightRPM(0);
  setLeftRPM(0);
  g_req_linear_vel_x = 0;
  g_req_linear_vel_y = 0;
  g_req_angular_vel_z = 0;
}

void moveBase() {
  Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
  setLeftRPM(req_rpm.motor1);
  setRightRPM(req_rpm.motor2);

}
/*******************************************************************************
  Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;

  if (isConnected)
  {
    if (wait_flag == false)
    {
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
  //char buffer[40];
  g_req_linear_vel_x = cmd_msg.linear.x;
  g_req_linear_vel_y = cmd_msg.linear.y;
  g_req_angular_vel_z = cmd_msg.angular.z;
  //moveBase();
  //sprintf (buffer, "Encoder FrontLeft  : %i Encoder FrontRight  : %i ", currentLeftWheelRPM, currentRightWheelRPM);
  //nh.loginfo(buffer);

  g_prev_command_time = millis();

  //nh.spinOnce();
  // Wait the serial link time to process
  //waitForSerialLink(nh.connected());
}
float getPowerInVoltage(void)
{
  int adc_value;
  float vol_value;

  adc_value = analogRead(BDPIN_BAT_PWR_ADC);
  vol_value = map(adc_value, 0, 1023, 0, 330 * 57 / 10);
  vol_value = vol_value / 100.;

  return vol_value;
}
/*******************************************************************************
  Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  voltage_msg.data = getPowerInVoltage();
  pub_voltage.publish(&voltage_msg);
}
/*******************************************************************************
  Publish msgs (IO_state)
*******************************************************************************/
void publishIOMsg(void)
{
  //analog.data = "";
  /*
    String dataInput = "{\"1\":{A1},\"2\":{A2},\"3\":{A3},\"4\":{A4},\"5\":{A5},\"6\":{A6},\"7\":{A7},\"8\":{A8},\"9\":{A9}}";
    dataInput.replace("{A1}", String(analogRead(A1)));
    dataInput.replace("{A2}", String(analogRead(A2)));
    dataInput.replace("{A3}", String(analogRead(A3)));
    dataInput.replace("{A4}", String(analogRead(A4)));
    dataInput.replace("{A5}", String(analogRead(A5)));
    dataInput.replace("{A6}", String(analogRead(A6)));
    dataInput.replace("{A7}", String(analogRead(A7)));
    dataInput.replace("{A8}", String(analogRead(A8)));
    dataInput.replace("{A9}", String(analogRead(A9)));
    input.data = (char*)dataInput.c_str();
    //analog_pub.publish(&analog);
    input_pub.publish(&input);
  */
  //------------------------------------------------------------------------------
  int16_t value[9] = {(int16_t)analogRead(A1), (int16_t)analogRead(A2), (int16_t)analogRead(A3), (int16_t)analogRead(A4), (int16_t)analogRead(A5), (int16_t)analogRead(A6), (int16_t)analogRead(A7), (int16_t)analogRead(A8), (int16_t)analogRead(A9)};
  inputAN.data = value;
  inputAN.data_length = 9;
  inputAN_pub.publish(&inputAN);
  
}
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void set_idMotor() {
  const char *log = NULL;
  bool result = false;

  if (Serial.available())
  {
    String read_string = Serial.readStringUntil('\n');
    Serial.println("[CMD] : " + String(read_string));

    read_string.trim();

    split(read_string, ' ', cmd);

    if (cmd[0] == "help")
    {
      printInst();
    }
    else if (cmd[0] == "end")
    {
      return;
    }
    else if (cmd[0] == "t")
    {
      result = dxl_wb.wheelMode(1, 0, &log);
      result = dxl_wb.wheelMode(2, 0, &log);
      result = dxl_wb.goalVelocity(1, 200, &log);
      result = dxl_wb.goalVelocity(2, -200, &log);
    }
    else if (cmd[0] == "s")
    {
      result = dxl_wb.goalVelocity(1, 0, &log);
      result = dxl_wb.goalVelocity(2, 0, &log);
    }
    else if (cmd[0] == "scan")
    {
      if (cmd[1] == '\0')
        cmd[1] = String("253");

      uint8_t range = cmd[1].toInt();
      result = dxl_wb.scan(get_id, &scan_cnt, range);
      if (result == false)
      {
        Serial.println(log);
        Serial.println("Failed to scan");
      }
      else
      {
        Serial.print("Find ");
        Serial.print(scan_cnt);
        Serial.println(" Dynamixels");

        for (int cnt = 0; cnt < scan_cnt; cnt++)
        {
          Serial.print("id : ");
          Serial.print(get_id[cnt]);
          Serial.print(" model name : ");
          Serial.println(dxl_wb.getModelName(get_id[cnt]));
        }
      }
    }
    else if (cmd[0] == "c")
    {
      //uint8_t id = cmd[1].toInt();
      uint8_t id = 2;
      uint16_t model_number = 0;
      uint16_t set_model = cmd[2].toInt();
      if (cmd[1] == '\0')
        id = 2;
      if (cmd[2] == '\0')
        set_model = 0;
      result = dxl_wb.scan(get_id, &scan_cnt, 10);
      if (result == false)
      {
        Serial.println(log);
        Serial.println("Failed to scan");
      }
      else
      {
        Serial.print("Find ");
        Serial.print(scan_cnt);
        Serial.println(" Dynamixels");

        for (int cnt = 0; cnt < scan_cnt; cnt++)
        {
          Serial.print("id : ");
          Serial.print(get_id[cnt]);
          Serial.print(" model name : ");
          Serial.println(dxl_wb.getModelName(get_id[cnt]));
        }

        uint8_t id_s    = get_id[0];
        result = dxl_wb.changeID(id_s, id, &log);
        if (result == false)
        {
          Serial.println(log);
          return;
        }
        else
        {
          Serial.println(log);
        }
        result = dxl_wb.ping(id, &model_number, &log);
        if (result == false)
        {
          Serial.println(log);
          Serial.println("Failed to ping");
        }
        else
        {
          Serial.println("Succeeded to ping");
          Serial.print("id : ");
          Serial.print(id);
          Serial.print(" model_number : ");
          Serial.println(model_number);
        }

        if (set_model == 1) {
          result = dxl_wb.jointMode(id, 0, 0, &log);
          if (result == false)
          {
            Serial.println(log);
            Serial.println("Failed to change joint mode");
          }
          else
          {
            Serial.println("Succeed to change joint mode");
            Serial.println("Dynamixel is moving...");

            for (int count = 0; count < 3; count++)
            {
              dxl_wb.goalPosition(id, (int32_t)0);
              delay(3000);

              dxl_wb.goalPosition(id, (int32_t)1023);
              delay(3000);
            }
          }
        }
      }
    }
        else if (cmd[0] == "setid")
        {
          uint8_t id = cmd[1].toInt();
          uint16_t model_number = 0;
          uint16_t set_model = cmd[2].toInt();
          if (cmd[1] == '\0')
            id = 1;
          if (cmd[2] == '\0')
            set_model = 0;
          result = dxl_wb.scan(get_id, &scan_cnt, 10);
          if (result == false)
          {
            Serial.println(log);
            Serial.println("Failed to scan");
          }
          else
          {
            Serial.print("Find ");
            Serial.print(scan_cnt);
            Serial.println(" Dynamixels");

            for (int cnt = 0; cnt < scan_cnt; cnt++)
            {
              Serial.print("id : ");
              Serial.print(get_id[cnt]);
              Serial.print(" model name : ");
              Serial.println(dxl_wb.getModelName(get_id[cnt]));
            }

            uint8_t id_s    = get_id[0];
            result = dxl_wb.changeID(id_s, id, &log);
            if (result == false)
            {
              Serial.println(log);
              return;
            }
            else
            {
              Serial.println(log);
            }
            result = dxl_wb.ping(id, &model_number, &log);
            if (result == false)
            {
              Serial.println(log);
              Serial.println("Failed to ping");
            }
            else
            {
              Serial.println("Succeeded to ping");
              Serial.print("id : ");
              Serial.print(id);
              Serial.print(" model_number : ");
              Serial.println(model_number);
            }

            if (set_model == 1) {
              result = dxl_wb.jointMode(id, 0, 0, &log);
              if (result == false)
              {
                Serial.println(log);
                Serial.println("Failed to change joint mode");
              }
              else
              {
                Serial.println("Succeed to change joint mode");
                Serial.println("Dynamixel is moving...");

                for (int count = 0; count < 3; count++)
                {
                  dxl_wb.goalPosition(id, (int32_t)0);
                  delay(3000);

                  dxl_wb.goalPosition(id, (int32_t)1023);
                  delay(3000);
                }
              }
            } else {
              int32_t goal  = 300;
              result = dxl_wb.wheelMode(id, 0, &log);
              if (result == false)
              {
                Serial.println(log);
                return;
              }
              else
              {
                Serial.println(log);
              }

              result = dxl_wb.goalVelocity(id, (int32_t)goal, &log);
              if (result == false)
              {
                Serial.println(log);
                return;
              }
              else
              {
                Serial.println(log);
              }
            }
          }
        }
        else
        {
          Serial.println("Please check ID");
        }
      }
    }

    void split(String data, char separator, String * temp)
    {
      int cnt = 0;
      int get_index = 0;

      String copy = data;

      while (true)
      {
        get_index = copy.indexOf(separator);

        if (-1 != get_index)
        {
          temp[cnt] = copy.substring(0, get_index);

          copy = copy.substring(get_index + 1);
        }
        else
        {
          temp[cnt] = copy.substring(0, copy.length());
          break;
        }
        ++cnt;
      }
    }

    bool isAvailableID(uint8_t id)
    {
      for (int dxl_cnt = 0; dxl_cnt < (scan_cnt + ping_cnt); dxl_cnt++)
      {
        if (get_id[dxl_cnt] == id)
          return true;
      }

      return false;
    }

    void printInst(void)
    {
      Serial.print("-------------------------------------\n");
      Serial.print("Set begin before scan or ping\n");
      Serial.print("-------------------------------------\n");
      Serial.print("help\n");
      Serial.print("scan   (RANGE)\n");
      Serial.print("setid     (ID) (Mode[0=wheelMode,1=JointMode])\n");
      Serial.print("end\n");
      Serial.print("-------------------------------------\n");
      Serial.print("Press Enter Key\n");

    }
