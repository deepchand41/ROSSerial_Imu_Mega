/*
   rosserial Publisher Example
   Prints "hello world!"
*/

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/transform_datatypes.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>


Adafruit_MPU6050 mpu;

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);

sensor_msgs::Imu imu_raw;
ros::Publisher imu("imu", &imu_raw);


//char hello[13] = "hello world!";

char base_link[] = "/base_link";
char odom[] = "/odom";
char frameid[] = "/imu";
uint8_t seq = 0;

float  roll, pitch, yaw = 0.0;
float z_accel;
geometry_msgs::Quaternion quat;

//imu_raw.header.frame_id = frameid

void setup()
{
  nh.initNode();
  nh.advertise(imu);
//  nh.advertise(chatter);
  broadcaster.init(nh);

  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }

   Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
 }
}

void loop()
{

  t.header.frame_id = base_link;
  t.child_frame_id = odom;
  

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  roll = 57.295*atan2((float)a.acceleration.y, (float)a.acceleration.z-9.8);  //180/PI = 57.295
  pitch = 57.295*atan2((float)-a.acceleration.x, (float)a.acceleration.z - 9.8);
  yaw = 57.295*atan2((float)a.acceleration.y, (float)a.acceleration.x);

  imu_raw.header.frame_id = frameid;
  imu_raw.header.seq = seq;

  quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);


  imu_raw.orientation_covariance[0] = -1;
  imu_raw.orientation.x = quat.x;
  imu_raw.orientation.y = quat.y;
  imu_raw.orientation.z = quat.z;
  imu_raw.orientation.w = quat.w;

  imu_raw.linear_acceleration_covariance[0] = -1;

  z_accel = a.acceleration.z - 9.8;

  imu_raw.linear_acceleration.x = a.acceleration.x;
  imu_raw.linear_acceleration.y = a.acceleration.y;
  imu_raw.linear_acceleration.z = z_accel;

  imu_raw.angular_velocity_covariance[0] = -1;
  imu_raw.angular_velocity.x = g.gyro.x;
  imu_raw.angular_velocity.y = g.gyro.y;
  imu_raw.angular_velocity.z = g.gyro.z;

 // imu_broadcaster.sendTransform(t);
  imu_raw.header.stamp = nh.now();
  imu.publish(&imu_raw);

//  str_msg.data = hello;
//  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(100);
}
