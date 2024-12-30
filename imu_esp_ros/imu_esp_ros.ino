#include <WiFi.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <MPU6050_tockn.h>
#include <std_msgs/Header.h>
MPU6050 mpu6050(Wire);

//////////////////////
// WiFi Definitions //
//////////////////////
const char* ssid = "Trai Đep Phong 11";
const char* password = "09032003";

IPAddress server(192, 168, 100, 66); // IP of your ROS server
IPAddress ip_address;
int status = WL_IDLE_STATUS;
long timer = 0;

WiFiClient client;

class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    // Initialize the TCP connection to the ROS master
    client.connect(server, 11411);
  }

  // Read a byte from the TCP connection
  int read() {
    return client.read(); // Will return -1 if no data is available
  }

  // Write data to the TCP connection to ROS
  void write(uint8_t* data, int length) {
    for(int i = 0; i < length; i++) {
      client.write(data[i]);
    }
  }

  // Return the time in milliseconds since the start of the program
  unsigned long time() {
    return millis();
  }
};

// Servo variable (for potential future use)
// Servo s;
int i;

void chatterCallback(const std_msgs::String& msg) {
  i = atoi(msg.data);
//  s.write(i);
}

// Change the IMU message type to sensor_msgs::Imu
sensor_msgs::Imu imu_msg;
ros::Subscriber<std_msgs::String> sub("message", &chatterCallback);
ros::Publisher pub("imu_data", &imu_msg);
ros::NodeHandle_<WiFiHardware> nh;

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to "); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
  Wire.begin();
  mpu6050.begin();
}

void setup() {
  Serial.begin(57600);
  setupWiFi();
  delay(2000);
  nh.initNode();
  nh.advertise(pub);
}

void loop() {
  mpu6050.update();
  if (millis() - timer > 20) {  
    timer = millis();
    
    // Get accelerometer and gyroscope data
    float ax = mpu6050.getAccX();
    float ay = mpu6050.getAccY();
    float az = mpu6050.getAccZ();
    float gx = mpu6050.getGyroX();
    float gy = mpu6050.getGyroY();
    float gz = mpu6050.getGyroZ();

    // Populate sensor_msgs::Imu message
    imu_msg.header.stamp.sec = millis() / 1000;
    imu_msg.header.stamp.nsec = (millis() % 1000) * 1000000;
    imu_msg.header.frame_id = "imu_link";

    // Accelerometer data (m/s^2)
    imu_msg.linear_acceleration.x = ax * 9.81; // Convert to m/s^2 if needed
    imu_msg.linear_acceleration.y = ay * 9.81;
    imu_msg.linear_acceleration.z = az * 9.81;

    // Gyroscope data (rad/s)
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;

    Serial.println("Publishing IMU data");
    pub.publish(&imu_msg);
  }
  nh.spinOnce();
}
