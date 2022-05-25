#include "rpLidar.h"
#include "rpLidarTypes.h"
#include <esp_task_wdt.h>
#include <Servo.h>
#include <vector>

static int drivepin = 32;
static int steerpin = 27;
static int straight = 100;
static int lidarpin = 19;
static int minSpeed = 1570;

Servo drive;
Servo steer;

rpLidar lidar(&Serial2,115200,13,12);

static void readPoints(void * parameter){
  while(true){
    int result = lidar.cacheUltraCapsuledScanData();
    Serial.println(result,HEX);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(19, OUTPUT);
  drive.attach(drivepin);
  steer.attach(steerpin);
  esp_task_wdt_init(36000, false); //turn off watchdog so core 0 task doesn't cause reset
  lidar.stopDevice(); //reset the device to be sure that the status is good
  delay(1);
  if(!lidar.start(express)){
    Serial.println("failed to start");
//    return;
  } //start the express scan of the lidar\  esp_task_wdt_init(36000, false); //turn off watchdog so core 0 task doesn't cause reset

  xTaskCreatePinnedToCore(readPoints, "LidarPolling", 65536, NULL, 2, NULL, 0);
  digitalWrite(19, HIGH);  
  steer.write(straight);
  drive.writeMicroseconds(minSpeed+1);
}

float average(float input[]) {
  float sum = 0;
  for (int i = 0; i < sizeof(input); i++) {
    sum += input[i];
  }
  return sum / sizeof(input);
}

float headings[] = {0, 0, 0, 0, 0};

struct range {
  float lower;
  float upper;
};
struct measurement {
  float angle;
  float dist;
};
void loop()
{
//// number of data points in cache: lidar._cached_scan_node_hq_count
// float angle = (((float)_cached_scan_node_hq_buf[index].angle_z_q14) * 90.0 / 16384.0);
// float distance = _cached_scan_node_hq_buf[index].dist_mm_q2 /4.0f;
//// each cache load contains a full 360 scan. If you slow down the rotations too much it will not fit and data will be lost (too many points per 360 deg for cache size allowable on ESP32)
  float minDist = 1000.0;
  double angle_change = 0;
  int measurements = 0;
  float maxDist = 0;
  float maxDistAngle = 0;
  int range = 100;
  std::vector<measurement> angles;
  for (int pos = 0; pos < lidar._cached_scan_node_hq_count; ++pos) {
      scanDot dot;
      if (!lidar._cached_scan_node_hq_buf[pos].dist_mm_q2) continue;
      //dot.quality = _cached_scan_node_hq_buf[pos].quality; //quality is broken for some reason
      dot.angle = (((float)lidar._cached_scan_node_hq_buf[pos].angle_z_q14) * 90.0 / 16384.0);
      dot.dist = lidar._cached_scan_node_hq_buf[pos].dist_mm_q2 /4.0f;
      if (dot.dist > 100000) {
        continue;
      }
      dot.dist = min(dot.dist, 10000.0f);
      if (dot.angle > 360 - range || dot.angle < range) {
        angles.push_back({dot.angle, dot.dist});
        if (dot.dist < 130) {
          drive.writeMicroseconds(minSpeed-20);
          steer.write(straight);
          Serial.print("STOP");
          delay(1000*30);
        }
      }
  }
  int coneSize = 9;
  int externalCheckAngle = 30;
  for (int i = coneSize; i < angles.size(); i++){
    int center = i - coneSize / 2;
    bool safe = true;
    for (int j = i - coneSize; j <= i; j++) {
      if (angles[j].dist < minDist) {
        safe = false;
      }
    }
//    if (angles[center - externalCheckAngle + 1].dist < minDist ||
//        angles[center + externalCheckAngle - 1].dist < minDist) {
//      safe = false;
//    }
    if (safe && angles[center].dist > maxDist) {
      maxDist = angles[center].dist;
      maxDistAngle = angles[center].angle;
    }
  }
  if (angles.size() > 50) {
    for (int i = 0; i < sizeof(headings); i++) {
      headings[i] = headings[i + 1];
    }
    if (maxDistAngle >= 180) {
      maxDistAngle -= 360;
    }
    headings[sizeof(headings) - 1] = maxDistAngle;
//    float heading = average(headings);
    float heading = headings[sizeof(headings) - 3];
    if (heading < straight) {
      heading *= 1.1;
    }
    if (heading > straight) {
      heading *= 0.9;
    }
    Serial.println(heading);
    steer.write(straight + heading);
  }
}
