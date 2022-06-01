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

using namespace std;

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
}

float average(float input[]) {
  float sum = 0;
  for (int i = 0; i < sizeof(input); i++) {
    sum += input[i];
  }
  return sum / sizeof(input);
}

float headings[] = {0, 0, 0, 0, 0};

struct measurement {
  float angle;
  float dist;
};
float last_heading = 0;
bool driving = false;
void loop()
{
  int range = 100;
  vector<float> angles;
  vector<float> distances;
  for (int pos = 0; pos < lidar._cached_scan_node_hq_count; ++pos) {
    scanDot dot;
    if (!lidar._cached_scan_node_hq_buf[pos].dist_mm_q2) continue;
    dot.angle = (((float)lidar._cached_scan_node_hq_buf[pos].angle_z_q14) * 90.0 / 16384.0);
    dot.dist = lidar._cached_scan_node_hq_buf[pos].dist_mm_q2 /4.0f;
    if (dot.dist > 100000) {
      continue;
    }
    if (dot.angle > 360 - range || dot.angle < range) {
      angles.push_back(dot.angle);
      distances.push_back(dot.dist);
      if (dot.dist < 130) {
        drive.writeMicroseconds(minSpeed-20);
        steer.write(straight);
        driving = false;
        Serial.println("STOP");
        delay(1000*30);
      }
    }
  }
  if (angles.size() > 50) {
    if (!driving) {
        drive.writeMicroseconds(minSpeed+1);
        driving = true;
    }
    float heading = safestAngle(angles, distances);
    if (heading > 180) {
      heading -= 360;
    }
    Serial.println(heading);
    steer.write(straight + heading);
  }
}


float safeRadius = 200;
float safestAngle(vector<float> angles, vector<float> distances) {
  vector<vector<float>> sequences;
  vector<float> sequence;
  bool safe;
  for (int i = 0; i < angles.size(); i++) {
    safe = distances[i] > safeRadius;
    if (safe) {
      sequence.push_back(angles[i]);
    } else {
      if (sequence.size() > 0) {
        sequences.push_back(sequence);
        sequence.clear();
      }
    }
  }
  vector<float> longestSequence = sequences[0];
  for (vector<float> s: sequences) {
    if (s.size() > longestSequence.size()) {
      longestSequence = s;
    }
  }
  return longestSequence[(int)longestSequence.size()/2];
}
