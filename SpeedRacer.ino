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
      if (dot.angle > 180) dot.angle -= 360;
      angles.push_back(dot.angle);
      distances.push_back(dot.dist);
      if (dot.dist < 130) {
//        drive.writeMicroseconds(minSpeed-20);
//        steer.write(straight);
//        driving = false;
//        Serial.println("STOP");
//        delay(1000*30);
        steer.write(straight);
        drive.writeMicroseconds(1000);
        delay(1000*0.3);
        drive.writeMicroseconds(minSpeed+1);
      }
    }
  }
  if (angles.size() > 50) {
    if (!driving) {
        drive.writeMicroseconds(minSpeed+1);
        driving = true;
    }
    int bestI = safestAngle(angles, distances);
    float heading = angles[bestI];
    float dist = distances[bestI];
    Serial.print("Heading: ");
    Serial.println(heading);
    steer.write(straight + heading);
    int spd = minSpeed + 10 + min(1.0f, (dist / 4000))* (1600-minSpeed) * abs(heading/90.0f);
    Serial.print("Speed: ");
    Serial.println(spd);
    drive.writeMicroseconds(spd);
  }
}


float average(vector<float> vec) {
  float sum = 0;
  for (float f: vec) sum += f;
  return sum / vec.size();
}

float safestAngle(vector<float> angles, vector<float> distances) {
  float safeRadius = 10000;
  for (float d: distances) {
    safeRadius = min(d, safeRadius);
  }
  safeRadius += 100;
  vector<vector<int>> sequences;
  vector<int> sequence;
  for (int i = 0; i < angles.size(); i++) {
    if (distances[i] > safeRadius) {
      sequence.push_back(i);
    } else {
      if (sequence.size() > 0) {
        vector<int> s = sequence;
        sequences.push_back(s);
//        Serial.println(s[0]);
        sequence.clear();
      }
    }
  }
  sequences.push_back(sequence);
//  Serial.println(sequences.size());
  vector<int> longestSequence = sequences[0];
  for (vector<int> s: sequences) {
    if (s.size() > longestSequence.size()) {
      longestSequence = s;
    }
  }
  return chooseBestPoint(longestSequence, distances);
}

int edgeBlock = 15;
int chooseBestPoint(vector<int> indices, vector<float> distances) {
    int len = indices.size();
    int mid = len / 2;
    int maxI = indices[mid];
    float maxDist = distances[maxI];
    int left_i = mid - 1;
    int right_i = mid + 1;
    int leftI;
    int rightI;
    float dist;
    while (left_i > edgeBlock) {
        leftI = indices[left_i];
        dist = distances[leftI];
        if (dist > maxDist) {
            maxI = leftI;
            maxDist = dist;
        }
        rightI = indices[right_i];
        dist = distances[rightI];
        if (dist > maxDist) {
            maxI = rightI;
            maxDist = dist;
        }
        left_i--;
        right_i++;
    }
    return maxI;
}
