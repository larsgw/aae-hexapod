#include <Servo.h>
#include <stdlib.h>
#include <math.h>


/*
 * Units: 
 *    Position vectors: mm
 *    Angles: degrees
 *    
 * The front is where the arduino is located
 * x-axis: back to front (x = 0: centre)
 * y-axis: bottom to top (y = 0: bottom plate)
 * z-axis: left to right (z = 0: centre)
 */

class Vector3 {
  public:
    Vector3 () {}
    Vector3 (float x_, float y_, float z_) {
      
      x = x_;
      y = y_;
      z = z_;
    }

    float get_x() { return x; }
    float get_y() { return y; }
    float get_z() { return z; }

    std::string to_string() {

      return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
    }

    Vector3 scale(float scalar) {

      return {x * scalar, y * scalar, z * scalar};
    }

    Vector3 add(Vector3 vector) {

      return {x + vector.get_x(), y + vector.get_y(), z + vector.get_z()};
    }

    Vector3 sub(Vector3 vector) {

      return {x - vector.get_x(), y - vector.get_y(), z - vector.get_z()};
    }

    Vector3 cross(Vector3 vector) {

      return {y*vector.get_z() - z*vector.get_y(), 
              z*vector.get_x() - x*vector.get_z(), 
              x*vector.get_y() - y*vector.get_x()};
    }

    float magnitude() {

      return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    }

    float dot(Vector3 vector) {

      return x * vector.get_x() + y * vector.get_y() + z * vector.get_z();
    }

  private:
  
    float x;
    float y;
    float z;  
};



float getAngle(Vector3 vec1, Vector3 vec2, Vector3 axis) {

  Vector3 vecPar1 = axis.scale(vec1.dot(axis));
  Vector3 vecPerp1 = vec1.sub(vecPar1);
  Vector3 vecPar2 = axis.scale(vec2.dot(axis));
  Vector3 vecPerp2 = vec2.sub(vecPar2);

  float x = vecPerp1.dot(vecPerp2);
  float y = vecPerp1.cross(axis).dot(vecPerp2);

  return degrees(atan2(y, x));
}

Vector3 rotate(Vector3 vec1, Vector3 axis, float angle) {

  float angleR = radians(angle);

  Vector3 vecPar = axis.scale(vec1.dot(axis));
  Vector3 vecPerp1 = vec1.sub(vecPar);

  Vector3 vecPerp2 = vecPerp1.scale(cos(angleR)).add(vecPerp1.cross(axis).scale(sin(angleR)));
  Vector3 vec2 = vecPar.add(vecPerp2);

  return vec2;
}

float invCosRule(float a, float b, float c) {

  return degrees(acos((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * b * c)));
}


class Joint {
  public:
  
    Joint(Vector3 restPos_, Vector3 axis_, float angleScale_, float angleOffset_, float resetAngle_) {
      
      angleScale = angleScale_;
      angleOffset = angleOffset_;
      resetAngle = resetAngle_;

      restPos = restPos_;
      axis = axis_;
    }

    void attach(int pin) {
  
      servo.attach(pin);
    }

    float read() {

      return servo2joint(servo.read());
    }

    void write(float targetP, float maxSpeedP) {

      target = targetP;
      maxSpeed = maxSpeedP;
    }

    void reset() {

      write(resetAngle, 0.1);
    }

    void update() {

      float g = 1.0;
      if (target < read()) {
        g = -1.0;
      }
      servo.write(joint2servo(read()+g*min(maxSpeed, abs(target-read()))));
      //servo.write(joint2servo(target));
      
      //Serial.println(joint2servo(read()+g*min(maxSpeed, abs(target-read()))));
      //if (Mathf.Abs(joint.target-joint.read()) < 0.1) joint.lockServo = false;
    }

    Servo get_servo() { return servo; }
    
    Vector3 get_restPos() { return restPos; }
    Vector3 get_axis() { return axis; }
  
  private:

    Servo servo;

    Vector3 restPos;
    Vector3 axis;
    
    float angleScale;
    float angleOffset;
    float resetAngle;
    float target = 0;
    float maxSpeed = 0.01;

    float joint2servo(float angleP) {
      return angleP * angleScale + angleOffset;
    }
    float servo2joint(float angleP) {
      return (angleP - angleOffset) / angleScale;
    }
};

class Leg {

  public:

    Leg() {}
    
    Leg (Vector3 restPosP, Joint *joint1, Joint *joint2, Joint *joint3, float fps) {

      joints[0] = joint1;
      joints[1] = joint2;
      joints[2] = joint3;

      restPos = restPosP;

      FPS = fps;
    }

    void update() {

      //walk();
      
      joints[0]->update();
      joints[1]->update();
      joints[2]->update();
    }

    Vector3 get_pos() {

      Vector3 pos2 = rotate(joints[1]->get_restPos().sub(joints[0]->get_restPos()), joints[0]->get_axis(), joints[0]->read()).add(joints[0]->get_restPos());
      Vector3 axis2 = rotate(joints[1]->get_axis(), joints[0]->get_axis(), joints[0]->read());
      
      Vector3 pos3_1 = rotate(joints[2]->get_restPos().sub(joints[0]->get_restPos()), joints[0]->get_axis(), joints[0]->read()).add(joints[0]->get_restPos());   
      Vector3 pos3 = rotate(pos3_1.sub(pos2), axis2, joints[1]->read()).add(pos2);
      Vector3 axis3 = rotate(joints[2]->get_axis(), joints[0]->get_axis(), joints[0]->read());

      Vector3 pos4_1 = rotate(restPos.sub(joints[0]->get_restPos()), joints[0]->get_axis(), joints[0]->read()).add(joints[0]->get_restPos());
      Vector3 pos4_2 = rotate(pos4_1.sub(pos2), axis2, joints[1]->read()).add(pos2);
      Vector3 pos4 = rotate(pos4_2.sub(pos3), axis3, joints[2]->read()).add(pos3);
      
      return pos4;
    }

    Vector3 get_pos(int joint) {

      Vector3 pos2 = rotate(joints[1]->get_restPos().sub(joints[0]->get_restPos()), joints[0]->get_axis(), joints[0]->read()).add(joints[0]->get_restPos());
      Vector3 axis2 = rotate(joints[1]->get_axis(), joints[0]->get_axis(), joints[0]->read());
      
      Vector3 pos3_1 = rotate(joints[2]->get_restPos().sub(joints[0]->get_restPos()), joints[0]->get_axis(), joints[0]->read()).add(joints[0]->get_restPos());   
      Vector3 pos3 = rotate(pos3_1.sub(pos2), axis2, joints[1]->read()).add(pos2);
      Vector3 axis3 = rotate(joints[2]->get_axis(), joints[0]->get_axis(), joints[0]->read());

      if (joint == 0) return joints[0]->get_restPos();
      else if (joint == 1) return pos2;
      else if (joint == 2) return pos3;
    }
    

    void writePos(Vector3 target, float maxSpeed) {

      Vector3 joint1ToTarget = target.sub(joints[0]->get_restPos());

      float angle1 = getAngle(restPos.sub(joints[0]->get_restPos()), joint1ToTarget, joints[0]->get_axis());

      Vector3 target2 = rotate(target.sub(joints[0]->get_restPos()), joints[0]->get_axis(), -angle1).add(joints[0]->get_restPos());

      Vector3 joint2ToTarget = target2.sub(joints[1]->get_restPos());

      float offsetAngle = invCosRule(restPos.sub(joints[2]->get_restPos()).magnitude(), joint2ToTarget.magnitude(), joints[1]->get_restPos().sub(joints[2]->get_restPos()).magnitude());
      float angle2 = getAngle(joints[2]->get_restPos().sub(joints[1]->get_restPos()), joint2ToTarget, joints[1]->get_axis()) - offsetAngle;

      float angle3 = invCosRule(joint2ToTarget.magnitude(), joints[1]->get_restPos().sub(joints[2]->get_restPos()).magnitude(), restPos.sub(joints[2]->get_restPos()).magnitude()) - invCosRule(restPos.sub(joints[1]->get_restPos()).magnitude(), joints[1]->get_restPos().sub(joints[2]->get_restPos()).magnitude(), restPos.sub(joints[2]->get_restPos()).magnitude());

      joints[0]->write(angle1, maxSpeed);
      joints[1]->write(angle2, maxSpeed);
      joints[2]->write(angle3, maxSpeed);
    }

    void moveTo(Vector3 start, Vector3 end, float height, float time) {

      writePos(start.add(end.sub(start).scale(status/time)).add({0, height * (1 - pow(1 - 2*status/time, 2)), 0}), 10);
    }

    void setRefPos(Vector3 refPosP) { refPos = refPosP; }
    void set_status(float stat) { status = stat; }

    Vector3 get_refPos() { return refPos; }

  private:

    Vector3 restPos;
    Vector3 refPos;

    Joint* joints[3];

    float FPS;
    float status = 0;
};

class LegGroup {

  public:

    LegGroup() {};
    LegGroup(Leg* leg1, Leg* leg2, Leg* leg3, float fps) {

      legs[0] = leg1;
      legs[1] = leg2;
      legs[2] = leg3;

      FPS = fps;
    }

    void update() {

      status = max(0, status - 1000/FPS); 

      for (int i = 0; i < 3; i++) {

        legs[i]->update();
      }

     // Serial.println(status);
    }

    void walk(Vector3 translation, float rotation, float height, float time) {

      for (int i = 0; i < 3; i++) {

        if (lifted) {

          Vector3 rotatedPos1 = rotate(legs[i]->get_refPos(), {0, 1, 0}, rotation/2);
          Vector3 rotatedPos2 = rotate(legs[i]->get_refPos(), {0, 1, 0}, -rotation/2);
          
          legs[i]->set_status(status);
          legs[i]->moveTo(rotatedPos1.add(translation.scale(0.5)), rotatedPos2.sub(translation.scale(0.5)), height, time);
          
        } else {

          Vector3 rotatedPos = rotate(legs[i]->get_refPos(), {0, 1, 0}, -rotation/2 + status/time * rotation);
          
          legs[i]->writePos(rotatedPos.sub(translation.scale(0.5).sub(translation.scale(status/time))), 20);
        }
      }
    }

    void set_status(float statusP) { status = statusP; }
    void set_lifted(bool liftedP) { lifted = liftedP; }

    float get_status() { return status; }
    bool get_lifted() { return lifted; }

  private:

    Leg* legs[3];

    float status = 0;
    float FPS;

    bool lifted = false;
};


class Command {

  public:

    Vector3 translation;

    String type;
    
    float duration;
    float rotation;
    float stepHight;
    float stepDuration;

    Command() {}

    Command(String type_) { type = type_; }

    Command(String type_, float duration_, Vector3 translation_, float rotation_, float stepHight_, float stepDuration_) {

      type = type_; duration = duration_; translation = translation_; rotation = rotation_; stepHight = stepHight_; stepDuration = stepDuration_;
    }
};


class Hexapod {

  public:
    
    Hexapod(LegGroup legGroup1, LegGroup legGroup2, float fps) {

      legGroup[0] = legGroup1;
      legGroup[1] = legGroup2;
      FPS = fps;
    }

    void update() {

      legGroup[0].update();
      legGroup[1].update();
    }

    void sequence() {

      commandStatus = max(0, commandStatus - 1000/FPS);

      if (commands[commandIndex].type == "end") {

        commandIndex = -1;
        
      } else if (commandStatus == 0) {

        commandIndex += 1;
        commandStatus = commands[commandIndex].duration;
      }

      if (commands[commandIndex].type == "walk") {

        walk(commands[commandIndex].translation, commands[commandIndex].rotation, commands[commandIndex].stepHight, commands[commandIndex].stepDuration);
      }
    }

    void walk(Vector3 translation, float rotation, float height, float time) {

      if (legGroup[0].get_status() == 0 && legGroup[1].get_status() == 0 && legGroup[0].get_lifted() == false && legGroup[1].get_lifted() == false) {
        
        legGroup[0].set_status(time);
        legGroup[0].set_lifted(true);
        legGroup[1].set_status(time);
        legGroup[1].set_lifted(false);
        
      } else if (legGroup[0].get_lifted() && legGroup[0].get_status() == 0) {

        legGroup[0].set_status(time);
        legGroup[0].set_lifted(false);
        legGroup[1].set_status(time);
        legGroup[1].set_lifted(true);
        
      } else if (legGroup[1].get_lifted() && legGroup[1].get_status() == 0) {

        legGroup[0].set_status(time);
        legGroup[0].set_lifted(true);
        legGroup[1].set_status(time);
        legGroup[1].set_lifted(false);
      }

      legGroup[0].walk(translation, rotation, height, time);
      legGroup[1].walk(translation, rotation, height, time);
    }

  private:

    LegGroup legGroup[2];
    Command commands[7] = {

      {"walk", 2000, {80, 0, 0}, 0, 50, 300},
      {"walk", 2000, {-80, 0, 0}, 0, 50, 300},
      {"walk", 1800, {80, 0, 0}, 10, 50, 300},
      {"walk", 2000, {0, 0, 0}, -20, 50, 300},
      {"walk", 2000, {-30, 0, -30}, 0, 50, 300},
      {"walk", 6400, {-0, 0, -0}, 20, 50, 300},
      {"end"}
    };

    int commandIndex = -1;
    float commandStatus = 0;
    float FPS;
};

const float FPS = 60;

Vector3 vec = {0, 0, 0};

Joint joints[18] = {
  {{130, 21, 75}, {0, -1, 0}, -1, 90, 0},            
  {{130, 21, 110}, {-1, 0, 0}, 1, 45, 0}, 
  {{130, 101, 110}, {1, 0, 0}, 1, 0, 0},

  {{0, 21, 75}, {0, -1, 0}, -1, 73, 0},
  {{0, 21, 110}, {-1, 0, 0}, 1, 45, 0},
  {{0, 101, 110}, {1, 0, 0}, 1, 10, 0},

  {{-130, 21, 75}, {0, -1, 0}, -1, 75, 0},
  {{-130, 21, 110}, {-1, 0, 0}, 1, 45, 0},
  {{-130, 101, 110}, {1, 0, 0}, 1, 0, 0},

  {{-130, 21, -75}, {0, 1, 0}, 1, 105, 0},
  {{-130, 21, -110}, {1, 0, 0}, 1, 45, 0},
  {{-130, 101, -110}, {-1, 0, 0}, 1, 0, 0},

  {{0, 21, -75}, {0, 1, 0}, 1, 110, 0},
  {{0, 21, -110}, {1, 0, 0}, 1, 30, 0},
  {{0, 101, -110}, {-1, 0, 0}, 1, 10, 0},

  {{130, 21, -75}, {0, 1, 0}, 1, 110, 0},
  {{130, 21, -110}, {1, 0, 0}, 1, 40, 0},
  {{130, 101, -110}, {-1, 0, 0}, 1, 0, 0}
  
};

Leg legs[6] {

  {{130, -33, 168}, &joints[0], &joints[1], &joints[2], FPS},      // front right
  {{0, -33, 168}, &joints[3], &joints[4], &joints[5], FPS},        // mid right
  {{-130, -33, 168}, &joints[6], &joints[7], &joints[8], FPS},     // back right
  {{-130, -33, -168}, &joints[9], &joints[10], &joints[11], FPS},  // back left
  {{0, -33, -168}, &joints[12], &joints[13], &joints[14], FPS},    // mid left
  {{130, -33, -168}, &joints[15], &joints[16], &joints[17], FPS}   // front left
};

Hexapod hexapod = {
  
  {&legs[0], &legs[2], &legs[4], FPS},
  {&legs[1], &legs[3], &legs[5], FPS},
  FPS
};

Vector3 centre = {252.33, -113.00, 197.33};
float radius = 30;
float angle = 0;

void setup() {

  joints[0].attach(13); //First
  joints[1].attach(53); //Second
  joints[2].attach(12); //Third

  joints[3].attach(11);
  joints[4].attach(51);
  joints[5].attach(10);

  joints[6].attach(9);
  joints[7].attach(49);
  joints[8].attach(8);
  
  joints[9].attach(7);
  joints[10].attach(47);
  joints[11].attach(6);

  joints[12].attach(5);
  joints[13].attach(45);
  joints[14].attach(4);

  joints[15].attach(3);
  joints[16].attach(43);
  joints[17].attach(2);

  //Serial.begin(9600);

  //-140
//  legs[0].writePos({130, -140, 168}, 1);
//  legs[1].writePos({0, -140, 168}, 1);
//  legs[2].writePos({-130, -140, 168}, 1);
//  legs[3].writePos({-130, -140, -168}, 1);
//  legs[4].writePos({0, -140, -168}, 1);
//  legs[5].writePos({130, -140, -168}, 1);

  legs[0].setRefPos({170, -100, 165});
  legs[1].setRefPos({0, -100, 165});
  legs[2].setRefPos({-170, -100, 165});
  legs[3].setRefPos({-170, -100, -165});
  legs[4].setRefPos({0, -100, -165});
  legs[5].setRefPos({170, -100, -165});
}

void loop() {

  hexapod.update();
  //hexapod.walk({0, 0, 0}, 20, 50, 300);
  hexapod.sequence();

  angle += 0.02;

  delay(1000/FPS);
}
