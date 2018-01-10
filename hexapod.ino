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

    String to_string() {
      
      return "(" + String(x) + ", " + String(y) + ", " + String(z) + ")";
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
    
    Vector3 rotate(float angle, Vector3 axis) {

      float angleR = radians(angle);
    
      Vector3 vecPar = axis.scale(dot(axis));
      Vector3 vecPerp1 = sub(vecPar);
    
      Vector3 vecPerp2 = vecPerp1.scale(cos(angleR)).add(vecPerp1.cross(axis).scale(sin(angleR)));
      Vector3 vec2 = vecPar.add(vecPerp2);
    
      return vec2;
    }

    float getAngle(Vector3 vector, Vector3 axis) {

      Vector3 vecPar1 = axis.scale(dot(axis));
      Vector3 vecPerp1 = sub(vecPar1);
      Vector3 vecPar2 = axis.scale(vector.dot(axis));
      Vector3 vecPerp2 = vector.sub(vecPar2);
    
      float x = vecPerp1.dot(vecPerp2);
      float y = vecPerp1.cross(axis).dot(vecPerp2);
    
      return degrees(atan2(y, x));
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

Vector3 operator+ (Vector3 vectorA, Vector3 vectorB) {
  return vectorA.add(vectorB);
}

Vector3 operator- (Vector3 vectorA, Vector3 vectorB) {
  return vectorA.sub(vectorB);
}

Vector3 operator* (Vector3 vector, float scalar) {
  return vector.scale(scalar);
}

Vector3 operator* (float scalar, Vector3 vector) {
  return vector.scale(scalar);
}

float invCosRule(float a, float b, float c) {

  return degrees(acos((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * b * c)));
}

// A wrapper class for servos
class Joint {
  public:
  
    Joint(Vector3 restPos_, Vector3 axis_, float angleScale_, float angleOffset_, float resetAngle_, float fps) {
      
      angleScale = angleScale_;
      angleOffset = angleOffset_;
      resetAngle = resetAngle_;

      restPos = restPos_;
      axis = axis_;
      FPS = fps;
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

      write(resetAngle, 60);
    }

    void update() {

      float g = 1.0;
      if (target < read()) {
        g = -1.0;
      }
      servo.write(joint2servo(read() + g * min(maxSpeed / FPS, abs(target - read()))));
    }

    Servo get_servo() { return servo; }
    
    Vector3 get_restPos() { return restPos; }
    Vector3 get_axis() { return axis; }
  
  private:

    Servo servo;

    Vector3 restPos;        // The absolute position of the servo's axis of rotation when all the angles of previous joints on the same leg are 0
    Vector3 axis;           // A unit vector that defines the axis of rotation itself
    
    float angleScale;       // Angle multiplier for callibration
    float angleOffset;
    float resetAngle;
    float target = 0;  
    float maxSpeed = 0.01;  // the maximum angular velocity in degrees per second

    float FPS;

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
    
    Leg (Vector3 restPos_, Joint *joint1, Joint *joint2, Joint *joint3, float fps) {

      joints[0] = joint1;
      joints[1] = joint2;
      joints[2] = joint3;

      restPos = restPos_;

      FPS = fps;
    }

    void update() {
      
      joints[0]->update();
      joints[1]->update();
      joints[2]->update();
    }

    Vector3 get_pos() {

      return get_pos(3);
    }

    Vector3 get_pos(int jointIndex) {

      if (jointIndex == 0) {
        return get_restPos(0);
      }

      Vector3 pos = get_restPos(jointIndex);

      for (int i = jointIndex; i > 0; i--) {

        Vector3 deltaPos = pos - get_restPos(i - 1);
        pos = get_restPos(i - 1) + deltaPos.rotate(joints[i - 1]->read(), joints[i - 1]->get_axis());
      }

      return pos;
    }
   
    void writePos(Vector3 target, float maxSpeed) {

      Vector3 joint1ToTarget = target - get_restPos(0);
      Vector3 joint1ToLeg = restPos - get_restPos(0);

      Vector3 joint2ToLeg = restPos - get_restPos(1);

      Vector3 joint3ToJoint2 = get_restPos(2) - get_restPos(1);
      Vector3 joint3ToLeg = restPos - get_restPos(2);

      float angle1 = joint1ToLeg.getAngle(joint1ToTarget, joints[0]->get_axis());                                                 // The rotation of the first servo

      Vector3 target2 = joint1ToTarget.rotate(-angle1, joints[0]->get_axis()) + get_restPos(0);                                   // The target position with the rotation of the first joint canceled out
      Vector3 joint2ToTarget2 = target2 - get_restPos(1);

      float offsetAngle1 = invCosRule(joint3ToLeg.magnitude(), joint2ToTarget2.magnitude(), joint3ToJoint2.magnitude());          // The angle: target-joint2-joint3
      float angle2 = joint3ToJoint2.getAngle(joint2ToTarget2, joints[1]->get_axis()) - offsetAngle1;                              // The rotation of the second servo

      float offsetAngle2 = invCosRule(joint2ToLeg.magnitude(), joint3ToJoint2.magnitude(), joint3ToLeg.magnitude());              // The angle: leg-joint3-joint2 at rest
      float angle3 = invCosRule(joint2ToTarget2.magnitude(), joint3ToJoint2.magnitude(), joint3ToLeg.magnitude()) - offsetAngle2; // The rotation of the third joint

      joints[0]->write(angle1, maxSpeed);
      joints[1]->write(angle2, maxSpeed);
      joints[2]->write(angle3, maxSpeed);
    }

    // This function is used to move the leg from A to B using a parabolic trajectory while walking
    void moveTo(Vector3 start, Vector3 end, float height, float time) {

      writePos(start.add(end.sub(start).scale(status / time)).add({0, height * (1 - pow(1 - 2 * status / time, 2)), 0}), 600);
    }

    void setRefPos(Vector3 refPosP) { refPos = refPosP; }
    void set_status(float stat) { status = stat; }

    Vector3 get_refPos() { return refPos; }
    Vector3 get_restPos() { return get_restPos(3); }
    Vector3 get_restPos(int index) {

      if (index > 2) {
        
        return restPos;
        
      } else {

        return joints[index]->get_restPos();
      }
    }

  private:

    Vector3 restPos; // The absolute position of the end of the leg when the angle of every joint on the leg is 0
    Vector3 refPos;  // The center location of the motion of the end of the leg while walking

    Vector3 start;   // A start location defined used in the moveTo() function
    Vector3 end;     // An end location defined used in the moveTo() function

    Joint* joints[3];

    float FPS;
    float status = -1; // A value that defines the linear progress of the motion of the moveTo() function
};

// A legGroup containes the two outer most legs of one side and the middle leg from the other side of the hexapod
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

      status = max(0, status - 1000 / FPS); 

      for (int i = 0; i < 3; i++) {

        legs[i]->update();
      }
    }

    void walk(Vector3 translation, float rotation, float height, float time) {

      for (int i = 0; i < 3; i++) {

        if (lifted) {

          Vector3 rotatedPos1 = legs[i]->get_refPos().rotate(rotation / 2, {0, 1, 0});
          Vector3 rotatedPos2 = legs[i]->get_refPos().rotate(-rotation / 2, {0, 1, 0});
          
          legs[i]->set_status(status); // Sets the leg.status equal to the legGroup.status so it can be used in the moveTo() function 
          legs[i]->moveTo(rotatedPos1 + (0.5 * translation), rotatedPos2 - (0.5 * translation), height, time);
          
        } else {

          Vector3 rotatedPos = legs[i]->get_refPos().rotate(-rotation/2 + status/time * rotation, {0, 1, 0});
          
          legs[i]->writePos(rotatedPos - ((0.5 - status / time) * translation), 600);
        }
      }
    }

    void set_status(float statusP) { status = statusP; }
    void set_lifted(bool liftedP) { lifted = liftedP; }

    float get_status() { return status; }
    bool get_lifted() { return lifted; }

  private:

    Leg* legs[3];

    float status = 0; // A value that defines the progress of a single step in the walkcycle
    float FPS;

    bool lifted = false;
};

// Data structure used to define an task the hexapod could perform for a certain amount of time
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

      type = type_;
      duration = duration_;
      translation = translation_;
      rotation = rotation_;
      stepHight = stepHight_;
      stepDuration = stepDuration_;
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

      commandStatus = max(0, commandStatus - 1000 / FPS);

      if (commands[commandIndex].type == "end") {

        commandIndex = -1;

      } else if (commandStatus == 0) {

        commandIndex += 1;
        commandStatus = commands[commandIndex].duration;
      }

      if (commands[commandIndex].type == "walk") {

        walk(commands[commandIndex]);
      }
    }

    void walk(Command command) {
      walk(command.translation, command.rotation, command.stepHight, command.stepDuration);
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

    // A sequence of commands that will be executed while the sequence function is being called
    Command commands[7] = {

      {"walk", 2000, {80, 0, 0}, 0, 50, 300},
      {"walk", 2000, {-80, 0, 0}, 0, 50, 300},
      {"walk", 1800, {80, 0, 0}, 10, 50, 300},
      {"walk", 2000, {0, 0, 0}, -20, 50, 300},
      {"walk", 2000, {-30, 0, -30}, 0, 50, 300},
      {"walk", 6400, {-0, 0, -0}, 20, 50, 300},
      {"end"}
    };

    int commandIndex = -1;   // The command that is currently being executed
    float commandStatus = 0; // A value that defines the linear progress of the to be executed command
    float FPS;
};

const float FPS = 45; // the framerate of the main loop

Joint joints[18] = {
  {{130.5, 15, 75}, {0, -1, 0}, -1, 90, 0, FPS},            
  {{141, 15, 110.45}, {-1, 0, 0}, 1, 44, 0, FPS}, 
  {{141, 95, 110.45}, {1, 0, 0}, 1, 12, 0, FPS},

  {{0, 15, 75}, {0, -1, 0}, -1, 70, 0, FPS},
  {{-10.5, 15, 110.45}, {-1, 0, 0}, 1, 29, 0, FPS},
  {{-10.5, 95, 110.45}, {1, 0, 0}, 1, 9, 0, FPS},

  {{-130.5, 15, 75}, {0, -1, 0}, -1, 75, 0, FPS},
  {{-141, 15, 110.45}, {-1, 0, 0}, 1, 39, 0, FPS},
  {{-141, 95, 110.45}, {1, 0, 0}, 1, 15, 0, FPS},

  {{-130.5, 15, -75}, {0, 1, 0}, 1, 105, 0, FPS},
  {{-141, 15, -110.45}, {1, 0, 0}, 1, 34, 0, FPS},
  {{-141, 95, -110.45}, {-1, 0, 0}, 1, 4, 0, FPS},

  {{0, 15, -75}, {0, 1, 0}, 1, 108, 0, FPS},
  {{-10.5, 15, -110.45}, {1, 0, 0}, 1.2, 10, 0, FPS},
  {{-10.5, 95, -110.45}, {-1, 0, 0}, 1, 7, 0, FPS},

  {{130.5, 15, -75}, {0, 1, 0}, 1, 85, 0, FPS},
  {{141, 15, -110.45}, {1, 0, 0}, 1, 43, 0, FPS},
  {{141, 95, -110.45}, {-1, 0, 0}, 1, 1, 0, FPS}
  
};

Leg legs[6] {

  {{141, -41.23, 155.88}, &joints[0], &joints[1], &joints[2], FPS},       // front right
  {{-10.5, -41.23, 155.88}, &joints[3], &joints[4], &joints[5], FPS},     // mid right
  {{-141, -41.23, 155.88}, &joints[6], &joints[7], &joints[8], FPS},      // back right
  {{-141, -41.23, -155.88}, &joints[9], &joints[10], &joints[11], FPS},   // back left
  {{-10.5, -41.23, -155.88}, &joints[12], &joints[13], &joints[14], FPS}, // mid left
  {{141, -41.23, -155.88}, &joints[15], &joints[16], &joints[17], FPS}    // front left
};

Hexapod hexapod = {
  
  {&legs[0], &legs[2], &legs[4], FPS},
  {&legs[1], &legs[3], &legs[5], FPS},
  FPS
};

void setup() {

  joints[0].attach(13); // First
  joints[1].attach(53); // Second
  joints[2].attach(12); // Third

  joints[3].attach(11); // First
  joints[4].attach(51); // ...
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

  Serial.begin(9600);

  //-140
//  legs[0].writePos(legs[0].get_restPos().add({0, -50, 0}), 60);
//  legs[1].writePos(legs[1].get_restPos().add({0, -50, 0}), 60);
//  legs[2].writePos(legs[2].get_restPos().add({0, -50, 0}), 60);
//  legs[3].writePos(legs[3].get_restPos().add({0, -50, 0}), 60);
//  legs[4].writePos(legs[4].get_restPos().add({0, -50, 0}), 60);
//  legs[5].writePos(legs[5].get_restPos().add({0, -50, 0}), 60);

  legs[0].setRefPos({180, -100, 165});
  legs[1].setRefPos({0, -100, 165});
  legs[2].setRefPos({-180, -100, 165});
  legs[3].setRefPos({-180, -100, -165});
  legs[4].setRefPos({0, -100, -165});
  legs[5].setRefPos({180, -100, -165});

//  for (int i = 0; i < 18; i++) {
//    joints[i].write(10, 60);
//  }
}

void loop() {

  hexapod.update();
  
  hexapod.sequence();

  delay(1000/FPS);
}
