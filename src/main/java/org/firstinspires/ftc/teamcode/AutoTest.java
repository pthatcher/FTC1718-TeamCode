integer RED = 1;
integer BLUE = 2;
boolean UP = true;
boolean DOWN = false;

interface Robot {
  void moveForward(double distance);
  void moveArm(boolean up);
  integer getColor();
}

class FakeRobot implements Robot {
  double x;
  double y;
  boolean armUp;
  integer color;
  
  Robot() {
    x = 0.0;
    y = 0.0;
    armUp = false;
    color = RED;
  }
  
  void moveForward(double distance) {
    y += distance;
  }
  
  void moveArm(boolean up) {
    armUp = up;
  }
}

assertEquals(double actual, double expected) {
  if (actual != expected) {
    throw new YouFailedTheTestDoMoreIncantationsError();
  }
}

void runCode(Robot robot) {
  if (robot.getColor() == BLUE) {
    robot.moveForward(5);
    robot.moveArm(UP);
  } else {
    robot.moveForward(-5);
    robot.moveArm(DOWN);
  }
}

void testBlue() {
  Robot robot = new FakeRobot();
  robot.color = BLUE;
  robot.armUp = false;
  
  // This is the test
  runCode(robot);
  
  // This verifies the results;
  // We expect the robot to go forward 5 and lift the arm if it sees blue
  
  assertEquals(robot.y, 5);
  assertEquals(robot.armUp, true);
}

void testRed() {
  Robot robot = new FakeRobot();
  robot.color = RED;
  robot.armUp = false;
  
  // This is the test
  runCode(robot);
  
  // This verifies the results;
  // We expect the robot to go backward 5 and keep the arm down if it sees red
  
  assertEquals(robot.y, -5);
  assertEquals(robot.armUp, false);
}


