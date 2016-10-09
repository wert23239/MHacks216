//
//  ROSWorld.cpp
//  BasicMarkovBrainTemplate
//
//  Copyright (c) 2015 Arend Hintze. All rights reserved.
//

#include "ROSWorld.h"

shared_ptr<ParameterLink<string>> ROSWorld::environmentModePL = Parameters::register_parameter("WORLD_ROS-environmentMode", (string) "grid", "[grid, simulation]");
shared_ptr<ParameterLink<int>> ROSWorld::robotLifespanPL = Parameters::register_parameter("WORLD_ROS-robotLifespan", 1000, "Blah");
shared_ptr<ParameterLink<int>> ROSWorld::runRatePL = Parameters::register_parameter("WORLD_ROS-runRate", 5, "Blah");
shared_ptr<ParameterLink<double>> ROSWorld::obstacleThresholdPL = Parameters::register_parameter("WORLD_ROS-obstacleThreshold", 1.0, "Blah");
shared_ptr<ParameterLink<double>> ROSWorld::filterThresholdPL = Parameters::register_parameter("WORLD_ROS-filterThreshold", 0.1, "blah");
shared_ptr<ParameterLink<string>> ROSWorld::gridFilePathPL = Parameters::register_parameter("WORLD_ROS-gridFilePath", (string) "/home/parallels/indigo_ws/src/ros_mabe/scripts/grid.dat", "blah");



ROSWorld::ROSWorld(shared_ptr<ParametersTable> _PT) : AbstractWorld(_PT) {
  environmentMode = (PT == nullptr) ? environmentModePL->lookup() : PT->lookupString("WORLD_ROS-environmentMode");
  globalMode = (PT == nullptr) ? Global::modePL->lookup() : PT->lookupString("GLOBAL-mode");
  robotLifespan = (PT == nullptr) ? robotLifespanPL->lookup() : PT->lookupInt("WORLD_ROS-robotLifespan");
  runRate = (PT == nullptr) ? runRatePL->lookup() : PT->lookupInt("WORLD_ROS-runRate");
  obstacleThreshold = (PT == nullptr) ? obstacleThresholdPL->lookup() : PT->lookupDouble("WORLD_ROS-obstacleThreshold");
  filterThreshold = (PT == nullptr) ? filterThresholdPL->lookup() : PT->lookupDouble("WORLD_ROS-filterThreshold");
  outputNodesCount = 4;
  gridFilePath = (PT == nullptr) ? gridFilePathPL->lookup() : PT->lookupString("WORLD_ROS-gridFilePath");
  inputNodesCount = numRangeSlices;
  aveFileColumns.clear();

  cout << "ROSWORLD -- globalMode: " << globalMode << endl;
  cout << "ROSWORLD -- evaluateMode: " << environmentMode << endl;

  // Setup publishers
  cmdVelPub = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  // Setup subscribers
  laserSub = nodeHandle.subscribe("base_scan", 1000, &ROSWorld::laserCallback, this);
  odomSub = nodeHandle.subscribe("odom", 1000, &ROSWorld::odomCallback, this);

  if (environmentMode == "grid")
    loadGrid();
  // for (int i = 0; i < (int) worldGrid.size(); i++) {
  //   for (int k = 0; k < (int) worldGrid[i].size(); k++) {
  //     cout << worldGrid[i][k];
  //   }
  //   cout << endl;
  // }
  // exit(-1);
}

void ROSWorld::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  /* Callback function for laser range finder data topic.
      Responsibility: update current laser data with new laser data.
  */
  currentLaser = msg;
}

void ROSWorld::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  /* Callback function for odometry data topic.
      Responsibility: update current odometry with new data.
  */
  currentOdom = msg;
}

vector<float> ROSWorld::filterLaser(const sensor_msgs::LaserScan::ConstPtr& laserMsg) {
  /* Laser data filter. Given laser message, return filtered result.
      This function is not general. It is world specific.
      For this world, filterLaser works as follows:
        * EXPECTS: 180 degree FOV
        * Reduces 180 degree arbitrary resolution data to 5 binary values.
        * Each binary value represents a 36 degree range.
        * 1: something is there (10% of lasers are under threshold), 0: nothing is there
      Output: [0:36, 36:72, 72:108, 108:144, 144:180]
  */
  vector<float> filteredRanges;
  // Calculate slice thresholds (for convenience later)
  float angWindowSize = ((laserFOV / numRangeSlices) * M_PI) / 180.0;
  float curAngle = laserMsg->angle_min;
  float angAccum = 0.0;
  int numSightings = 0;
  int totalViews = 0;
  int ri = 0;
  while (curAngle < laserMsg->angle_max) {
    // Are we ready to clip?
    if (!(angAccum < angWindowSize)) {
      // Clip!
      // 1) Did we see anything?
      if (numSightings >= (filterThreshold * totalViews)) {
        filteredRanges.push_back(1);
      } else {
        filteredRanges.push_back(0);
      }
      // 2) Reset everything
      // Reset the accumulator
      angAccum = 0.0;
      // Reset total views
      totalViews = 0;
      // Reset number of sightings
      numSightings = 0;
    }
    float range = laserMsg->ranges[ri];
    // Did we see anything?
    if (range < obstacleThreshold) numSightings += 1;
    totalViews += 1;
    angAccum += laserMsg->angle_increment;
    curAngle += laserMsg->angle_increment;
    ri++;
  }
  return filteredRanges;
}

void ROSWorld::runWorldSolo(shared_ptr<Organism> org, bool analyze, bool visualize, bool debug) {
  ros::Rate rate(runRate);
  if (globalMode == "run" && environmentMode == "simulation") {
    /* RUN MODE DESCRIPTION: This mode just runs the organism passed to runWorldSolo until ROS dies.
      RUN in SIMULATOR
    */
    // Each time step consists of the following steps:
    //  1) SENSE, 2) THINK (brain update), 3) ACTUATE, 4) WORLD UPDATE
    // Wait for a laser scan message
	  //change this part for us
    ros::topic::waitForMessage<sensor_msgs::LaserScan>("base_scan", nodeHandle);
    ros::spinOnce();  // We'll want to spin once when we receive it (calls our callbacks for us)
    while (ros::ok()) {
      /////////////////////////////////////////////
      // 1: SENSE: range data
      /////////////////////////////////////////////

		//Change this
      vector<float> filteredRanges = filterLaser(currentLaser);
      cout << "FILTERED RANGES: ";
      for (int i = 0; i < (int) filteredRanges.size(); i++) {
        cout << filteredRanges[i] << " ";
        org->brain->setInput(i, filteredRanges[i]);
      }
      cout << endl;
      /////////////////////////////////////////////
      // 2: UPDATE
      /////////////////////////////////////////////
      org->brain->update();
      /////////////////////////////////////////////
      // 3: ACTUATE
      /////////////////////////////////////////////
      // Brain:
      //  Linear Velocity: 0, 1
      //  Angular Velocity: 2, 3
      // Action Table:
      //  Linear Velocity:
      //    00: Reverse (-1)
      //    01: None    (0)
      //    10: None    (0)
      //    11: Forward (+1)
      //  Angular Velocity:
      //    00: None  (0)
      //    01: Right (+1)
      //    10: Left  (-1)
      //    11: None  (0)
      int linearAction = (org->brain->readOutput(1)) + (org->brain->readOutput(0) * 2);
      int angularAction = (org->brain->readOutput(2)) + (org->brain->readOutput(3) * 2);
      geometry_msgs::Twist cmdVelMsg;
      cmdVelMsg.linear.x = linearVelocityLookup(linearAction);
      cmdVelMsg.angular.z = angularVelocityLookup(angularAction);
      // Send off the command!
      cmdVelPub.publish(cmdVelMsg);
      cout << "Linear action: " << linearAction << endl;
      cout << "Angular action: " << angularAction << endl;
      /////////////////////////////////////////////
      // 4: WORLD UPDATE
      /////////////////////////////////////////////
      // Step through time!
      ros::spinOnce();
      rate.sleep();
	};
  } else if (globalMode == "evolve" && environmentMode == "simulation") {
    /* TODO (amlalejini): This mode currently does nothing.
          EVOLVE MODE DESCRPTION: This mode will be used to evolve brains in the stage simulation. It will call fitness tracking service for
           fitness information.
    */
    // EVOLUTION MODE
    int robotTime = 0;
    // Wait for a laser scan message
    ros::topic::waitForMessage<sensor_msgs::LaserScan>("base_scan", nodeHandle);
    ros::spinOnce();  // We'll want to spin once when we receive it (calls our callbacks for us)
    while (ros::ok() && (robotTime < robotLifespan)) {
      // What time is it?
      cout << "The time: " + to_string(robotTime) << endl;
      vector<float> filteredRanges = filterLaser(currentLaser);
      cout << "FILTERED RANGES: ";
      for (int i = 0; i < (int) filteredRanges.size(); i++) cout << filteredRanges[i] << " ";
      cout << endl;
      // Step through time!
      ros::spinOnce();
      rate.sleep();
      robotTime += 1;
    }
  } else if (globalMode == "evolve" && environmentMode == "grid") {
    /* EVOLVE-GRID MODE DESCRIPTION: This mode will evolve brains to navigate to a target in a grid world.
        Fitness is the A* distance to the objective.
    */
    int x = gridStartLocation[0];
    int y = gridStartLocation[1];
    int dir = gridStartDirection;
    int maxDist = getGridDiagDist(0, 0, (int) worldGrid[0].size(), (int) worldGrid.size());
    for (int t = 0; t < robotLifespan; t++) {
      // 1) SENSE, 2) THINK (brain update), 3) ACTUATE, 4) WORLD UPDATE
      /////////////////////////////////////////////
      // 1: SENSE
      /////////////////////////////////////////////
      // Visual Sensors:
      //  v0 v1
      //  -> v2
      //  v4 v3
      for (int v = 0; v < numGridVisualSensors; v++) {
        // Rotate sensor as defined by visualSensors
        int vDir = rotate(dir, gridVisualSensors[v][1]);
        // Get location of vision
        int visx = (x + (xfm[vDir] * gridVisualSensors[v][0]));
        int visy = (y + (yfm[vDir] * gridVisualSensors[v][0]));
        // What are we sensing?
        int data = getGridValue(visx, visy);
        org->brain->setInput(v, data);
      }
      /////////////////////////////////////////////
      // 2: UPDATE
      /////////////////////////////////////////////
      org->brain->update();
      /////////////////////////////////////////////
      // 3: ACTUATE
      /////////////////////////////////////////////
      // Brain:
      //  Linear Velocity: 0, 1
      //  Angular Velocity: 2, 3
      // Action Table:
      //  Linear Velocity:
      //    00: Reverse (-1)
      //    01: None    (0)
      //    10: None    (0)
      //    11: Forward (+1)
      //  Angular Velocity:
      //    00: None  (0)
      //    01: Right (+1)
      //    10: Left  (-1)
      //    11: None  (0)
      int linearAction = (org->brain->readOutput(1)) + (org->brain->readOutput(0) * 2);
      int angularAction = (org->brain->readOutput(3)) + (org->brain->readOutput(2) * 2);
      int linearVelocity = linearVelocityLookup(linearAction);
      int angularVelocity = angularVelocityLookup(angularAction);
      // Rotate according to angular velocity
      dir = rotate(dir, angularVelocity);
      // Move according to linear velocity
      if (linearVelocity < 0) {
        // Reverse
        int targx = x + xrm[dir];
        int targy = y + yrm[dir];
        // Make sure target isn't a wall
        // if target not wall: move to target x, y
        if (getGridValue(targx, targy) != WALL) {
          // move
          // worldGrid[y][x] = 0;
          // worldGrid[y][x] = 0;
          x = targx;
          y = targy;
        }
      } else if (linearVelocity > 0) {
        // Forward
        int targx = x + xfm[dir];
        int targy = y + yfm[dir];
        // if target not wall: move to target x, y
        if (getGridValue(targx, targy) != WALL) {
          // move
          // worldGrid[y][x] = 0;
          // worldGrid[y][x] = 0;
          x = targx;
          y = targy;
        }
      } // Otherwise, no movement
      /////////////////////////////////////////////
      // 4: WORLD UPDATE
      /////////////////////////////////////////////
      // The world is static.
      // worldGrid[y][x] = 8;
      // cout << showWorld() << endl;
      // cout << "DIR: " << dir << endl;
    }
    // What's my fitness, again? Simple Diagnal distance to goal (susceptible to deception, but whatever, this is for testing.)
    org->score = abs(maxDist - getGridDiagDist(x, y, gridTargetLocation[0], gridTargetLocation[1]));
  } else if (globalMode == "run" && environmentMode == "grid") {
    cout << "RUN mode in GRID environment is not implemented." << endl;
    exit(-1);
  } else {
    cout << "Unable to resolve ROSWORLD::environmentMode + GLOBAL::mode: \n\tROSWORLD::environmentMode: " << environmentMode << "; GLOBAL::mode: " << globalMode << endl;
    exit(-1);
  }
}

void ROSWorld::loadGrid() {
  // Empty out the world grid
  worldGrid.clear();
  ifstream gridFile(gridFilePath);
  if (gridFile.is_open()) {
    string line;
    while (getline(gridFile, line)) {
      vector<int> row;
      for (int i = 0; i < (int) line.size(); i++) {
        row.push_back(line[i] - '0');
      }
      worldGrid.push_back(row);
    }
    gridFile.close();
  } else {
    cout << "Failed to open grid file." << endl;
    exit(-1);
  }
}

int ROSWorld::rotate(int direction, int angularVelocity) {
  /* Given a current direction and an angular velocity (direction and magnitude of rotation), return new direction.
    Angular velocity explained: (+) Value will rotate clockwise, (-) Value will rotate anti-clockwise.
      Magnitude determines how many 45 degree turns to make.
  */
  if (angularVelocity == 0) {
    return direction;
  } else {
    int newDirection = direction;
    int magnitude = abs(angularVelocity);
    int turnDirection = angularVelocity / abs(angularVelocity);
    for (int t = 0; t < magnitude; t++) {
      newDirection = (newDirection + turnDirection) % numberOfGridDirections;
      if (newDirection < 0) newDirection += numberOfGridDirections;
    }
    return newDirection;
  }
}

int ROSWorld::getGridValue(int x, int y) {
  /* Given an x,y coordinate, this function will return grid value at that location (resource or not resource?).
     Any x,y given that lies beyond the size of the grid will return EMPTY. */
  if (y <= (int) worldGrid.size()) {
    if (x <= (int) worldGrid[y].size()) {
      return worldGrid[y][x];
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}

int ROSWorld::linearVelocityLookup(int action) {
  switch(action) {
    case 0: // 00
      // Reverse
      return -1;
      break;
    case 1: // 01
      // Nothing
      return 0;
      break;
    case 2: // 10
      // Nothing
      return 0;
      break;
    case 3: // 11
      // Forward
      return 1;
      break;
  }
  return 0;
}

int ROSWorld::angularVelocityLookup(int action) {
  switch (action) {
    case 0: // 00
      // Nothing
      return 0;
      break;
    case 1: // 01
      // Right
      return 1;
      break;
    case 2: // 10
      // Left
      return -1;
      break;
    case 3: // 11
      // Nothing
      return 0;
      break;
  }
  return 0;
}

string ROSWorld::showWorld() {
  string gridStr = "";
  for (int y = 0; y < (int) worldGrid.size(); y++) {
    for (int x = 0; x < (int) worldGrid[y].size(); x++) {
      gridStr += to_string(worldGrid[y][x]) + " ";
    }
    gridStr += "\n";
  }
  return gridStr;
}

int ROSWorld::getGridDiagDist(int x1, int y1, int x2, int y2) {
  int moveCost = 1;
  int diagMoveCost = 1;
  int dx = abs(x1 - x2);
  int dy = abs(y1 - y2);
  return moveCost * (dx + dy) + (diagMoveCost - 2 * moveCost) * min(dx, dy);
}
