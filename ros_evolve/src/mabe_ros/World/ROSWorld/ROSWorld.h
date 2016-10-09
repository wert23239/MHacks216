//
//  ROSWorld.h
//  BasicMarkovBrainTemplate
//
//  Copyright (c) 2015 Arend Hintze. All rights reserved.
//

#ifndef __BasicMarkovBrainTemplate__ROSWorld__
#define __BasicMarkovBrainTemplate__ROSWorld__

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometryfr4.h"

#include "../../Global.h"
#include "../AbstractWorld.h"

using namespace std;

class ROSWorld: public AbstractWorld {
private:
  int outputNodesCount;
  int inputNodesCount;
  string gridFilePath;
  vector<vector<int>> worldGrid;
  int robotLifespan;
  int runRate;                // Rate of ROS run loop (in HZ)
  float obstacleThreshold;    // How close (in meters) does something need to be to be considered an obstacle?
  float filterThreshold;      // Proportion of scans in a window that need to report as seeing an obstacle before setting a filtered bit.
  sensor_msgs::LaserScan::ConstPtr ;
  nav_msgs::Odometry::ConstPtr currentOdom;
  string environmentMode;        // Grid or simulator?
  string globalMode;
  ros::NodeHandle nodeHandle;
  ros::Publisher cmdVelPub;
  ros::Subscriber odomSub;
  ros::Subscriber laserSub;

  // Some constants
  const float laserFOV = 180.0;
  const int numRangeSlices = 5;
  const int gridStartLocation[2] = {440, 685};
  const int gridTargetLocation[2] = {10, 10};
  const int gridStartDirection = 0;
  const int numberOfGridDirections = 8;
  const int WALL = 1;
  const int EMPTY = 0;
  // Forward movement tables (indexed by direction)
  const int xfm[8] = {-1,  0,  1,  1,  1,  0, -1, -1};  // Starts in top left corner, moves clockwise. (right turn, increase by 1, left turn decrease by 1)
  const int yfm[8] = {-1, -1, -1,  0,  1,  1,  1,  0};
  // Reverse movement tables (indexed by direction)
  const int xrm[8] = { 1,  0, -1, -1, -1,  0,  1, 1};
  const int yrm[8] = { 1,  1,  1,  0, -1, -1, -1, 0};
  const int numGridVisualSensors = 5;
  // Visual sensors: (distance, rotation)
  const int gridVisualSensors[5][2] = {
                                        {1, -2}, {1, -1},
                                                 {1,  0},
                                        {1,  2}, {1,  1}
                                      };

public:
  // P-server parameters
  static shared_ptr<ParameterLink<string>> environmentModePL;
  static shared_ptr<ParameterLink<int>> robotLifespanPL;
  static shared_ptr<ParameterLink<int>> runRatePL;
  static shared_ptr<ParameterLink<double>> obstacleThresholdPL;
  static shared_ptr<ParameterLink<double>> filterThresholdPL;
  static shared_ptr<ParameterLink<string>> gridFilePathPL;

  ROSWorld(shared_ptr<ParametersTable> _PT = nullptr);
  virtual void runWorldSolo(shared_ptr<Organism> org, bool analyze, bool visualize, bool debug) override;
  virtual int requiredInputs() override {
    return inputNodesCount;
  }
  virtual int requiredOutputs() override {
    return outputNodesCount;
  }
  virtual int maxOrgsAllowed() override {
    return -1;
  }
  virtual int minOrgsAllowed() override {
    return 1;
  }
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  vector<float> filterLaser(const sensor_msgs::LaserScan::ConstPtr& laserMsg);
  void loadGrid();
  int rotate(int direction, int angularVelocity);
  int getGridValue(int x, int y);
  int linearVelocityLookup(int action);
  int angularVelocityLookup(int action);
  string showWorld();

  int getGridDiagDist(int x1, int y1, int x2, int y2);

};

#endif /* defined(__BasicMarkovBrainTemplate__ROSWorld__) */
