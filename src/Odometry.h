#pragma once
#include <Arduino.h>

/** Odometries are represented similar to ROS */

struct Pose2D
{
    float x;
    float y;
    float yaw;
};

struct Point
{
    float x;
    float y;
    float z;
};

struct Quaternion
{
    float w;
    float x;
    float y;
    float z;
};

struct Twist
{
    float x;
    float y;
    float z;
};

struct Odometry
{
    Pose2D pose;
    Twist linear;
    Twist angular;
};

struct Odometry2D
{
    Pose2D pose;
};