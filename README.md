# ESP32-localization:
### This projects aims to achieve the localization and mapping of an autonomous Rover or ROV.

## Localization options: 
Both options should output a set of data representing the _position_ and _orientation_ 
in 3d space, _angular_ and _linear_ velocities.
    
- **Odometery:** 
     primary uses wheel encoders combined with IMU data. Used on ground vehicles as dead reckoning is prone to errors from acceleration sensors.
linear errors occur as a result of uneven or slippery surfaces. Filters and sensor fusion can be applied. 
  - [Basics](https://www.youtube.com/watch?v=LrsTBWf6Wsc&t=498s)
  - [More accurate derivation and equations](https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299)
  
![img.png](img.png)




- **Dead reckoning:** relies on sensors that output acceleration, velocity, and direction. Integration drift error accumulates over time. Used in situation where encoders 
are impractical as water and air vehicles.


### TO DO: (Prioritized)
- __Finish odom__
- __IMU FILTERING (1D kalman)__
- __Research Sensor Fusion__
- __Mapping techniques__

