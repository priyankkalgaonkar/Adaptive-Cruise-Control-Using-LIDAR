# Adaptive Cruise Control Using LIDAR
Code was developed in NXP S32 Design Studio for Arm IDE and Matlab Simulink 2019b.

The design of Adaptive Cruise Control system uses two microcontroller units, a Garmin LIDAR module which is compact and high-performance optical distance measurement sensor, and Brushless DC (BLDC) electric motor. The main objective of Adaptive Cruise Control Using LIDAR project is to enable the cruise control system used in road vehicles to automatically slow down or speed up the vehicle up to a speed limit set by the driver. The result is improved driver's comfort, and safety of the vehicle and other traffic on the road by minimizing rear-end collisions.

According to a report from the National Highway Transportation Safety Administration, rear-end crashes are the most frequently occurring type of collision. About 29 percent of all car crashes are rear-end collisions. These crashes result in a substantial number of injuries and fatalities each year. In fact, roughly 1.7 million rear-end collisions take place in the United States each year. Of these nearly 2 million accidents, about 1,700 people die and another 500,000 are injured in these types of crashes [1]. These numbers constitute a significant portion of highway accidents, injuries, and fatalities.

Our proposed Automatic Cruise Control (ACC) system utilizes a LIDAR sensor that communicates with the on-board ECU (MCU for our project) using I2C protocol to enable this feature. The ACC system will control the speed of the vehicle the same way a driver does – by adjusting the throttle (accelerator) position. I2C master (MCU #1) will receive data from I2C Slave (LIDAR sensor) which will then be transferred to MCU #2 via UART. MCU #2 will then utilize this information to control the BLDC motor which will simulate the ECU controlling the speed of the road vehicle.

Video of our working prototype: https://youtu.be/jsxPeRWvAOA 
