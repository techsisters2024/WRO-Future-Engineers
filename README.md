Engineering materials
====
We used Arduino, raspberry pi, webcam, and wheels from our school’s Lego set.
### Raspberry Pi 4 
`We used it because it had the storage for all the sensors we choose to use and was better for running the complex algorithm of the colour sensing camera.`
### Raspberry Pico 2
`Added additional capacity and connects the dc motors and batteries. It is responsible for the movement of the car. `
### PiicoDev Colour Sensor VEML6040
`Specifically made for colour sensing. Works as a back up for the main webcam. In some lightings it works better. `
### Webcam 
`Great vision and colour recognition and can be connected by cable, which freed up space for the other sensors.`
### Battery
`We found it at school, so it lowered our costs. It is rechargeable.`
### Ultrasonic Sensor HC-SRO4 
`It’s best advantage is the price. Very accessible.`
### Lego Wheels and parts
`They were the ones available to us at school and the size was perfect for us.`
### Wires


## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains several schematic diagrams of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Introduction

This part provides an overview of the code used to control an autonomous vehicle equipped with ultrasonic distance sensors, motors, servo steering, and a web camera for color detection. The vehicle is designed to avoid obstacles by measuring distances and detecting colors, then adjusting its path based on these inputs.

### Code Components and Modules
The code integrates several modules to manage the electromechanical components of the vehicle and process color information from the web camera:

### 1. Motor Control:

* The motors are controlled through Pulse Width Modulation (PWM) signals. The machine module defines GPIO pins and generates PWM signals to manage motor speeds and directions.
* Functions move_forward(speed), move_backward(speed), and stop_motors() control the vehicle’s movement by adjusting PWM signals.
Steering Control:

### 2. Servo Control:
* The steering is managed by a servo motor using PWM, set to a frequency of 50Hz.
* The function set_servo_angle(angle) adjusts the steering angle by modifying the duty cycle of the PWM signal.

### 3. Distance Measurement:

* The ultrasonic sensor detects obstacles by measuring the time it takes for an ultrasonic pulse to return.
* The function measure_distance() calculates the distance to obstacles and returns the measurement.
  
### 4. Color Detection:
* A web camera is used to capture real-time images for color detection.  
* The code processes the camera feed to identify red and green colors based on RGB, determining the presence and location of colored obstacles.

### Relationship to Electromechanical Components
* Motors: Controlled through PWM signals to dictate the vehicle’s speed and direction.
* Servo: Adjusted via PWM to steer the vehicle left or right based on detected obstacles.
* Ultrasonic Sensor: Measures the distance to obstacles to facilitate navigation and decision-making.
* Web Camera: Captures images to detect and interpret colors, guiding the vehicle’s movement and direction.

### Example Workflow
1. The code initializes the motor, servo, and sensor modules. It also configures the web camera for color detection.
2. The vehicle continuously measures distance and captures camera images. It processes color data to detect obstacles and adjust its path based on both distance and color information.
3. Depending on detected colors (red or green) and proximity to obstacles, the vehicle will turn, move forward, or stop to navigate effectively.

In order to integrate all of the electromechanical components into the vehicle check the schemes and the src folders for additional details about wiring abd coding. 
