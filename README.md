# Kidnapped Vehicle
This is a Udacity Self-Driving Car NanoDegree project submission that uses a particle filter to estimate the position and heading of a simulated moving vehicle with noisy sensor measurements of simulated landmarks. 

![](./screenshot.png)

## Installation
* Clone or fork this repository. 
* Install [Udacity's Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases) which allows the user to visualize the moving object, lidar and radar sensor data, and the kalman filter estimated positions.
* Install [uWebSocketIO](https://github.com/uNetworking/uWebSockets) which allows the project to send data to the Term 2 Simulator.
  * Linux: run the script `install-ubuntu.sh`.
  * Mac: run the script `install-mac.sh`.
  * Windows: install Ubuntu Bash enviroment, clone repo in Ubuntu Bash environment, then run the linux install script `install-ubuntu.sh`.
* Troubleshooting tips can be found on this [Udacity knowledge post](https://knowledge.udacity.com/questions/5184).

## Usage
Intended user is the Udacity evaluator for this project. 

1. Run the following from the project directory:
   * `cd build`
   * `./particle_filter`
2. Run the Term 2 Simulator.
   * In the main menu screen select Project 3: Kidnapped Vehicle.
   * Click the START button to observe the vehicle moving.
     * The vehicle symbol indicates the ground truth position and heading of the kidnapped vehicle.
     * The blue circle with arrow indicates the particle filter's estimated position and heading of the kidnapped vehicle.
     * Green lines extending from vehicle are ground truth measurements to landmarks around vehicle.
     * Blue lines extending from vehicle are estimated measurements to landmarks around vehicle.
     * Error values for vehicle's position and heading components are also displayed.

## Main Project Files
The C++ code and headers can be found in the `src` folder.
* `particle_filter.cpp`: manages the particle_filter class for calculating the estimated vehicle position and heading.

### The Particle Filter

The screenshot above indicates the particle filter error at the last time step was 0.107 and 0.098 meters along the x and y axis respectively, and the heading error was 0.004 radians. The particle filter was able to complete the simulation in 49 seconds.

#### Initialization

The particle filter initializes 1000 particles with a Gaussian distribution around first position and all the weights to 1.

```python
void ParticleFilter::init(double x, double y, double theta, double std[]) {

  num_particles = 1000;  // TODO: Set the number of particles
  particles.resize(num_particles);
  weights.resize(particles.size());
  
  normal_distribution<double> dist_x(x, std[X]);
  normal_distribution<double> dist_y(y, std[Y]);
  normal_distribution<double> dist_theta(theta, std[THETA]);
   
  for (int i = 0; i < particles.size(); i++) {
    Particle& p = particles[i];
    p.id       = i;
    p.x        = dist_x(Gen);
    p.y        = dist_y(Gen);
    p.theta    = dist_theta(Gen);
    p.weight   = 1.0;
    weights[i] = p.weight;
  }

  is_initialized = true;
}
```

#### Prediction

The state of each particle is then predicted with the vehicle's (noiseless) velocity and yaw rate using basic geometry.

To predict the particle state with zero yaw, the following was used:

```python
void predictionYawZero(double delta_t, double std_pos[], double velocity, 
    std::vector<Particle>& particles) {
          
  double dist = velocity * delta_t;

  for (int i = 0; i < particles.size(); i ++) {

    Particle& p = particles[i];
    
    double pred_x = p.x + dist * cos(p.theta);
    double pred_y = p.y + dist * sin(p.theta);
    
    setState(pred_x, pred_y, p.theta, std_pos, p);
  }
}
```

To predict the particle state with yaw rate, the following was used:

```python
void predictionYawNonZero(double delta_t, double std_pos[], double velocity, 
      double yaw_rate, std::vector<Particle>& particles)  {
        
  double v_per_yaw_rate = velocity / yaw_rate;
  
  for (int i = 0; i < particles.size(); i ++) {

    Particle& p = particles[i];
    
    double pred_theta = p.theta + yaw_rate*delta_t;
    double pred_x = p.x + v_per_yaw_rate * (sin(pred_theta) - sin(p.theta));
    double pred_y = p.y + v_per_yaw_rate * (cos(p.theta) - cos(pred_theta));
    
    setState(pred_x, pred_y, pred_theta, std_pos, p);
  }
```

Once the predicted position and heading of each particle is calculated, noise is applied and the state is set.

```python
void setState (double pred_x, double pred_y, double pred_theta, 
    double std_pos[], Particle& p) {
  
  normal_distribution<double> dist_x    (pred_x, std_pos[X]);
  normal_distribution<double> dist_y    (pred_y, std_pos[Y]);
  normal_distribution<double> dist_theta(pred_theta, std_pos[THETA]);
  
  p.x     = dist_x(Gen);
  p.y     = dist_y(Gen);
  p.theta = dist_theta(Gen);
}
```


To compile the source code, run the following from the main project directory:
* `cd build`
* `cmake ..`
* `make`


---------------------------------
# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

#### Submission
All you will need to submit is your `src` directory. You should probably do a `git pull` before submitting to verify that your project passes the most up-to-date version of the grading code (there are some parameters in `src/main.cpp` which govern the requirements on accuracy and run time).

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Success Criteria
If your particle filter passes the current grading code in the simulator (you can make sure you have the current version at any time by doing a `git pull`), then you should pass!

The things the grading code is looking for are:


1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
