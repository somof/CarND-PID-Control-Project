
# Project

In this project you'll revisit the lake race track from the Behavioral Cloning Project.
This time, however, you'll implement a PID controller in C++ to maneuver the vehicle around the track!

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

One more thing. 
The speed limit has been increased from 30 mph to 100 mph. 
Get ready to channel your inner Vin Diesel and try to drive SAFELY as fast as possible! NOTE: you don't have to meet a minimum speed to pass.

# Rubrics

- The PID procedure follows what was taught in the lessons.

  - It's encouraged to be creative, particularly around hyperparameter
    tuning/optimization. However, the base algorithm should follow
    what's presented in the lessons.

- Describe the effect each of the P, I, D components had in your implementation.

  - Student describes the effect of the P, I, D component of the PID
    algorithm in their implementation. Is it what you expected?

  - Visual aids are encouraged, i.e. record of a small video of the
    car in the simulator and describe what each component is set to.

- Describe how the final hyperparameters were chosen.

  - Student discusses how they chose the final hyperparameters (P, I,
    D coefficients). This could be have been done through manual
    tuning, twiddle, SGD, or something else, or a combination!

- The vehicle must successfully drive a lap around the track.

  - No tire may leave the drivable portion of the track surface. The
    car may not pop up onto ledges or roll over any surfaces that
    would otherwise be considered unsafe (if humans were in the
    vehicle).

