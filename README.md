# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Build and Run

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


## Result

My project has two PIDs for throttle and steering, and I got the coefficients for each PID, by manual and automatic algorithm.

### Manual tuning

Parameters after some trial are as follows.

- 65mph:

  - pid_throttle: Kp=0.20, Ki=0.0001, Kd=1.0
  - pid_steering: Kp=0.10, Ki=1.e-05, Kd=1.9

### Automatic tuning

Parameters after auto tuning with Twiddle Algorithm are as follows.

- 35mph: after 12H auto tuning

  - pid_throttle: Kp=3.422630, Ki=0.000366662, Kd=1.96983
  - pid_steering: Kp=0.232654, Ki=7.50166e-06, Kd=1.08217

- 60mph: after 2H auto tuning

  - pid_throttle: Kp=0.3064270, Ki=0.000104040, Kd=1.0200
  - pid_steering: Kp=0.0982223, Ki=1.01644e-05, Kd=1.7773
  - <a href="img/demo-video-2017-11-15_11.21.57.mkv">video</a>

- over 60mph: fail to tune

  - The vehicle goes out of the course, and the auto tuning process can not keep working.


## Description

### Effect of the P.I.D.

PID method controls a vehicle with difference signals from the reference path plans, called as CTE(Cross Track Error).

<img width=600 src="img/PID_CTE.png">

PID has 3 components, P, I and D,
that have own coefficient as Kp, Ki and Kd with their attribute as follows:

- P: Proportion effect (CTE(t))

  - Directly reduce CTE, but also has heavy overshoot.
  - In the project, tight corner requires large Kp, but it cause awful crawling after the corner.

- D: Differential effect (ΔCTE(t))

  - Suppress argent moving and oscillations.
  - In the project, it works to prevent overshoot that would be mainly caused by Kp.

- I: Integration effect (ΣCTE)

  - Reduce piled up error between CTE and PID controls.
  - In the project, it is a really small value and works to make up sum of CTE.


### How the final hyperparameters were chosen.

Tuning method is same as twiddle algorithm at the lesson.

And plus my project keeps up with tuning PIDs coefficients while the simulator is working on-line.

## Implementation

### porting Twiddle Algorithm

For consecutively tuning PIDs,
I had the Twiddle Algorithm converted into state-machine style as bellows:

original Twiddle Algorithm pseudo code from the lesson:

        best_err = measurement()
        for param, dparam in zip(all_params, all_dparam):
            param += dparam
            err = measurement()
            if err < best_err:
                best_err = err
                dparam *= 1.1
            else:
                param -= 2 * dparam
                err = measurement()
                if err < best_err:
                    best_err = err
                    dparam *= 1.1
                else:
                    param += dparam
                    dparam *= 0.9


Statemachine-style algorithm pseudo code:
    
        state0: first measurement
            best_err = measurement()
            param += dparam // for positive trial
            err = measurement()
            goto sate 1
        
        state1: post positive trial
            if err < best_err:
                best_err = err
                dparam *= 1.1
                next_param += next_dparam
                err = measurement()
                goto state 1 w/ next param
            else:
                param -= 2 * dparam // negative trial
                err = measurement()
                goto sate 2
        
        state2: post negative trial
            if err < best_err:
                best_err = err
                dparam *= 1.1
                next_param += next_dparam
                err = measurement()
                goto state 1 w/ next param
            else:
                param += dparam // back to positive gain (state 1)
                dparam *= 0.9 // suppress gain
                next_param += next_dparam
                err = measurement()
                goto state 1 w/ next param


While the original Twiddle Algorithm code assumes off-line use,
my code is suitable on-line use.

This feature is convenient to tune plural PIDs automatically.


## Consideration

As the target speed becomes faster,
large Kp and Kd(depending on Kp) values are required.
And my project works well under 65mph speed if adequate initial values are given.

For more over 65mph, more careful adjustment with the causal relationship was recommended by manual.

In spite of the situation,
Twiddle algorithm starts to search answers from the edges of the search area,
then the vehicle on the simulator is easy to fail the road.

As next step,
the program could re-start automatically the simulator when the vehicle digress the road.


# EOF
