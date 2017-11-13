# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---


## Build and Run

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


## Result

My project automatically tunes two PIDs for throttle and steering with Twiddle Algorithm.

And following PID parameters are the results after about 30-60 minute auto-tuning for each speed.

- 35mph:

  - pid_throttle: Kp=0.135212, Ki=7.68274e-05, Kd=0.989856
  - pid_steering: Kp=0.060845, Ki=2.35935e-05, Kd=8.190700

optimazed param:(4)  Kp=0.1216910, Ki=8.45101e-05, Kd=0.989856
updated params:(15)  Kp=0.0669295, Ki=2.36171e-05, Kd=8.190700

- 40mph: 

  - pid_throttle: Kp=0.194840, Ki=0.000101412, Kd=0.989856
  - pid_steering: Kp=0.104785, Ki=2.35935e-05, Kd=8.272610

- 45mph: unstable to safty drive

  - pid_throttle: Kp=0.255046, Ki=0.000102426, Kd=1.20861
  - pid_steering: Kp=0.104785, Ki=2.59505e-05, Kd=8.27261

optimazed param:(3)  Kp=0.3086060, Ki=0.000112669, Kd=1.20861
 updated params:(6)  Kp=0.1058330, Ki=2.59505e-05, Kd=9.09987

optimazed param:(3)  Kp=0.373413, Ki=0.000123936, Kd=1.20861


- 50mph: fail to tune


## Implementation

### Effect of the P.I.D.

CTE(Cross Track Error)

- P: Proportion effect

  - 

- I: Integration effect

  - 

- D: Differential effect

  - 



### porting Twiddle Algorithm

To consecutively tune the two PIDs,
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
                param += - 2 * dparam
                err = measurement()
                if err < best_err:
                    best_err = err
                    dparam *= 1.1
                else:
                    param += dparam
                    dparam *= 0.9

state-machine style pseudo code:
    
        state0: first measurement
            best_err = measurement()
            param += dparam // positive trial
            goto sate 1
        
        state1: post positive trial
            if err < best_err:
                best_err = err
                dparam *= 1.1
                next_param += next_dparam
                goto state 1 w/ next param
            else:
                param += - 2 * dparam // negative trial
                goto sate 2
        
        state2: post negative trial
            if err < best_err:
                best_err = err
                dparam *= 1.1
                next_param += next_dparam
                goto state 1 w/ next param
            else:
                param += dparam // back to positive gain (state 1)
                dparam *= 0.9 // suppress gain
                next_param += next_dparam
                goto state 1 w/ next param


While the original Twiddle Algorithm code assumes off-line use,
my code enables on-line tuning.

This feature is very convenient to tune plural PIDs automatically.


### Exclusive Twiddling

Runing two twiddle algorithms concurrently caused conflict and vehicle control failure,
before getting adequate PID parameters.

So I added an exclusive logic to the two Twiddle functions in main() as following code.

     // PID for Steering
     pid_steering.UpdateError(cte);
     steer_value = std::max(-1., std::min(+1., - pid_steering.TotalError())); // limit in [-1, 1].
     double speed_err = target_speed - speed;

     // exclusive control for two twiddles
     if (pid_steering.is_tuned && pid_throttle.is_tuned) {
        std::cout << "start twiddle for throttle" << std::endl;
        pid_throttle.is_tuned = false;
     }

     // PID for Throttle
     pid_throttle.UpdateError(speed_err);
     throttle_value = std::min(1., pid_throttle.TotalError());

     // exclusive control for two twiddles
     if (pid_throttle.is_tuned && pid_steering.is_tuned) {
        std::cout << "start twiddle for steering" << std::endl;
        pid_steering.is_tuned = false;
     }


Therefore,
my project can prevent the conflict and tune the two PIDs alternately.


### Consideration

As the speed became faster,
Twiddle Algorithm seemed to take large Kp(=proportinal weight).

This is reasonable behaviour to tune to high speed drive,
but Kp's overshoot exceed road width around 45mph speed.

For more improvement,
stabilization may be needed for my project.




# EOF
