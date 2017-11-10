#include <vector>
#include "PID.h"

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
   // Errors
   pre_cte = 0.;
   p_error = 0.;
   i_error = 0.;
   d_error = 0.;

   // Coefficients
   Kp = 0.;
   Ki = 0.;
   Kd = 0.;

   // Twiddle
   Twiddle_error          = 0.;
   Twiddle_best_error     = 0.;
   Twiddle_param_no       = 0;
   Twiddle_param_no_phase = 0;
   Twiddle_count          = 0;
   Twiddle_count_unit     = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double Kp_inc, double Ki_inc, double Kd_inc) {
   // Errors
   p_error = 0.;
   i_error = 0.;
   d_error = 0.;

   // Coefficients
   this->Kp = Kp;
   this->Ki = Ki;
   this->Kd = Kd;

   // Twiddle
   Twiddle_error          = 0.;
   Twiddle_best_error     = -1.;
   Twiddle_param_no       = 0;
   Twiddle_param_no_phase = 0;
   Twiddle_count          = 0;
   Twiddle_count_unit     = 5;

   new_param.resize(3);
   new_param[0]     = Kp;
   new_param[1]     = Ki;
   new_param[2]     = Kd;
   new_param_inc.resize(3);
   new_param_inc[0] = Kp_inc;
   new_param_inc[1] = Ki_inc;
   new_param_inc[2] = Kd_inc;
}

void PID::UpdateError(double cte) {
   p_error  = cte;
   i_error += cte;
   d_error  = cte - pre_cte;
   pre_cte  = cte;
}

double PID::TotalError() {
   return Kp * p_error + Ki * i_error + Kd * d_error;
}

void PID::RunningTwiddle(double cte) {

   if (Twiddle_count < Twiddle_count_unit) {
      // measure err
      Twiddle_error += cte;
      Twiddle_count ++;
   }


   if (Twiddle_count == Twiddle_count_unit) {

      if (Twiddle_param_no_phase == 0) {
         new_param[Twiddle_param_no] += new_param_inc[Twiddle_param_no];

         if (Twiddle_error < Twiddle_best_error) {
            Twiddle_best_error = Twiddle_error;
            new_param_inc[Twiddle_param_no] *= 1.1;
            Twiddle_param_no_phase = 2; // will finish this tern
         } else {
            Twiddle_param_no_phase = 1; // twiddle
         }

      } else 

      if (Twiddle_param_no_phase == 1) {


      }

      } else {
      new_param[Twiddle_param_no] -= 2. * new_param_inc[Twiddle_param_no];
      Twiddle_param_no_phase = 1;

      }


      //
      Twiddle_count = 0;
      Twiddle_error = 0.;
   }
}

/**
# Make this tolerance bigger if you are timing out!
def twiddle(tol=0.2): 
    # Don't forget to call `make_robot` before every call of `run`!
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    
    return p, best_err

 **/
