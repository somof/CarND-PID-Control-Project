#include <vector>
#include <iostream>
#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double Kp_inc, double Ki_inc, double Kd_inc) {
   // Errors
   p_error = 0.;
   i_error = 0.;
   d_error = 0.;
   pre_cte = 0.;

   error_count = 0;
   error_sum   = 0.;
   curr_error  = 0.;
   best_error  = 0.;
   error_measure_num = 0;

   // Coefficients
   this->Kp = Kp;
   this->Ki = Ki;
   this->Kd = Kd;

   // Twiddle
   twiddle_phase      = 0;
   twiddle_param_no   = 0;

   twiddle_unit.resize(PID_PARAMETER_NUM);
   twiddle_unit[0] = Kp_inc;
   twiddle_unit[1] = Ki_inc;
   twiddle_unit[2] = Kd_inc;

   twiddle_finished = false;
}

void PID::UpdateError(double cte) {
   p_error  = cte;
   i_error += cte;
   d_error  = cte - pre_cte;
   pre_cte  = cte;

   //
   if (error_count < PID_COLLECT_DATA_NUM) {
      // count err
      error_count ++;
      error_sum += cte * cte;
   } else {
      // measure average err
      curr_error  = error_sum / PID_COLLECT_DATA_NUM;
      error_count = 0;
      error_sum   = 0.;
      error_measure_num ++;
      //
      RunningTwiddle();
   }
}

double PID::TotalError() {
   return Kp * p_error + Ki * i_error + Kd * d_error;
}

void PID::updateParam(const int pnum, const double param_inc) {
   if (0 == pnum) {
      Kp += param_inc;
   } else if (1 == pnum) {
      Ki += param_inc;
   } else if (2 == pnum) {
      Kd += param_inc;
   }
}

void PID::RunningTwiddle(void) {
   if (twiddle_finished) {
      return;
   }

   // the first measurement
   if (error_measure_num < 1) {
      best_error = curr_error;
      return;
   }

   // check completion
   if (curr_error < best_error) {
      // increae gain and goto next param, if best error is updated
      best_error = curr_error;
      twiddle_unit[twiddle_param_no] *= 1.1; // proceed more
// !!      updateParam(twiddle_param_no, twiddle_unit[twiddle_param_no]);
      // goto next param
      twiddle_param_no = (twiddle_param_no + 1) % PID_PARAMETER_NUM;
      twiddle_phase = 0;



      double sum_twiddle_unit = twiddle_unit[0] + twiddle_unit[1] + twiddle_unit[2];
      if (sum_twiddle_unit < PID_TOLERANCE_THRESHOLD) {
         std::cout << "optimazed param:"
                   << "(" << error_measure_num << ")"
                   << " Kp " << Kp
                   << " Ki " << Ki
                   << " Kd " << Kd
                   << std::endl;
         twiddle_finished = true;
      }
      return;
   }


   // Debug
   std::cout << "twiddle:" << std::endl;
   std::cout
      << "  pnum " << twiddle_param_no
      << "  phase " <<  twiddle_phase
      << std::endl;


   if (twiddle_phase == 0) {
      // Phase0: try normal side twiddle with gain 1.1

      twiddle_phase = 1;

   } else if (twiddle_phase == 1) {
      // Phase1: try another side twiddle with minus-gain

//      updateParam(twiddle_param_no, - 2. * twiddle_unit[twiddle_param_no]); // will try another side
      twiddle_phase = 2;

   } else if (twiddle_phase == 2) {
      // Phase2: try normal side twiddle with gain 0.9

//      updateParam(twiddle_param_no, 0.9 * twiddle_unit[twiddle_param_no]); // will try another side


      // goto next param
      twiddle_param_no = (twiddle_param_no + 1) % PID_PARAMETER_NUM;
      twiddle_phase = 0;
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
