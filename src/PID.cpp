#include <vector>
#include <iostream>
#include <math.h>
#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(const double Kp, const double Ki, const double Kd,
               const int error_count_max,
               const double error_threshold,
               const bool is_tuned) {
   // Errors
   first_error = true;
   p_error = 0.;
   i_error = 0.;
   d_error = 0.;
   pre_cte = 0.;

   error_count = 0;
   error_sum   = 0.;
   curr_error  = 0.;
   best_error  = 0.;
   error_count_no      = 0;
   best_error_count_no = 0;

   // Coefficients
   this->Kp = Kp;
   this->Ki = Ki;
   this->Kd = Kd;
   this->error_count_max   = error_count_max;
   this->error_threshold   = error_threshold;
   this->is_tuned          = is_tuned;

   // Twiddle
   twiddle_state    = 0;
   twiddle_param_no = 0;

   twiddle_unit.resize(PID_PARAMETER_NUM);
   twiddle_unit[0] = Kp / 50.;
   twiddle_unit[1] = Ki / 50.;
   twiddle_unit[2] = Kd / 50.;
}

void PID::UpdateError(double cte) {
   if (first_error) {
      first_error = false;
      d_error  = 0;
   } else {
      d_error  = cte - pre_cte;
   }
   p_error  = cte;
   i_error += cte;
   pre_cte  = cte;

   //
   if (error_count < error_count_max && !is_tuned) {
      // count err
      error_count ++;
      error_sum += cte * cte;

   } else if (0 < error_count_max && !is_tuned) {
      // calc average err
      curr_error  = error_sum / error_count_max;
      error_count = 0;
      error_sum   = 0.;
      error_count_no ++;
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
   // Twiddle Algorithm pseudo code:
   // 
   //     best_err = measurement()
   //     for param, dparam in zip(all_params, all_dparam):
   //         param += dparam
   //         err = measurement()
   //         if err < best_err:
   //             best_err = err
   //             dparam *= 1.1
   //         else:
   //             param -= 2 * dparam
   //             err = measurement()
   //             if err < best_err:
   //                 best_err = err
   //                 dparam *= 1.1
   //             else:
   //                 param += dparam
   //                 dparam *= 0.9
   // 
   // 
   // statemachine-style algorithm pseudo code:
   // 
   //     state0: first measurement
   //         best_err = measurement()
   //         param += dparam // for positive trial
   //         err = measurement()
   //         goto sate 1
   //     
   //     state1: post positive trial
   //         if err < best_err:
   //             best_err = err
   //             dparam *= 1.1
   //             next_param += next_dparam
   //             err = measurement()
   //             goto state 1 w/ next param
   //         else:
   //             param -= 2 * dparam // negative trial
   //             err = measurement()
   //             goto sate 2
   //     
   //     state2: post negative trial
   //         if err < best_err:
   //             best_err = err
   //             dparam *= 1.1
   //             next_param += next_dparam
   //             err = measurement()
   //             goto state 1 w/ next param
   //         else:
   //             param += dparam // back to positive gain (state 1)
   //             dparam *= 0.9 // suppress gain
   //             next_param += next_dparam
   //             err = measurement()
   //             goto state 1 w/ next param


   if (is_tuned) {
      return;
   }

   if (error_count_max * error_count_no < 100) {
      // vehicle speed would be not stable yet
      return;
   }


   // the first measurement
   // force state0 by the best_error is determined
   if (error_count_no < 1 || 0. == best_error || 0 == twiddle_state) {
      best_error = curr_error;
      best_error_count_no = error_count_no;

      // start state machine
      twiddle_param_no = 0;
      updateParam(twiddle_param_no, twiddle_unit[twiddle_param_no]);
      twiddle_state    = 1;
      std::cout
         << "  start SM:       "
         << "  error " << curr_error
         << std::endl;
      return;
   }


   { // Debug Msg
      // std::cout << "twiddle:" << std::endl;
      std::cout
         << "  param " << twiddle_param_no
         << "  state " << twiddle_state
         << "  error " << curr_error;
      // << "  best error " << std::min(best_error, curr_error);
      if (curr_error < best_error) {
         std::cout << "  new params:"
                   << "(" << error_count_no << ")"
                   << "  " << Kp
                   << ", " << Ki
                   << ", " << Kd;
      } else {
         std::cout << "  +" << error_count_no - best_error_count_no << " loop";
      }
      std::cout << std::endl;
   }


   // next twiddle when convergence
   if (curr_error < error_threshold) {

      std::cout << "optimazed param:"
                << "(" << error_count_no << ")"
                << "  Kp=" << Kp
                << ", Ki=" << Ki
                << ", Kd=" << Kd
                << std::endl
                << std::endl;

      // twiddle_finished = true;
      best_error       = 0.;
      twiddle_param_no = 0;
      twiddle_state    = 0;
      is_tuned         = true;
      return;
   }


   if (best_error_count_no + PID_PARAMETER_NUM * 4 <= error_count_no) {
      // give up optimaization
      return;
   }


   // determine next state
   if (1 == twiddle_state) {
      // state1: post positive trial
      if (curr_error < best_error) {
         best_error = curr_error;
         best_error_count_no = error_count_no;
         twiddle_unit[twiddle_param_no] *= 1.1; // increase gain

         // goto next param
         twiddle_param_no = (twiddle_param_no + 1) % PID_PARAMETER_NUM;
         updateParam(twiddle_param_no, twiddle_unit[twiddle_param_no]);
         twiddle_state = 1;

      } else {
         // goto negative trial
         updateParam(twiddle_param_no, - 2. * twiddle_unit[twiddle_param_no]);
         twiddle_state = 2;
      }

   } else if (2 == twiddle_state) {
      // state2: post negative trial
      if (curr_error < best_error) {
         best_error = curr_error;
         best_error_count_no = error_count_no;
         twiddle_unit[twiddle_param_no] *= 1.1; // increase gain

         // goto next param
         twiddle_param_no = (twiddle_param_no + 1) % PID_PARAMETER_NUM;
         updateParam(twiddle_param_no, twiddle_unit[twiddle_param_no]);
         twiddle_state = 1;

      } else {
         updateParam(twiddle_param_no, twiddle_unit[twiddle_param_no]);
         twiddle_unit[twiddle_param_no] *= 0.9; // decrease gain

         // goto next param
         twiddle_param_no = (twiddle_param_no + 1) % PID_PARAMETER_NUM;
         updateParam(twiddle_param_no, twiddle_unit[twiddle_param_no]);
         twiddle_state = 1;
      }
   }
}
