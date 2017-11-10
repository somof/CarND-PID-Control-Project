#include "PID.h"

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
   // Errors
   p_error = 0.;
   i_error = 0.;
   d_error = 0.;

   // Coefficients
   Kp = 0.;
   Ki = 0.;
   Kd = 0.;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
   /*
    * Errors
    */
   this->p_error = 0;
   this->i_error = 0;
   this->d_error = 0;



   /*
    * Coefficients
    */ 
   this->Kp = Kp;
   this->Ki = Ki;
   this->Kd = Kd;
}

void PID::UpdateError(double cte) {

   d_error  = cte - prev_cte;
   i_error += cte;
   p_error  = cte;

   // double steer = - Kp * cte - Ki * i_error - Kd * d_error;


}

double PID::TotalError() {
   
}

