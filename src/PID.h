#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double pre_cte;
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle
  */
  int Twiddle_param_no;
  int Twiddle_param_no_phase;
  int Twiddle_count;
  int Twiddle_count_unit;
  double Twiddle_error;
  double Twiddle_best_error;

  std::vector< double > new_param;
  std::vector< double > new_param_inc;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
   //void Init(double Kp, double Ki, double Kd, double Kp_inc, double Ki_inc, double Kd_inc);
   void Init(double Kp, double Ki, double Kd, double Kp_inc = 0., double Ki_inc = 0., double Kd_inc = 0.);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();


  /*
  * Additional Method
  */
  void RunningTwiddle(double cte);
};

#endif /* PID_H */
