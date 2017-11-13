#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double pre_cte;

  int    error_count;
  int    error_count_max;
  int    best_error_count_no;
  int    error_count_no;

  double error_sum;
  double curr_error;
  double best_error;
  double error_threshold;
  double twiddle_threshold;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle
  */
  int twiddle_state;
  int twiddle_param_no;
  std::vector< double > twiddle_unit;
  bool is_tuned;

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
  void Init(const double Kp, const double Ki, const double Kd, const int count_max, const double error_threshold, const double twiddle_threshold, const bool is_tuned);

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
  void updateParam(const int pnum, const double param_inc);
   void RunningTwiddle(void);
};

const int    PID_PARAMETER_NUM       = 3;


#endif /* PID_H */
