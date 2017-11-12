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

  int    error_measure_num;
  int    error_count;
  double error_sum;
  double curr_error;
  double best_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle
  */
  int twiddle_phase;
  int twiddle_param_no;
  std::vector< double > twiddle_unit;
  bool twiddle_finished;

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
  void updateParam(const int pnum, const double param_inc);
   void RunningTwiddle(void);
};

const int    PID_COLLECT_DATA_NUM    = 10;
const int    PID_PARAMETER_NUM       = 3;
const double PID_TOLERANCE_THRESHOLD = 0.2;


#endif /* PID_H */
