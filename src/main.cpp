#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid_steering;
  PID pid_throttle;

  //constexpr double target_speed = 35.;
  pid_throttle.Init(0.1216910, 8.45101e-05, 0.989856, 300, 1.00, 0.05, false);
  pid_steering.Init(0.0669295, 2.36171e-05, 8.190700, 400, 0.20, 0.03, true);
  pid_throttle.Init(0.1740180, 9.29611e-05, 1.08884, 300, 1.00, 0.05, false);
  pid_steering.Init(0.0729532, 2.85767e-05, 9.00977, 400, 0.20, 0.03, true);
// optimazed param:(9)  Kp=0.1740180, Ki=9.29611e-05, Kd=1.08884
// optimazed param:(8)  Kp=0.0729532, Ki=2.85767e-05, Kd=9.00977

  //constexpr double target_speed = 40.;
  pid_throttle.Init(0.535975, 0.000102257, 1.08884, 300, 1.00, 0.05, false);
  pid_steering.Init(0.253895, 4.25563e-05, 8.41554, 400, 0.20, 0.03, true);
// optimazed param:(39)  Kp=0.535975, Ki=0.000102257, Kd=1.08884
// optimazed param:(91)  Kp=0.253895, Ki=4.25563e-05, Kd=8.41554

  constexpr double target_speed = 43.;
  pid_throttle.Init(0.606188, 0.000123731, 1.19772, 300, 1.00, 0.05, false);
  pid_steering.Init(0.309419, 6.13236e-05, 9.25709, 400, 0.20, 0.03, true);
// optimazed param:(10)  Kp=0.606188, Ki=0.000123731, Kd=1.19772
// optimazed param:(17)  Kp=0.309419, Ki=6.13236e-05, Kd=9.25709



//   pid_throttle.Init(0.194097, 8.53552e-05, 0.989856, 200, 2.00, 0.05, false);
//   pid_steering.Init(0.104684, 3.45778e-05, 11.46700, 400, 0.10, 0.03, true);

//   pid_throttle.Init(0.246503, 7.68197e-05, 1.08884, 200, 2.00, 0.05, false);
//   pid_steering.Init(0.114860, 3.49236e-05, 10.6311, 400, 0.10, 0.03, true);

//   pid_throttle.Init(0.813460, 8.45017e-05, 1.19772, 200, 2.00, 0.05, false);
//   pid_steering.Init(0.361949, 7.56820e-05, 12.4313, 400, 0.10, 0.03, true);


//   constexpr double target_speed = 40.;
//   pid_throttle.Init(0.1216910, 8.45101e-05, 0.989856, 200, 1.00, 0.05, false);
//   pid_steering.Init(0.0669295, 2.36171e-05, 8.190700, 400, 0.20, 0.03, true);

// optimazed param:(3)  Kp=0.1338600, Ki=9.29611e-05, Kd=0.989856
//      new params:(3)  Kp=0.0602366, Ki=2.36171e-05, Kd=8.190700

  // constexpr double target_speed = 40.;
  // pid_throttle.Init(0.194840, 0.000101412, 0.989856, 200, 2.00, 0.05, false);
  // pid_steering.Init(0.104785, 2.35935e-05, 8.272610, 400, 0.10, 0.03, true);

  // constexpr double target_speed = 45.;
  // pid_throttle.Init(0.255046, 0.000102426, 1.20861, 200, 2.00, 0.05, false);
  // pid_steering.Init(0.104785, 2.59505e-05, 8.27261, 400, 0.20, 0.03, true);

  // pid_throttle.Init(0.3086060, 0.000112669, 1.20861, 200, 2.00, 0.05, false);
  // pid_steering.Init(0.1058330, 2.59505e-05, 9.09987, 400, 0.20, 0.03, true);


  h.onMessage([&pid_steering, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

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

          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << "  angle: " << angle << std::endl;
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          //msgJson["throttle"] = 0.3;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
