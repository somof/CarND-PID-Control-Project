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

  // 10H tuning for 35mph
  // optimazed param:(419)   Kp=3.422630, Ki=0.000366662, Kd=1.96983
  // optimazed param:(1031)  Kp=0.232654, Ki=7.50166e-06, Kd=1.08217

  constexpr double target_speed = 66.;
  pid_throttle.Init(0.20, 0.0000, 1.0, 500, 8.00, true); // first params
  pid_steering.Init(0.10, 1.e-05, 1.9, 500, 0.05, false); // first params

  pid_steering.Init(0.102, 1.0002e-05, 1.9, 500, 0.5, false); // after 1H tuning
  pid_steering.Init(0.100348, 1.0002e-05, 1.9342, 500, 0.5, false); //

  // pid_throttle.Init(0.20808, 0.0000, 1.0, 500, 8.00, false);
  // pid_steering.Init(0.0995876, 9.83800e-06, 1.90000, 500, 0.5, false);
  // pid_steering.Init(0.1013362, 9.72735e-06, 1.83184, 500, 0.5, false);
  // pid_steering.Init(0.0993095, 9.72735e-06, 1.83184, 500, 0.5, false);

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
          double speed_err = target_speed - speed;

          // PID for Steering
          pid_steering.UpdateError(cte);
          steer_value = std::max(-1., std::min(+1., - pid_steering.TotalError())); // limit in [-1, 1].

          // exclusive control for two twiddles
          if (pid_steering.is_tuned && pid_throttle.is_tuned) {
             std::cout << "start twiddle for throttle" << std::endl;
             pid_throttle.is_tuned = false;
          }

          // PID for Throttle
          pid_throttle.UpdateError(speed_err);
          //throttle_value = pid_throttle.TotalError();
          throttle_value = std::min(1., pid_throttle.TotalError());

          // exclusive control for two twiddles
          if (pid_throttle.is_tuned && pid_steering.is_tuned) {
             std::cout << "start twiddle for steering" << std::endl;
             pid_steering.is_tuned = false;
          }


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
