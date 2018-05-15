#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#define USE_TANH

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

  PID pid_steer, pid_throttle;
  // TODO: Initialize the pid variables.
  double Kp, Ki, Kd;

  Kp = 0.135;
  Ki = 0.0003;
  Kd = 3.0; 
  pid_steer.Init(Kp, Ki, Kd);

  Kp = 0.2;
  Ki = 0.00001;
  Kd = 0.1; 
  pid_throttle.Init(Kp, Ki, Kd);

  // one more pid is added to onMessage
  h.onMessage([&pid_steer, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          const double max_speed = 100.0;
          const double min_speed = 20.0;
          const double max_cte = 2.0; // try to be no more than this value
          const double coeff_steer = 30.0;
          const double coeff_throttle = 1.5*max_speed;
          
          pid_steer.UpdateError(cte);
#ifdef USE_TANH
          // steer_value must be proportial to the total error and inversely proportional to speed
          steer_value = -tanh(pid_steer.TotalError()*coeff_steer/((speed > 1.0) ? speed : 1.0));
#else 
          steer_value = -pid_steer.TotalError();
          steer_value = (steer_value > 1.0) ? 1.0 : steer_value;
          steer_value = (steer_value < -1.0) ? -1.0 : steer_value;
#endif
          
          pid_throttle.UpdateError(fabs(cte));
#ifdef USE_TANH
          // throttle_value must be proportial to the total error and inversely proportional to speed
          throttle_value = tanh(pid_throttle.TotalError()*coeff_throttle/((speed > 1.0) ? speed : 1.0));
#else
          const double coeff = 0.75;
          throttle_value = coeff - pid_throttle.TotalError();

          throttle_value = (throttle_value > 1.0) ? 1.0 : throttle_value;
          throttle_value = (throttle_value < -1.0) ? -1.0 : throttle_value;
#endif

          /*
          ** post processing of throttle_value
          */

          // 1. post processing of throttle_value based on speed
          if (speed > max_speed*0.8)
            throttle_value -= 0.5; // slow down 
          else if (speed < min_speed)
            throttle_value += 0.2;  // accelerating
          else
            throttle_value += 0.1;  // slowling accelerating
          
          // 2. post processing of throttle_value based on cte
          if (fabs(cte) > max_cte)
            throttle_value -= 0.5; // slow down 

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          // changed throttle value from originally o.3
          msgJson["throttle"] = throttle_value; // 0.3
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
