#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        std::cout << sdata << std::endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    // The global x positions of the waypoints in global frame coordinates
                    const vector<double> ptsx = j[1]["ptsx"];
                    // The global y positions of the waypoints in global frame coordinates
                    const vector<double> ptsy = j[1]["ptsy"];
                    const double px = j[1]["x"];
                    const double py = j[1]["y"];
                    const double psi = j[1]["psi"];
                    const double v = j[1]["speed"];
                    const double delta= j[1]["steering_angle"];
                    const double a = j[1]["throttle"];

                    // The vehicle model works in vehicle local frame coordinates. The waypoint coordinates must be
                    // transformed into the vehicle frame
                    auto ptsx_veh = Eigen::VectorXd(ptsx.size());
                    auto ptsy_veh = Eigen::VectorXd(ptsy.size());
                    for (size_t i = 0; i < ptsx.size(); ++i) {
                        // translation followed by rotation
                        double x = ptsx[i] - px;
                        double y = ptsy[i] - py;
                        ptsx_veh(i) = x * cos(psi) + y * sin(psi);
                        ptsy_veh(i) = - x * sin(psi) + y * cos(psi);
                    }

                    // Fit polynomial
                    constexpr int ORDER = 3;
                    auto cPoly = polyfit(ptsx_veh, ptsy_veh, ORDER);

                    // Calculate current errors for cte & epsi
                    // cte is the polynomial f(x=0) - y (y=0)
                    const double cte = cPoly(0);

                    // epsi is the angle of the tangent of f(x) at x = 0;
                    // epsi = psi - atan(f'(x=0)) with f' being the derivative of f and psi=0
                    const double epsi = -atan(cPoly(1));

                    const double Lf = 2.67;
                    const double dt = 0.1;

                    // To account for the delay the kinematic model is applied
                    // Due to the transformation in vehicle coordinates we have px=py=psi=0.0
                    const double px_veh = v * dt;
                    const double py_veh = 0.0;
                    const double psi_veh = - v * (-delta) / Lf * dt;
                    const double v_kin = v + a * dt;
                    const double cte_kin = cte + v * sin(epsi) * dt;
                    const double epsi_kin = epsi - v * delta / Lf * dt;

                    // prepare state vector
                    constexpr size_t STATE_SIZE = 6;
                    Eigen::VectorXd state(STATE_SIZE);
                    state << px_veh, py_veh, psi_veh, v_kin, cte_kin, epsi_kin;

                    // calcualte next actions using MPC
                    auto result = mpc.Solve(state, cPoly);

                    double steer_value = result.delta;
                    double throttle_value = result.a;

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the
                    //   steering value back. Otherwise the values will be in between
                    //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    // Display the MPC predicted trajectory

                    msgJson["mpc_x"] = std::move(result.mpc_x_vals);
                    msgJson["mpc_y"] = std::move(result.mpc_y_vals);

                    // Display the waypoints/reference line
                    vector<double> next_x_vals(ptsx.size());
                    vector<double> next_y_vals(ptsy.size());

                    for (size_t i = 0; i < ptsx_veh.size(); ++i) {
                        next_x_vals[i] = ptsx_veh(i);
                        next_y_vals[i] = ptsy_veh(i);
                    }

                    /**
                     * TODO: add (x,y) points to list here, points are in reference to
                     *   the vehicle's coordinate system the points in the simulator are
                     *   connected by a Yellow line
                     */

                    msgJson["next_x"] = std::move(next_x_vals);
                    msgJson["next_y"] = std::move(next_y_vals);


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    //   the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    //   around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}