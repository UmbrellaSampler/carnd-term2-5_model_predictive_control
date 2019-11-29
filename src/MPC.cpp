#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

// the timestep length and duration
static constexpr size_t N = 10;
static constexpr double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Index offsets of model variables
static constexpr size_t X_START = 0;
static constexpr size_t Y_START = X_START + N;
static constexpr size_t PSI_START = Y_START + N;
static constexpr size_t V_START = PSI_START + N;
static constexpr size_t CTE_START = V_START + N;
static constexpr size_t EPSI_START = CTE_START + N;
static constexpr size_t DELTA_START = EPSI_START + N;
static constexpr size_t A_START = DELTA_START + N - 1;

// Cost weights
// Error

constexpr double V_REF = 60;
constexpr double W_CTE = 3000.0;
constexpr double W_EPSI = 3000.0;

// Actuators
constexpr double W_DELTA = 300;
constexpr double W_A = 60;

// Actuator changes
constexpr double W_D_DELTA = 300000;
constexpr double W_D_A = 1000;


class FG_eval {
public:
    // Fitted polynomial coefficients
    VectorXd coeffs;

    FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        fg[0] = 0;

        // The part of the cost based on the reference state.
        for (int t = 0; t < N; t++) {
            fg[0] += W_CTE * CppAD::pow(vars[CTE_START + t], 2);
            fg[0] += W_EPSI * CppAD::pow(vars[EPSI_START + t], 2);
            fg[0] += CppAD::pow(vars[V_START + t] - V_REF, 2);
        }

        // Minimize change-rate of actuators
        for (int t = 0; t < N - 1; t++) {
            fg[0] += W_DELTA * CppAD::pow(vars[DELTA_START + t], 2);
            fg[0] += W_A * CppAD::pow(vars[A_START + t], 2);
        }

        // Increase cost for consecutive high actuators changes (smoothness factor)
        for (int t = 0; t < N - 2; t++) {
            fg[0] += W_D_DELTA * CppAD::pow(vars[DELTA_START + t + 1] - vars[DELTA_START + t], 2);
            fg[0] += W_D_A * CppAD::pow(vars[A_START + t + 1] - vars[A_START + t], 2);
        }

        fg[1 + X_START] = vars[X_START];
        fg[1 + Y_START] = vars[Y_START];
        fg[1 + PSI_START] = vars[PSI_START];
        fg[1 + V_START] = vars[V_START];
        fg[1 + CTE_START] = vars[CTE_START];
        fg[1 + EPSI_START] = vars[EPSI_START];

        for (int t = 1; t < N; t++) {

            AD<double> x1 = vars[X_START + t];
            AD<double> x0 = vars[X_START + t - 1];

            AD<double> y1 = vars[Y_START + t];
            AD<double> y0 = vars[Y_START + t - 1];

            AD<double> psi1 = vars[PSI_START + t];
            AD<double> psi0 = vars[PSI_START + t - 1];

            AD<double> v1 = vars[V_START + t];
            AD<double> v0 = vars[V_START + t - 1];

            AD<double> cte1 = vars[CTE_START + t];
            AD<double> cte0 = vars[CTE_START + t - 1];

            AD<double> epsi1 = vars[EPSI_START + t];
            AD<double> epsi0 = vars[EPSI_START + t - 1];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[DELTA_START + t - 1];
            AD<double> a0 = vars[A_START + t - 1];

            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
            AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

            fg[1 + X_START + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + Y_START + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + PSI_START + t] = psi1 - (psi0 - v0 / Lf * delta0 * dt);
            fg[1 + V_START + t] = v1 - (v0 + a0 * dt);
            fg[1 + CTE_START + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[1 + EPSI_START + t] = epsi1 - ((psi0 - psides0) - v0 / Lf * delta0 * dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}

MPC::~MPC() {}

MPC::Result MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    const double x = state[0];
    const double y = state[1];
    const double psi = state[2];
    const double v = state[3];
    const double cte = state[4];
    const double epsi = state[5];

    /**
     * TODO: Set the number of model variables (includes both states and inputs).
     * For example: If the state is a 4 element vector, the actuators is a 2
     *   element vector and there are 10 timesteps. The number of variables is:
     *   4 * 10 + 2 * 9
     */
    // number of model variables (includes both states and inputs).
    constexpr size_t n_vars = N * 6 + (N - 1) * 2;
    /**
     * TODO: Set the number of constraints
     */
    // number of constraints
    constexpr size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; ++i) {
        vars[i] = 0;
    }

    vars[X_START] = x;
    vars[Y_START] = y;
    vars[PSI_START] = psi;
    vars[V_START] = v;
    vars[CTE_START] = cte;
    vars[EPSI_START] = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set lower and upper limits for variables.

    // max values for non actuator constraints
    for (int i = 0; i < DELTA_START; i++) {
        vars_lowerbound[i] = std::numeric_limits<double>::lowest();
        vars_upperbound[i] = std::numeric_limits<double>::max();
    }

    // steering angle between -25 and 25 degrees
    constexpr double DELTA_ABS = M_PI / 180.0 * 25.0;
    for (int i = DELTA_START; i < A_START; i++) {
        vars_lowerbound[i] = -DELTA_ABS;
        vars_upperbound[i] = DELTA_ABS;
    }

    // throttle between -1.0 and 1.0
    for (int i = A_START; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; ++i) {
        constraints_lowerbound[i] = 0.0;
        constraints_upperbound[i] = 0.0;
    }

    constraints_lowerbound[X_START] = x;
    constraints_lowerbound[Y_START] = y;
    constraints_lowerbound[PSI_START] = psi;
    constraints_lowerbound[V_START] = v;
    constraints_lowerbound[CTE_START] = cte;
    constraints_lowerbound[EPSI_START] = epsi;

    constraints_upperbound[X_START] = x;
    constraints_upperbound[Y_START] = y;
    constraints_upperbound[PSI_START] = psi;
    constraints_upperbound[V_START] = v;
    constraints_upperbound[CTE_START] = cte;
    constraints_upperbound[EPSI_START] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    // NOTE: You don't have to worry about these options
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    //   of sparse routines, this makes the computation MUCH FASTER. If you can
    //   uncomment 1 of these and see if it makes a difference or not but if you
    //   uncomment both the computation time should go up in orders of magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    Result result{};
    result.delta = solution.x[DELTA_START];
    result.a = solution.x[A_START];

    for (int i = 0; i < N - 1; i++) {
        result.mpc_x_vals.push_back(solution.x[X_START + i + 1]);
        result.mpc_y_vals.push_back(solution.x[Y_START + i + 1]);
    }

    return result;
}