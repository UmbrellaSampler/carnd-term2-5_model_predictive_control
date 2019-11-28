#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC {
public:
    MPC();

    virtual ~MPC();

    struct Result {
        double delta;
        double a;
        std::vector<double> mpc_x_vals;
        std::vector<double> mpc_y_vals;
    };

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    Result Solve(const Eigen::VectorXd &state,
                              const Eigen::VectorXd &coeffs);


};

#endif  // MPC_H
