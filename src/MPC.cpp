#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"


using CppAD::AD;

// TODO: Set the timestep length and duration

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    // cout << "All Vars: " <<
    //   X_START << ", " <<
    //   Y_START << ", " <<
    //   PSI_START <<  ", " <<
    //   V_START << ", " <<
    //   CTE_START << ", " <<
    //   EPSI_START << ", " << endl;

    // Set initial values
    fg[0] = 0;
    fg[X_START + 1] = vars[X_START];
    fg[Y_START + 1] = vars[Y_START];
    fg[PSI_START + 1] = vars[PSI_START];
    fg[V_START + 1] = vars[V_START];
    fg[CTE_START + 1] = vars[CTE_START];
    fg[EPSI_START + 1] = vars[EPSI_START];

    // Add costs
    for(int i=0; i < MPC_N; i++){
      fg[0] += CTE_COST_FACTOR * CppAD::pow(vars[CTE_START + i], 2);
      fg[0] += EPSI_COST_FACTOR * CppAD::pow(vars[EPSI_START + i], 2);
      fg[0] += CppAD::pow(vars[V_START + i] - MPC_REF_V, 2);
    }

    for(int j=0;j<MPC_N-1; ++j){
      // Add actuators cost to fg[0]
      fg[0] += DELTA_COST_FACTOR * CppAD::pow(vars[DELTA_START + j], 2);
      fg[0] += A_COST_FACTOR * CppAD::pow(vars[A_START + j], 2);
    }

    for (int t = 0; t < MPC_N - 2; t++) {
      fg[0] += 1000*CppAD::pow(vars[DELTA_START + t + 1] - vars[DELTA_START + t], 2);
      fg[0] += CppAD::pow(vars[A_START + t + 1] - vars[A_START + t], 2);
    }

    // Set values for fg[1:]
    for(int t=1; t< MPC_N; ++t){
      AD<double> x1 = vars[X_START + t];
      AD<double> y1 = vars[Y_START + t];
      AD<double> psi1 = vars[PSI_START + t];
      AD<double> v1 = vars[V_START + t];
      AD<double> cte1 = vars[CTE_START + t];
      AD<double> epsi1 = vars[EPSI_START + t];

      // The state at time t.
      AD<double> x0 = vars[X_START + t - 1];
      AD<double> y0 = vars[Y_START + t - 1];
      AD<double> psi0 = vars[PSI_START + t - 1];
      AD<double> v0 = vars[V_START + t - 1];
      AD<double> cte0 = vars[CTE_START + t - 1];
      AD<double> epsi0 = vars[EPSI_START + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[DELTA_START + t - 1];
      AD<double> a0 = vars[A_START + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + X_START + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * MPC_DT);
      fg[1 + Y_START + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * MPC_DT);
      fg[1 + PSI_START + t] = psi1 - (psi0 + v0 * delta0 / MPC_LF * MPC_DT);
      fg[1 + V_START + t] = v1 - (v0 + a0 * MPC_DT);
      fg[1 + CTE_START + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * MPC_DT));
      fg[1 + EPSI_START + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / MPC_LF * MPC_DT);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Assign state values to local variables
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6 * MPC_N + 2 * (MPC_N - 1);
  // TODO: Set the number of constraints
  size_t n_constraints = 6 * MPC_N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  // cout << n_vars << endl;
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[X_START] = x;
  vars[Y_START] = y;
  vars[PSI_START] = psi;
  vars[V_START] = v;
  vars[CTE_START] = cte;
  vars[EPSI_START] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // set the range of values DELTA to [-25, 25] in radians
  for(int i=DELTA_START, j=A_START; i<A_START && j<n_vars; ++i, ++j){
    vars_upperbound[i] = DELTA_THRESHOLD;
    vars_lowerbound[i] = -1.0 * DELTA_THRESHOLD;
    vars_upperbound[j] = A_THRESHOLD;
    vars_lowerbound[j] = -1.0 * A_THRESHOLD;
  }
  // Set other limits as double numeric limits:
  for(int i = 0; i<DELTA_START; ++i){
    vars_lowerbound[i] = std::numeric_limits<double>::lowest();
    vars_upperbound[i] = std::numeric_limits<double>::max();
  }
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // TODO: Handle initial states seperately
  constraints_upperbound[X_START] = constraints_lowerbound[X_START] = x;
  constraints_upperbound[Y_START] = constraints_lowerbound[Y_START] = y;
  constraints_upperbound[PSI_START] = constraints_lowerbound[PSI_START] = psi;
  constraints_upperbound[V_START] = constraints_lowerbound[V_START] = v;
  constraints_upperbound[CTE_START] = constraints_lowerbound[CTE_START] = cte;
  constraints_upperbound[EPSI_START] = constraints_lowerbound[EPSI_START] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
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

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  std::vector<double> result = {solution.x[DELTA_START], solution.x[A_START]};
  for(int i=0; i < MPC_N; ++i) {
    result.push_back(solution.x[X_START + i + 1]);
  }
  for(int i=0; i < MPC_N; ++i) {
    result.push_back(solution.x[Y_START + i + 1]);
  }
  return result;
}
