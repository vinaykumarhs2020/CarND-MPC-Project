#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// Define compile time constants:
#define MPC_N 10
#define MPC_DT 0.15
#define X_START 0
#define Y_START (X_START+MPC_N)
#define PSI_START (Y_START+MPC_N)
#define V_START (PSI_START+MPC_N)
#define CTE_START (V_START+MPC_N)
#define EPSI_START (CTE_START+MPC_N)
#define DELTA_START (EPSI_START+MPC_N)
#define A_START (DELTA_START+MPC_N-1)

#define DELTA_THRESHOLD 0.436332
#define A_THRESHOLD 1.0
#define DELTA_CHANGE_THRESHOLD 10000
#define A_CHANGE_THRESHOLD 1000
#define PSI_CHANGE_THRESHOLD 1000

#define CTE_COST_FACTOR 1000
#define EPSI_COST_FACTOR 50000
#define DELTA_COST_FACTOR 50
#define VEL_COST_FACTOR 1.0
#define A_COST_FACTOR 50
#define MPC_LF 2.67
#define MPC_REF_V 65

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();
  const double Lf = MPC_LF;
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
