#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t N = 15;
double dt = 0.05;
int late = static_cast<int>(0.1/dt);

const double Lf = 2.67;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    fg[0] = 0; 
    for (size_t t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - 85, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 20*CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += 600*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
     
    fg[1+x_start] = vars[x_start];
    fg[1+y_start] = vars[y_start];
    fg[1+psi_start] = vars[psi_start];
    fg[1+v_start] = vars[v_start];
    fg[1+cte_start] = vars[cte_start];
    fg[1+epsi_start] = vars[epsi_start];

    for (size_t t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
  
      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
  
      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
  
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 +coeffs[2] * x0*x0 + coeffs[3] * x0*x0*x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2] * x0 + 3*coeffs[3] * x0*x0);
  
      // Recall the equations for the model:
      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      // v_[t] = v[t-1] + a[t-1] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
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
  typedef CPPAD_TESTVECTOR(double) Dvector;

  size_t n_vars = 8*N-2;
  size_t n_constraints = 6*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
 
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[epsi_start] = state[5];
 
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
 
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (size_t i = delta_start; i < delta_start + late; i++) {
    vars_lowerbound[i] = prev_delta;
    vars_upperbound[i] = prev_delta;
  }


  for (size_t i = delta_start + late; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  for (size_t i = a_start; i < a_start + late; i++) {
    vars_lowerbound[i] = prev_a;
    vars_upperbound[i] = prev_a;
  }

  for (size_t i = a_start + late; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] =  1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  constraints_lowerbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_lowerbound[psi_start] = state[2];
  constraints_lowerbound[v_start] = state[3];
  constraints_lowerbound[cte_start] = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  constraints_upperbound[x_start] = state[0];
  constraints_upperbound[y_start] = state[1];
  constraints_upperbound[psi_start] = state[2];
  constraints_upperbound[v_start] = state[3];
  constraints_upperbound[cte_start] = state[4];
  constraints_upperbound[epsi_start] = state[5];


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

  mpc_x = vector<double>();
  mpc_y = vector<double>();

  for (size_t i = 0; i < N; i++){
    mpc_x.push_back(solution.x[x_start+i]);
    mpc_y.push_back(solution.x[y_start+i]);
  }

  prev_delta = solution.x[delta_start+late];
  prev_a = solution.x[a_start+late];

  return {solution.x[delta_start+late],solution.x[a_start+late]};
}
