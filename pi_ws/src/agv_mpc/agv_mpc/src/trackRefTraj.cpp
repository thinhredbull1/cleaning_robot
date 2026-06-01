

#include "trackRefTraj.h"
//#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <Eigen/Core>



using CppAD::AD;

// =========================================
// FG_eval class definition implementation.
// =========================================
class FG_eval 
{
    public:
  
        Eigen::VectorXd coeffs;

        double _dt, _ref_cte, _ref_etheta, _ref_vel; 
        double  _w_cte, _w_etheta, _w_vel, _w_angvel, _w_accel, _w_angvel_d, _w_accel_d;
        int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start;

        AD<double> cost_cte, cost_etheta, cost_vel;
        // Constructor
        FG_eval(Eigen::VectorXd coeffs) 
        { 
            this->coeffs = coeffs; 

      
            _dt = 0.1;  // in sec
            _ref_cte   = 0;
            _ref_etheta  = 0;
            _ref_vel   = 0.5; // m/s
            _w_cte     = 100;
            _w_etheta    = 100;
            _w_vel     = 1;
            _w_angvel   = 100;
            _w_accel   = 50;
            _w_angvel_d = 0;
            _w_accel_d = 0;

            _mpc_steps   = 40;
            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _theta_start   = _y_start + _mpc_steps;
            _v_start     = _theta_start + _mpc_steps;
            _cte_start   = _v_start + _mpc_steps;
            _etheta_start  = _cte_start + _mpc_steps;
            _angvel_start = _etheta_start + _mpc_steps;
            _a_start     = _angvel_start + _mpc_steps - 1;
        }

        // Load parameters for constraints
        void LoadParams(const std::map<string, double> &params)
        {
            _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
            _mpc_steps = params.find("STEPS") != params.end()    ? params.at("STEPS") : _mpc_steps;
            _ref_cte   = params.find("REF_CTE") != params.end()  ? params.at("REF_CTE") : _ref_cte;
            _ref_etheta  = params.find("REF_ETHETA") != params.end() ? params.at("REF_ETHETA") : _ref_etheta;
            _ref_vel   = params.find("REF_V") != params.end()    ? params.at("REF_V") : _ref_vel;
            
            _w_cte   = params.find("W_CTE") != params.end()   ? params.at("W_CTE") : _w_cte;
            _w_etheta  = params.find("W_EPSI") != params.end()  ? params.at("W_EPSI") : _w_etheta;
            _w_vel   = params.find("W_V") != params.end()     ? params.at("W_V") : _w_vel;
            _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;
            _w_accel = params.find("W_A") != params.end()     ? params.at("W_A") : _w_accel;
            _w_angvel_d = params.find("W_DANGVEL") != params.end() ? params.at("W_DANGVEL") : _w_angvel_d;
            _w_accel_d = params.find("W_DA") != params.end()     ? params.at("W_DA") : _w_accel_d;

            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _theta_start   = _y_start + _mpc_steps;
            _v_start     = _theta_start + _mpc_steps;
            _cte_start   = _v_start + _mpc_steps;
            _etheta_start  = _cte_start + _mpc_steps;
            _angvel_start = _etheta_start + _mpc_steps;
            _a_start     = _angvel_start + _mpc_steps - 1;
            
            
        }

   
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
          
        void operator()(ADvector& fg, const ADvector& vars) 
        {
            // fg[0] for cost function
            fg[0] = 0;
            cost_cte =  0;
            cost_etheta = 0;
            cost_vel = 0;

            /*
            for (int i = 0; i < _mpc_steps; i++) 
            {
                cout << i << endl;
                cout << "_x_start" << vars[_x_start + i] <<endl;
                cout << "_y_start" << vars[_y_start + i] <<endl;
                cout << "_theta_start" << vars[_theta_start + i] <<endl;
                cout << "_v_start" << vars[_v_start + i] <<endl;
                cout << "_cte_start" << vars[_cte_start + i] <<endl;
                cout << "_etheta_start" << vars[_etheta_start + i] <<endl;
            }*/

            for (int i = 0; i < _mpc_steps; i++) 
            {
              fg[0] += _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2); // cross deviation error

              fg[0] += _w_etheta * CppAD::pow(vars[_etheta_start + i] - _ref_etheta, 2); // heading error
              fg[0] += _w_vel * CppAD::pow(vars[_v_start + i] - _ref_vel, 2); // speed error
              /// adding
            //   fg[0] += 300.0 * CppAD::pow(vars[_angvel_start + i], 2) * CppAD::pow(vars[_etheta_start + i], 2);
            //   fg[0] += 200.0 * CppAD::pow(vars[_v_start + i] * CppAD::sin(vars[_etheta_start + i]), 2);

              //
              cost_cte +=  _w_cte * CppAD::pow(vars[_cte_start + i] - _ref_cte, 2);
              cost_etheta +=  (_w_etheta * CppAD::pow(vars[_etheta_start + i] - _ref_etheta, 2)); 
              cost_vel +=  (_w_vel * CppAD::pow(vars[_v_start + i] - _ref_vel, 2)); 
            }
            // cout << "-----------------------------------------------" <<endl;
            cout << "cost_cte, etheta, velocity: " << cost_cte << ", " << cost_etheta  << ", " << cost_vel << endl;
            

   
            for (int i = 0; i < _mpc_steps - 1; i++) {
              fg[0] += _w_angvel * CppAD::pow(vars[_angvel_start + i], 2);
              fg[0] += _w_accel * CppAD::pow(vars[_a_start + i], 2);
            }
            // cout << "cost of actuators: " << fg[0] << endl; 

            // Minimize the value gap between sequential actuations.
            for (int i = 0; i < _mpc_steps - 2; i++) {
              fg[0] += _w_angvel_d * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
              fg[0] += _w_accel_d * CppAD::pow(vars[_a_start + i + 1] - vars[_a_start + i], 2);
            }
            // cout << "cost of gap: " << fg[0] << endl; 
            

            // fg[x] for constraints
            // Initial constraints
            fg[1 + _x_start] = vars[_x_start];
            fg[1 + _y_start] = vars[_y_start];
            fg[1 + _theta_start] = vars[_theta_start];
            fg[1 + _v_start] = vars[_v_start];
            fg[1 + _cte_start] = vars[_cte_start];
            fg[1 + _etheta_start] = vars[_etheta_start];

            //  system dynamic model 
            for (int i = 0; i < _mpc_steps - 1; i++)
            {
                // The state at time t+1 .
                AD<double> x1 = vars[_x_start + i + 1];
                AD<double> y1 = vars[_y_start + i + 1];
                AD<double> theta1 = vars[_theta_start + i + 1];
                AD<double> v1 = vars[_v_start + i + 1];
                AD<double> cte1 = vars[_cte_start + i + 1];
                AD<double> etheta1 = vars[_etheta_start + i + 1];

                // The state at time t.
                AD<double> x0 = vars[_x_start + i];
                AD<double> y0 = vars[_y_start + i];
                AD<double> theta0 = vars[_theta_start + i];
                AD<double> v0 = vars[_v_start + i];
                AD<double> cte0 = vars[_cte_start + i];
                AD<double> etheta0 = vars[_etheta_start + i];


                AD<double> w0 = vars[_angvel_start + i];
                AD<double> a0 = vars[_a_start + i];


                //AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
                AD<double> f0 = 0.0;
                for (int i = 0; i < coeffs.size(); i++) 
                {
                    f0 += coeffs[i] * CppAD::pow(x0, i); //f(0) = y
                }

                //AD<double> trj_grad0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
                AD<double> trj_grad0 = 0.0;
                for (int i = 1; i < coeffs.size(); i++) 
                {
                    trj_grad0 += i*coeffs[i] * CppAD::pow(x0, i-1); // f'(x0) = f(1)/1
                }
                trj_grad0 = CppAD::atan(trj_grad0);


          
                fg[2 + _x_start + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * _dt);
                fg[2 + _y_start + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * _dt);
                fg[2 + _theta_start + i] = theta1 - (theta0 +  w0 * _dt);
                fg[2 + _v_start + i] = v1 - (v0 + a0 * _dt);
                
                fg[2 + _cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(etheta0) * _dt));
                fg[2 + _etheta_start + i] = etheta1 - ((theta0 - trj_grad0) + w0 * _dt);//theta0-trj_grad0)->etheta : it can have more curvature prediction, but its gradient can be only adjust positive plan.   
                //fg[2 + _etheta_start + i] = etheta1 - (etheta0 + w0 * _dt);
            }
        }
};

// ====================================
// MPC class definition implementation.
// ====================================
MPC::MPC() 
{
    // Set default value    
    _mpc_steps = 20;
    _max_angvel = 3.0; // 
    _max_throttle = 1.0; //
    _bound_value  = 1.0e3; // 

    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;
    _v_start     = _theta_start + _mpc_steps;
    _cte_start   = _v_start + _mpc_steps;
    _etheta_start  = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _a_start     = _angvel_start + _mpc_steps - 1;

}

void MPC::LoadParams(const std::map<string, double> &params)
{
    _params = params;
    //Init parameters for MPC object
    _mpc_steps = _params.find("STEPS") != _params.end() ? _params.at("STEPS") : _mpc_steps;
    _max_angvel = _params.find("ANGVEL") != _params.end() ? _params.at("ANGVEL") : _max_angvel;
    _max_throttle = _params.find("MAXTHR") != _params.end() ? _params.at("MAXTHR") : _max_throttle;
    _bound_value  = _params.find("BOUND") != _params.end()  ? _params.at("BOUND") : _bound_value;
    
    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;
    _v_start     = _theta_start + _mpc_steps;
    _cte_start   = _v_start + _mpc_steps;
    _etheta_start  = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _a_start     = _angvel_start + _mpc_steps - 1;

    cout << "\n!! MPC Obj parameters updated !! " << endl; 
}
void MPC::UpdateWeights(
    const std::map<string,double>& params)
{
    _params["W_CTE"] =
        params.at("W_CTE");

    _params["W_EPSI"] =
        params.at("W_EPSI");

    _params["W_V"] =
        params.at("W_V");

    _params["W_ANGVEL"] =
        params.at("W_ANGVEL");

    _params["W_A"] =
        params.at("W_A");

    _params["W_DANGVEL"] =
        params.at("W_DANGVEL");

    _params["W_DA"] =
        params.at("W_DA");

    _params["REF_V"] =
        params.at("REF_V");
}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double v = state[3];
    const double cte = state[4];
    const double etheta = state[5];


    size_t n_vars = _mpc_steps * 6 + (_mpc_steps - 1) * 2;
    
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * 6;


    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_x_start] = x;
    vars[_y_start] = y;
    vars[_theta_start] = theta;
    vars[_v_start] = v;
    vars[_cte_start] = cte;
    vars[_etheta_start] = etheta;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
  
    for (int i = 0; i < _angvel_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }
   
    for (int i = _angvel_start; i < _a_start; i++) 
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }

    for (int i = _a_start; i < n_vars; i++)  
    {
        vars_lowerbound[i] = -_max_throttle;
        vars_upperbound[i] = _max_throttle;
    }


   
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    constraints_lowerbound[_v_start] = v;
    constraints_lowerbound[_cte_start] = cte;
    constraints_lowerbound[_etheta_start] = etheta;
    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    constraints_upperbound[_v_start] = v;
    constraints_upperbound[_cte_start] = cte;
    constraints_upperbound[_etheta_start] = etheta;

   
    FG_eval fg_eval(coeffs);
    fg_eval.LoadParams(_params);

    fg_eval.cost_cte;

   
    std::string options;
  
    options += "Integer print_level  0\n";
 
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    options += "Numeric max_cpu_time          0.5\n";

   
    CppAD::ipopt::solve_result<Dvector> solution;

    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    // std::cout << "------------ Total Cost(solution): " << cost << "------------" << std::endl;
    // cout << "-----------------------------------------------" <<endl;
    _mpc_totalcost = cost;
    _mpc_ctecost = Value(fg_eval.cost_cte);
    _mpc_ethetacost = Value(fg_eval.cost_etheta);
    _mpc_velcost = Value(fg_eval.cost_vel);

    this->mpc_x = {};
    this->mpc_y = {};
    this->mpc_theta = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
        this->mpc_theta.push_back(solution.x[_theta_start + i]);
    }
    
    vector<double> result;
    result.push_back(solution.x[_angvel_start]);
    result.push_back(solution.x[_a_start]);
    return result;
}
