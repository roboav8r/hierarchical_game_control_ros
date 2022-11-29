#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

using Eigen::Vector2d;
using Eigen::Vector4d;

/*
DEFINE CONTROL VARIABLES TO SOLVE
*/
class ControlVariables : public ifopt::VariableSet {
public:
  // Every variable set has a name, here "control_vars". this allows the constraints
  // and costs to define values and Jacobians specifically w.r.t this variable set.
  ControlVariables() : ControlVariables("control_vars") {};
  ControlVariables(const std::string& name) : VariableSet(2, name)
  {
    // the initial values where the NLP starts iterating from
    yawRate_ = 0.0;
    acc_ = 0.0;
  }

  // Here is where you can transform the Eigen::Vector into whatever
  // internal representation of your variables you have (here two doubles, but
  // can also be complex classes such as splines, etc..
  void SetVariables(const VectorXd& x) override
  {
    yawRate_ = x(0);
    acc_ = x(1);
  };

  // Here is the reverse transformation from the internal representation to
  // to the Eigen::Vector
  VectorXd GetValues() const override
  {
    return Vector2d(yawRate_, acc_);
  };

  // TODO add SetBounds() and member initializer list
  // Each variable has an upper and lower bound set here
  VecBound GetBounds() const override
  {
    VecBound bounds(GetRows());
    bounds.at(0) = ifopt::Bounds(-ifopt::inf, ifopt::inf); // TODO replace this with actual limiting values
    bounds.at(1) = ifopt::Bounds(-ifopt::inf, ifopt::inf);
    return bounds;
  }

private:
  double yawRate_, acc_;
};

class StateVariables : public ifopt::VariableSet {
public:
  // Every variable set has a name, here "control_vars". this allows the constraints
  // and costs to define values and Jacobians specifically w.r.t this variable set.
  StateVariables() : StateVariables("state_vars") {};
  StateVariables(const std::string& name) : VariableSet(4, name)
  {
    // // the initial values where the NLP starts iterating from
    // yawRate_ = 0.0;
    // acc_ = 0.0;
  }
  StateVariables(const std::string& name, float min_vel, float max_vel) : 
    VariableSet(4, name),
    minVel_(min_vel), maxVel_(max_vel) {}

  // Here is where you can transform the Eigen::Vector into whatever
  // internal representation of your variables you have (here two doubles, but
  // can also be complex classes such as splines, etc..
  void SetVariables(const VectorXd& x) override
  {
    xPos_ = x(0);
    yPos_ = x(1);
    heading_ = x(2);
    speed_ = x(3);
  };

  // Here is the reverse transformation from the internal representation to
  // to the Eigen::Vector
  VectorXd GetValues() const override
  {
    return Vector4d(xPos_, yPos_, heading_, speed_);
  };

  // Each variable has an upper and lower bound set here
  VecBound GetBounds() const override
  {
    VecBound bounds(GetRows());
    bounds.at(0) = ifopt::Bounds(-ifopt::inf, ifopt::inf); 
    bounds.at(1) = ifopt::Bounds(-ifopt::inf, ifopt::inf);
    bounds.at(2) = ifopt::Bounds(-ifopt::inf, ifopt::inf);
    bounds.at(3) = ifopt::Bounds(-ifopt::inf, ifopt::inf);
    return bounds;
  }

private:
  double xPos_, yPos_, heading_, speed_;
  float minVel_, maxVel_;
};




/*
DEFINE CONSTRAINTS
*/

class DynConstraint : public ifopt::ConstraintSet {
public:
  // Constructors
  DynConstraint() : DynConstraint("constraint1") {}
  DynConstraint(const std::string& name) : ConstraintSet(4, name) {}
  DynConstraint(const std::string& name, const float dt) : ConstraintSet(4, name), dt_(dt) {}

  // Member variables
  float dt_{0.1};

  // The constraint value minus the constant value "1", moved to bounds.
  VectorXd GetValues() const override
  {
    VectorXd g(GetRows());
    VectorXd u = GetVariables()->GetComponent("control_vars")->GetValues();
    VectorXd x = GetVariables()->GetComponent("curr_state_vars")->GetValues();
    VectorXd x_tgt = GetVariables()->GetComponent("tgt_state_vars")->GetValues();
    g(0) = x(0) + dt_*x_tgt(3)*cos(x_tgt(2)) - x_tgt(0);
    g(1) = x(1) + dt_*x_tgt(3)*sin(x_tgt(2)) - x_tgt(1);
    g(2) = x(2) + u(0)*dt_ - x_tgt(2);
    g(3) = x(3) + u(1)*dt_ - x_tgt(3);
    return g;
  };

  // The only constraint in this set is an equality constraint to 1.
  // Constant values should always be put into GetBounds(), not GetValues().
  // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    b.at(0) = ifopt::Bounds(0, 0);
    b.at(1) = ifopt::Bounds(0, 0);
    b.at(2) = ifopt::Bounds(0, 0);
    b.at(3) = ifopt::Bounds(0, 0);
    return b;
  }

  // This function provides the first derivative of the constraints.
  // In case this is too difficult to write, you can also tell the solvers to
  // approximate the derivatives by finite differences and not overwrite this
  // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
  // Attention: see the parent class function for important information on sparsity pattern.
  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
  {
    // must fill only that submatrix of the overall Jacobian that relates
    // to this constraint and "control_vars". even if more constraints or variables
    // classes are added, this submatrix will always start at row 0 and column 0,
    // thereby being independent from the overall problem.
    if (var_set == "control_vars") {

      jac_block.coeffRef(0, 0) = 0.0;       // derivative of first constraint w.r.t yaw rate
      jac_block.coeffRef(0, 1) = 0.0;       // derivative of first constraint w.r.t acceleration
      jac_block.coeffRef(1, 0) = 0.0;       // derivative of 2nd constraint w.r.t yaw rate
      jac_block.coeffRef(1, 1) = 0.0;       // derivative of 2nd constraint w.r.t acceleration
      jac_block.coeffRef(2, 0) = dt_;       // derivative of 3rd constraint w.r.t yaw rate
      jac_block.coeffRef(2, 1) = 0.0;       // derivative of 3rd constraint w.r.t acceleration
      jac_block.coeffRef(3, 0) = 0.0;       // derivative of 4th constraint w.r.t yaw rate
      jac_block.coeffRef(3, 1) = dt_;       // derivative of 4th constraint w.r.t acceleration
    }

    if (var_set == "curr_state_vars") {
      VectorXd x = GetVariables()->GetComponent("curr_state_vars")->GetValues();

      jac_block.coeffRef(0, 0) = 1.0;       // derivative of 1st constraint w.r.t p_x
      jac_block.coeffRef(0, 1) = 0.0;       // derivative of 1st constraint w.r.t p_y
      jac_block.coeffRef(0, 2) = 0.0;   // derivative of 1st constraint w.r.t heading
      jac_block.coeffRef(0, 3) = 0.0;         // derivative of 1st constraint w.r.t velocity
      jac_block.coeffRef(1, 0) = 0.0;       // derivative of 2nd constraint w.r.t p_x
      jac_block.coeffRef(1, 1) = 1.0;       // derivative of 2nd constraint w.r.t p_y
      jac_block.coeffRef(1, 2) = 0.0;   // derivative of 2nd constraint w.r.t heading
      jac_block.coeffRef(1, 3) = 0.0;         // derivative of 2nd constraint w.r.t velocity
      jac_block.coeffRef(2, 0) = 0.0;       // derivative of 3rd constraint w.r.t p_x
      jac_block.coeffRef(2, 1) = 0.0;       // derivative of 3rd constraint w.r.t p_y
      jac_block.coeffRef(2, 2) = 1.0;       // derivative of 3rd constraint w.r.t heading
      jac_block.coeffRef(2, 3) = 0.0;         // derivative of 3rd constraint w.r.t velocity
      jac_block.coeffRef(3, 0) = 0.0;       // derivative of 4th constraint w.r.t p_x
      jac_block.coeffRef(3, 1) = 0.0;       // derivative of 4th constraint w.r.t p_y
      jac_block.coeffRef(3, 2) = 0.0;       // derivative of 4th constraint w.r.t heading
      jac_block.coeffRef(3, 3) = 1.0;         // derivative of 4th constraint w.r.t velocity

    }

    if (var_set == "tgt_state_vars") {
      VectorXd x = GetVariables()->GetComponent("tgt_state_vars")->GetValues();
      jac_block.coeffRef(0, 0) = -1.0;       // derivative of 1st constraint w.r.t p_x
      jac_block.coeffRef(0, 1) = 0.0;       // derivative of 1st constraint w.r.t p_y
      jac_block.coeffRef(0, 2) = -dt_*x(3)*sin(x(2));;   // derivative of 1st constraint w.r.t heading
      jac_block.coeffRef(0, 3) = dt_*cos(x(2));         // derivative of 1st constraint w.r.t velocity
      jac_block.coeffRef(1, 0) = 0.0;       // derivative of 2nd constraint w.r.t p_x
      jac_block.coeffRef(1, 1) = -1.0;       // derivative of 2nd constraint w.r.t p_y
      jac_block.coeffRef(1, 2) = dt_*x(3)*cos(x(2));   // derivative of 2nd constraint w.r.t heading
      jac_block.coeffRef(1, 3) = dt_*sin(x(2));         // derivative of 2nd constraint w.r.t velocity
      jac_block.coeffRef(2, 0) = 0.0;       // derivative of 3rd constraint w.r.t p_x
      jac_block.coeffRef(2, 1) = 0.0;       // derivative of 3rd constraint w.r.t p_y
      jac_block.coeffRef(2, 2) = -1.0;       // derivative of 3rd constraint w.r.t heading
      jac_block.coeffRef(2, 3) = 0.0;         // derivative of 3rd constraint w.r.t velocity
      jac_block.coeffRef(3, 0) = 0.0;       // derivative of 4th constraint w.r.t p_x
      jac_block.coeffRef(3, 1) = 0.0;       // derivative of 4th constraint w.r.t p_y
      jac_block.coeffRef(3, 2) = 0.0;       // derivative of 4th constraint w.r.t heading
      jac_block.coeffRef(3, 3) = -1.0;         // derivative of 4th constraint w.r.t velocity

    }


  }

};

class StateConstraint : public ifopt::ConstraintSet {
public:
  // Constructors
  StateConstraint() : StateConstraint("state_constraints") {}
  StateConstraint(const std::string& name) : ConstraintSet(4, name) {}
  //CurrStateConstraint(const std::string& name, const float dt) : ConstraintSet(4, name), dt_(dt) {}

  // Member variables
  float x_actual_;
  float y_actual_;
  float heading_actual_;
  float vel_actual_;

  void SetTargetState(float x, float y, float h, float v){
    x_actual_ = x;
    y_actual_ = y;
    heading_actual_ = h;
    vel_actual_ = v;
  }

  // The constraint value minus the constant value "1", moved to bounds.
  VectorXd GetValues() const override
  {
    VectorXd g(GetRows());
    VectorXd x = GetVariables()->GetComponent("curr_state_vars")->GetValues();
    g(0) = x(0);
    g(1) = x(1);
    g(2) = x(2);
    g(3) = x(3);
    return g;
  };

  // The only constraint in this set is an equality constraint to 1.
  // Constant values should always be put into GetBounds(), not GetValues().
  // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    b.at(0) = ifopt::Bounds(x_actual_, x_actual_);
    b.at(1) = ifopt::Bounds(y_actual_, y_actual_);
    b.at(2) = ifopt::Bounds(heading_actual_, heading_actual_);
    b.at(3) = ifopt::Bounds(heading_actual_, heading_actual_);
    return b;
  }

  // This function provides the first derivative of the constraints.
  // In case this is too difficult to write, you can also tell the solvers to
  // approximate the derivatives by finite differences and not overwrite this
  // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
  // Attention: see the parent class function for important information on sparsity pattern.
  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
  {

    if (var_set == "curr_state_vars") {
      jac_block.coeffRef(0, 0) = 1.0;       // derivative of 1st constraint w.r.t p_x
      jac_block.coeffRef(0, 1) = 0.0;       // derivative of 1st constraint w.r.t p_y
      jac_block.coeffRef(0, 2) = 0.0;   // derivative of 1st constraint w.r.t heading
      jac_block.coeffRef(0, 3) = 0.0;         // derivative of 1st constraint w.r.t velocity
      jac_block.coeffRef(1, 0) = 0.0;       // derivative of 2nd constraint w.r.t p_x
      jac_block.coeffRef(1, 1) = 1.0;       // derivative of 2nd constraint w.r.t p_y
      jac_block.coeffRef(1, 2) = 0.0;   // derivative of 2nd constraint w.r.t heading
      jac_block.coeffRef(1, 3) = 0.0;         // derivative of 2nd constraint w.r.t velocity
      jac_block.coeffRef(2, 0) = 0.0;       // derivative of 3rd constraint w.r.t p_x
      jac_block.coeffRef(2, 1) = 0.0;       // derivative of 3rd constraint w.r.t p_y
      jac_block.coeffRef(2, 2) = 1.0;       // derivative of 3rd constraint w.r.t heading
      jac_block.coeffRef(2, 3) = 0.0;         // derivative of 3rd constraint w.r.t velocity
      jac_block.coeffRef(3, 0) = 0.0;       // derivative of 4th constraint w.r.t p_x
      jac_block.coeffRef(3, 1) = 0.0;       // derivative of 4th constraint w.r.t p_y
      jac_block.coeffRef(3, 2) = 0.0;       // derivative of 4th constraint w.r.t heading
      jac_block.coeffRef(3, 3) = 1.0;         // derivative of 4th constraint w.r.t velocity
    }

  }

};

class StateBounds : public ifopt::ConstraintSet {
public:
  StateBounds() : StateBounds("state_bounds") {}

  // This constraint set just contains 1 constraint, however generally
  // each set can contain multiple related constraints.
  StateBounds(const std::string& name) : ConstraintSet(4, name) {}

  // The constraint value minus the constant value "1", moved to bounds.
  VectorXd GetValues() const override
  {
    VectorXd g(GetRows());
    Vector4d x = GetVariables()->GetComponent("tgt_state_vars")->GetValues();
    g(0) = x(0);
    g(1) = x(1);
    g(0) = x(2);
    g(1) = x(3);
    return g;
  };

  // The only constraint in this set is an equality constraint to 1.
  // Constant values should always be put into GetBounds(), not GetValues().
  // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    b.at(0) = ifopt::Bounds(-100.0, 100.0);
    b.at(1) = ifopt::Bounds(-100.0, 100.0);
    b.at(2) = ifopt::Bounds(-3.2, 3.2);
    b.at(3) = ifopt::Bounds(-10, 10);

    return b;
  }

  // This function provides the first derivative of the constraints.
  // In case this is too difficult to write, you can also tell the solvers to
  // approximate the derivatives by finite differences and not overwrite this
  // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
  // Attention: see the parent class function for important information on sparsity pattern.
  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
  {
    // must fill only that submatrix of the overall Jacobian that relates
    // to this constraint and "var_set1". even if more constraints or variables
    // classes are added, this submatrix will always start at row 0 and column 0,
    // thereby being independent from the overall problem.
    if (var_set == "tgt_state_vars") {
      jac_block.coeffRef(0, 0) = 1.0; // derivative of first constraint w.r.t x0
      jac_block.coeffRef(0, 1) = 1.0;      // derivative of first constraint w.r.t x1
      jac_block.coeffRef(0, 2) = 1.0;
      jac_block.coeffRef(0, 3) = 1.0; 

    }
  }
};




/*
DEFINE COSTS/OBJECTIVE
*/
class InputCostFunction: public ifopt::CostTerm {
public:
  // Constructors
  InputCostFunction() : InputCostFunction("input_cost") {}
  InputCostFunction(const std::string& name) : CostTerm(name) {}
  InputCostFunction(const std::string& name, const float& wt_omega, const float& wt_acc ) : 
    CostTerm(name),
    weight_yawrate_(wt_omega),
    weight_acc_(wt_acc) {}

  double GetCost() const override
  {
    Vector2d u = GetVariables()->GetComponent("control_vars")->GetValues();
    return weight_yawrate_*std::pow(u(0),2) + weight_acc_*std::pow(u(1),2);
  };

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "control_vars") {
      Vector2d u = GetVariables()->GetComponent("control_vars")->GetValues();

      jac.coeffRef(0, 0) = 2.0*weight_yawrate_*u(0);    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 1) = 2.0*weight_acc_*u(1);        // derivative of cost w.r.t acceleration
    }
    if (var_set == "curr_state_vars") {

      jac.coeffRef(0, 0) = 0.0;    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 1) = 0.0;        // derivative of cost w.r.t acceleration
      jac.coeffRef(0, 2) = 0.0;    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 3) = 0.0;        // derivative of cost w.r.t acceleration
    }
    if (var_set == "tgt_state_vars") {

      jac.coeffRef(0, 0) = 0.0;    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 1) = 0.0;        // derivative of cost w.r.t acceleration
      jac.coeffRef(0, 2) = 0.0;    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 3) = 0.0;        // derivative of cost w.r.t acceleration
    }


  }

private:
  float weight_yawrate_ = 0.5;
  float weight_acc_ = 0.1;
};



class GoalCostFunction: public ifopt::CostTerm {
public:
  // Constructors
  GoalCostFunction() : GoalCostFunction("goal_cost") {}
  GoalCostFunction(const std::string& name) : CostTerm(name) {}
  GoalCostFunction(const std::string& name, const float& wt_goal, float dt) : 
    CostTerm(name),
    weight_goal_(wt_goal),
    dt_(dt) {}

  void SetGoal(float x_g, float y_g) {
    x_goal_ = x_g;
    y_goal_ = y_g;
  };

  double GetCost() const override
  {
    //Vector4d x_curr = GetVariables()->GetComponent("curr_state_vars")->GetValues();
    Vector4d x_tgt = GetVariables()->GetComponent("tgt_state_vars")->GetValues();
    //return weight_goal_*std::pow((x(0)-x_goal_ + dt_*x(3)*cos(x(2))),2) + weight_goal_*std::pow((x(1)-y_goal_+ dt_*x(3)*sin(x(2))),2);
    return weight_goal_*std::pow((x_tgt(0)-x_goal_),2) + weight_goal_*std::pow((x_tgt(1)-y_goal_),2);
  };

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "control_vars") {
      jac.coeffRef(0, 0) = 0.0;    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 1) = 0.0;    // derivative of cost w.r.t acceleration
    }

    if (var_set == "curr_state_vars") {
      jac.coeffRef(0, 0) = 0.0;    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 1) = 0.0;     // derivative of cost w.r.t acceleration
      jac.coeffRef(0, 2) = 0.0;    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 3) = 0.0;        // derivative of cost w.r.t acceleration
    }

    if (var_set == "tgt_state_vars") {
      Vector4d x_curr = GetVariables()->GetComponent("curr_state_vars")->GetValues();
      Vector4d x_tgt = GetVariables()->GetComponent("tgt_state_vars")->GetValues();
      jac.coeffRef(0, 0) = 2*weight_goal_*(x_tgt(0)-x_goal_);    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 1) = 2*weight_goal_*(x_tgt(1)-y_goal_);        // derivative of cost w.r.t acceleration
      jac.coeffRef(0, 2) = 0.0;
      jac.coeffRef(0, 3) = 0.0;
      // jac.coeffRef(0, 2) = 2*weight_goal_*dt_*x_tgt(3)*((x_curr(0)-x_goal_)*sin(x_tgt(2)) + (x_curr(1)-y_goal_)*cos(x_tgt(2)));
      // jac.coeffRef(0, 3) = 2*weight_goal_*dt_*(dt_*x_tgt(3) + (x_curr(0)-x_goal_)*cos(x_tgt(2)) + (x_curr(1)-y_goal_)*sin(x_tgt(2)));
    }
  }

private:
  float weight_goal_ = 1.0;
  float x_goal_=0.0;
  float y_goal_=0.0;
  float dt_ = 0.1;
};

class ObstacleCostFunction: public ifopt::CostTerm {
public:
  // Constructors
  ObstacleCostFunction() : ObstacleCostFunction("obstacle_cost") {}
  ObstacleCostFunction(const std::string& name) : CostTerm(name) {}
  ObstacleCostFunction(const std::string& name, const float& wt_lateral, const float& wt_axial, const int& num_scans) : 
    CostTerm(name),
    weight_lateral_(wt_lateral),
    weight_axial_(wt_axial),
    nScans_ (num_scans),
    x_scans_(std::vector<float>(num_scans)),
    y_scans_(std::vector<float>(num_scans)) {}
  
  void SetScanData(std::vector<float> x_scans_map, std::vector<float> y_scans_map) {
    x_scans_ = x_scans_map;
    y_scans_ = y_scans_map;
  };

  double GetCost() const override
  {
    Vector4d x_tgt = GetVariables()->GetComponent("tgt_state_vars")->GetValues();
    float cost=0;
    for (int ii=0; ii < nScans_; ii++ ){
      cost += weight_axial_*-log( pow(cos(x_tgt(2))*(x_scans_[ii] - x_tgt(0)) + sin(x_tgt(2))*(y_scans_[ii] - x_tgt(1)),2) ); // x/vertical distances
      cost += weight_lateral_-log( pow(-sin(x_tgt(2))*(x_scans_[ii] - x_tgt(0)) + cos(x_tgt(2))*(y_scans_[ii] - x_tgt(1)),2) ); // y/lateral distances
    }
    return cost;
  };

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "control_vars") {
      jac.coeffRef(0, 0) = 0.0;    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 1) = 0.0;    // derivative of cost w.r.t acceleration
    }

    if (var_set == "curr_state_vars") {
      jac.coeffRef(0, 0) = 0.0;    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 1) = 0.0;     // derivative of cost w.r.t acceleration
      jac.coeffRef(0, 2) = 0.0;    // derivative of cost w.r.t yaw rate
      jac.coeffRef(0, 3) = 0.0;        // derivative of cost w.r.t acceleration
    }

    if (var_set == "tgt_state_vars") {
      Vector4d x_tgt = GetVariables()->GetComponent("tgt_state_vars")->GetValues();
      jac.coeffRef(0, 0) = 0.0;
      jac.coeffRef(0, 1) = 0.0;
      jac.coeffRef(0, 2) = 0.0;
      jac.coeffRef(0, 3) = 0.0;
    
      for (int ii=0; ii < nScans_; ii++ ){
        jac.coeffRef(0, 0) += 2*weight_axial_*cos(x_tgt(2)) / ( cos(x_tgt(2))*( x_scans_[ii] - x_tgt(0) ) + sin(x_tgt(2))*( y_scans_[ii] - x_tgt(1) ) );
        jac.coeffRef(0, 0) += 2*weight_lateral_*sin(x_tgt(2)) / ( sin(x_tgt(2))*( x_scans_[ii] - x_tgt(0) ) - cos(x_tgt(2))*( y_scans_[ii] - x_tgt(1) ) );
        jac.coeffRef(0, 1) += 2*weight_axial_*sin(x_tgt(2)) / ( cos(x_tgt(2))*( x_scans_[ii] - x_tgt(0) ) + sin(x_tgt(2))*( y_scans_[ii] - x_tgt(1) ) );
        jac.coeffRef(0, 1) += -2*weight_lateral_*cos(x_tgt(2)) / ( sin(x_tgt(2))*( x_scans_[ii] - x_tgt(0) ) - cos(x_tgt(2))*( y_scans_[ii] - x_tgt(1) ) );
        jac.coeffRef(0, 2) += 2*weight_axial_*( sin(x_tgt(2))*( x_scans_[ii] - x_tgt(0) ) - cos(x_tgt(2))*( y_scans_[ii] - x_tgt(1) ) ) / ( cos(x_tgt(2))*( x_scans_[ii] - x_tgt(0) ) + sin(x_tgt(2))*( y_scans_[ii] - x_tgt(1) ) );
        jac.coeffRef(0, 2) += -2*weight_lateral_*( cos(x_tgt(2))*( x_scans_[ii] - x_tgt(0) ) + sin(x_tgt(2))*( y_scans_[ii] - x_tgt(1) ) ) / ( sin(x_tgt(2))*( x_scans_[ii] - x_tgt(0) ) - cos(x_tgt(2))*( y_scans_[ii] - x_tgt(1) ) );
      }
    }
  }

private:
  float weight_lateral_ = 0.5;
  float weight_axial_ = 2.0;
  int nScans_ = 11;
  float dt_ = 0.1;
  std::vector<float> x_scans_ = std::vector<float>(nScans_);
  std::vector<float> y_scans_ = std::vector<float>(nScans_);
};