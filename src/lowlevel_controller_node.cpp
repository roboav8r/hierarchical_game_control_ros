#include "lowlevel_controller_node.h"
#include "solver_params.h"




int main(int argc, char **argv)
{
  /*
  CREATE ROS OBJECTS
  */
 
  // Create ROS node
  ros::init(argc, argv, "ll_controller_node");
  ros::NodeHandle nh;

  // Create control publisher
  ros::Publisher control_pub = nh.advertise<geometry_msgs::Twist>("/base_controller/command",10);

  // Get current robot position
  ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 1, robotOdomCallback);

  // Get navigation goal
  ros::Subscriber navgoal_sub = nh.subscribe<move_base_msgs::MoveBaseGoal>("/nav_goal",1,navGoalCallback);
  goal.target_pose.pose.position.x = 0; // set initial nav goal as the robot's initial value - TODO replace with callback values or wait for position measurement
  goal.target_pose.pose.position.y = 10.0;

  // Get LiDAR scan data
  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/base_scan", 1, scanCallback);


  /*
  DEFINE VARIABLES
  */
  // Initialize scan parameters
  for (int ii = 0; ii < numScans; ii++) 
  {
      scanAngles[ii] = minScanAngle + (maxScanAngle - minScanAngle)*((float)ii/(numScans-1));
      scanIndices[ii] = (int)(numMeasScans-1)*((float)ii/(numScans-1));
  };

  /*
  NONLINEAR PROGRAM AND SOLVER PARAMETERS
  */
  float dt = 0.1;
  // Variables
  std::shared_ptr<ControlVariables> control_var_ptr = std::make_shared<ControlVariables>();
  std::shared_ptr<StateVariables> curr_state_var_ptr = std::make_shared<StateVariables>("curr_state_vars", minvel, maxvel);
  std::shared_ptr<StateVariables> tgt_state_var_ptr = std::make_shared<StateVariables>("tgt_state_vars", minvel, maxvel);
  // Constraints
  std::shared_ptr<DynConstraint> dyn_const_ptr = std::make_shared<DynConstraint>("dyn_const", dt);
  std::shared_ptr<StateConstraint> state_const_ptr = std::make_shared<StateConstraint>("curr_state_const");
  // std::shared_ptr<StateBounds> state_bounds_ptr = std::make_shared<StateBounds>("state_bounds");
  // Costs
  std::shared_ptr<InputCostFunction> input_cost_ptr = std::make_shared<InputCostFunction>("input_cost",yawRateCost,accCost);
  std::shared_ptr<GoalCostFunction> goal_cost_ptr = std::make_shared<GoalCostFunction>("goal_cost",goalCost,dt);
  std::shared_ptr<ObstacleCostFunction> obst_cost_ptr = std::make_shared<ObstacleCostFunction>("obstacle_cost", latObstCost, axObstCost, numScans);

  ifopt::Problem llControlProgram;
  llControlProgram.AddVariableSet(control_var_ptr);
  llControlProgram.AddVariableSet(curr_state_var_ptr);
  llControlProgram.AddVariableSet(tgt_state_var_ptr);
  llControlProgram.AddConstraintSet(dyn_const_ptr);
  llControlProgram.AddConstraintSet(state_const_ptr);
  llControlProgram.AddCostSet(input_cost_ptr);
  llControlProgram.AddCostSet(goal_cost_ptr);
  llControlProgram.AddCostSet(obst_cost_ptr);
  llControlProgram.PrintCurrent();
  
  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("jacobian_approximation", "exact");
 

  /*
  MAIN TIMER/LOOP
  */

  // Setup main propagation/visualization/publisher loop (timer)
  // ros::Rate loopRate(10);
  ros::Timer mainTimer = nh.createTimer(ros::Duration(dt),[&](const ros::TimerEvent& event)
  {
    // Process current state
    float currentX = robotX;
    float currentY = robotY;
    float currentH = robotHeading;
    float currentV = robotV;

    curr_state_var_ptr->SetVariables(Vector4d(currentX, currentY,currentH,currentV));
    tgt_state_var_ptr->SetVariables(Vector4d(currentX, currentY,currentH,currentV));
    state_const_ptr->SetTargetState(currentX, currentY,currentH,currentV);

    // Update goal position
    goal_cost_ptr->SetGoal(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

    // Update scan data for obstacle avoidance constraint
    obst_cost_ptr->SetScanData(scanXMap, scanYMap);

    // Update constraints and solve nonlinear control program
    ipopt.Solve(llControlProgram);
    uOptimal = llControlProgram.GetOptVariables()->GetValues();
    std::cout << "Optimal yaw rate: " << uOptimal(0) << std::endl;
    std::cout << "Commanded yaw rate " << uOptimal(0) << std::endl;
    std::cout << "Optimal acc: " << uOptimal(1) << std::endl;
    std::cout << "Commanded vel " << currentV + uOptimal(1)*dt << std::endl;
    std::cout << "Solver current x: " << uOptimal(2) << std::endl;
    std::cout << "Solver current y: " << uOptimal(3) << std::endl;
    std::cout << "Solver current h: " << uOptimal(4) << std::endl;
    std::cout << "Solver current v: " << uOptimal(5) << std::endl;
    std::cout << "Solver tgt x: " << uOptimal(6) << std::endl;
    std::cout << "Solver tgt y: " << uOptimal(7) << std::endl;
    std::cout << "Solver tgt h: " << uOptimal(8) << std::endl;
    std::cout << "Solver tgt v: " << uOptimal(9) << std::endl;


    // Update and publish control message
    controlMsg.angular.z = uOptimal(0); // yaw rate
    controlMsg.linear.x = currentV + uOptimal(1)*dt; // acceleration
    control_pub.publish(controlMsg);

  });

  // ros::spin();
  ros::spin();

  return 0;
}