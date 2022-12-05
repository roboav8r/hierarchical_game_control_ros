#include "highlevel_planner_node.h"
#include "graph_datatypes.h"

// Get ROS map parameters

// Create CV matrix

// Perform BFS to find neighbors

int main(int argc, char** argv)
{
  ros::init(argc, argv, "highlevel_planner_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it_(nh);

  /*
  Subscriptions/inputs
  */
  ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 1, robotOdomCallback); // Current robot position

  /*
  Publishers
  */
  image_transport::Publisher image_pub_ = it_.advertise("planner_image", 1);
  ros::Publisher goal_pub = nh.advertise<move_base_msgs::MoveBaseGoal>("/nav_goal",10);

  /*
  Parameters
  */
  nh.getParam("map_path",mapPath);
  nh.getParam("resolution",mapResolution);
  nh.getParam("free_thresh", freeThresh);
  nh.getParam("negate", mapNegate);
  pixStepsize = movementStepSize/mapResolution;
  std::cout << "Pixel Step size: " << pixStepsize << std::endl;

  // OpenCV: Get map, save as image. Create window.
  mapImg = cv::imread(mapPath, cv::IMREAD_UNCHANGED);
  cv::namedWindow("Planner");

  /*
  MAIN TIMER/LOOP
  */

  // Setup main propagation/visualization/publisher loop (timer)
  ros::Timer mainTimer = nh.createTimer(ros::Duration(dt),[&](const ros::TimerEvent& event)
  {

    // Draw visuals
    //plannerImg.setTo(cv::Scalar(0,0,0));
    plannerImg = mapImg.clone(); // 
    updateRobotPixels(); // Compute robot's pixel location on the map
    updateCargoPixels(); // Compute cargo pixel locations from 
    cv::circle(plannerImg, cv::Point(robotXpixel, plannerImg.rows - robotYpixel), 10, cv::Scalar(0,255,255), cv::FILLED);// Add a circle where the robot is
    cv::circle(plannerImg, cv::Point(pickupXpixel, plannerImg.rows - pickupYpixel), (int)(cargoDist/mapResolution), CV_RGB(0,0,0));// Add a circle at cargo pickup
    cv::circle(plannerImg, cv::Point(dropoffXpixel, plannerImg.rows - dropoffYpixel), (int)(cargoDist/mapResolution), CV_RGB(0,0,0));// Add a circle at cargo dropoff
    cv::putText(plannerImg, "Robot", cv::Point(robotXpixel, plannerImg.rows - robotYpixel - (int)(cargoDist/mapResolution)), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 1);
    cv::putText(plannerImg, "Pickup", cv::Point(pickupXpixel, plannerImg.rows - pickupYpixel - (int)(cargoDist/mapResolution)), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 1);
    cv::putText(plannerImg, "Dropoff", cv::Point(dropoffXpixel, plannerImg.rows - dropoffYpixel - (int)(cargoDist/mapResolution)), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 1);
    if (robotHasCargo){
        cv::putText(plannerImg, "[cargo]", cv::Point(robotXpixel, plannerImg.rows - robotYpixel + (int)(cargoDist/mapResolution)), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 1);
    }
    if (cargoAtPickup){
        cv::putText(plannerImg, "[cargo]", cv::Point(pickupXpixel, plannerImg.rows - pickupYpixel + (int)(cargoDist/mapResolution)), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 1);
    }
    if (cargoAtDest){
        cv::putText(plannerImg, "[cargo]", cv::Point(dropoffXpixel, plannerImg.rows - dropoffYpixel + (int)(cargoDist/mapResolution)), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(118, 185, 0), 1);
    }


    // Initialize node & tree at current state
    currentState.reset();
    currentState = std::make_shared<GameState>(robotXpixel, robotYpixel, robotHasCargo, pickupXpixel, pickupYpixel, cargoAtPickup, dropoffXpixel, dropoffYpixel, cargoAtDest, cargoDist, &mapImg, pixStepsize, freeThresh);
    Tree gameTree = Tree(currentState); // Create tree with current state as root

    // Run the search
    gameTree.Search();

    // plotAvailMoves
    if( std::find(gameTree.rootNode->availMoves.begin(), gameTree.rootNode->availMoves.end(), Move::PlusX) != gameTree.rootNode->availMoves.end() ) {
        cv::line(plannerImg, cv::Point(robotXpixel, plannerImg.rows - robotYpixel), cv::Point(robotXpixel+pixStepsize, plannerImg.rows - robotYpixel), cv::Scalar(0,0,0), 1, cv::LINE_8);
    }
    if ( std::find(gameTree.rootNode->availMoves.begin(), gameTree.rootNode->availMoves.end(), Move::PlusY) != gameTree.rootNode->availMoves.end() ) {
        cv::line(plannerImg, cv::Point(robotXpixel, plannerImg.rows - robotYpixel), cv::Point(robotXpixel, plannerImg.rows - (robotYpixel +pixStepsize)), cv::Scalar(0,0,0), 1, cv::LINE_8);
    }
    if( std::find(gameTree.rootNode->availMoves.begin(), gameTree.rootNode->availMoves.end(), Move::MinusX) != gameTree.rootNode->availMoves.end() ) {
        cv::line(plannerImg, cv::Point(robotXpixel, plannerImg.rows - robotYpixel), cv::Point(robotXpixel-pixStepsize, plannerImg.rows - robotYpixel), cv::Scalar(0,0,0), 1, cv::LINE_8);
    }
    if ( std::find(gameTree.rootNode->availMoves.begin(), gameTree.rootNode->availMoves.end(), Move::MinusY) != gameTree.rootNode->availMoves.end() ) {
        cv::line(plannerImg, cv::Point(robotXpixel, plannerImg.rows - robotYpixel), cv::Point(robotXpixel, plannerImg.rows - (robotYpixel -pixStepsize)), cv::Scalar(0,0,0), 1, cv::LINE_8);
    }

    // Handle the selected action
    if( gameTree.bestMove == Move::PlusX) {
        cv::line(plannerImg, cv::Point(robotXpixel, plannerImg.rows - robotYpixel), cv::Point(robotXpixel+pixStepsize, plannerImg.rows - robotYpixel), cv::Scalar(0,0,0), 3, cv::LINE_8);
        sendGoalMessage(goal_pub, movementStepSize,0);
    }
    if ( gameTree.bestMove == Move::PlusY) {
        cv::line(plannerImg, cv::Point(robotXpixel, plannerImg.rows - robotYpixel), cv::Point(robotXpixel, plannerImg.rows - (robotYpixel +pixStepsize)), cv::Scalar(0,0,0), 3, cv::LINE_8);
        sendGoalMessage(goal_pub, 0,movementStepSize);
    }
    if( gameTree.bestMove == Move::MinusX) {
        cv::line(plannerImg, cv::Point(robotXpixel, plannerImg.rows - robotYpixel), cv::Point(robotXpixel-pixStepsize, plannerImg.rows - robotYpixel), cv::Scalar(0,0,0), 3, cv::LINE_8);
        sendGoalMessage(goal_pub, -movementStepSize,0);
    }
    if ( gameTree.bestMove == Move::MinusY) {
        cv::line(plannerImg, cv::Point(robotXpixel, plannerImg.rows - robotYpixel), cv::Point(robotXpixel, plannerImg.rows - (robotYpixel -pixStepsize)), cv::Scalar(0,0,0), 1, cv::LINE_8);
        sendGoalMessage(goal_pub, 0,-movementStepSize);
    }


    // Resize and display
    float scaleFactor{0.5};
    int heightScaled{(int)(plannerImg.rows*scaleFactor)};
    int widthScaled{(int)(plannerImg.cols*scaleFactor)};
    cv::resize(plannerImg, scaledImg, cv::Size(widthScaled, heightScaled), cv::INTER_LINEAR);
    cv::imshow("Planner", scaledImg);
    cv::waitKey(1);

    // std::cout << "End Main Timer: " << event.current_real << std::endl;
  });



  ros::spin();
  return 0;
}