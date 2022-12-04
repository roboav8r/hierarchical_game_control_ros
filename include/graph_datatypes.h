#ifndef GRAPH_DATATYPES_H_
#define GRAPH_DATATYPES_H_

#include <random>

#include <stdio.h>
#include <stdlib.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Generic Helper functions
// Find maxima function
// From https://stackoverflow.com/questions/61350093/finding-multiple-max-elements-in-a-vector-c 
template<typename T>
auto findMaxima(const std::vector<T>& v) {
   std::vector<size_t> indexes;

   for (auto it_max = std::max_element(v.begin(), v.end()); it_max != v.end();
        it_max = std::find(it_max+1, v.end(), *it_max))
   {
      auto index = std::distance(v.begin(), it_max);
      indexes.push_back(index);
   }

   return indexes;
}

// Game-specific data type
enum Move{
    Neutral,
    PlusX,
    PlusY,
    MinusX,
    MinusY,
    PickUp,
    DropOff
};

// Game-specific class
class GameState {
public:
    // Default constructor
    GameState() {}

    // Root node/initial constructor
    GameState(uint& x_px, uint& y_px, bool& rob_cargo, uint& pickup_x_px, uint& pickup_y_px, bool& pickup_cargo, uint& dest_x_px, uint& dest_y_px, bool& dest_cargo, float& cargo_dist, cv::Mat* map_img, int& pix_stepsize, float& free_thresh) : 
    robotXPix(x_px), robotYPix(y_px), robotHasCargo(rob_cargo), // Robot State
    pickupXPix(pickup_x_px), pickupYPix(pickup_y_px), cargoAtPickup(pickup_cargo), // Pickup site state
    destXPix(dest_x_px), destYPix(dest_y_px), cargoAtDest(dest_cargo), // Destination/dropoff site state
    cargoDist(cargo_dist),
    mapImg(map_img), // Map/MCTS parameters
    pixelStepsize(pix_stepsize),
    freeThreshold(free_thresh),
    parent(nullptr)
    {
        nMoves=0;
        nTimesVisited=0;
        score=0;
        completedMoves.clear();
        availMoves.clear();
        getAvailMoves();
        srand(time(0));
    }

    // Child constructor: Create new node/GameState from a parent state + move
    GameState(std::shared_ptr<GameState> last_state, Move& move): parent(last_state),
    robotXPix(last_state->robotXPix), robotYPix(last_state->robotYPix), robotHasCargo(last_state->robotHasCargo),
    pickupXPix(last_state->pickupXPix), pickupYPix(last_state->pickupYPix), cargoAtPickup(last_state->cargoAtPickup), // Pickup site state
    destXPix(last_state->destXPix), destYPix(last_state->destYPix), cargoAtDest(last_state->cargoAtDest), // Destination/dropoff site state
    mapImg(last_state->mapImg),
    pixelStepsize(last_state->pixelStepsize),
    freeThreshold(last_state->freeThreshold),
    nMoves(last_state->nMoves) 
    {
        // std::cout << "Creating new child node from parent at :" << parent->robotXPix <<", " << parent->robotYPix << std::endl;
        //std::cout << "Got move:" << move << std::endl;


        // Propagate state from parent -> move -> child node 
        propagate(move);

        // Initialize MCTS variables
        nTimesVisited = 0;
        score=0;
        completedMoves.clear();
        availMoves.clear();
        getAvailMoves();
    }; 

    // Game State variables
    uint robotXPix; // Robot's current X-pixel in the map
    uint robotYPix; // Robot's current Y-pixel in the map
    bool robotHasCargo; // If the robot is currently carrying cargo
    uint pickupXPix; // Pickup location X-pixel in the map
    uint pickupYPix; // Pickup location Y-pixel in the map
    bool cargoAtPickup; // If the cargo is currently at the pickup location
    uint destXPix; // Destination X-pixel in the map
    uint destYPix; // Destination Y-pixel in the map
    bool cargoAtDest; // If the cargo is currently at the destination

    // Map parameters
    float cargoDist; // Distance within which the robot can reach the cargo to pick up or drop off
    cv::Mat* mapImg;
    uint pixelStepsize;
    float freeThreshold;

    // MCTS 
    std::shared_ptr<GameState>  parent;
    std::vector<Move>       availMoves;
    std::vector<Move>       completedMoves;
    std::vector<std::shared_ptr<GameState>> children;
    std::vector<float>      childUctValue;
    std::vector<float>      moveValue;
    int nTimesVisited;
    int score;
    uint nMoves;
    int8_t maxNumMoves{50}; // TODO make this a parameter

    // Random number generation
    std::default_random_engine generator;

    // Class-specific helper functions
    bool robotCanPickupCargo()
    {
        return cargoAtPickup && !robotHasCargo && sqrt(pow(robotXPix - pickupXPix,2) + pow(robotYPix - pickupYPix,2)) <= cargoDist;
    }

    bool robotCanDeliverCargo()
    {
        return !cargoAtDest && robotHasCargo && sqrt(pow(robotXPix - destXPix,2) + pow(robotYPix - destYPix,2)) <= cargoDist;
    }

    // Find out which moves are currently available
    void getAvailMoves() 
    {

        // check for pickup
        if (robotCanPickupCargo())
        {
            availMoves.push_back(Move::PickUp);
        }
        
        // check for dropoff
        if (robotCanDeliverCargo())
        {
            availMoves.push_back(Move::DropOff);
        }

        // +x
        // expand in the +x direction in the occupancy grid, ensuring that pixels are below occupancy threshold 
        for (int x = 1; x <= pixelStepsize ; x++)
        {
            float occ = (float)(255 - mapImg->at<uint8_t>(robotXPix + x, mapImg->rows - robotYPix))/255;
            //std::cout << "Pixel at " <<  robotXPix + x <<", " << mapImg->rows - robotYPix << " has occupancy " << occ << std::endl;
            if (occ > freeThreshold ) { break;} // if pixel is not free, exit this loop.
            //std::cout<<x<<std::endl;
            if (x==pixelStepsize) 
            {
                // If all pixels are below occupancy threshold, moving +x is a valid move
                availMoves.push_back(Move::PlusX);
                //std::cout << "Plus X is a valid move" << std::endl;
            }

        }
        // +y
        // expand in the +y direction in the occupancy grid, ensuring that pixels are below occupancy threshold
        // NOTE: +y in the map is -y in the pixel images
        for (int y = 1; y <= pixelStepsize ; y++)
        {
            float occ = (float)(255 - mapImg->at<uint8_t>(robotXPix, mapImg->rows - (robotYPix + y)))/255;
            //std::cout << "Pixel at " <<  robotXPix + x <<", " << mapImg->rows - robotYPix << " has occupancy " << occ << std::endl;
            if (occ > freeThreshold ) { break;} // if pixel is not free, exit this loop.
            //std::cout<<x<<std::endl;
            if (y==pixelStepsize) 
            {
                // If all pixels are below occupancy threshold, moving +x is a valid move
                availMoves.push_back(Move::PlusY);
                //std::cout << "Plus X is a valid move" << std::endl;
            }

        }

        // -x
        for (int x = 1; x <= pixelStepsize ; x++)
        {
            float occ = (float)(255 - mapImg->at<uint8_t>(robotXPix - x, mapImg->rows - robotYPix))/255;
            //std::cout << "Pixel at " <<  robotXPix + x <<", " << mapImg->rows - robotYPix << " has occupancy " << occ << std::endl;
            if (occ > freeThreshold ) { break;} // if pixel is not free, exit this loop.
            //std::cout<<x<<std::endl;
            if (x==pixelStepsize) 
            {
                // If all pixels are below occupancy threshold, moving +x is a valid move
                availMoves.push_back(Move::MinusX);
                //std::cout << "Plus X is a valid move" << std::endl;
            }

        }

        // -y 
        // NOTE: +y in the map is -y in the pixel images
        for (int y = 1; y <= pixelStepsize ; y++)
        {
            float occ = (float)(255 - mapImg->at<uint8_t>(robotXPix, mapImg->rows - (robotYPix - y)))/255;
            //std::cout << "Pixel at " <<  robotXPix + x <<", " << mapImg->rows - robotYPix << " has occupancy " << occ << std::endl;
            if (occ > freeThreshold ) { break;} // if pixel is not free, exit this loop.
            //std::cout<<x<<std::endl;
            if (y==pixelStepsize) 
            {
                // If all pixels are below occupancy threshold, moving +x is a valid move
                availMoves.push_back(Move::MinusY);
                //std::cout << "Plus X is a valid move" << std::endl;
            }

        }
    }

    // MCTS Member Functions
    // Return if the current node is a leaf. A leaf is any node that has a potential child from which no simulation (playout) has yet been initiated.
    bool isLeaf() {
        return children.size() < availMoves.size();
    };

    // During node selection, use UCT equation to compute value of child nodes and assign to childSelectionValue vector
    void computeChildUctValue()
    {
        this->childUctValue.clear();
        this->childUctValue.reserve(this->children.size());

        for (auto& child : this->children)
        {
            
            this->childUctValue.push_back((float)(child->score/child->nTimesVisited + sqrt(2)*sqrt(log(this->nTimesVisited)/child->nTimesVisited)));
        }
    }

    // During move selection, compute the expected value of the move (score over times visited)/AKA exploitation and assign to moveValue vector
    void computeMoveValue()
    {
        this->moveValue.clear();
        this->moveValue.reserve(this->children.size());

        for (auto& child : this->children)
        {
            this->moveValue.push_back((float)child->score/child->nTimesVisited);
        }
    }

    // During search, select the best-valued child from known children using UCT. Randomly select from tied children.
    std::shared_ptr<GameState> selectNextNode()
    {

        computeChildUctValue();

        auto maxIndices = findMaxima(childUctValue);
        
        if (maxIndices.empty()) {
            std::cout<< "Could not find a maximum!"<< std::endl;
            //TODO throw error

        } else if (maxIndices.size()==1){
            //std::cout<< "Found a single maximum at child index " << maxIndices[0] << std::endl;
            return children[maxIndices[0]];

        } else {
            //std::cout << "Found multiple maxima at child indices " << std::endl;
            // for (int ii=0 ; ii< maxIndices.size(); ii++) {
            //     std::cout << maxIndices[ii] << std::endl;
            // }
            std::uniform_int_distribution<int> distribution(0,maxIndices.size()-1);
            int randInt = distribution(generator); 

            // int randInt = rand()%(maxIndices.size());

            //std::cout << "Multiple maxima when selecting next node, choosing random child " << randInt+1 << "/" << maxIndices.size() <<std::endl;
            return children[randInt];
        }
    };

    Move selectBestMove()
    {

        computeMoveValue();

        auto maxIndices = findMaxima(moveValue);
        
        if (maxIndices.empty()) {

        } else if (maxIndices.size()==1){
            std::cout<< "Found a single maximum at child index " << maxIndices[0] << std::endl;
            return completedMoves[maxIndices[0]];

        } else {
            std::cout << "Found multiple maxima at child indices " << std::endl;
            for (int ii=0 ; ii< maxIndices.size(); ii++) {
                std::cout << maxIndices[ii] << std::endl;
            }
            std::cout << "Found multiple maxima, choosing random best move" << std::endl;
            std::uniform_int_distribution<int> distribution(0,maxIndices.size()-1);
            int randInt = distribution(generator); 
            return completedMoves[randInt];
        }
    };

    // Propagate state forward (when creating a child or when simulating)
    void propagate(Move move)
    {
         // Compute transition logic
        if (move==0) {
            std::cout << "Neutral" << std::endl;
        }
        if (move==1) {
            //std::cout << "PlusX" << std::endl;
            this->nMoves+=1;
            this->robotXPix+=this->pixelStepsize;
        }
        if (move==2) {
            //std::cout << "PlusY" << std::endl;
            this->nMoves+=1;
            this->robotYPix+=this->pixelStepsize;
        }
        if (move==3) {
            //std::cout << "MinusX" << std::endl;
            nMoves+=1;
            this->robotXPix-=this->pixelStepsize;
        }
        if (move==4) {
            //std::cout << "MinusY" << std::endl;
            nMoves+=1;
            this->robotYPix-=this->pixelStepsize;
        }
        if (move==5) {
            //std::cout << "PickUp" << std::endl;
            this->robotHasCargo=true;
            this->cargoAtPickup=false;
        }
        if (move==6) {
            //std::cout << "DropOff" << std::endl;
            this->robotHasCargo=false;
            this->cargoAtDest=true;
        }
    }

    // Determine if current game state is terminal, and report result
    int gameResult()
    {
        // If the cargo is delivered, return a 1 for a win
        if (cargoAtDest) {
            return 1;
        };
        // If the robot makes too many moves, return a 0 for a loss
        if (nMoves > maxNumMoves) {
            return 0;
        };
        
        // If none of the above conditions are met, return -1 (not terminal)
        return -1;
    }

};

class Tree {
public:
    Tree(){}

    // Initialize tree with game state at root
    Tree(std::shared_ptr<GameState> gs) : rootNode(gs) {
        //nodes.push_back(gs);
    }

    // Member variables
    std::shared_ptr<GameState> rootNode;
    std::shared_ptr<GameState> currentNode;
    //std::vector<std::shared_ptr<GameState>> nodes;
    float searchTime{2.5}; // TODO make this a parameter

    Move bestMove;


    void Search()
    {
        // Start at the root
        currentNode = rootNode;
        time_t searchStartTime = time(0);

        // Main, timed search loop
        do {
            // SELECT: From the current node, select the best option (or randomly from equal options) until a leaf is reached
            while(!currentNode->isLeaf()) {
                std::shared_ptr<GameState> nextNode = currentNode->selectNextNode();
                currentNode = nextNode;
            }

            // EXPAND: Expand the node by creating a child from a random, available move
            // Randomly select a move that hasn't already been done
            std::uniform_int_distribution<int> distribution(0,currentNode->availMoves.size()-1);
            int randInt = distribution(currentNode->generator); 
            Move nextMove = currentNode->availMoves[randInt];
            // Keep trying if move was already done
            while (std::count(currentNode->completedMoves.begin(), currentNode->completedMoves.end(), nextMove)) 
            {
                randInt = distribution(currentNode->generator); 
                nextMove = currentNode->availMoves[randInt];
            }

            // // Create new node, add pointer to nodes vector, based on next move
            // std::shared_ptr<GameState> childNode;
            // childNode.reset();
            // childNode = std::make_shared<GameState>(currentNode, nextMove);
            std::shared_ptr<GameState> childNode = std::make_shared<GameState>(currentNode, nextMove);
            currentNode->children.push_back(childNode);
            currentNode->completedMoves.push_back(nextMove);
            //nodes.push_back(childNode);


            // std::cout << "Current Node has " << currentNode->children.size() << " children" << std::endl;
            // for (auto& child : currentNode->children){
            //     std::cout << child->robotXPix << ", "<< child->robotYPix << std::endl;
            //     std::cout << child << std::endl;
            // }
            // std::cout << "Current Node has done moves: " << std::endl;
            // for (auto& move : currentNode->completedMoves){
            //     std::cout << move << std::endl;
            // }

            // Simulate from child node until a terminal state is reached
            GameState simNode = *(childNode.get()); // Dereference the child node to simulate without altering the actual child node's state
            while (simNode.gameResult()==-1) {
                // Compute available moves at this node
                simNode.getAvailMoves();

                // Randomly select a move, and advance the simulated state
                std::uniform_int_distribution<int> distribution(0,simNode.availMoves.size()-1);
                int randSimInt = distribution(simNode.generator); 
                simNode.propagate(simNode.availMoves[randSimInt]);
            }


            // Backpropagate
            currentNode = childNode; // Move to the child node
            currentNode->nTimesVisited+=1; // increment counter
            currentNode->score += simNode.gameResult(); // increment score
            //std::cout << "BACKPROP: node at " << currentNode->robotXPix << ", " << currentNode->robotYPix << " has been visited " << currentNode->nTimesVisited << " times with score " << currentNode->score << std::endl;

            while (currentNode->parent != nullptr){ // While the current node isn't the root
                // std::cout << "Current node: " << currentNode << std::endl;
                // std::cout << "Parent node: " << currentNode->parent << std::endl;
                // std::cout << "Root node: " << rootNode << std::endl;
                // std::cout << "Root Parent node: " << rootNode->parent << std::endl;
                currentNode = currentNode->parent; // Move back toward the root
                currentNode->nTimesVisited+=1; // increment counter
                currentNode->score += simNode.gameResult(); // increment score
                //std::cout << "BACKPROP: node at " << currentNode->robotXPix << ", " << currentNode->robotYPix << " has been visited " << currentNode->nTimesVisited << " times with score " << currentNode->score << std::endl;
            }
            //std::cout << "Made it back to root" << std::endl;

            //}

        }// End main search loop
        while (time(0) - searchStartTime < searchTime);

        std::cout << "SEARCH COMPLETE" << std::endl;
        std::cout << "Time: " << time(0) - searchStartTime <<  std::endl;
        std::cout << "Root node at " << rootNode->robotXPix << ", " << rootNode->robotYPix << " has been visited " << rootNode->nTimesVisited << " times with score " << rootNode->score << std::endl;        
        for (auto child : rootNode->children)
        {
            std::cout <<"Child " << child.get() << " has been visited " << child->nTimesVisited << " times, with score " << child->score << std::endl;
        }
        bestMove = rootNode->selectBestMove();
        std::cout << "Best Move: " << bestMove << std::endl;

    } // Search method

}; // Tree class

#endif //GRAPH_DATATYPES_H_