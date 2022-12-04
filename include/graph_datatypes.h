#ifndef GRAPH_DATATYPES_H_
#define GRAPH_DATATYPES_H_

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
    GameState(uint& x_px, uint& y_px, bool& rob_cargo, bool& map_cargo, bool& cargo_pickup, bool& cargo_dest, cv::Mat* map_img, int& pix_stepsize, float& free_thresh) : 
    xPix(x_px), yPix(y_px), 
    robotHasCargo(rob_cargo),
    mapHasCargo(map_cargo),
    isCargoPickup(cargo_pickup),
    isCargoDestination(cargo_dest),
    mapImg(map_img),
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
    GameState(GameState* last_state, Move& move): parent(last_state),
    xPix(last_state->xPix), yPix(last_state->yPix), 
    robotHasCargo(last_state->robotHasCargo),
    mapHasCargo(last_state->mapHasCargo),
    isCargoPickup(last_state->isCargoPickup),
    isCargoDestination(last_state->isCargoDestination),
    mapImg(last_state->mapImg),
    pixelStepsize(last_state->pixelStepsize),
    freeThreshold(last_state->freeThreshold),
    nMoves(last_state->nMoves) 
    {
        std::cout << "Creating new child node from parent at :" << parent->xPix <<", " << parent->yPix << std::endl;
        std::cout << "Got move:" << move << std::endl;

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
    uint xPix;
    uint yPix;
    uint nMoves;
    bool robotHasCargo;
    bool mapHasCargo;
    bool isCargoPickup;    
    bool isCargoDestination;

    // Map parameters
    cv::Mat* mapImg;
    uint pixelStepsize;
    float freeThreshold;

    // MCTS 
    GameState*              parent;
    std::vector<Move>       availMoves;
    std::vector<Move>       completedMoves;
    std::vector<GameState*> children;
    std::vector<float>      childValue;
    int nTimesVisited;
    int score;
    int8_t maxNumMoves{30}; // TODO make this a parameter

    // MCTS Member Functions
    void getAvailMoves() 
    {

        // check for pickup
        if (mapHasCargo && isCargoPickup)
        {
            availMoves.push_back(Move::PickUp);
        }
        
        // check for dropoff
        if (robotHasCargo && isCargoDestination )
        {
            availMoves.push_back(Move::DropOff);
        }

        // +x
        // expand in the +x direction in the occupancy grid, ensuring that pixels are below occupancy threshold 
        for (int x = 1; x <= pixelStepsize ; x++)
        {
            float occ = (float)(255 - mapImg->at<uint8_t>(xPix + x, mapImg->rows - yPix))/255;
            //std::cout << "Pixel at " <<  xPix + x <<", " << mapImg->rows - yPix << " has occupancy " << occ << std::endl;
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
            float occ = (float)(255 - mapImg->at<uint8_t>(xPix, mapImg->rows - (yPix + y)))/255;
            //std::cout << "Pixel at " <<  xPix + x <<", " << mapImg->rows - yPix << " has occupancy " << occ << std::endl;
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
            float occ = (float)(255 - mapImg->at<uint8_t>(xPix - x, mapImg->rows - yPix))/255;
            //std::cout << "Pixel at " <<  xPix + x <<", " << mapImg->rows - yPix << " has occupancy " << occ << std::endl;
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
            float occ = (float)(255 - mapImg->at<uint8_t>(xPix, mapImg->rows - (yPix - y)))/255;
            //std::cout << "Pixel at " <<  xPix + x <<", " << mapImg->rows - yPix << " has occupancy " << occ << std::endl;
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

    // Return if the current node is a leaf. A leaf is any node that has a potential child from which no simulation (playout) has yet been initiated.
    bool isLeaf() {
        return children.size() < availMoves.size();
    };

    // Using UCT equation, compute value of child nodes and assign to childValue vector
    void computeChildValue()
    {
        childValue.clear();
        childValue.reserve(children.size());

        for (auto& child : children)
        {
            childValue.push_back((float)(child->score/child->nTimesVisited + sqrt(2)*sqrt(log(nTimesVisited)/child->nTimesVisited)));
        }
    }

    // Select the best-valued child from known children. Randomly select from tied children.
    GameState* selectNext()
    {

        computeChildValue();

        auto maxIndices = findMaxima(childValue);
        
        if (maxIndices.empty()) {
            std::cout<< "Could not find a maximum!"<< std::endl;
            //TODO throw error

        } else if (maxIndices.size()==1){
            std::cout<< "Found a single maximum at child index " << maxIndices[0] << std::endl;
            return children[maxIndices[0]];

        } else {
            std::cout << "Found multiple maxima at child indices " << std::endl;
            for (int ii=0 ; ii< maxIndices.size(); ii++) {
                std::cout << maxIndices[ii] << std::endl;
            }
            std::cout << "Choosing random child" << std::endl;
            return children[(int)(rand()%maxIndices.size())];
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
            std::cout << "PlusX" << std::endl;
            this->nMoves+=1;
            this->xPix+=this->pixelStepsize;
        }
        if (move==2) {
            std::cout << "PlusY" << std::endl;
            this->nMoves+=1;
            this->yPix+=this->pixelStepsize;
        }
        if (move==3) {
            std::cout << "MinusX" << std::endl;
            nMoves+=1;
            this->xPix-=this->pixelStepsize;
        }
        if (move==4) {
            std::cout << "MinusY" << std::endl;
            nMoves+=1;
            this->yPix-=this->pixelStepsize;
        }
        if (move==5) {
            std::cout << "PickUp" << std::endl;
            this->robotHasCargo=true;
            this->mapHasCargo=false;
        }
        if (move==6) {
            std::cout << "DropOff" << std::endl;
            this->robotHasCargo=false;
            this->mapHasCargo=true;
        }
    }

    // Determine if current game state is terminal, and report result
    int gameResult()
    {
        // TODO define win condition

        // If the robot makes too many moves, return a 0 for a loss
        if (nMoves > maxNumMoves) {
            //std::cout << "LOSS: Exceeded max number of moves" << std::endl;
            return 0;
        };
        
        // If none of the above conditions are met, return -1 (not terminal)
        return -1;
    }

    // Propagate the state forward


    // Helper functions
};

class Tree {
public:
    Tree(){}

    // Initialize tree with game state at root
    Tree(GameState* gs) : rootNode(gs) {
        nodes.push_back(gs);
    }

    // Member variables
    GameState* rootNode;
    GameState* currentNode;
    std::vector<GameState*> nodes;
    float searchTime{0.9};


    void Search()
    {
        // Start at the root
        currentNode = rootNode;

        // From the current node, select the best option (or randomly from equal options) until a leaf is reached
        while(!currentNode->isLeaf()) {
            // Select 
            GameState* nextNode = currentNode->selectNext();
            currentNode = nextNode;
        }
        std::cout << "Found leaf at: " << currentNode->xPix << ", " << currentNode->yPix << std::endl;

        // If the current node (leaf) is not terminal, create and choose a child node
        if (currentNode->gameResult() ==-1) // TODO maybe remove this?
        {
            std::cout << "Current Node not terminal. Creating new child." << std::endl;

            // Create a child node by randomly selecting a move that hasn't already been done
            Move nextMove = currentNode->availMoves[(int)(rand()%currentNode->availMoves.size())];
            std::cout << "Want to select move " << nextMove << std::endl; //<< " out of " << currentNode->availMoves.size() << std::endl;
            std::cout << "Already did moves:" << std::endl;
            for (auto& move : currentNode->completedMoves){
                std::cout << move << std::endl;
            }

            while (std::count(currentNode->completedMoves.begin(), currentNode->completedMoves.end(), nextMove)) // keep trying if move was already done
            {
                std::cout << "Move was already done, trying again" << std::endl;
                nextMove = currentNode->availMoves[(int)(rand()%currentNode->availMoves.size())];
            }

            // Create new node, add pointer to nodes vector
            std::shared_ptr<GameState> childNode = std::make_shared<GameState>(currentNode, nextMove);
            currentNode->children.push_back(childNode.get());
            currentNode->completedMoves.push_back(nextMove);
            nodes.push_back(childNode.get());
            //currentNode = childNode.get();

            std::cout << "Current Node has children: " << currentNode->children.size() << std::endl;
            for (auto& child : currentNode->children){
                std::cout << child->xPix << ", "<< child->yPix << std::endl;
            }
            std::cout << "Current Node has done moves: " << currentNode->completedMoves.size() << std::endl;
            for (auto& move : currentNode->completedMoves){
                std::cout << move << std::endl;
            }
            // Simulate from child node
            GameState simNode = (*childNode);
            while (simNode.gameResult()==-1) {
                // Compute available moves at this node
                simNode.getAvailMoves();

                // Randomly select a move, and advance the simulated state
                simNode.propagate(simNode.availMoves[(int)(rand()%simNode.availMoves.size())]);

            }
            std::cout << "Finished child simulation with result: " << simNode.gameResult() << std::endl;

            // Backpropagate
            currentNode = childNode.get(); // Set current node to the child node
            currentNode->nTimesVisited=+1; // increment counter
            currentNode->score += simNode.gameResult(); // increment score
            std::cout << "BACKPROP: node at " << currentNode->xPix << ", " << currentNode->yPix << " has been visited " << currentNode->nTimesVisited << " times with score " << currentNode->score << std::endl;

            while (currentNode->parent != nullptr){ // While the current node isn't the root
                currentNode = currentNode->parent; // Move back toward the root
                currentNode->nTimesVisited=+1; // increment counter
                currentNode->score += simNode.gameResult(); // increment score
                std::cout << "BACKPROP: node at " << currentNode->xPix << ", " << currentNode->yPix << " has been visited " << currentNode->nTimesVisited << " times with score " << currentNode->score << std::endl;
            }

        }

    }

};

#endif //GRAPH_DATATYPES_H_