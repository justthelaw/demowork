// Author: Alex Gomez

#include "route_planner.h"
#include "motor_driver.h"
//#include "Particle.h"

// Compare the cost + heuristic values
bool RoutePlanner::Compare(const std::vector<int> a, const std::vector<int> b){
    // f = g + h 
    int f1 = a[2] + a[3];
    int f2 = b[2] + b[3]; 
    return f1 > f2;
}

// Sort the 2-D vector of ints in descending order
void RoutePlanner::CellSort(std::vector<std::vector<int>> *v){
    sort(v->begin(), v->end(), Compare); // if true, don't swap
}

// Calculate the manhattan distance
int RoutePlanner::Heuristic(int x1, int y1, int x2, int y2){
    return 0.5*(abs(x2-x1) + abs(y2-y1));
}

// Check if the cell is valid. That is, no obstacle. 
bool RoutePlanner::CheckValidCell(int x, int y, std::vector<std::vector<State>> &grid){
    bool on_grid_x = (x >= 0 && x < grid.size());
    bool on_grid_y = (y >= 0 && y < grid[0].size());
    if (on_grid_x && on_grid_y)
        return grid[x][y] == State::kEmpty;
    return false;
}

// Add a node to the open list and mark it as open
void RoutePlanner::AddToOpen(int x, int y, int g, int h,
std::vector<std::vector<int>> &openlist, std::vector<std::vector<State>> &grid){
    // Add node to open vector, and mark grid cell as closed.
    openlist.push_back(std::vector<int>{x, y, g, h});
    grid[x][y] = State::kClosed;
}

// Expand current node's neighbors and add them to the open list
void RoutePlanner::ExpandNeighbors(const std::vector<int> &current, std::vector<int> goal, 
std::vector<std::vector<int>> &openlist, std::vector<std::vector<State>> &grid, Node* prev){
    int x = current[0];
    int y = current[1];
    int g = current[2];
    int g2, h2;
    
    // Loop through current node's potential neighbors.
    for (int i = 0; i < 4; i++) {
        int x2 = x + deltaGrid[i][0];
        int y2 = y + deltaGrid[i][1];

        // Check that the potential neighbor's x2 and y2 values are on the grid and not closed.
        if (CheckValidCell(x2, y2, grid)) {
            grid[x2][y2] = State::kClosed;
            g2 = g + 1; // increase the cost
            Node* child = new Node; // new node initialized
            child -> parent = prev;
            child -> g_val = g2;
            if (x2 == x-1 and y2 == y) { // up
                h2 = Heuristic(x-2, y2, goal[0], goal[1]); 
                AddToOpen(x-2, y, g2, h2, openlist, grid);
                child -> x_coor = x-2;
                child -> y_coor = y;
                child -> h_val = h2;

            }
            else if (x2 == x and y2 == y-1) { // left
                h2 = Heuristic(x, y-2, goal[0], goal[1]);
                AddToOpen(x, y-2, g2, h2, openlist, grid);
                child -> x_coor = x;
                child -> y_coor = y-2;
                child -> h_val = h2;
            }
            else if (x2 == x+1 and y2 == y){ // down
                h2 = Heuristic(x+2, y, goal[0], goal[1]);
                AddToOpen(x+2, y, g2, h2, openlist, grid);
                child -> x_coor = x+2;
                child -> y_coor = y;
                child -> h_val = h2;
            }
            else if (x2 == x and y2 == y+1){ // right
                h2 = Heuristic(x, y+2, goal[0], goal[1]);
                AddToOpen(x, y+2, g2, h2, openlist, grid);
                child -> x_coor = x;
                child -> y_coor = y+2;
                child -> h_val = h2;
            }
            //std::cout << "Child coordinates: " << child -> x_coor << " " << child -> y_coor << std::endl;
            pList.push_back(child);
        }
        if (h2 == 0) // the goal is found. No further exploration necessary
            break;
    }
}
void RoutePlanner::CreateMap(std::vector<std::vector<State>> &grid){
    if (fwdSensor() && CheckValid(grid, Dir::Fwd)){
      Serial.println("Front Empty");
      AddCellWall(grid, Dir::Fwd, State::kEmpty);
    }
    else if (!fwdSensor() && CheckValid(grid, Dir::Fwd)){
      Serial.println("Front Wall");
      AddCellWall(grid, Dir::Fwd, State::kObstacle);
    }
    
    if (rightSensor() && CheckValid(grid, Dir::Right)){
      Serial.println("Right empty");
      AddCellWall(grid, Dir::Right, State::kEmpty);
    }
    else if (!rightSensor() && CheckValid(grid, Dir::Right)){
      Serial.println("Right wall");
      AddCellWall(grid, Dir::Right, State::kObstacle);
    }
    if (leftSensor() && CheckValid(grid, Dir::Left)){
      Serial.println("Left empty");
      AddCellWall(grid, Dir::Left, State::kEmpty);
    }
    else if (!leftSensor() && CheckValid(grid, Dir::Left)){
      Serial.println("Left wall");
      AddCellWall(grid, Dir::Left, State::kObstacle);
    }
}

bool RoutePlanner::CheckValid(std::vector<std::vector<State>> &grid, Dir dir){
    int x, y;
    if (dir == Dir::Fwd){
        switch (curr_head){
            case Head::North:
                x = curr_x - 2;
                y = curr_y;
                break;
            case Head::South:
                x = curr_x + 2;
                y = curr_y;
                break;
            case Head::East:
                x = curr_x;
                y = curr_y + 2;
                break;
            case Head::West:
                x = curr_x;
                y = curr_y - 2;
                break;
            default:
                Serial.println("No fwd Match!");
        }   
    }
    else if (dir == Dir::Right){
        switch (curr_head){
            case Head::North:
                x = curr_x;
                y = curr_y + 2;
                break;
            case Head::South:
                x = curr_x;
                y = curr_y - 2;
                break;
            case Head::East:
                x = curr_x + 2;
                y = curr_y;
                break;
            case Head::West:
                x = curr_x - 2;
                y = curr_y;
                break;
            default:
                Serial.println("No right Match!");
        }
    }
    else if (dir == Dir::Left){
        switch (curr_head){
            case Head::North:
                x = curr_x;
                y = curr_y - 2;
                break;
            case Head::South:
                x = curr_x;
                y = curr_y + 2;
                break;
            case Head::East:
                x = curr_x - 2;
                y = curr_y;
                break;
            case Head::West:
                x = curr_x + 2;
                y = curr_y;
                break;
            default:
                Serial.println("No left Match!");
        }
    }

    bool on_grid_x = (x >= 0 && x < grid.size());
    bool on_grid_y = (y >= 0 && y < grid[0].size());
    if (on_grid_x && on_grid_y)
        return grid[x][y] == State::kUndefined;
    return false;
}

void RoutePlanner::AddCellWall(std::vector<std::vector<State>> &grid, Dir dir, State state){
    if (dir == Dir::Fwd){
        switch (curr_head){
            case Head::North:
                if (state == State::kEmpty){
                    grid[curr_x-1][curr_y] = State::kEmpty;
                    grid[curr_x-2][curr_y] = State::kEmpty; 
                }
                else 
                    grid[curr_x-1][curr_y] = State::kObstacle;
                break;
            case Head::South:
                if (state == State::kEmpty){
                    grid[curr_x+1][curr_y] = State::kEmpty;
                    grid[curr_x+2][curr_y] = State::kEmpty;
                }
                else
                    grid[curr_x+1][curr_y] = State::kObstacle;
                break;
            case Head::East:
                if (state == State::kEmpty){
                    grid[curr_x][curr_y+1] = State::kEmpty;
                    grid[curr_x][curr_y+2] = State::kEmpty;
                }
                else
                    grid[curr_x][curr_y+1] = State::kObstacle;
                break;
            case Head::West:
                if (state == State::kEmpty){
                    grid[curr_x][curr_y-1] = State::kEmpty;
                    grid[curr_x][curr_y-2] = State::kEmpty;
                }
                else
                    grid[curr_x][curr_y-1] = State::kObstacle;
                break;
            default:
                Serial.println("No fwd Match!");
        }   
    }
    else if (dir == Dir::Right){
        switch (curr_head){
            case Head::North:
                if (state == State::kEmpty){
                    grid[curr_x][curr_y+1] = State::kEmpty;
                    grid[curr_x][curr_y+2] = State::kEmpty; // x-1 is unpopulated **
                }
                else 
                    grid[curr_x][curr_y+1] = State::kObstacle;
                break;
            case Head::South:
                if (state == State::kEmpty){
                    grid[curr_x][curr_y-1] = State::kEmpty;
                    grid[curr_x][curr_y-2] = State::kEmpty;
                }
                else
                    grid[curr_x][curr_y-1] = State::kObstacle;
                break;
            case Head::East:
                if (state == State::kEmpty){
                    grid[curr_x+1][curr_y] = State::kEmpty;
                    grid[curr_x+2][curr_y] = State::kEmpty;
                }
                else
                    grid[curr_x+1][curr_y] = State::kObstacle;
                break;
            case Head::West:
                if (state == State::kEmpty){
                    grid[curr_x-1][curr_y] = State::kEmpty;
                    grid[curr_x-2][curr_y] = State::kEmpty;
                }
                else
                    grid[curr_x-1][curr_y] = State::kObstacle;
                break;
            default:
                Serial.println("No right Match!");
        }   
    }
    else if (dir == Dir::Left){
        switch (curr_head){
            case Head::North:
                if (state == State::kEmpty){
                    grid[curr_x][curr_y-1] = State::kEmpty;
                    grid[curr_x][curr_y-2] = State::kEmpty; // x-1 is unpopulated **
                }
                else 
                    grid[curr_x][curr_y-1] = State::kObstacle;
                break;
            case Head::South:
                if (state == State::kEmpty){
                    grid[curr_x][curr_y+1] = State::kEmpty;
                    grid[curr_x][curr_y+2] = State::kEmpty;
                }
                else
                    grid[curr_x][curr_y+1] = State::kObstacle;
                break;
            case Head::East:
                if (state == State::kEmpty){
                    grid[curr_x-1][curr_y] = State::kEmpty;
                    grid[curr_x-2][curr_y] = State::kEmpty;
                }
                else
                    grid[curr_x-1][curr_y] = State::kObstacle;
                break;
            case Head::West:
                if (state == State::kEmpty){
                     grid[curr_x+1][curr_y] = State::kEmpty;
                    grid[curr_x+2][curr_y] = State::kEmpty;
                }
                else
                    grid[curr_x+1][curr_y] = State::kObstacle;
                break;
            default:
                Serial.println("No left Match!");
        }
    }
}

std::vector<std::vector<State>> RoutePlanner::AStarSearch(std::vector<std::vector<State>> grid){
    // Create the vector of open nodes.
    std::vector<std::vector<int>> open;
    // A copy of the grid to be used for the optimal route
    std::vector<std::vector<State>> cp;
    std::copy(grid.begin(), grid.end(), back_inserter(cp));
    
    // Initialize the starting node.
    int g = 0; // initializing cost
    int h = Heuristic(curr_x, curr_y, goal[0],goal[1]); // initializing heuristic

    // prev used to link the children to it's parent node
    Node* prev = nullptr; 
    // curr used to store the current node's address
    Node* curr = nullptr;
    // Initializing node with coordinates and parent   
    Node *node = new Node;
    node -> parent = prev;
    node -> x_coor = curr_x;
    node -> y_coor = curr_y;
    node -> g_val = g;
    node -> h_val = h;

    //CreateMap(grid);
    //ExpandNeighbors(current, goal, open, grid, curr);
    pList.push_back(node);
    curr = node;
    //prev = node; 

    AddToOpen(curr_x, curr_y, g, h, open, grid); // initializing the open list with the first node

    int count = 1;
    while (open.size() > 0) {
        Serial.print("Iteration: ");
        Serial.println(count);
        // check if the current node is neighbor to previous node. if not, call TrackBack()
        std::vector<int> current;
        //CreateMap(grid);
        PrintBoard(grid);
        count += 1;
        // Get the next node
        CellSort(&open); 
        current = open.back();
        open.pop_back();
        curr_x = current[0]; // from the lowest g + h value
        curr_y = current[1]; // from the lowest g + h value
        Serial.print("curr_x, curr_y: ");
        Serial.print(curr_x);
        Serial.print(" ");
        Serial.println(curr_y);
        grid[curr_x][curr_y] = State::kPath; // close the path for the current node
    
        // Find the parent node address (used in ExpandNeighbors() to link the child)
        for (Node *p : pList){
            if (p -> x_coor == curr_x and p -> y_coor == curr_y){
                prev = curr;
                curr = p;
                p = pList.at(pList.size()-1); // last element to stop the loop
            }
        }
        //CreateMap(grid);
        if (curr -> x_coor == prev -> x_coor and curr -> y_coor == prev -> y_coor);
        else if (Neighbor(curr, prev, grid)){
            Go(curr -> x_coor, prev -> x_coor, curr -> y_coor, prev -> y_coor);
        }
        else if (!(Neighbor(curr, prev, grid))){
            Serial.println("Initiate Trackback");
            TrackBack(curr, prev);
        }

        // if the neighbor is in pList and the neigbors cost is less than the parent's cost
        for (Node *ptr : pList){
            if ((ptr -> x_coor == curr_x-2 and ptr -> y_coor == curr_y and grid[curr_x-1][curr_y] != State::kObstacle)
            or (ptr -> x_coor == curr_x+2 and ptr -> y_coor == curr_y and grid[curr_x+1][curr_y] != State::kObstacle)
            or (ptr -> x_coor == curr_x and ptr -> y_coor == curr_y-2 and grid[curr_x][curr_y-1] != State::kObstacle)
            or (ptr -> x_coor == curr_x and ptr -> y_coor == curr_y + 2 and grid[curr_x][curr_y+1] != State::kObstacle)) {
                if (ptr -> g_val < curr -> parent -> g_val)
                    curr -> parent = ptr;
            }
        }

        // Check if we're done.
        if (curr_x == goal[0] && curr_y == goal[1]) {
            Serial.println("FOUND GOAL!");
            grid[init[0]][init[1]] = State::kStart;
            grid[goal[0]][goal[1]] = State::kFinish;
            Serial.print("Iteration: ");
            Serial.println(count);
            PrintBoard(grid);
            Serial.println();
            ConstructFinalPath(); // generates the optimal path coordinates
            auto finalBoard = MakeBoard(cp); // generates the board with optimal path coordinates
            PrintBoard(finalBoard);
            //killMotor();
            delay(15000);
            curr_head = Head::East;
            FinalRun();
            return finalBoard; 
        }
        // If we're not done, expand search to current node's neighbors.
        CreateMap(grid);
        ExpandNeighbors(current, goal, open, grid, curr);
    }
    
    // We've run out of new nodes to explore and haven't found a path.
    Serial.println("No path found!");
    return std::vector<std::vector<State>>{};
}

void RoutePlanner::FinalRun(){
    int prevX = 0;
    int prevY = 0;
    for (const auto row : path_found){
        int currX = row -> x_coor;
        int currY = row -> y_coor;
        Go(currX, prevX, currY, prevY);
        prevX = currX;
        prevY = currY;
    }
  }

std::vector<std::vector<State>> RoutePlanner::MakeBoard(std::vector<std::vector<State>> board){
    int x,y;
    for (auto p:path_found){
        x = p -> x_coor;
        y = p -> y_coor;
        board[x][y] = State::kPath;
    }
    board[init[0]][init[1]] = State::kStart;
    board[goal[0]][goal[1]] = State::kFinish;
    return board;
}

// Constructs the final path of the maze (for the second run...)
void RoutePlanner::ConstructFinalPath(){
    Node *current_node = pList.back();
    while(current_node != nullptr){
        path_found.push_back(current_node);
        current_node = current_node -> parent;
    }
    std::reverse(path_found.begin(), path_found.end());
    /*
    for (auto p:path_found){
        std::cout << p -> x_coor << " " << p -> y_coor << std::endl;
    }
    */
}

/*
std::vector<std::vector<State>> RoutePlanner::MakeBoard(std::vector<std::vector<State>> board){
    int x,y;
    for (auto p:path_found){
        x = p -> x_coor;
        y = p -> y_coor;
        board[x][y] = State::kPath;
    }
    return board;
}
*/

void RoutePlanner::CellString(State cell){
    switch(cell) {
        case State::kObstacle: 
          Serial.print("|  ");
          break;
        case State::kPath: 
          Serial.print("P  ");
          break;
        //case State::kClosed: Serial.print("C  "); // comment back
        case State::kStart: 
          Serial.print("S  ");
          break;
        case State::kFinish: 
          Serial.print("E  ");
          break;
        case State::kUndefined: 
          Serial.print("U  ");
          break;
        default: Serial.print("0  "); 
    }
}

std::vector<std::vector<State>> RoutePlanner::PrintBoard(std::vector<std::vector<State>> board){
    //board[init[0]][init[1]] = State::kStart;
    //board[goal[0]][goal[1]] = State::kFinish;
    volatile int count = 0;
    Serial.println("Before print");
    for (int i = 0; i < board.size(); i++) {
        for (int j = 0; j < board[i].size(); j++) {
            //delay(10);
            count ++;
            CellString(board[i][j]);
        }
        Serial.println();
        //delay(100);
    }
    Serial.println(count);
    Serial.println("After print");
    return board;
}

// Iterate until the root and store the coordinates for both the nodes
// Pop the common ancestors until the most recent ancestor (where they branch off)
// Allign the coordinates accordingly
void RoutePlanner::TrackBack(Node* curr, Node* prev){

    //delay(3000);
    std::vector<std::vector<int>> currRoot;
    std::vector<std::vector<int>> prevRoot;
    std::vector<std::vector<int>> path;
    Node* temp_curr = curr;
    Node* temp_prev = prev;
    
    while (temp_curr != nullptr){
        currRoot.push_back(std::vector<int>{temp_curr -> x_coor, temp_curr -> y_coor});
        temp_curr = temp_curr -> parent;
    }
    while (temp_prev != nullptr){
        prevRoot.push_back(std::vector<int>{temp_prev -> x_coor, temp_prev -> y_coor});
        temp_prev = temp_prev -> parent;
    }
    bool ancestorFound = false;
    while (!ancestorFound){
        if (currRoot.back() == prevRoot.back()){
            if (currRoot.end()[-2] == prevRoot.end()[-2]){
                currRoot.pop_back();
                prevRoot.pop_back();
            }
            else {
                prevRoot.pop_back(); // taking the common node out
                prevRoot.erase(prevRoot.begin()); // taking the current position out
                ancestorFound = true;
            }
        }
    }

    reverse(currRoot.begin(), currRoot.end());
    path = prevRoot;
    path.insert(path.end(), currRoot.begin(), currRoot.end());

    // let's print out the trackback path
    Serial.println("Trackback start");
    int prevX = prev -> x_coor;
    int prevY = prev -> y_coor;

    for (int i = 0; i < path.size(); i++)
    {
        for (int j = 0; j < path[i].size(); j++)
        {
            Serial.print(path[i][j]);
            Serial.println(" ");
        }    
        Serial.println();
    }

    Serial.println("Prev x, Prev y");
    Serial.print(prevX);
    Serial.print(" ");
    Serial.println(prevY);
    for (const auto row : path){
        int currX = row[0];
        int currY = row[1];
        Go(currX, prevX, currY, prevY);
        delay(150);
        prevX = currX;
        prevY = currY;
    }
    Serial.println("Trackback end");
}

// Check the neighbor relationship between current and previous node
bool RoutePlanner::Neighbor(Node* curr, Node* prev, std::vector<std::vector<State>> grid){
    for (int i = 0; i < 4; i++) {
        int x2 = prev -> x_coor + deltaNode[i][0];
        int x3 = prev -> x_coor + deltaGrid[i][0]; // wall position
        int y2 = prev -> y_coor + deltaNode[i][1]; 
        int y3 = prev -> y_coor + deltaGrid[i][1]; // wall position
        //std::cout << curr -> x_coor << " " << curr -> y_coor << std::endl;
        //std::cout << prev -> x_coor << " " << prev -> y_coor << std::endl;
       // CreateMap(grid);
       /*
       if (i == 0){
        if (getDistance(trig[1], echo[1]) < 12 && curr_head == Head::North)
          grid[x3][y3] = State::kObstacle;
        
       }
          */
        if (curr -> x_coor == x2 and curr -> y_coor == y2 and grid[x3][y3] != State::kObstacle){
            std::cout << "Returning true" << std::endl;
            return true; // when curr is the prev neighbor and there is no wall in between them
        }
        else if (curr -> x_coor == x2 and curr -> y_coor == y2 and grid[x3][y3] == State::kObstacle){
          //delay(3000);
          return false; // when curr is the prev neighbor but there is a wall in between 
          }
    }
    std::cout << "returning false" << std::endl;
    return false; // when the curr is not a prev neighbor
}

void RoutePlanner::Go(int currX, int prevX, int currY, int prevY){
    switch (curr_head){
        case Head::North:
            if (currX == prevX - 2 && currY == prevY){
                fwd();
            }
            else if (currX == prevX + 2 && currY == prevY){
                rotate();
                fwd();
                curr_head = Head::South;
            }
            else if (currX == prevX && currY == prevY + 2){
                turnRight();
                fwd();
                curr_head = Head::East;
            }
            else if (currX == prevX && currY == prevY - 2){
                turnLeft();
                fwd();
                curr_head = Head::West;
            }
            break;
        case Head::South:
            if (currX == prevX - 2 && currY == prevY){
                rotate();
                fwd();
                curr_head = Head::North;
            }
            else if (currX == prevX + 2 && currY == prevY){
                fwd();
            }
            else if (currX == prevX && currY == prevY + 2){
                turnLeft();
                fwd();
                curr_head = Head::East;
            }
            else if (currX == prevX && currY == prevY - 2){
                turnRight();
                fwd();
                curr_head = Head::West;
            }
            break;
        case Head::East:
            if (currX == prevX - 2 && currY == prevY){
                turnLeft();
                fwd();
                curr_head = Head::North;
            }
            else if (currX == prevX + 2 && currY == prevY){
                Serial.println("going right!");
                turnRight();
                fwd();
                curr_head = Head::South;
            }
            else if (currX == prevX && currY == prevY + 2){
                fwd();
            }
            else if (currX == prevX && currY == prevY - 2){
                Serial.println("rotating!");
                rotate();
                fwd();
                curr_head = Head::West;
            }
            break;
        case Head::West:
            if (currX == prevX - 2 && currY == prevY){
                turnRight();
                fwd();
                curr_head = Head::North;
            }
            else if (currX == prevX + 2 && currY == prevY){
                turnLeft();
                fwd();
                curr_head = Head::South;
            }
            else if (currX == prevX && currY == prevY + 2){
                rotate();
                fwd();
                curr_head = Head::East;
            }
            else if (currX == prevX && currY == prevY - 2){
                fwd();
            }
            break;
    }
}
