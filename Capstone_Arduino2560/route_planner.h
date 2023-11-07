// Author: Alex Gomez

#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H
//#include "Particle.h"
//#include <ArxContainer.h>
#include <Arduino_AVRSTL.h>
//#include <vector>
//#include <string>
#include <cstdlib>
//#include <algorithm>
#include <iterator>

enum class State {kUndefined, kEmpty, kObstacle, kClosed, kPath, kStart, kFinish};
enum class Dir {Fwd, Right, Left};
enum class Head {North, South, East, West};

struct Coordinate{
    int x_coor{0};
    int y_coor{0};
};

class Node : public Coordinate{
    public:
        Node *parent = nullptr;
        int g_val{0};
        int h_val{0};
};

class RoutePlanner {
    public:
        RoutePlanner(std::vector<int> start, std::vector<int> end_goal):
            init(start), goal(end_goal), curr_x(start[0]), curr_y(start[1]){}
        static bool Compare(const std::vector<int> a, const std::vector<int> b);
        void CellSort(std::vector<std::vector<int>> *v);
        int Heuristic(int x1, int y1, int x2, int y2);
        bool CheckValidCell(int x, int y, std::vector<std::vector<State>> &grid);
        void AddToOpen(int x, int y, int g, int h,
        std::vector<std::vector<int>> &openlist, std::vector<std::vector<State>> &grid);
        void ExpandNeighbors(const std::vector<int> &current, std::vector<int> goal, 
        std::vector<std::vector<int>> &openlist, std::vector<std::vector<State>> &grid, Node* prev);
        std::vector<std::vector<State>> AStarSearch(std::vector<std::vector<State>> grid);
        void ConstructFinalPath(); 
        std::vector<std::vector<State>> MakeBoard(std::vector<std::vector<State>> board);
        void CellString(State cell);
        std::vector<std::vector<State>> PrintBoard(std::vector<std::vector<State>> board);

        void TrackBack(Node* curr, Node* prev);
        bool Neighbor(Node * curr, Node* prev, std::vector<std::vector<State>> grid);

        void CreateMap(std::vector<std::vector<State>> &grid);
        bool CheckValid(std::vector<std::vector<State>> &grid, Dir dir);
        void AddCellWall(std::vector<std::vector<State>> &grid, Dir dir, State state); 
        void FinalRun();
        void Go(int currX, int prevX, int currY, int prevY);
    private:
        std::vector<int> init, goal; // start and end coordinates
        int curr_x, curr_y;
        Head curr_head{Head::East};
        std::vector<Node *> pList; // list of all the parents
        std::vector<Node *> path_found; // the optimal path coordinates
        // directional deltas
        const int deltaGrid[4][2]{{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
        const int deltaNode[4][2]{{-2, 0}, {0, -2}, {2, 0}, {0, 2}}; 

};

#endif //ROUTE_PLANNER_H
