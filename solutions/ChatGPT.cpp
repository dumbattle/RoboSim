/*

Stack Based DFS

Strategy:
The robot explores the map using a DFS-style algorithm.

- It keeps an internal map of visited tiles and blocked tiles (walls/cliffs).
- From each position, it tries to move to any unexplored neighboring tile.
- If a move is possible, it goes there and saves the position on a stack.
- If all neighbors are explored or blocked, it backtracks using the stack.
- Sensors are only used when moving into unknown tiles.

This allows the robot to systematically explore every reachable tile
without random movement or repeated work.
*/


#include <stack>
#include <vector>
#include "robot_api.h"
#include "robot_params.h"



bool visited[MAP_WIDTH][MAP_HEIGHT] = {false};
bool blocked[MAP_WIDTH][MAP_HEIGHT] = {false};


Direction dir = EAST;

struct Node {
    int x,y;
};

std::stack<Node> path;

int rx, ry;

bool inBounds(int x,int y)
{
    return x>=0 && x<MAP_WIDTH && y>=0 && y<MAP_HEIGHT;
}

void TurnTo(Direction target)
{
    int diff = (target - dir + 4) % 4;

    if(diff == 1){
        TurnRight();
    }
    else if(diff == 2){
        TurnRight();
        TurnRight();
    }
    else if(diff == 3){
        TurnLeft();
    }

    dir = target;
}

bool MoveDir(Direction d)
{
    TurnTo(d);

    int nx = rx;
    int ny = ry;
    Translate(nx, ny, d);
    if(!inBounds(nx,ny)) return false;

    if(!visited[nx][ny] && IsWallAhead())
    {
        blocked[nx][ny] = true;
        return false;
    }

    MoveForward();

    rx = nx;
    ry = ny;

    visited[rx][ry] = true;

    return true;
}

void Explore()
{
    GetPosition(rx,ry);

    visited[rx][ry] = true;
    path.push({rx,ry});

    while(!path.empty())
    {
        bool moved = false;

        for(int i=0;i<4;i++)
        {
            Direction d = GetDirection();
            for(int j=0;j<i;j++) {
                d = Right(d);
            }

            int nx = rx;
            int ny = ry;
            Translate(nx, ny, d);
            if(!inBounds(nx,ny)) continue;
            if(visited[nx][ny]) continue;
            if(blocked[nx][ny]) continue;

            if(MoveDir(d))
            {
                path.push({rx,ry});
                moved = true;
                break;
            }
        }

        if(!moved)
        {
            path.pop();
            if(path.empty()) break;

            Node back = path.top();

            int dx2 = back.x - rx;
            int dy2 = back.y - ry;

            Direction backDir;

            if(dx2 == 1) backDir = EAST;
            else if(dx2 == -1) backDir = WEST;
            else if(dy2 == 1) backDir = NORTH;
            else backDir = SOUTH;

            MoveDir(backDir);
        }
    }

}

int main()
{
    Reset();
    Explore();
    PrintStatus();
}