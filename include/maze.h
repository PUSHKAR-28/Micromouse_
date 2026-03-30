#ifndef MAZE_H
#define MAZE_H

#include <cstdint>

// Constants
constexpr int Size = 8;
constexpr int MAX_PATH_LENGTH = Size*Size;
constexpr int8_t goal = Size / 2;
constexpr int8_t goal2 = goal - 1;

// Bit positions for wall flags
constexpr uint8_t WALL_NORTH = 0b00000001;
constexpr uint8_t WALL_SOUTH = 0b00000010;
constexpr uint8_t WALL_EAST = 0b00000100;
constexpr uint8_t WALL_WEST = 0b00001000;
constexpr uint8_t VISITED = 0b00010000;

// Simple Queue for BFS (Memory Efficient)
class SimpleQueue
{
private:
    int8_t xQueue[MAX_PATH_LENGTH];
    int8_t yQueue[MAX_PATH_LENGTH];
    int head;
    int tail;

public:
    SimpleQueue() : head(0), tail(0) {}
    inline bool empty() const { return head >= tail; }
    inline void push(int8_t x, int8_t y)
    {
        if (tail < MAX_PATH_LENGTH)
        {
            xQueue[tail] = x;
            yQueue[tail] = y;
            tail++;
        }
    }
    inline void pop() { head++; }
    inline int8_t frontX() const { return xQueue[head]; }
    inline int8_t frontY() const { return yQueue[head]; }
    inline void clear()
    {
        head = 0;
        tail = 0;
    }
};

// Cell Class
class cell
{
public:
    uint8_t manhattan;
    uint8_t flags;

    cell() : manhattan(MAX_PATH_LENGTH-1), flags(0) {}

    inline bool isVisited() const { return (flags & VISITED) != 0; }
    inline void setVisited() { flags |= VISITED; }

    inline bool hasWall(char dir) const
    {
        if (dir == 'N')
            return (flags & WALL_NORTH) != 0;
        if (dir == 'S')
            return (flags & WALL_SOUTH) != 0;
        if (dir == 'E')
            return (flags & WALL_EAST) != 0;
        if (dir == 'W')
            return (flags & WALL_WEST) != 0;
        return false;
    }

    inline void setWall(char dir)
    {
        if (dir == 'N')
            flags |= WALL_NORTH;
        else if (dir == 'S')
            flags |= WALL_SOUTH;
        else if (dir == 'E')
            flags |= WALL_EAST;
        else if (dir == 'W')
            flags |= WALL_WEST;
    }
};

// Mouse Class
class Mouse
{
public:
    int8_t X;
    int8_t Y;
    char dir;
    Mouse() : X(0), Y(Size-1), dir('N') {}
};

// Function Declarations
void updateManhattan(cell maze[Size][Size], int8_t StoG, bool visitedOnly = false);
void explore(cell maze[Size][Size], Mouse &mouse, int StoG);
inline bool NOTatGoal(Mouse &mouse)
{
    return !((mouse.X == goal2 && mouse.Y == goal2) || (mouse.X == goal && mouse.Y == goal) || (mouse.X == goal2 && mouse.Y == goal) || (mouse.X == goal && mouse.Y == goal2));
}

inline bool NOTatGoal2(Mouse &mouse)
{
    return !((mouse.X == 0 && mouse.Y == Size - 1));
}

void getShortestPath(cell maze[Size][Size], int8_t startX, int8_t startY, int8_t goalX, int8_t goalY,
                     int pathX[], int pathY[], int &pathLength);

void compressPath(const int pathX[], const int pathY[], int pathLength,
                  int compressedX[], int compressedY[], int &compressedLength);

#endif