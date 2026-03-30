#include <cstdint>
#include "hardware.h"
#include "sensor.h"
#include "maze.h"

using namespace std; 

// direction helpers :
constexpr int dx[4] = {-1, 1, 0, 0};
constexpr int dy[4] = {0, 0, -1, 1};

void updateManhattan(cell maze[Size][Size], int8_t StoG, bool visitedOnly)
{
    // Reset cells
    for (int8_t x = 0; x < Size; ++x)
        for (int8_t y = 0; y < Size; ++y)
            if (!visitedOnly || maze[x][y].isVisited())
                maze[x][y].manhattan = MAX_PATH_LENGTH-1;

    SimpleQueue q;

    // Center goals
    if (StoG == 1)
    {
        constexpr pair<int8_t, int8_t> goals[4] = {{goal2, goal2}, {goal2, goal}, {goal, goal2}, {goal, goal}};
        for (auto g : goals)
        {
            int8_t gx = g.first, gy = g.second;
            if (!visitedOnly || maze[gx][gy].isVisited())
            {
                maze[gx][gy].manhattan = 0;
                q.push(gx, gy);
            }
        }
    }
    else if (StoG == 2)
    {
        if (!visitedOnly || maze[0][Size - 1].isVisited())
        {
            maze[0][Size - 1].manhattan = 0;
            q.push(0, Size - 1);
        }
    }

    // BFS
    while (!q.empty())
    {
        int8_t x = q.frontX();
        int8_t y = q.frontY();
        q.pop();
        uint8_t d = maze[x][y].manhattan;

        for (int8_t k = 0; k < 4; ++k)
        {
            int8_t nx = x + dx[k];
            int8_t ny = y + dy[k];

            if (nx < 0 || nx >= Size || ny < 0 || ny >= Size)
                continue;
            if (visitedOnly && !maze[nx][ny].isVisited())
                continue; // skip unvisited if in visitedOnly mode

            // check walls
            if (k == 0 && maze[x][y].hasWall('W'))
                continue;
            if (k == 1 && maze[x][y].hasWall('E'))
                continue;
            if (k == 2 && maze[x][y].hasWall('N'))
                continue;
            if (k == 3 && maze[x][y].hasWall('S'))
                continue;

            if (maze[nx][ny].manhattan > d + 1)
            {
                maze[nx][ny].manhattan = d + 1;
                q.push(nx, ny);
            }
        }
    }
}

void explore(cell maze[Size][Size], Mouse &mouse, int StoG)
{
    // current cell = visited
    int8_t x = mouse.X;
    int8_t y = mouse.Y;
    maze[x][y].setVisited();
    //------------------------------------------------------------------------------------------------------------------------------------------------

    // check walls around - update maze
    if (Sensor::wallFront())
    {
        maze[x][y].setWall(mouse.dir); // Set wall in current direction

        // Also set the opposite wall in neighbor cell
        int8_t nx = x, ny = y;
        char oppositeDir;

        switch (mouse.dir)
        {
        case 'N':
            ny = y - 1;
            oppositeDir = 'S';
            break;
        case 'S':
            ny = y + 1;
            oppositeDir = 'N';
            break;
        case 'E':
            nx = x + 1;
            oppositeDir = 'W';
            break;
        case 'W':
            nx = x - 1;
            oppositeDir = 'E';
            break;
        }

        if (nx >= 0 && nx < Size && ny >= 0 && ny < Size)
        {
            maze[nx][ny].setWall(oppositeDir);
        }
    }

    if (Sensor::wallLeft())
    {
        // Determine left direction
        char leftDir;
        switch (mouse.dir)
        {
        case 'N':
            leftDir = 'W';
            break;
        case 'S':
            leftDir = 'E';
            break;
        case 'E':
            leftDir = 'N';
            break;
        case 'W':
            leftDir = 'S';
            break;
        }

        maze[x][y].setWall(leftDir);

        // Set opposite wall in neighbor
        int8_t nx = x, ny = y;
        char oppositeDir;
        switch (leftDir)
        {
        case 'N':
            ny = y - 1;
            oppositeDir = 'S';
            break;
        case 'S':
            ny = y + 1;
            oppositeDir = 'N';
            break;
        case 'E':
            nx = x + 1;
            oppositeDir = 'W';
            break;
        case 'W':
            nx = x - 1;
            oppositeDir = 'E';
            break;
        }

        if (nx >= 0 && nx < Size && ny >= 0 && ny < Size)
        {
            maze[nx][ny].setWall(oppositeDir);
        }
    }

    if (Sensor::wallRight())
    {
        // Determine right direction
        char rightDir;
        switch (mouse.dir)
        {
        case 'N':
            rightDir = 'E';
            break;
        case 'S':
            rightDir = 'W';
            break;
        case 'E':
            rightDir = 'S';
            break;
        case 'W':
            rightDir = 'N';
            break;
        }

        maze[x][y].setWall(rightDir);

        // Set opposite wall in neighbor
        int nx = x, ny = y;
        char oppositeDir;
        switch (rightDir)
        {
        case 'N':
            ny = y - 1;
            oppositeDir = 'S';
            break;
        case 'S':
            ny = y + 1;
            oppositeDir = 'N';
            break;
        case 'E':
            nx = x + 1;
            oppositeDir = 'W';
            break;
        case 'W':
            nx = x - 1;
            oppositeDir = 'E';
            break;
        }

        if (nx >= 0 && nx < Size && ny >= 0 && ny < Size)
        {
            maze[nx][ny].setWall(oppositeDir);
        }
    }
    //---------------------------------------------------------------------------------------
    // check for avaialable cells
    int8_t min_nx = -1;
    int8_t min_ny = -1;
    char n_dir = '\0';

    int distance = maze[x][y].manhattan;
    for (int k = 0; k < 4; ++k)
    {
        int nx = x + dx[k];
        int ny = y + dy[k];
        if (nx < 0 || nx >= Size || ny < 0 || ny >= Size)
            continue;

        if (k == 0)
        {
            if (maze[x][y].hasWall('W'))
            {
                continue;
            }
            else if (maze[nx][ny].manhattan < distance)
            {
                min_nx = nx;
                min_ny = ny;
                n_dir = 'W';
                distance = maze[nx][ny].manhattan;
            }
        }
        else if (k == 1)
        {
            if (maze[x][y].hasWall('E'))
            {
                continue;
            }
            else if (maze[nx][ny].manhattan < distance)
            {
                min_nx = nx;
                min_ny = ny;
                n_dir = 'E';
                distance = maze[nx][ny].manhattan;
            }
        }
        else if (k == 2)
        {
            if (maze[x][y].hasWall('N'))
            {
                continue;
            }
            else if (maze[nx][ny].manhattan < distance)
            {
                min_nx = nx;
                min_ny = ny;
                n_dir = 'N';
                distance = maze[nx][ny].manhattan;
            }
        }
        else if (k == 3)
        {
            if (maze[x][y].hasWall('S'))
            {
                continue;
            }
            else if (maze[nx][ny].manhattan < distance)
            {
                min_nx = nx;
                min_ny = ny;
                n_dir = 'S';
                distance = maze[nx][ny].manhattan;
            }
        }
    }
    //------------------------------------------------------------------------------------------------------------------------------------------------

    // if no viable cell is found - updateManhattan
    if (n_dir == '\0')
    {
        updateManhattan(maze, (int8_t)StoG);
        return;
    }
    //------------------------------------------------------------------------------------------------------------------------------------------------

    // move to the cell - SMARTER TURNING
    if (n_dir != mouse.dir)
    {
        // Calculate relative direction
        if ((mouse.dir == 'N' && n_dir == 'E') ||
            (mouse.dir == 'E' && n_dir == 'S') ||
            (mouse.dir == 'S' && n_dir == 'W') ||
            (mouse.dir == 'W' && n_dir == 'N'))
        {
            // Right turn
            Hardware::turnRight();
        }
        else if ((mouse.dir == 'N' && n_dir == 'W') ||
                 (mouse.dir == 'W' && n_dir == 'S') ||
                 (mouse.dir == 'S' && n_dir == 'E') ||
                 (mouse.dir == 'E' && n_dir == 'N'))
        {
            // Left turn
            Hardware::turnLeft();
        }
        else
        {
            // 180 degree turn (do 2 rights or 2 lefts)
            Hardware::turnRight();
            Hardware::turnRight();
        }

        mouse.dir = n_dir;
    }

    Hardware::moveForward(1);
    mouse.X = min_nx;
    mouse.Y = min_ny;

    return; // one step done
    //------------------------------------------------------------------------------------------------------------------------------------------------
}

void getShortestPath(cell maze[Size][Size], int8_t startX, int8_t startY, int8_t goalX, int8_t goalY,
                     int pathX[], int pathY[], int &pathLength)
{
    // ADD THESE LINES at the start of the function:
    pathLength = 0;
    pathX[pathLength] = startX;
    pathY[pathLength] = startY;
    pathLength++;
    int8_t x = startX;
    int8_t y = startY;

    // Also remove the initial push_back since we already did it

    while (!(x == goalX && y == goalY))
    {
        int curManhattan = maze[x][y].manhattan;
        bool moved = false;

        // Check all 4 directions
        if (!maze[x][y].hasWall('W') && x - 1 >= 0 && maze[x - 1][y].manhattan == curManhattan - 1)
        {
            x = x - 1;
            moved = true;
        }
        else if (!maze[x][y].hasWall('E') && x + 1 < Size && maze[x + 1][y].manhattan == curManhattan - 1)
        {
            x = x + 1;
            moved = true;
        }
        else if (!maze[x][y].hasWall('N') && y - 1 >= 0 && maze[x][y - 1].manhattan == curManhattan - 1)
        {
            y = y - 1;
            moved = true;
        }
        else if (!maze[x][y].hasWall('S') && y + 1 < Size && maze[x][y + 1].manhattan == curManhattan - 1)
        {
            y = y + 1;
            moved = true;
        }

        if (!moved)
        {
            // No valid neighbor, break
            break;
        }

        // NEW:
        pathX[pathLength] = x;
        pathY[pathLength] = y;
        pathLength++;

        // Also remove the initial push_back since we already did it
    }
}

void compressPath(const int pathX[], const int pathY[], int pathLength,
                  int compressedX[], int compressedY[], int &compressedLength)
{
    // NEW START:
    if (pathLength <= 2)
    {
        // Copy entire path
        for (int i = 0; i < pathLength; i++)
        {
            compressedX[i] = pathX[i];
            compressedY[i] = pathY[i];
        }
        compressedLength = pathLength;
        return;
    }

    // Start with first point
    compressedX[0] = pathX[0];
    compressedY[0] = pathY[0];
    compressedLength = 1;

    for (int i = 1; i < pathLength - 1; ++i)
    {
        int x0 = pathX[i - 1], y0 = pathY[i - 1];
        int x1 = pathX[i], y1 = pathY[i];
        int x2 = pathX[i + 1], y2 = pathY[i + 1];

        // check if current point is in straight line between previous and next
        if ((x0 == x1 && x1 == x2) || (y0 == y1 && y1 == y2))
        {
            continue; // skip middle point
        }

        // otherwise it's a turn, keep it
        compressedX[compressedLength] = pathX[i];
        compressedY[compressedLength] = pathY[i];
        compressedLength++;
    }

    compressedX[compressedLength] = pathX[pathLength - 1];
    compressedY[compressedLength] = pathY[pathLength - 1];
    compressedLength++;
}