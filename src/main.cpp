#include "maze.h"
#include "sensor.h"
#include "hardware.h"
#include <Arduino.h>
// #include <ESP32Encoder.h>
// #include <PID_v1.h>

// Global objects
Mouse mouse;
cell maze[Size][Size];
int8_t StoG = 1;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ;
  } // Wait for serial port to connect (mainly for native USB)
  delay(1000); // Give the monitor time to open

  Serial.println("\n--- MICROMOUSE STARTING ---");
  Sensor::init();
  Hardware::init();
  // Border Initialization
  for (int8_t i = 0; i < Size; ++i)
  {
    maze[0][i].setWall('W');
    maze[Size - 1][i].setWall('E');
    maze[i][0].setWall('N');
    maze[i][Size - 1].setWall('S');
  }

  updateManhattan(maze, StoG, false);

  // Physical Exploration Loop
  while (NOTatGoal(mouse))
  {
    explore(maze, mouse, StoG);
  }

  StoG = 2;
  updateManhattan(maze, StoG, false);
  while (NOTatGoal2(mouse))
  {
    explore(maze, mouse, StoG);
  }

  // PHASE 3: THE FINAL RUN (Shortest Path)
  // 1. Calculate the manhattan distances based ONLY on known walls
  updateManhattan(maze, 1, true);

  int pathX[MAX_PATH_LENGTH], pathY[MAX_PATH_LENGTH];
  int pathLength = 0;

  // 2. Generate the path from start (current position) to goal
  getShortestPath(maze, mouse.X, mouse.Y, goal, goal, pathX, pathY, pathLength);
  for (int i = 0; i < pathLength; ++i)
  {
    Serial.print("(");
    Serial.print(pathX[i]);
    Serial.print(", ");
    Serial.print(pathY[i]);
    Serial.print(") -> ");
  }
  Serial.println("");

  // 3. Compress it (turns (0,15)->(0,14)->(0,13) into (0,15)->(0,13))
  int compressedX[MAX_PATH_LENGTH], compressedY[MAX_PATH_LENGTH];
  int compressedLength = 0;
  compressPath(pathX, pathY, pathLength, compressedX, compressedY, compressedLength);
  for (int i = 0; i < compressedLength; ++i)
  {
    Serial.print("(");
    Serial.print(compressedX[i]);
    Serial.print(", ");
    Serial.print(compressedY[i]);
    Serial.print(") -> ");
  }
  Serial.println("");

  // 4. DRIVE THE PATH
  // We skip index 0 because it's where the mouse currently is.
  // for (int i = 1; i < compressedLength; ++i)
  // {
  //   driveToCell(compressedX[i], compressedY[i]);
  // }

  // Mouse has finished!
  Hardware::stop();
}

void loop()
{
  // updateYaw();
  // delay(10);
  // Nothing here because we used a "blocking" approach in setup
}

