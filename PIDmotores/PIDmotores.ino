// uncomment to use the ETL without the STL when using mega2560
// #define ETL_NO_STL
#include"Embedded_Template_Library.h"
#include<etl/vector.h>
#include<etl/stack.h>
#include "map.h"
#include "Tile.h"
#include "TileDirection.h"
#include "coord.h"
#include "CustomSerial.h"
#include "Movement.h"
#include "Pins.h"
#include "Encoder.h"

#define DEBUG_ALGORITHM 1

Movement robot;

// double targetOrientation = 0.0; pq global?
unsigned long iterations = 0;
bool hasArrived = false;

constexpr TileDirection directions[] = {TileDirection::kUp, TileDirection::kDown, TileDirection::kLeft, TileDirection::kRight};

uint16_t robotOrientation = 0;
coord robotCoord = coord{0,0,0};

void turnRobot(const int targetOrientation) {
    int difference = targetOrientation - robotOrientation;
    if (difference == 0) {
        return;
    }

    if (difference == 90 || difference == -270) {
        robot.turnRight(targetOrientation);
        robotOrientation = (robotOrientation + 90) % 360;
    } else if (difference == -90 || difference == 270) {
        robot.turnLeft(targetOrientation);
        robotOrientation = (robotOrientation + 270) % 360;
    } else if (difference == 180 || difference == -180) {
        robot.turnRight(targetOrientation);
        robotOrientation = (robotOrientation + 180) % 360;
    }
}

void followPath(etl::stack<coord, kMaxMapSize>& path) {
    while(!path.empty()) {
        const coord& next = path.top();
        path.pop();
        if (next.x > robotCoord.x) {
            turnRobot(270);
        } else if (next.x < robotCoord.x) {
            turnRobot(90);
        } else if (next.y > robotCoord.y) {
            turnRobot(0);
        } else if (next.y < robotCoord.y) {
            turnRobot(180);
        }
        robot.goForward(robotOrientation);
        robotCoord = next;
    }
}

void dijsktra(const coord& start, const coord& end, const Map& tilesMap, const etl::vector<Tile, kMaxMapSize>& tiles) {
    etl::vector<bool, kMaxMapSize> explored;
    etl::vector<int, kMaxMapSize> distance;
    etl::vector<coord, kMaxMapSize> previousPositions;
    etl::stack<coord, kMaxMapSize> path;
    // initialize distance.
    for (int i = tilesMap.positions.size() - 1; i >= 0; --i) {
        distance.push_back(INT_MAX);
        explored.push_back(false);
    }
    distance[tilesMap.getIndex(start)] = 0;
    explored[tilesMap.getIndex(start)] = true;
    // explore the map.
    coord currentCoord = start;
    int minDistance;
    while (!explored[tilesMap.getIndex(end)]) {
        // update distance.
        for (int i = 0; i < 4; i++) {
            const TileDirection& direction = directions[i];
            const Tile& currentTile = tiles[tilesMap.getIndex(currentCoord)];
            const coord& adjacentCoord = currentTile.adjacentTiles_[static_cast<int>(direction)]->position_;
            // check if there's an adjecent tile and there's no wall.
            //TODO: check if the tile to explore is black
            if (currentTile.adjacentTiles_[static_cast<int>(direction)] != NULL && !currentTile.hasWall(direction)) {
                const int weight = currentTile.weights_[static_cast<int>(direction)] + distance[tilesMap.getIndex(currentCoord)];
                // check if the new weight to visit the adjecent tile is less than the current weight.
                if (weight < distance[tilesMap.getIndex(adjacentCoord)]) {
                    distance[tilesMap.getIndex(adjacentCoord)] = weight;
                    previousPositions[tilesMap.getIndex(adjacentCoord)] = currentCoord;
                }
            }
        }
        // find next tile.
        minDistance = INT_MAX;
        for (int i = tilesMap.positions.size() - 1; i >= 0; --i) {
            const coord& current = tilesMap.positions[i];
            const int currentDistance = distance[tilesMap.getIndex(current)];
            if (currentDistance < minDistance && !explored[tilesMap.getIndex(current)]) {
                minDistance = currentDistance;
                currentCoord = current;
            }
        }
        explored[tilesMap.getIndex(currentCoord)] = true;
    }
    // find path.
    coord current = end;
    while (current != start) {
        path.push(current);
        current = previousPositions[tilesMap.getIndex(current)];
    }
    path.push(start);
    // followPath(path);
}

// bool checkForWall(const etl::vector<etl::vector<char, kMaxMapSize>, kMaxMapSize>& maze, const TileDirection& direction, const coord& currentTileCoord) {
//     switch(direction) {
//         case TileDirection::kRight:
//             return maze[currentTileCoord.y][currentTileCoord.x + 1] == '#';
//         case TileDirection::kUp:
//             return maze[currentTileCoord.y + 1][currentTileCoord.x] == '#';
//         case TileDirection::kLeft:
//             return maze[currentTileCoord.y][currentTileCoord.x - 1] == '#';
//         case TileDirection::kDown:
//             return maze[currentTileCoord.y - 1][currentTileCoord.x] == '#';
//     }
// }

void test() {
    customPrintln("Test");
}

void depthFirstSearch(Map& tilesMap, etl::vector<Tile, kMaxMapSize>& tiles) {
    Map visitedMap = Map();
    etl::vector<bool, kMaxMapSize> visited;
    etl::stack<coord, kMaxMapSize> unvisited;
    Tile* currentTile;
    bool wall;
    bool alreadyConnected;
    bool visitedFlag;
    coord nextTileCoord;
    TileDirection oppositeDirection;
    unvisited.push(robotCoord);
    #if DEBUG_ALGORITHM
    customPrintln("inicio DFS");
    #endif
    // explore the map.
    while (!unvisited.empty()){ // unvisited.size() != 256
        // get the next tile to explore.
        coord currentTileCoord = unvisited.top();
        unvisited.pop();
        // check if the tile has been visited.
        visitedFlag = false;
        for (int i=0; i<visitedMap.positions.size(); ++i) {
            if (visitedMap.positions[i] == currentTileCoord) {
                visitedFlag = true;
                break;
            }
        }
        if (visitedFlag) {
            continue;
        }
        dijsktra(robotCoord, currentTileCoord, tilesMap, tiles);
        #if DEBUG_ALGORITHM
        customPrint("currentTileCoord: ");
        customPrint(currentTileCoord.x);
        customPrint(" ");
        customPrint(currentTileCoord.y);
        customPrintln(unvisited.size());
        // customPrintln("After dijsktra");
        #endif
        robotCoord = currentTileCoord;
        visitedMap.positions.push_back(currentTileCoord);
        visited.push_back(true);
        // check walls the 4 adjacent tiles.
        for (const TileDirection& direction : directions) {
            wall = false;
            switch(direction) {
                case TileDirection::kRight:
                    nextTileCoord = coord{currentTileCoord.x + 2, currentTileCoord.y, 1}; // checkRamp(direction);
                    currentTile = &tiles[tilesMap.getIndex(currentTileCoord)];
                    oppositeDirection = TileDirection::kLeft;
                    break;
                case TileDirection::kUp:
                    nextTileCoord = coord{currentTileCoord.x, currentTileCoord.y + 2, 1}; // checkRamp(direction);
                    currentTile = &tiles[tilesMap.getIndex(currentTileCoord)];
                    oppositeDirection = TileDirection::kDown;
                    break;
                case TileDirection::kLeft:
                    nextTileCoord = coord{currentTileCoord.x - 2, currentTileCoord.y, 1}; // checkRamp(direction);
                    currentTile = &tiles[tilesMap.getIndex(currentTileCoord)];
                    oppositeDirection = TileDirection::kRight;
                    break;
                case TileDirection::kDown:
                    nextTileCoord = coord{currentTileCoord.x, currentTileCoord.y - 2, 1}; // checkRamp(direction);
                    currentTile = &tiles[tilesMap.getIndex(currentTileCoord)];
                    oppositeDirection = TileDirection::kUp;
                    break;
            }
            // check if the tile has not been checked.
            if (currentTile->adjacentTiles_[static_cast<int>(direction)] == NULL) {
                // check for a wall.
                // wall = robot.checkWallsDistances(direction, robotOrientation);
                wall = false;
                // create a pointer to the next tile and asign its coordenate if it's a new Tile.
                tilesMap.positions.push_back(nextTileCoord);
                tiles[tilesMap.getIndex(nextTileCoord)] = Tile(nextTileCoord);
                Tile* nextTile = &tiles[tilesMap.getIndex(nextTileCoord)];
                if (nextTile->position_ == kInvalidPosition) {
                     nextTile->setPosition(nextTileCoord);
                }
                // Link the two adjacent Tiles.
                currentTile->addAdjacentTile(direction, nextTile, wall);
                nextTile->addAdjacentTile(oppositeDirection, currentTile, wall);
                // Check if there's a wall between the two adjacent Tiles.
                if (!wall) {
                    // if the tile has not been visited, add it to the queue.
                    visitedFlag = false;
                    for (int i = 0; i < visitedMap.positions.size(); ++i) {
                        if (visitedMap.positions[i] == nextTileCoord) {
                            visitedFlag = true;
                            break;
                        }
                    }
                    if(!visitedFlag) {
                        #if DEBUG_ALGORITHM
                        customPrintln("finished loop");
                        // customPrint("nextTileCoord: ");
                        // customPrint(nextTileCoord.x);
                        // customPrint(" ");
                        // customPrintln(nextTileCoord.y);
                        #endif
                        unvisited.push(nextTileCoord);
                    }
                }
            }
        }
    }
    #if DEBUG_ALGORITHM
    customPrintln("termino DFS");
    #endif
}

void startAlgorithm() {
    Map tilesMap = Map();
    etl::vector<Tile, kMaxMapSize> tiles;
    tilesMap.positions.push_back(robotCoord);
    tiles[tilesMap.getIndex(robotCoord)] = Tile(robotCoord);
    #if DEBUG_ALGORITHM
    customPrintln("Start algorithm");
    #endif
    depthFirstSearch(tilesMap, tiles);
}

void setup(){
    Serial.begin(9600);
    while (!Serial) delay(10); // wait for serial port to open!
    #if DEBUG_ALGORITHM
    customPrintln("Serial ready");
    #endif
    // robot.setup();
    startAlgorithm();
    /* robot.moveMotors(MovementState::kForward, 0, 1.5);
    robot.moveMotors(MovementState::kTurnLeft, 270, 0);
    robot.moveMotors(MovementState::kForward, 270, 1.64);
    robot.moveMotors(MovementState::kTurnLeft, 90, 0);
    robot.moveMotors(MovementState::kForward, 90, 1.70);
    robot.moveMotors(MovementState::kTurnRight, 180, 0);
    robot.moveMotors(MovementState::kForward, 180, 1.5); */
    //robot.moveMotors(MovementState::kTurnRight, 90, 0);
    /* for (int i = 0; i < 1000; ++i) {
        robot.moveMotors(MovementState::kForward, 0);
    } */

    /* while (true) {
        vlx1.printDistance();
        delay(1000);
    } */

    /* while (robot.moveMotors(MovementState::kTurnRight, 0) == false) {
        customPrintln("Turning right");
    }
    customPrintln("Arrived to 0"); */
    //robot.moveMotors(MovementState::kForward, 0, 0.3);
    //robot.moveMotors(MovementState::kBackward, 0, 0.3);
}
    
void loop() {
    #if DEBUG_ALGORITHM
    customPrintln("Loop");
    #endif
    // WARNING: by using a while or for loop here, the robot will not follow the instruction
    // robot.moveMotors(MovementState::kForward, 0, 1);
    //robot.setSpeed(0);
    //robot.moveMotors(MovementState::kTurnRight, 90, 0);
    //robot.moveMotors(MovementState::kTurnLeft, 0, 0);
    //robot.moveMotors(MovementState::kForward, 0, 0.5);
    //robot.moveMotors(MovementState::kBackward, 0, 0);
    //robot.moveMotors(MovementState::kTurnRight, 90, 0);
    //robot.moveMotors(MovementState::kTurnLeft, 0, 0);
    

   /*  robot.moveMotors(MovementState::kForward, 0, 1.5);
    robot.moveMotors(MovementState::kTurnLeft, 270, 0);
    robot.moveMotors(MovementState::kForward, 270, 1.64);
    robot.moveMotors(MovementState::kTurnLeft, 89, 0);
    robot.moveMotors(MovementState::kForward, 91, 1.75);
    robot.moveMotors(MovementState::kTurnRight, 180, 0);
    robot.moveMotors(MovementState::kForward, 180, 1.5); */

    
    // robot.moveMotors(MovementState::kBackward, 0, 0.5);
    
    /* digitalWrite(18, HIGH);
    digitalWrite(19, LOW);
    analogWrite(23, 200); */

    /* if (hasArrived == false && robot.moveMotors(MovementState::kForward, 0, 0.5) == false) {
        customPrintln("Moving forward...");
    }
    else {
        customPrintln("Arrived");
        hasArrived = true;
    } */

   /*  while (hasArrived == false && robot.moveMotors(MovementState::kForward, 0, 0.5) == false){
        delay(100);
    }
    delay(1000);
    while (hasArrived == false && robot.moveMotors(MovementState::kBackward, 0, 0.5) == false){
        delay(100);
    }
    hasArrived = true; */

    /* for (int i = 0; i < 1000; ++i) {
        robot.moveMotors(MovementState::kForward, 0);
    } */

    /* if (robot.moveMotors(MovementState::kTurnLeft, 270) == false) {
        customPrintln("Turining left...");
    }
    else { 
        customPrintln("Arrived");
    } */
    
    /* if (iterations < 25) { // TODO: juega con este numero
        robot.moveMotors(MovementState::kForward, 0);
    } else if (iterations < 50) { // TODO: juega con este numero
        robot.moveMotors(MovementState::kStop, 0);
    } else if (iterations < 75) {
        robot.moveMotors(MovementState::kBackward, 0);
    } else if (iterations < 100) {
        robot.moveMotors(MovementState::kStop, 0);
    } else if (iterations < 125) {
        robot.moveMotors(MovementState::kTurnLeft, 90);
    } else if (iterations < 150) {
        robot.moveMotors(MovementState::kStop, 0);
    } else if (iterations < 175) {
        robot.moveMotors(MovementState::kTurnRight, 270);
    }
    else {
        robot.moveMotors(MovementState::kStop, 0);
        delay(10000);
    } */
    
    // ++iterations;

    //LEER VELOCIDAD 
    // customPrint("BACK_LEFT: ");
    // customPrintln(robot.getBackLeftSpeed());
    // customPrint("FRONT_LEFT: ");
    // customPrintln(robot.getFrontLeftSpeed());
    // customPrint("BACK_RIGHT: ");
    // customPrintln(robot.getBackRightSpeed());
    // customPrint("FRONT_RIGHT: ");
    // customPrintln(robot.getFrontRightSpeed()); 

    // delay(70);

    
    
    
    // Asi se mueven los motores sin PID
    //robot.moveMotors(MotorState::kForward);

    /* delay(2000);

    robot.moveMotors(MotorState::kStop);

    delay(2000);

    robot.moveMotors(MotorState::kBackward); 
 */
}