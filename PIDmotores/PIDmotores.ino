// uncomment to use the ETL without the STL when using mega2560
// #define ETL_NO_STL
#include <Wire.h>
#include <Adafruit_GFX.h>
#include<Arduino.h>
#include"Embedded_Template_Library.h"
#include<etl/vector.h>
#include<etl/stack.h>
#include "Map.h"
#include "Tile.h"
#include "TileDirection.h"
#include "coord.h"
#include "CustomSerial.h"
#include "Movement.h"
#include "Pins.h"
#include "Encoder.h"

#define DEBUG_ALGORITHM 0
#define USING_SCREEN 1
#define DEBUG_MERGE 0
#define MOVEMENT 1
#define NO_ROBOT 0

Movement robot;

unsigned long iterations = 0;
bool hasArrived = false;

Map tilesMap = Map();
etl::vector<Tile, kMaxMapSize> tiles;
etl::vector<coord, kMaxMapSize> lastCheckpointVisitedCoords;
etl::vector<bool, kMaxMapSize> explored;
etl::vector<int, kMaxMapSize> distance;
etl::vector<coord, kMaxMapSize> previousPositions;
etl::stack<coord, kMaxMapSize> path;

constexpr TileDirection directions[] = {TileDirection::kUp, TileDirection::kDown, TileDirection::kLeft, TileDirection::kRight};

uint16_t robotOrientation = 0;
coord robotCoord = coord{0,0,0};

coord lastCheckpointCoord = robotCoord;

void updateLastCheckpoint(const coord& checkpointCoord) {
    lastCheckpointVisitedCoords = tilesMap.positions;
    lastCheckpointCoord = checkpointCoord;
}

void restartOnLastCheckpoint(const coord& checkpointCoord) {
    robotCoord = checkpointCoord;
    robotOrientation = 0;
    tilesMap.positions = lastCheckpointVisitedCoords;
    depthFirstSearch();
}

void turnAndMoveRobot(const int targetOrientation) {
    int difference = targetOrientation - robotOrientation;
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
    robot.goForward(robotOrientation, tiles[tilesMap.getIndex(robotCoord)].hasVictim());
    // If a victim was found, update the tile.
    if (robot.getVictimFound() && !tiles[tilesMap.getIndex(robotCoord)].hasVictim()) {
        tiles[tilesMap.getIndex(robotCoord)].setVictim();
    }
}

void followPath() {
    #if DEBUG_MERGE
    customPrint("path size: ");
    customPrintln(path.size());
    #endif
    while(!path.empty()) {
        const coord next = path.top();
        path.pop();
        #if DEBUG_MERGE
        customPrint("from: ");
        customPrint(robotCoord.x);
        customPrint(" ");
        customPrintln(robotCoord.y);
        customPrint("to: ");
        customPrint(next.x);
        customPrint(" ");
        customPrintln(next.y);
        #endif
        if(robotCoord == next) {
            continue;
        }
        if (next.x < robotCoord.x) {
            #if DEBUG_MERGE
            customPrintln("left");
            #endif
            #if USING_SCREEN
            // robot.screenPrint("left");
            #endif
            #if MOVEMENT
            turnAndMoveRobot(270);
            #endif
        } else if (next.x > robotCoord.x) {
            #if DEBUG_MERGE
            customPrintln("right");
            #endif
            #if USING_SCREEN
            // robot.screenPrint("right");
            #endif
            #if MOVEMENT
            turnAndMoveRobot(90);
            #endif
        } else if (next.y > robotCoord.y) {
            #if DEBUG_MERGE
            customPrintln("up");
            #endif
            #if USING_SCREEN
            // robot.screenPrint("up");
            #endif
            #if MOVEMENT
            turnAndMoveRobot(0);
            #endif
        } else if (next.y < robotCoord.y) {
            #if DEBUG_MERGE
            customPrintln("down");
            #endif
            #if USING_SCREEN
            // robot.screenPrint("down");
            #endif
            #if MOVEMENT
            turnAndMoveRobot(180);
            #endif
        }
        #if DEBUG_ALGORITHM
        customPrintln("robotOrientation: " + String(robotOrientation));
        customPrintln("robotCoord: " + String(robotCoord.x) + " " + String(robotCoord.y));
        #endif
        if (robot.wasBlackTile()) {
            tiles[tilesMap.getIndex(next)].setBlackTile();
            // robot.screenPrint("Black tile found " + String(robotCoord.x) + " " + String(robotCoord.y));
        } else if (robot.isBlueTile()) {
            robotCoord = next;
            tiles[tilesMap.getIndex(robotCoord)].weight_ = kBlueTileWeight;
            // robot.screenPrint("Blue tile found " + String(robotCoord.x) + " " + String(robotCoord.y));
        } else {
            robotCoord = next;
        }
    }
}

void dijsktra(const coord& start, const coord& end) {
    #if DEBUG_ALGORITHM
    customPrintln("End coord: " + String(end.x) + " " + String(end.y));
    #endif
    // empty path.
    // while (!path.empty()) {
    //     path.pop();
    // }
    path.clear();
    // initialize vectors.
    #if DEBUG_ALGORITHM 
    customPrintln("before distance");
    #endif
    for (int i = 0; i < tilesMap.positions.size(); ++i) {
        distance[i] = INT_MAX;
        explored[i] = false;
        previousPositions[i] = kInvalidPosition;
    }
    const int startIndex = tilesMap.getIndex(start);
    distance[startIndex] = 0;
    explored[startIndex] = true;
    // explore the map.
    coord currentCoord = start;
    int minDistance;
    #if DEBUG_ALGORITHM 
    customPrintln("before while"); //error in while
    #endif
    while (!explored[tilesMap.getIndex(end)]) {
        #if DEBUG_ALGORITHM
        customPrintln("Current tile: " + String(currentCoord.x) + " " + String(currentCoord.y));
        #endif
        // Get current tile.
        const int currentCoordIndex = tilesMap.getIndex(currentCoord);
        const Tile& currentTile = tiles[currentCoordIndex];
        // update adjecent tiles distances.
        for (const TileDirection& direction : directions) {
            const int staticDirection = static_cast<int>(direction);
            // check if there's an adjecent tile.
            if (currentTile.adjacentTiles_[staticDirection] != NULL) {
                const coord& adjacentCoord = currentTile.adjacentTiles_[staticDirection]->position_;
                const int adjacentCoordIndex = tilesMap.getIndex(adjacentCoord);
                #if DEBUG_ALGORITHM
                customPrintln("adjecent coord at: " + String(adjacentCoord.x) + " " + String(adjacentCoord.y));
                #endif
                // check if there's a wall between the two adjacent tiles and if there isn't a hole.
                if (!currentTile.hasWall(direction) && !tiles[adjacentCoordIndex].hasBlackTile()) {
                    const int weight = tiles[adjacentCoordIndex].weight_ + distance[currentCoordIndex];
                    // check if the new weight to visit the adjecent tile is less than the current weight.
                    if (weight < distance[adjacentCoordIndex]) {
                        #if DEBUG_ALGORITHM
                        customPrintln("updated weight: " + String(weight));
                        #endif
                        distance[adjacentCoordIndex] = weight;
                        previousPositions[adjacentCoordIndex] = currentCoord;
                    }
                }
            }
        }
        // find next tile.
        minDistance = INT_MAX;
        for (int i = 0; i < tilesMap.positions.size(); ++i) {
            const coord& current = tilesMap.positions[i];
            const int currentDistance = distance[tilesMap.getIndex(current)];
            if (currentDistance < minDistance && !explored[tilesMap.getIndex(current)]) {
                minDistance = currentDistance;
                currentCoord = current;
                #if DEBUG_ALGORITHM
                customPrintln("lowest distance: " + String(currentDistance) + " of " + String(current.x) + " " + String(current.y));
                #endif
            }
        }
        explored[tilesMap.getIndex(currentCoord)] = true;
    }
    #if DEBUG_ALGORITHM 
    customPrintln("before path");
    #endif
    // find and follow path.
    coord current = end;
    while (current != start) {
        path.push(current);
        current = previousPositions[tilesMap.getIndex(current)];
    }
    #if DEBUG_ALGORITHM 
    customPrintln("before followPath");
    #endif
    followPath();
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

void depthFirstSearch() {
    for (int i = 0; i < kMaxMapSize; ++i) {
        distance.push_back(INT_MAX);
        explored.push_back(false);
        previousPositions.push_back(kInvalidPosition);
    }
    Map visitedMap = Map();
    etl::stack<coord, kMaxMapSize> unvisited;
    Tile* currentTile;
    bool wall;
    bool alreadyConnected;
    coord nextTileCoord;
    TileDirection oppositeDirection;
    visitedMap.positions = lastCheckpointVisitedCoords;
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
        #if DEBUG_ALGORITHM
        customPrint("visitedMap size = ");
        customPrintln(visitedMap.positions.size());
        #endif
        if (visitedMap.getIndex(currentTileCoord) != kInvalidIndex) {
            continue;
        }
        #if DEBUG_ALGORITHM 
        customPrintln("before dijsktra"); 
        #endif
        #if USING_SCREEN
        // robot.screenPrint("robotCoord: " + String(robotCoord.x) + " " + String(robotCoord.y));
        // robot.screenPrint("Orientation: " + String(robotOrientation));
        // robot.screenPrint("CurrentTileCoord: " + String(currentTileCoord.x) + " " + String(currentTileCoord.y));
        #endif
        dijsktra(robotCoord, currentTileCoord);
        #if DEBUG_ALGORITHM || DEBUG_MERGE
        customPrint("currentTileCoord: ");
        customPrint(currentTileCoord.x);
        customPrint(" ");
        customPrintln(currentTileCoord.y);
        customPrint("unvisited size: ");
        customPrintln(unvisited.size());
        #endif
        #if USING_SCREEN
        // robot.screenPrint("robotCoord: " + String(robotCoord.x) + " " + String(robotCoord.y));
        // robot.screenPrint("Orientation: " + String(robotOrientation));
        // robot.screenPrint("CurrentTileCoord: " + String(currentTileCoord.x) + " " + String(currentTileCoord.y));
        #endif
        visitedMap.positions.push_back(currentTileCoord);
        if (robot.wasBlackTile()) {
            // robot.screenPrint("Black tile found");
            continue;
        }
        robotCoord = currentTileCoord; // Maybe not needed.
        currentTile = &tiles[tilesMap.getIndex(currentTileCoord)];
        //check for ramp
        if (robot.isRamp()) {
            // robot.screenPrint("Ramp found");
            currentTile->weight_ = kRampWeight;
            TileDirection direction;
            // check robots orientation to know the next Tile.
            switch (robotOrientation) {
                case 0:
                    nextTileCoord = coord{currentTileCoord.x, currentTileCoord.y + 1, currentTileCoord.z + robot.directionRamp()};
                    direction = TileDirection::kUp;
                    oppositeDirection = TileDirection::kDown;
                    break;
                case 90:
                    nextTileCoord = coord{currentTileCoord.x + 1, currentTileCoord.y, currentTileCoord.z + robot.directionRamp()};
                    direction = TileDirection::kRight;
                    oppositeDirection = TileDirection::kLeft;
                    break;
                case 180:
                    nextTileCoord = coord{currentTileCoord.x, currentTileCoord.y - 1, currentTileCoord.z + robot.directionRamp()};
                    direction = TileDirection::kDown;
                    oppositeDirection = TileDirection::kUp;
                    break;
                case 270:
                    nextTileCoord = coord{currentTileCoord.x - 1, currentTileCoord.y, currentTileCoord.z + robot.directionRamp()};
                    direction = TileDirection::kLeft;
                    oppositeDirection = TileDirection::kRight;
                    break;
            }
            // Creates a new tile if the next tile doesn't exist.
            if (tilesMap.getIndex(nextTileCoord) == kInvalidIndex) {
                tilesMap.positions.push_back(nextTileCoord);
                tiles[tilesMap.getIndex(nextTileCoord)] = Tile(nextTileCoord);
            }
            // Create a pointer to the next tile and asign its coordenate if it's a new Tile.
            Tile* nextTile = &tiles[tilesMap.getIndex(nextTileCoord)];
            if (nextTile->position_ == kInvalidPosition) { // maybe can be moved to the if statement above.
                nextTile->setPosition(nextTileCoord);
            }
            // Link the two adjacent Tiles.
            currentTile->addAdjacentTile(direction, nextTile, false);
            nextTile->addAdjacentTile(oppositeDirection, currentTile, false);
            if (visitedMap.getIndex(nextTileCoord) != kInvalidIndex) {
                continue;
            }
            unvisited.push(nextTileCoord);
            #if DEBUG_ALGORITHM || DEBUG_MERGE
            customPrint("nextTileCoord: ");
            customPrint(nextTileCoord.x);
            customPrint(" ");
            customPrintln(nextTileCoord.y);
            #endif
        } else {
            // check walls the 4 adjacent tiles.
            for (const TileDirection& direction : directions) {
                wall = false;
                switch(direction) {
                    case TileDirection::kRight:
                        nextTileCoord = coord{currentTileCoord.x + 1, currentTileCoord.y, currentTileCoord.z};
                        oppositeDirection = TileDirection::kLeft;
                        break;
                    case TileDirection::kUp:
                        nextTileCoord = coord{currentTileCoord.x, currentTileCoord.y + 1, currentTileCoord.z};
                        oppositeDirection = TileDirection::kDown;
                        break;
                    case TileDirection::kLeft:
                        nextTileCoord = coord{currentTileCoord.x - 1, currentTileCoord.y, currentTileCoord.z};
                        oppositeDirection = TileDirection::kRight;
                        break;
                    case TileDirection::kDown:
                        nextTileCoord = coord{currentTileCoord.x, currentTileCoord.y - 1, currentTileCoord.z};
                        oppositeDirection = TileDirection::kUp;
                        break;
                }
                // check if the tile has not been checked.
                if (currentTile->adjacentTiles_[static_cast<int>(direction)] == NULL) {
                    // check for a wall.
                    wall = robot.checkWallsDistances(direction, robotOrientation);
                    if (wall) {
                        #if USING_SCREEN
                        switch (direction)
                        {
                        case TileDirection::kUp:
                            // robot.screenPrint("Wall found up");
                            break;
                        case TileDirection::kDown:
                            // robot.screenPrint("Wall found down");
                            break;
                        case TileDirection::kLeft:
                            // robot.screenPrint("Wall found left");
                            break;
                        case TileDirection::kRight:
                            // robot.screenPrint("Wall found right");
                            break;
                        default:
                            break;
                        }
                        #endif
                    }
                    else {
                        #if USING_SCREEN
                        switch (direction)
                        {
                        case TileDirection::kUp:
                            // robot.screenPrint("No wall found up");
                            break;
                        case TileDirection::kDown:
                            // robot.screenPrint("No wall found down");
                            break;
                        case TileDirection::kLeft:
                            // robot.screenPrint("No wall found left");
                            break;
                        case TileDirection::kRight:
                            // robot.screenPrint("No wall found right");
                            break;
                        default:
                            break;
                        }
                        #endif
                    }
                    // Creates a new tile if the next tile doesn't exist.
                    if (tilesMap.getIndex(nextTileCoord) == kInvalidIndex) {
                        tilesMap.positions.push_back(nextTileCoord);
                        tiles[tilesMap.getIndex(nextTileCoord)] = Tile(nextTileCoord);
                    }
                    // create a pointer to the next tile and asign its coordenate if it's a new Tile.
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
                        if (visitedMap.getIndex(nextTileCoord) != kInvalidIndex) {
                            continue;
                        }
                        unvisited.push(nextTileCoord);
                        // #if DEBUG_ALGORITHM || DEBUG_MERGE
                        // customPrint("nextTileCoord: ");
                        // customPrint(nextTileCoord.x);
                        // customPrint(" ");
                        // customPrintln(nextTileCoord.y);
                        // #endif
                    }
                }
            }
        }
    }
    // robot.screenPrint("End of DFS");
    dijsktra(robotCoord, coord{0,0,0});
    #if DEBUG_ALGORITHM
    customPrintln("termino DFS");
    #endif
    loop();
}

void startAlgorithm() {
    tilesMap.positions.push_back(robotCoord);
    tiles[tilesMap.getIndex(robotCoord)] = Tile(robotCoord);
    #if DEBUG_ALGORITHM
    customPrintln("Start algorithm");
    #endif
    depthFirstSearch();
}

void setup(){
    Serial.begin(115200);
    #if USING_SCREEN
    // if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    //     customPrintln(F("SSD1306 allocation failed"));
    //     for(;;);
    // }
    #endif
    #if DEBUG_ALGORITHM
    customPrintln("Serial ready");
    #endif
    robot.setup();
    startAlgorithm();

    // Serial.println(1);
    // bool flag = false;
    // robot.screenPrint("nadota");
    // while(flag == false){
    //   if (Serial.available() > 0) {
    //       char input = Serial.read();
    //       robot.screenPrint("input: " + String(input));
    //       flag = true;
    //   }
    // }
}
    
void loop() {
    #if DEBUG_ALGORITHM
    customPrintln("Loop");
    delay(1000);
    #endif
    
    // Serial.println(1);
    // bool flag = false;
    // robot.screenPrint("nadota");
    // while(flag == false){
    //   if (Serial.available() > 0) {
    //       char input = Serial.read();
    //       robot.screenPrint("input: " + String(input));
    //       flag = true;
    //   }
    // }
    // delay(1000);

    // for (int robotOrientation = 0; robotOrientation < 360; robotOrientation += 90){
    //     customPrintln("Orientation: " + String(robotOrientation));
    //     for (TileDirection direction : directions) {
    //         if (robot.checkWallsDistances(direction, robotOrientation)) {
    //             switch (direction)
    //             {
    //             case TileDirection::kUp:
    //                 robot.screenPrint("Wall found up");
    //                 break;
    //             case TileDirection::kDown:
    //                 robot.screenPrint("Wall found down");
    //                 break;
    //             case TileDirection::kLeft:
    //                 robot.screenPrint("Wall found left");
    //                 break;
    //             case TileDirection::kRight:
    //                 robot.screenPrint("Wall found right");
    //                 break;
    //             default:
    //                 break;
    //             }
    //         }
    //         else {
    //             switch (direction)
    //             {
    //             case TileDirection::kUp:
    //                 robot.screenPrint("No wall found up");
    //                 break;
    //             case TileDirection::kDown:
    //                 robot.screenPrint("No wall found down");
    //                 break;
    //             case TileDirection::kLeft:
    //                 robot.screenPrint("No wall found left");
    //                 break;
    //             case TileDirection::kRight:
    //                 robot.screenPrint("No wall found right");
    //                 break;
    //             default:
    //                 break;
    //             }
    //         }
    //     }
    // }
    /* robot.goForward(0);
    delay(100); */
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