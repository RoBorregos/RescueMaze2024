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
#define USING_SCREEN 0
#define DEBUG_MERGE 0
#define MOVEMENT 1
#define NO_ROBOT 0
#define DEBUG_ONLINE 0

#if DEBUG_ONLINE
#include <WiFi.h>
#include <WiFiUdp.h>
const char* ssid = "RoBorregos2";
const char* password = "RoBorregos2024";
const char* udpServerIP = "192.168.0.123"; // Replace with your Python script's IP address
const int udpServerPort = 1;
WiFiUDP udp;
#endif

Movement robot;

unsigned long iterations = 0;
bool hasArrived = false;

Map tilesMap = Map();
Map visitedMap = Map();

etl::vector<Tile, kMaxMapSize> tiles;
etl::vector<bool, kMaxMapSize> explored;
etl::vector<int, kMaxMapSize> distance;
etl::vector<coord, kMaxMapSize> previousPositions;
etl::stack<coord, kMaxMapSize> path;
etl::stack<coord, kMaxMapSize> unvisited;

etl::vector<coord, kMaxMapSize> lastCheckpointCoords;
etl::vector<Tile, kMaxMapSize> lastCheckpointTiles;
etl::vector<coord, kMaxMapSize> lastCheckpointVisited;
etl::stack<coord, kMaxMapSize> lastCheckpointUnvisited;
coord endOfPath;

constexpr TileDirection directions[] = {TileDirection::kUp, TileDirection::kDown, TileDirection::kLeft, TileDirection::kRight};

uint16_t robotOrientation = 0;
coord robotCoord = coord{0,0,0};

coord lastCheckpointCoord = robotCoord;

long long int timeAtStart;
int kSevenMinutes = 420000;

bool firstRun = true;
bool isLackOfProgress = false;

void onIdle() {
    while (true) {
        isLackOfProgress = false;
        robot.resetLackOfProgress();
        robot.screenPrint("Idle");
        if (digitalRead(Pins::buttonPin) == LOW) {
            robot.screenPrint("Loading ...");
            delay(kOneSecInMs);
            long long int time = millis();
            while (millis() - time < 500) {
                if (digitalRead(Pins::buttonPin) == LOW) {
                    robot.calibrateColors();
                    break;      
                }
            }
            while (digitalRead(Pins::buttonPin) == HIGH) {
                robot.screenPrint("Press the button to start");
            }
            if (firstRun == true) {
                firstRun = false;
                startAlgorithm();
            } else {
                robot.screenPrint("Restarting algorithm");
                delay(kOneSecInMs);
                restartOnLastCheckpoint();
                // startAlgorithm();
            }
        }
    }
}

void updateLastCheckpoint() {
    lastCheckpointCoords = tilesMap.positions;
    lastCheckpointCoord = robotCoord;
    lastCheckpointTiles = tiles;
    lastCheckpointVisited = visitedMap.positions;
    lastCheckpointUnvisited = unvisited; 
    // Add coord to which I am going and didn't get there too.
    if (endOfPath != robotCoord) {
        lastCheckpointUnvisited.push(endOfPath);
    
    }
}

void restartOnLastCheckpoint() {
    robotCoord = lastCheckpointCoord;
    robotOrientation = 0;
    tilesMap.positions = lastCheckpointCoords;
    tiles = lastCheckpointTiles;
    visitedMap.positions = lastCheckpointVisited;
    unvisited = lastCheckpointUnvisited;
    #if DEBUG_ONLINE
    for (int i = 0; i < tilesMap.positions.size(); ++i) {
        customPrintln("TileMap: " + String(tilesMap.positions[i].x) + " " + String(tilesMap.positions[i].y) + " " + String(tilesMap.positions[i].z));
    }
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("TilesMap size: " + String(tilesMap.positions.size()));
    udp.endPacket();
    for (int i = 0; i < tiles.size(); ++i) {
        customPrintln("Tile: " + String(tiles[i].position_.x) + " " + String(tiles[i].position_.y) + " " + String(tiles[i].position_.z));
    }
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("visited: " + String(visitedMap.positions.size()));
    udp.endPacket();
    for (int i = 0; i < visitedMap.positions.size(); ++i) {
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("Visited: " + String(visitedMap.positions[i].x) + " " + String(visitedMap.positions[i].y));
        udp.endPacket();
        customPrintln("Visited: " + String(visitedMap.positions[i].x) + " " + String(visitedMap.positions[i].y));
    }
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("unvisited: " + String(unvisited.size()));
    udp.endPacket();
    while (!unvisited.empty()) {
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("unvisited: " + String(unvisited.top().x) + " " + String(unvisited.top().y));
        udp.endPacket();
        unvisited.pop();
    }
    unvisited = lastCheckpointUnvisited;
    #endif
    // Check if the robot hasn't detected a checkpoint.
    if (tilesMap.positions.size() == 0 && tiles.size() == 0) {
        tilesMap.positions.push_back(robotCoord);
        tiles.push_back(Tile(robotCoord));
    } else {
        // tiles[tilesMap.getIndex(robotCoord)] = Tile(robotCoord);
        // Make the four adjacent tiles of the last checkpoint to be NULL.
        for (const TileDirection& direction : directions) {
            const int staticDirection = static_cast<int>(direction);
            if (tiles[tilesMap.getIndex(robotCoord)].adjacentTiles_[staticDirection] != NULL) {
                tiles[tilesMap.getIndex(robotCoord)].adjacentTiles_[staticDirection] = NULL;
            }
        }
        // Check unvisited tiles adjecencies, if they're not visited make them NULL.
        while (!unvisited.empty()) {
            coord currentTileCoord = unvisited.top();
            unvisited.pop();
            for (const TileDirection& direction : directions) {
                const int staticDirection = static_cast<int>(direction);
                if (tiles[tilesMap.getIndex(currentTileCoord)].adjacentTiles_[staticDirection] != NULL) {
                    if (visitedMap.getIndex(tiles[tilesMap.getIndex(currentTileCoord)].adjacentTiles_[staticDirection]->position_) == kInvalidIndex) {
                        tiles[tilesMap.getIndex(currentTileCoord)].adjacentTiles_[staticDirection] = NULL;
                    }
                }
            }
        }
        unvisited = lastCheckpointUnvisited;
    }
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
    // If button was pressed while turning, set idle state.
    if (robot.getLackOfProgress()) {
        #if debug_algorithm
        customPrintln("Lack of progress");
        #endif
        isLackOfProgress = true;
        return;
    }
    // Check how to move (ramp/ground).
    if (robot.isRamp()) {
        robot.rampMovement(robotOrientation);
    } else {
        robot.goForward(robotOrientation, tiles[tilesMap.getIndex(robotCoord)].hasVictim());
    }
    // If button was pressed while going forward, set idle state.
    if (robot.getLackOfProgress()) {
        #if debug_algorithm
        customPrintln("Lack of progress");
        #endif
        isLackOfProgress = true;
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
        if(isLackOfProgress) {
            break;
        }
        if (robot.wasBlackTile()) {
            tiles[tilesMap.getIndex(next)].setBlackTile();
        } else if (robot.isBlueTile()) {
            robotCoord = next;
            tiles[tilesMap.getIndex(robotCoord)].weight_ = kBlueTileWeight;
        } else if (robot.isCheckpointTile()) {
            #if DEBUG_ONLINE
            udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("Checkpoint found at: " + String(next.x) + " " + String(next.y));
            udp.endPacket();
            #endif
            robotCoord = next;
            tiles[tilesMap.getIndex(robotCoord)].setCheckpoint();
            updateLastCheckpoint();
        } else {
            robotCoord = next;
        }
        // If a victim was found, update the tile.
        if (robot.getVictimFound() && !tiles[tilesMap.getIndex(robotCoord)].hasVictim()) {
            tiles[tilesMap.getIndex(robotCoord)].setVictim();
        }
    }
}

void dijsktra(const coord& start, const coord& end) {
    endOfPath = end;
    #if DEBUG_ALGORITHM
    customPrintln("End coord: " + String(end.x) + " " + String(end.y));
    #endif
    // empty path.
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
    Tile* currentTile;
    bool wall;
    bool alreadyConnected;
    coord nextTileCoord;
    TileDirection oppositeDirection;
    #if DEBUG_ONLINE
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("Starting at: " + String(robotCoord.x) + " " + String(robotCoord.y) + " " + String(robotCoord.z));
    udp.endPacket();
    #endif
    unvisited.push(robotCoord);
    #if DEBUG_ALGORITHM
    customPrintln("inicio DFS");
    #endif
    // explore the map.
    while (!unvisited.empty()) {
        if (millis() - timeAtStart > kSevenMinutes) {
            break;
        }
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
        #if DEBUG_ONLINE
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("Exploring: " + String(currentTileCoord.x) + " " + String(currentTileCoord.y) + " " + String(currentTileCoord.z));
        udp.endPacket();
        #endif
        #if DEBUG_ALGORITHM 
        customPrintln("before dijsktra"); 
        #endif
        #if USING_SCREEN
        // robot.screenPrint("robotCoord: " + String(robotCoord.x) + " " + String(robotCoord.y));
        // robot.screenPrint("Orientation: " + String(robotOrientation));
        // robot.screenPrint("CurrentTileCoord: " + String(currentTileCoord.x) + " " + String(currentTileCoord.y));
        #endif
        dijsktra(robotCoord, currentTileCoord);
        if (isLackOfProgress) {
            break;
        }
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
                    #if DEBUG_ONLINE
                    // udp.beginPacket(udpServerIP, udpServerPort);
                    // udp.print("Connecting wall at: " + String(nextTileCoord.x) + " " + String(nextTileCoord.y));
                    // udp.endPacket();
                    #endif
                    // check for a wall.
                    wall = robot.checkWallsDistances(direction, robotOrientation);
                    if (wall) {
                        #if USING_SCREEN
                        switch (direction)
                        {
                        case TileDirection::kUp:
                            robot.screenPrint("Wall found up");
                            break;
                        case TileDirection::kDown:
                            robot.screenPrint("Wall found down");
                            break;
                        case TileDirection::kLeft:
                            robot.screenPrint("Wall found left");
                            break;
                        case TileDirection::kRight:
                            robot.screenPrint("Wall found right");
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
                            robot.screenPrint("No wall found up");
                            break;
                        case TileDirection::kDown:
                            robot.screenPrint("No wall found down");
                            break;
                        case TileDirection::kLeft:
                            robot.screenPrint("No wall found left");
                            break;
                        case TileDirection::kRight:
                            robot.screenPrint("No wall found right");
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
                        // if the tile has not been visited, add it to the stack.
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
    if (isLackOfProgress) {
        #if DEBUG_ALGORITHM
        customPrintln("returned all the way");
        #endif
        return;
    }
    robot.screenPrint("End of DFS");
    dijsktra(robotCoord, coord{0,0,0});
    #if DEBUG_ALGORITHM
    customPrintln("termino DFS");
    #endif
    loop();
}

void startAlgorithm() {
    // Initialize dijkstra's vectors
    for (int i = 0; i < kMaxMapSize; ++i) {
        distance.push_back(INT_MAX);
        explored.push_back(false);
        previousPositions.push_back(kInvalidPosition);
    }
    tilesMap.positions.push_back(robotCoord);
    // tiles[tilesMap.getIndex(robotCoord)] = Tile(robotCoord);
    tiles.push_back(Tile(robotCoord));
    #if DEBUG_ALGORITHM
    customPrintln("Start algorithm");
    #endif
    timeAtStart = millis();
    robot.screenPrint("Start algorithm");
    depthFirstSearch();
}

void setup(){
    Serial.begin(115200);
    #if DEBUG_ALGORITHM
    customPrintln("Serial ready");
    #endif
    #if DEBUG_ONLINE
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(kOneSecInMs);
        customPrintln("Connecting to WiFi...");
    }
    customPrintln("Connected to WiFi");
    udp.begin(udpServerPort);
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("Connected to WiFi");
    udp.endPacket();
    #endif
    robot.setup();
    onIdle();
    // robot.moveMotors(MovementState::kForward, 0, 1);
}
    
void loop() {
    #if DEBUG_ALGORITHM
    customPrintln("Loop");
    delay(1000);
    #endif
}