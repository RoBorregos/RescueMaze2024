// make sure that we do not rely on the STL.
#define ETL_NO_STL

#include "Map.h"

using namespace etl;

constexpr TileDirection directions[] = {TileDirection::kUp, TileDirection::kDown, TileDirection::kLeft, TileDirection::kRight};

int robotOrientation = 0;
coord robotCoord = coord{1,1,1};

void turnRobot(const int& targetOrientation) {
    int difference = targetOrientation - robotOrientation;
    if (difference == 0) {
        return;
    }
    if (difference == 90 || difference == -270) {
        // turnRight(targetOrientation);
        robotOrientation = (robotOrientation + 90) % 360;
    } else if (difference == -90 || difference == 270) {
        // turnLeft(targetOrientation);
        robotOrientation = (robotOrientation + 270) % 360;
    } else if (difference == 180 || difference == -180) {
        // turnRight(targetOrientation);
        // turnRight(targetOrientation);
        robotOrientation = (robotOrientation + 180) % 360;
    }
}

void followPath(etl::stack<coord, kMaxMapSize>& path) {
    while(!path.empty()) {
        coord next = path.top();
        path.pop();
        if (next.x > robotCoord.x) {
            // turnRobot(270);
        } else if (next.x < robotCoord.x) {
            // turnRobot(90);
        } else if (next.y > robotCoord.y) {
            // turnRobot(0);
        } else if (next.y < robotCoord.y) {
            // turnRobot(180);
        }
        // goForward(robotOrientation);
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
        //push_back
        distance[i] = INT_MAX;
        explored[i] = false;
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
    followPath(path);
}

bool checkForWall(const etl::vector<etl::vector<char, kMaxMapSize>, kMaxMapSize>& maze, const TileDirection& direction, const coord& currentTileCoord) {
    switch(direction) {
        case TileDirection::kRight:
            return maze[currentTileCoord.y][currentTileCoord.x + 1] == '#';
        case TileDirection::kUp:
            return maze[currentTileCoord.y + 1][currentTileCoord.x] == '#';
        case TileDirection::kLeft:
            return maze[currentTileCoord.y][currentTileCoord.x - 1] == '#';
        case TileDirection::kDown:
            return maze[currentTileCoord.y - 1][currentTileCoord.x] == '#';
    }
}

// void printMap(unordered_map<coord, Tile>& map){
//     for (auto it = map.begin(); it != map.end(); ++it) {
//         cout << "Tile: " << it->first.x << " " << it->first.y << " " << it->first.z << endl;
//         for (int i = 0; i < 4; ++i) {
//             if (it->second.adjacentTiles_[i] != NULL) {
//                 cout << "Adjacent Tile: " << it->second.adjacentTiles_[i]->position_.x << " " << it->second.adjacentTiles_[i]->position_.y << " " << it->second.adjacentTiles_[i]->position_.z << endl;
//             }
//         }
//     }
//     return;
// }

// int checkRamp(const etl::vector<etl::vector<char, kMaxMapSize>, kMaxMapSize>& maze, const TileDirection& direction, const coord& currentTileCoord) {
//     int z = currentTileCoord.z;
//     switch(direction) {
//         case TileDirection::kRight:
//             if (maze[currentTileCoord.y][currentTileCoord.x] == 'r') {
//                 z++;
//             } else if (currentTileCoord.x+2 <= 10 && maze[currentTileCoord.y][currentTileCoord.x+2] == 'l' && currentTileCoord.z > 1) {
//                 z--;
//             }
//             break;
//         case TileDirection::kUp:
//             if (maze[currentTileCoord.y][currentTileCoord.x] == 'u') {
//                 z++;
//             } else if (currentTileCoord.y+2 <= 6 && maze[currentTileCoord.y+2][currentTileCoord.x] == 'd' && currentTileCoord.z > 1) {
//                 z--;
//             }
//             break;
//         case TileDirection::kLeft:
//             if (maze[currentTileCoord.y][currentTileCoord.x] == 'l') {
//                 z++;
//             } else if (currentTileCoord.x-2 >= 0 && maze[currentTileCoord.y][currentTileCoord.x-2] == 'r' && currentTileCoord.z > 1) {
//                 z--;
//             }
//             break;
//         case TileDirection::kDown:
//             if (maze[currentTileCoord.y][currentTileCoord.x] == 'd') {
//                 z++;
//             } else if (currentTileCoord.y-2 >= 0 && maze[currentTileCoord.y-2][currentTileCoord.x] == 'u' && currentTileCoord.z > 1) {
//                 z--;
//             }
//             break;
//     }
//     return z;
// }

void depthFirstSearch(Map& tilesMap, etl::vector<Tile, kMaxMapSize>& tiles) {
    Serial.println("inicio DFS");
    Map visitedMap = Map();
    etl::vector<bool, kMaxMapSize> visited;
    etl::stack<coord, kMaxMapSize> unvisited;
    Tile* currentTile;
    tilesMap.positions.push_back(robotCoord);
    tiles[tilesMap.getIndex(robotCoord)] = Tile(robotCoord);
    unvisited.push(robotCoord);
    bool wall;
    bool alreadyConnected;
    bool visitedFlag;
    coord nextTileCoord;
    TileDirection oppositeDirection;
    // explore the map.
    while (unvisited.size() != 256){
        delay(1000);
        // get the next tile to explore.
        coord currentTileCoord = unvisited.top();
        unvisited.pop();
        // check if the tile has been visited.
        visitedFlag = false;
        for (int i=0; i<visitedMap.positions.size(); i++) {
            if (visitedMap.positions[i] == currentTileCoord) {
                visitedFlag = true;
                break;
            }
        }
        if (visitedFlag) {
            continue;
        }
        dijsktra(robotCoord, currentTileCoord, tilesMap, tiles);
        Serial.print("currentTileCoord: ");
        Serial.print(currentTileCoord.x);
        Serial.print(" ");
        Serial.println(currentTileCoord.y);
    Serial.println(unvisited.size());
        robotCoord = currentTileCoord;
        visitedMap.positions.push_back(currentTileCoord);
        visited.push_back(true);
        // check walls the 4 adjacent tiles.
        for (const TileDirection& direction : directions) {
            wall = false;
            switch(direction) {
                case TileDirection::kRight:
                    nextTileCoord = coord{currentTileCoord.x+2,currentTileCoord.y,1}; // checkRamp(direction);
                    currentTile = &tiles[tilesMap.getIndex(currentTileCoord)];
                    oppositeDirection = TileDirection::kLeft;
                    break;
                case TileDirection::kUp:
                    nextTileCoord = coord{currentTileCoord.x,currentTileCoord.y+2,1}; // checkRamp(direction);
                    currentTile = &tiles[tilesMap.getIndex(currentTileCoord)];
                    oppositeDirection = TileDirection::kDown;
                    break;
                case TileDirection::kLeft:
                    nextTileCoord = coord{currentTileCoord.x-2,currentTileCoord.y,1}; // checkRamp(direction);
                    currentTile = &tiles[tilesMap.getIndex(currentTileCoord)];
                    oppositeDirection = TileDirection::kRight;
                    break;
                case TileDirection::kDown:
                    nextTileCoord = coord{currentTileCoord.x,currentTileCoord.y-2,1}; // checkRamp(direction);
                    currentTile = &tiles[tilesMap.getIndex(currentTileCoord)];
                    oppositeDirection = TileDirection::kUp;
                    break;
            }
            // check if the tile has not been checked.
            if (currentTile->adjacentTiles_[static_cast<int>(direction)] == NULL) {
                // check for a wall.
                wall = true; // por ahora
                // wall = checkForWall(direction);
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
                    for (int i=0; i<visitedMap.positions.size(); i++) {
                        if (visitedMap.positions[i] == nextTileCoord) {
                            visitedFlag = true;
                            break;
                        }
                    }
                    if(!visitedFlag) {
                        Serial.print("nextTileCoord: ");
                        Serial.print(nextTileCoord.x);
                        Serial.print(" ");
                        Serial.println(nextTileCoord.y);
                        delay(500);
                        unvisited.push(nextTileCoord);
                    }
                }
            }
        }
        Serial.println(unvisited.size());
    }
    Serial.println("termino BFS");
}

void startAlgorithm() {
    Map tilesMap = Map();
    etl::vector<Tile, kMaxMapSize> tiles;
    Serial.println("startAlgorithm");
    depthFirstSearch(tilesMap, tiles);
}

void setup() {
    // Serial.begin(9600);
    // Map tilesMap = Map();
    // etl::vector<Tile, kMaxMapSize> tiles;
    // Serial.println("setup");
    // depthFirstSearch(tilesMap, tiles);
}
void loop() {
    Serial.println("termino");
    delay(5000);
}