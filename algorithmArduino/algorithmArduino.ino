// make sure that we do not rely on the STL.
#define ETL_NO_STL

// #include<etl/unordered_map.h>

#include "Map.h"

using namespace etl;

constexpr TileDirection directions[] = {TileDirection::kUp, TileDirection::kDown, TileDirection::kLeft, TileDirection::kRight};

etl::vector<etl::vector<char, kMaxMapSize>, kMaxMapSize> maze = {
        // {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
        // {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'}, // 1.
        // {'#', ' ', '#', ' ', '#', ' ', '#', '#', '#', '#', '#'},
        // {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'}, // 3.
        // {'#', ' ', '#', '#', '#', ' ', '#', ' ', '#', '#', '#'},
        // {'#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#'}, // 5.
        // {'#', ' ', '#', '#', '#', ' ', '#', '#', ' ', ' ', '#'},
        // {'#', ' ', '#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'}, // 7.
        // {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#'},
        // {'#', ' ', ' ', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#'}, // 9.
        // {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'}
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
        {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'}, // 1.
        {'#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#'},
        {'#', ' ', ' ', 'r', '#', ' ', '#', 'l', ' ', ' ', '#'}, // 3.
        {'#', ' ', '#', '#', '#', ' ', '#', '#', '#', ' ', '#'},
        {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'}, // 5.
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'}
};
etl::vector<etl::vector<char, kMaxMapSize>, kMaxMapSize> mazeSecondLevel = {
    {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
    {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'}, // 1.
    {'#', ' ', ' ', ' ', '#', '#', '#', ' ', ' ', ' ', '#'},
    {'#', ' ', ' ', 'r', ' ', ' ', ' ', 'l', ' ', ' ', '#'}, // 3.
    {'#', ' ', ' ', ' ', '#', '#', '#', ' ', ' ', ' ', '#'},
    {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'}, // 5.
    {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'}
};

// void printMaze(const etl::vector<etl::vector<char,kMaxMapSize>,kMaxMapSize>& mazeToPrint) {
//     for (int i = 0; i < mazeToPrint.size(); ++i) {
//         for (int j = 0; j < mazeToPrint[i].size(); j++) {
//             etl::cout << mazeToPrint[i][j] << " ";
//         }
//         cout << endl;
//     }
// }

void printPath(etl::stack <coord, kMaxMapSize> path) {
    etl::vector<etl::vector<char, kMaxMapSize>, kMaxMapSize> newMaze = maze;
    int lastX = path.top().x;
    int lastY = path.top().y;
    path.pop();
    newMaze[lastY][lastX] = 'S';
    while (!path.empty()) {
        if (path.top().x == lastX) {
            if (path.top().y > lastY) {
                newMaze[lastY+1][lastX] = '/';
            } else {
                newMaze[lastY-1][lastX] = '^';
            }
        } else {
            if (path.top().x>lastX) {
                newMaze[lastY][lastX+1] = '>';
            } else {
                newMaze[lastY][lastX-1] = '<';
            }
        }
        lastX = path.top().x;
        lastY = path.top().y;
        path.pop();
    }
    newMaze[lastY][lastX] = 'E';
    // printMaze(newMaze);
    return;
}

void dijsktra(const coord& start, const coord& end, Map& map) {
    Map explored = Map("bool");
    // etl::unordered_map<coord, bool, kMaxMapSize> explored;
    Map distance = Map("int");
    // etl::unordered_map<coord, int, kMaxMapSize> distance;
    Map previousPositions = Map("coord");
    // etl::unordered_map<coord, coord, kMaxMapSize> previousPositions;
    etl::stack<coord, kMaxMapSize> path;
    // initialize distance.
    for (int i = map.indexes.size() - 1; i >= 0; --i) {
        distance.addInt(INT_MAX, map.indexes[i]);
        // distance[map.tiles[i].position_] = INT_MAX;
        explored.addBool(false, map.indexes[i]);
        // explored[it->first] = false;
    }
    distance.setInt(0,start);
    explored.setBool(true,start);

    // explore the map.
    coord currentCoord = start;
    int minDistance;
    while (!explored.getBool(end)) {
        // update distance.
        for (int i = 0; i < 4; i++) {
            const TileDirection& direction = directions[i];
            const Tile* currentTile = map.getTile(currentCoord);
            const coord& adjacentCoord = currentTile->adjacentTiles_[static_cast<int>(direction)]->position_;
            // check if there's an adjecent tile and there's no wall.
            //TODO: check if the tile to explore is black
            if (currentTile->adjacentTiles_[static_cast<int>(direction)] != NULL && !currentTile->hasWall(direction)) {
                const int weight = currentTile->weights_[static_cast<int>(direction)] + distance.getInt(currentCoord);
                // check if the new weight to visit the adjecent tile is less than the current weight.
                if (weight < distance.getInt(adjacentCoord)) {
                    distance.setInt(weight, adjacentCoord);
                    previousPositions.setCoord(currentCoord, adjacentCoord);
                }
            }
        }
        // find next tile.
        minDistance = INT_MAX;
        for (int i = map.indexes.size() - 1; i >= 0; --i) {
            const coord& current = map.indexes[i];
            const int currentDistance = distance.getInt(current);
            if (currentDistance < minDistance && !explored.getBool(current)) {
                minDistance = currentDistance;
                currentCoord = current;
            }
        }
        explored.setBool(true, currentCoord);
    }
    // find path.
    coord current = end;
    while (current != start) {
        path.push(current);
        current = previousPositions.getCoord(current);
    }
    path.push(start);
    // print path.
    printPath(path);
    return;
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

int checkRamp(const etl::vector<etl::vector<char, kMaxMapSize>, kMaxMapSize>& maze, const TileDirection& direction, const coord& currentTileCoord) {
    int z = currentTileCoord.z;
    switch(direction) {
        case TileDirection::kRight:
            if (maze[currentTileCoord.y][currentTileCoord.x] == 'r') {
                z++;
            } else if (currentTileCoord.x+2 <= 10 && maze[currentTileCoord.y][currentTileCoord.x+2] == 'l' && currentTileCoord.z > 1) {
                z--;
            }
            break;
        case TileDirection::kUp:
            if (maze[currentTileCoord.y][currentTileCoord.x] == 'u') {
                z++;
            } else if (currentTileCoord.y+2 <= 6 && maze[currentTileCoord.y+2][currentTileCoord.x] == 'd' && currentTileCoord.z > 1) {
                z--;
            }
            break;
        case TileDirection::kLeft:
            if (maze[currentTileCoord.y][currentTileCoord.x] == 'l') {
                z++;
            } else if (currentTileCoord.x-2 >= 0 && maze[currentTileCoord.y][currentTileCoord.x-2] == 'r' && currentTileCoord.z > 1) {
                z--;
            }
            break;
        case TileDirection::kDown:
            if (maze[currentTileCoord.y][currentTileCoord.x] == 'd') {
                z++;
            } else if (currentTileCoord.y-2 >= 0 && maze[currentTileCoord.y-2][currentTileCoord.x] == 'u' && currentTileCoord.z > 1) {
                z--;
            }
            break;
    }
    return z;
}

void depthFirstSearch(Map& map) {
    Map visited = Map("bool");
    etl::stack<coord, kMaxMapSize> unvisited;
    Tile* currentTile;
    coord robotCoord = coord{1,1,1};
    map.setTile(Tile(robotCoord), robotCoord);
    unvisited.push(robotCoord);
    bool wall;
    bool alreadyConnected;
    bool visitedFlag;
    coord nextTileCoord;
    TileDirection oppositeDirection;
    // explore the map.
    while (!unvisited.empty()) {
        // get the next tile to explore.
        coord currentTileCoord = unvisited.top();
        unvisited.pop();
        // check if the tile has been visited.
        visitedFlag = false;
        for (int i=0; i<visited.indexes.size(); i++) {
            if (visited.indexes[i] == currentTileCoord) {
                visitedFlag = true;
                break;
            }
        }
        if (visitedFlag) {
            continue;
        }
        // go to tile. TODO
        dijsktra(robotCoord, currentTileCoord, map);
        robotCoord = currentTileCoord;
        visited.setBool(true, currentTileCoord);
        // check walls the 4 adjacent tiles.
        // cout<<currentTileCoord.x<<" "<<currentTileCoord.y<<" "<<currentTileCoord.z<<endl;
        for (const TileDirection& direction : directions) {
            wall = false;
            switch(direction) {
                case TileDirection::kRight:
                    nextTileCoord = coord{currentTileCoord.x+2,currentTileCoord.y,checkRamp(maze, direction, currentTileCoord)};
                    currentTile = map.getTile(currentTileCoord);
                    oppositeDirection = TileDirection::kLeft;
                    break;
                case TileDirection::kUp:
                    nextTileCoord = coord{currentTileCoord.x,currentTileCoord.y+2,checkRamp(maze, direction, currentTileCoord)};
                    currentTile = map.getTile(currentTileCoord);
                    oppositeDirection = TileDirection::kDown;
                    break;
                case TileDirection::kLeft:
                    nextTileCoord = coord{currentTileCoord.x-2,currentTileCoord.y,checkRamp(maze, direction, currentTileCoord)};
                    currentTile = map.getTile(currentTileCoord);
                    oppositeDirection = TileDirection::kRight;
                    break;
                case TileDirection::kDown:
                    nextTileCoord = coord{currentTileCoord.x,currentTileCoord.y-2,checkRamp(maze, direction, currentTileCoord)};
                    currentTile = map.getTile(currentTileCoord);
                    oppositeDirection = TileDirection::kUp;
                    break;
            }
            // check if the tile has not been checked.
            if (currentTile->adjacentTiles_[static_cast<int>(direction)] == NULL) {
                // check for a wall.
                if (currentTileCoord.z == 1 && nextTileCoord.z == 1) {
                    wall = checkForWall(maze, direction, currentTileCoord);
                } else {
                    wall = checkForWall(mazeSecondLevel, direction, currentTileCoord);
                }
                //cout<<nextTileCoord.x<<" "<<nextTileCoord.y<<" "<<nextTileCoord.z<<" "<<wall<<endl;
                // create a pointer to the next tile and asign its coordenate if it's a new Tile.
                Tile* nextTile = map.getTile(nextTileCoord);
                if (nextTile->position_ == kInvalidPosition) {
                     nextTile->setPosition(nextTileCoord);
                    // map[nextTileCoord].setPosition(nextTileCoord);
                }
                // Link the two adjacent Tiles.
                currentTile->addAdjacentTile(direction, nextTile, wall);
                nextTile->addAdjacentTile(oppositeDirection, currentTile, wall);
                // Check if there's a wall between the two adjacent Tiles.
                if (!wall) {
                    // maze[currentTileCoord.y][currentTileCoord.x] = 'o';
                    // maze[nextTileCoord.y][nextTileCoord.x] = 'o';
                    // if the tile has not been visited, add it to the queue.
                    visitedFlag = false;
                    for (int i=0; i<visited.indexes.size(); i++) {
                        if (visited.indexes[i] == nextTileCoord) {
                            visitedFlag = true;
                            break;
                        }
                    }
                    if(!visitedFlag) {
                        unvisited.push(nextTileCoord);
                    }
                }
            }
        }
    }
    return;
}

void setup() {
    Serial.begin(9600);
    Map map = Map("Tile");
    depthFirstSearch(map); // hash problem.
}
void loop() {

}