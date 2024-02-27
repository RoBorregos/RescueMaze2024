#include<iostream>
#include<vector>
#include<unordered_map>
#include<stack>
#include<climits>

using namespace std;

#include "Tile.cpp"

constexpr TileDirection directions[] = {TileDirection::kUp, TileDirection::kDown, TileDirection::kLeft, TileDirection::kRight};

vector<vector<char>> maze = {
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
vector<vector<char>> mazeSecondLevel = {
    {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
    {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'}, // 1.
    {'#', ' ', ' ', ' ', '#', '#', '#', ' ', ' ', ' ', '#'},
    {'#', ' ', ' ', 'r', ' ', ' ', ' ', 'l', ' ', ' ', '#'}, // 3.
    {'#', ' ', ' ', ' ', '#', '#', '#', ' ', ' ', ' ', '#'},
    {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'}, // 5.
    {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'}
};

void printMaze(const vector<vector<char>>& mazeToPrint) {
    for (int i = 0; i < mazeToPrint.size(); ++i) {
        for (int j = 0; j < mazeToPrint[i].size(); j++) {
            cout << mazeToPrint[i][j] << " ";
        }
        cout << endl;
    }
}

void printPath(stack <coord> path) {
    vector<vector<char>> newMaze = maze;
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
    printMaze(newMaze);
    return;
}

void dijsktra(const coord& start, const coord& end, const unordered_map<coord,Tile>& map) {
    unordered_map<coord, bool> explored;
    unordered_map<coord, int> distance;
    unordered_map<coord, coord> previousPositions;
    stack<coord> path;
    // initialize distance.
    for (auto it = map.begin(); it != map.end(); ++it) {
        distance[it->first] = INT_MAX;
        explored[it->first] = false;
    }
    distance[start] = 0;
    explored[start] = true;

    // explore the map.
    coord currentCoord = start;
    int minDistance;
    while (!explored[end]) {
        // update distance.
        for (int i = 0; i < 4; i++) {
            const TileDirection& direction = directions[i];
            const Tile& currentTile = map.at(currentCoord);
            const coord& adjacentCoord = currentTile.adjacentTiles_[static_cast<int>(direction)]->position_;
            // check if there's an adjecent tile and there's no wall.
            //TODO: check if the tile to explore is black
            if (currentTile.adjacentTiles_[static_cast<int>(direction)] != NULL && !currentTile.hasWall(direction)) {
                const int weight = currentTile.weights_[static_cast<int>(direction)] + distance[currentCoord];
                // check if the new weight to visit the adjecent tile is less than the current weight.
                if (weight < distance[adjacentCoord]) {
                    distance[adjacentCoord] = weight;
                    previousPositions[adjacentCoord] = currentCoord;
                }
            }
        }
        // find next tile.
        minDistance = INT_MAX;
        for (auto it = distance.begin(); it != distance.end(); ++it) {
            const coord& current = it->first;
            const int currentDistance = it->second;
            if (currentDistance < minDistance && !explored[current]) {
                minDistance = currentDistance;
                currentCoord = current;
            }
        }
        explored[currentCoord] = true;
    }
    // find path.
    coord current = end;
    while (current != start) {
        path.push(current);
        current = previousPositions[current];
    }
    path.push(start);
    // print path.
    printPath(path);
    return;
}

bool checkForWall(const vector<vector<char>>& maze, const TileDirection& direction, const coord& currentTileCoord) {
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

void printMap(unordered_map<coord, Tile>& map){
    for (auto it = map.begin(); it != map.end(); ++it) {
        cout << "Tile: " << it->first.x << " " << it->first.y << " " << it->first.z << endl;
        for (int i = 0; i < 4; ++i) {
            if (it->second.adjacentTiles_[i] != NULL) {
                cout << "Adjacent Tile: " << it->second.adjacentTiles_[i]->position_.x << " " << it->second.adjacentTiles_[i]->position_.y << " " << it->second.adjacentTiles_[i]->position_.z << endl;
            }
        }
    }
    return;
}

int checkRamp(const vector<vector<char>>& maze, const TileDirection& direction, const coord& currentTileCoord) {
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

void depthFirstSearch(unordered_map<coord, Tile>& map) {
    unordered_map<coord, bool> visited;
    stack<coord> unvisited;
    Tile* currentTile;
    coord robotCoord = coord{1,1,1};
    map[robotCoord] = Tile(robotCoord);
    unvisited.push(robotCoord);
    bool wall;
    bool alreadyConnected;
    coord nextTileCoord;
    TileDirection oppositeDirection;
    // explore the map.
    while (!unvisited.empty()) {
        // get the next tile to explore.
        coord currentTileCoord = unvisited.top();
        unvisited.pop();
        // check if the tile has been visited.
        if (visited.find(currentTileCoord) != visited.end()) {
            continue;
        }
        // go to tile. TODO
        dijsktra(robotCoord, currentTileCoord, map);
        robotCoord = currentTileCoord;
        visited[currentTileCoord] = true;
        // check walls the 4 adjacent tiles.
        // cout<<currentTileCoord.x<<" "<<currentTileCoord.y<<" "<<currentTileCoord.z<<endl;
        for (const TileDirection& direction : directions) {
            wall = false;
            switch(direction) {
                case TileDirection::kRight:
                    nextTileCoord = coord{currentTileCoord.x+2,currentTileCoord.y,checkRamp(maze, direction, currentTileCoord)};
                    currentTile = &map[currentTileCoord];
                    oppositeDirection = TileDirection::kLeft;
                    break;
                case TileDirection::kUp:
                    nextTileCoord = coord{currentTileCoord.x,currentTileCoord.y+2,checkRamp(maze, direction, currentTileCoord)};
                    currentTile = &map[currentTileCoord];
                    oppositeDirection = TileDirection::kDown;
                    break;
                case TileDirection::kLeft:
                    nextTileCoord = coord{currentTileCoord.x-2,currentTileCoord.y,checkRamp(maze, direction, currentTileCoord)};
                    currentTile = &map[currentTileCoord];
                    oppositeDirection = TileDirection::kRight;
                    break;
                case TileDirection::kDown:
                    nextTileCoord = coord{currentTileCoord.x,currentTileCoord.y-2,checkRamp(maze, direction, currentTileCoord)};
                    currentTile = &map[currentTileCoord];
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
                Tile* nextTile = &map[nextTileCoord];
                if (nextTile->position_ == kInvalidPosition) {
                    map[nextTileCoord].setPosition(nextTileCoord);
                }
                // Link the two adjacent Tiles.
                currentTile->addAdjacentTile(direction, nextTile, wall);
                nextTile->addAdjacentTile(oppositeDirection, currentTile, wall);
                // Check if there's a wall between the two adjacent Tiles.
                if (!wall) {
                    // maze[currentTileCoord.y][currentTileCoord.x] = 'o';
                    // maze[nextTileCoord.y][nextTileCoord.x] = 'o';
                    // if the tile has not been visited, add it to the queue.
                    if (visited.find(nextTileCoord) == visited.end()) {
                        unvisited.push(nextTileCoord);
                    }
                }
            }
        }
    }
    return;
}

int main() {
    unordered_map<coord, Tile> map;
    depthFirstSearch(map);
    return 0;
}

/*TODO:
dijsktra's algorithm on bfs
check black tiles
multiple heights

(19/12/2023):
- Tile.h
- coord.h

(02/01/2024):
- dijsktra's algorithm
- visualize and read ascii map for testing

(15/01/2024):
- Tile.cpp
- enum class TileDirection
- visualize path

(23/01/2024):
- char walls_
- victim, obstacle and wall storage

(30/01/2024):
- maps to array storage
- checkpoint storage
- black tiles storage and check

(24/02/2024):
- arduino integration
*/