#include<iostream>
#include<vector>
#include<unordered_map>
#include<stack>
#include<climits>

using namespace std;

#include "Tile.cpp"

constexpr TileDirection directions[] = {TileDirection::kUp, TileDirection::kDown, TileDirection::kLeft, TileDirection::kRight};

vector<vector<char>> maze = {
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
        {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', '#', '#', '#', '#', '#'},
        {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'},
        {'#', ' ', '#', '#', '#', ' ', '#', ' ', '#', '#', '#'},
        {'#', ' ', ' ', ' ', ' ', ' ', '#', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', '#', '#', ' ', '#', '#', ' ', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#'},
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'}
};

void printMaze(vector<vector<char>> mazeToPrint){
    for(int i = 0; i < mazeToPrint.size(); i++){
        for(int j = 0; j < mazeToPrint[i].size(); j++){
            cout << mazeToPrint[i][j] << " ";
        }
        cout << endl;
    }
}

void printPath(stack <coord> path){
    vector<vector<char>> newMaze = maze;
    int lastX = path.top().x;
    int lastY = path.top().y;
    path.pop();
    newMaze[lastY][lastX] = 'S';
    while(!path.empty()){
        if(path.top().x == lastX){
            if(path.top().y > lastY){
                newMaze[lastY+1][lastX] = '/';
            }else{
                newMaze[lastY-1][lastX] = '^';
            }
        }else{
            if(path.top().x>lastX){
                newMaze[lastY][lastX+1] = '>';
            }else{
                newMaze[lastY][lastX-1] = '<';
            }
        }
        lastX = path.top().x;
        lastY = path.top().y;
        path.pop();
    }
    newMaze[lastY][lastX] = 'E';
    // print path on maze
    printMaze(newMaze);
    return;
}

void dijsktra(const coord& start, const coord& end, unordered_map<coord,Tile> map){
    unordered_map<coord, bool> explored;
    unordered_map<coord, int> distance;
    unordered_map<coord, coord> previousPositions;
    stack<coord> path;
    // initialize distance.
    for(auto it = map.begin(); it != map.end(); ++it){
        distance[it->first] = INT_MAX;
        explored[it->first] = false;
    }
    distance[start] = 0;
    explored[start] = true;

    // explore the map.
    coord currentCoord = start;
    int minDistance, weight;
    while(!explored[end]){
        // update distance.
        for(int i = 0; i < 4; i++){
            TileDirection direction = directions[i];
            Tile currentTile = map[currentCoord];
            coord adjacentCoord = currentTile.adjacentTiles_[direction]->position_;
            // check if there's an adjecent tile and there's no wall.
            // cout<<currentTile.hasWall(direction)<<' '<<currentTile.walls_[direction]<<endl;
            if(currentTile.adjacentTiles_[direction] != NULL && !currentTile.hasWall(direction)){
                weight = currentTile.weights_[direction] + distance[currentCoord];
                // check if the new weight to visit the adjecent tile is less than the current weight.
                if(weight < distance[adjacentCoord]){
                    distance[adjacentCoord] = weight;
                    previousPositions[adjacentCoord] = currentCoord;
                }
            }
        }
        // find next tile.
        minDistance = INT_MAX;
        for(auto it = distance.begin(); it != distance.end(); ++it){
            coord current = it->first; // las declaro antes?
            int currentDistance = it->second;
            if(currentDistance < minDistance && !explored[current]){
                minDistance = currentDistance;
                currentCoord = current;
            }
        }
        explored[currentCoord] = true;
    }
    // find path.
    coord current = end;
    while(current != start){
        path.push(current);
        current = previousPositions[current];
    }
    path.push(start);
    // print path.
    printPath(path);
    return;
}

int main(){
    unordered_map<coord, Tile> map;
    unordered_map<coord, bool> visited;
    stack<coord> unvisited;
    Tile currentTile;
    coord robotCoord = coord{1,1};
    unvisited.push(coord{1,1});
    // depth first search.
    while(!unvisited.empty()){
        // get the next tile to explore.
        coord currentTileCoord = unvisited.top();
        unvisited.pop();
        // check if the tile has been visited.
        if(visited.find(currentTileCoord) != visited.end()){
            //cout<<currentTileCoord.x<<","<<currentTileCoord.y<<" Already visited"<<endl;
            continue;
        }
        // go to tile. TODO
        // dijsktra(robotCoord, currentTileCoord, map);
        // robotCoord = currentTileCoord;

        visited[currentTileCoord] = true;
        bool wall, alreadyConnected;
        //cout<<"Exploring tile ("<<currentTileCoord.x<<","<<currentTileCoord.y<<")"<<endl;
        // check walls the 4 adjacent tiles.
        for(int i = 0; i < 4; i++){
            wall = false;
            coord nextTileCoord;
            TileDirection direction, oppositeDirection;
            switch(i){
                case 0:
                    nextTileCoord = coord{currentTileCoord.x+2,currentTileCoord.y};
                    currentTile = map[currentTileCoord];
                    direction = TileDirection::kRight;
                    oppositeDirection = TileDirection::kLeft;
                    break;
                case 1:
                    nextTileCoord = coord{currentTileCoord.x,currentTileCoord.y+2};
                    currentTile = map[currentTileCoord];
                    direction = TileDirection::kUp;
                    oppositeDirection = TileDirection::kDown;
                    break;
                case 2:
                    nextTileCoord = coord{currentTileCoord.x-2,currentTileCoord.y};
                    currentTile = map[currentTileCoord];
                    direction = TileDirection::kLeft;
                    oppositeDirection = TileDirection::kRight;
                    break;
                case 3:
                    nextTileCoord = coord{currentTileCoord.x,currentTileCoord.y-2};
                    currentTile = map[currentTileCoord];
                    direction = TileDirection::kDown;
                    oppositeDirection = TileDirection::kUp;
                    break;
            }
            // check if the tile has been checked.
            if(currentTile.adjacentTiles_[direction] == NULL){
                // check for a wall.
                if(maze[currentTileCoord.y][currentTileCoord.x+1] == '#' && direction == TileDirection::kRight){
                    wall = true;
                }else if(maze[currentTileCoord.y+1][currentTileCoord.x] == '#' && direction == TileDirection::kUp){
                    wall = true;
                }else if(maze[currentTileCoord.y][currentTileCoord.x-1] == '#' && direction == TileDirection::kLeft){
                    wall = true;
                }else if(maze[currentTileCoord.y-1][currentTileCoord.x] == '#' && direction == TileDirection::kDown){
                    wall = true;
                }
                // if there is no wall, add the connection.
                if(!wall){
                    maze[currentTileCoord.y][currentTileCoord.x] = 'o';
                    maze[nextTileCoord.y][nextTileCoord.x] = 'o';
                    //cout<<"Adding connection: ("<<currentTileCoord.x<<","<<currentTileCoord.y<<") - ("<<nextTileCoord.x<<","<<nextTileCoord.y<<")"<<endl;
                    map[currentTileCoord].addAdjacentTile(direction, &map[nextTileCoord], false, nextTileCoord);
                    map[nextTileCoord].addAdjacentTile(oppositeDirection, &map[currentTileCoord], false, currentTileCoord);
                    // if the tile has not been visited, add it to the queue.
                    if(visited.find(nextTileCoord) == visited.end()){
                        unvisited.push(nextTileCoord);
                    }
                }else{
                    map[currentTileCoord].addAdjacentTile(direction, &map[currentTileCoord], true, nextTileCoord);
                    map[nextTileCoord].addAdjacentTile(oppositeDirection, &map[currentTileCoord], true, currentTileCoord);
                }
            }
        }
    }
    dijsktra(coord{1,1},coord{9,7},map);
    return 0;
}

/*TODO:
- visualize map (how tf) done
- weight between nodes (new map?)
- movement comments
- dijsktra's algorithm
- almacenar victimas y tiles negras, checkpoints, etc.

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
*/