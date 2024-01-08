#include<iostream>
#include<vector>
#include<unordered_map>
#include<stack>

using namespace std;

#include "Tile.h"

void dijsktra(coord start, coord end, unordered_map<coord,Tile> map){
    unordered_map<coord,bool> explored;
    unordered_map<coord,int> distance;
    unordered_map<coord,coord> previous;
    stack<coord> path;
    string directions[] = {"up", "down", "left", "right"};
    //initialize distance
    for(auto it = map.begin(); it != map.end(); it++){
        distance[it->first] = 1000000;
        explored[it->first] = false;
    }
    distance[start] = 0;
    explored[start] = true;

    //explore the map
    coord currentTile = start;
    while(!explored[end]){
        //update distance
        for(int i = 0; i < 4; i++){
            if(map[currentTile].adjacentTiles[directions[i]] != NULL && !map[currentTile].walls[directions[i]]){
                int weight = map[currentTile].weights[directions[i]] + distance[currentTile];
                if(weight < distance[map[currentTile].adjacentTiles[directions[i]]->position]){
                    distance[map[currentTile].adjacentTiles[directions[i]]->position] = weight;
                    previous[map[currentTile].adjacentTiles[directions[i]]->position] = currentTile;
                }
            }
        }
        //find next tile
        int minDistance = 1000000;
        for(auto it = distance.begin(); it != distance.end(); it++){
            if(it->second < minDistance && !explored[it->first]){
                minDistance = it->second;
                currentTile = it->first;
            }
        }
        explored[currentTile] = true;
    }
    //find path
    coord current = end;
    while(current.x != start.x && current.y != start.y){
        path.push(current);
        current = previous[current];
    }
    path.push(start);
    //print path
    while(!path.empty()){
        cout<<"("<<path.top().x<<","<<path.top().y<<")"<<endl;
        path.pop();
    }
    return;
}

int main(){
    unordered_map<coord,Tile> map;
    unordered_map<coord,bool> visited;

    //pruebas insanas
    //mapa.insert(pair<coord, int>(coord{0,0},123));
    /*vector<coord> v;
    v.push_back(coord{-1,0});
    mapa.insert(pair<coord,vector<coord>>(coord{0,0},v));*/

    /*explore(coord{0,0},map,visited);
    cout<<"End of exploration"<<endl;*/

    /*map[coord{0,0}].push_back(coord{1,0});
    map[coord{0,0}].push_back(coord{0,1});
    map[coord{0,0}].push_back(coord{-1,0});
    vector<coord> v;
    v=map[coord{0,0}];
    for(int i = 0; i < v.size(); i++){
        cout<<"("<<v[i].x<<","<<v[i].y<<")"<<endl;
    }*/
    //map[coord{0,0}].push_back(coord{1,0});

    vector<vector<char>> maze = {
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
        {'#', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', '#', '#', '#', '#', '#'},
        {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', '#', ' ', '#'},
        {'#', ' ', '#', '#', '#', ' ', '#', ' ', '#', '#', '#'},
        {'#', ' ', ' ', ' ', '#', ' ', '#', ' ', ' ', ' ', '#'},
        {'#', ' ', '#', '#', '#', ' ', '#', '#', ' ', ' ', '#'},
        {'#', ' ', '#', ' ', '#', ' ', ' ', ' ', ' ', ' ', '#'},
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'}
    };

    stack<coord> unvisited;
    Tile currentTile;
    unvisited.push(coord{1,1});
    //depth first search
    while(!unvisited.empty()){
        //get the next tile to explore
        coord currentTileCoord = unvisited.top();
        unvisited.pop();
        //check if the tile has been visited
        if(visited.find(currentTileCoord) != visited.end()){
            cout<<currentTileCoord.x<<","<<currentTileCoord.y<<" Already visited"<<endl;
            continue;
        }
        //go to tile

        visited[currentTileCoord] = true;
        bool wall, alreadyConnected;
        cout<<"Exploring tile ("<<currentTileCoord.x<<","<<currentTileCoord.y<<")"<<endl;
        //check walls the 4 adjacent tiles
        for(int i = 0; i < 4; i++){
            wall = false;
            coord nextTileCoord;
            string direction, oppositeDirection;
            switch(i){
                case 0:
                    nextTileCoord = coord{currentTileCoord.x+2,currentTileCoord.y};
                    currentTile = map[currentTileCoord];
                    direction = "right";
                    oppositeDirection = "left";
                    break;
                case 1:
                    nextTileCoord = coord{currentTileCoord.x,currentTileCoord.y+2};
                    currentTile = map[currentTileCoord];
                    direction = "up";
                    oppositeDirection = "down";
                    break;
                case 2:
                    nextTileCoord = coord{currentTileCoord.x-2,currentTileCoord.y};
                    currentTile = map[currentTileCoord];
                    direction = "left";
                    oppositeDirection = "right";
                    break;
                case 3:
                    nextTileCoord = coord{currentTileCoord.x,currentTileCoord.y-2};
                    currentTile = map[currentTileCoord];
                    direction = "down";
                    oppositeDirection = "up";
                    break;
            }
            //check if the tile has been checked
            if(currentTile.adjacentTiles[direction] == NULL){
                //check for a wall
                if(maze[currentTileCoord.y][currentTileCoord.x+1] == '#' && direction == "right"){
                    wall = true;
                }else if(maze[currentTileCoord.y+1][currentTileCoord.x] == '#' && direction == "up"){
                    wall = true;
                }else if(maze[currentTileCoord.y][currentTileCoord.x-1] == '#' && direction == "left"){
                    wall = true;
                }else if(maze[currentTileCoord.y-1][currentTileCoord.x] == '#' && direction == "down"){
                    wall = true;
                }
                //if there is no wall, add the connection
                if(!wall){
                    maze[currentTileCoord.y][currentTileCoord.x] = 'o';
                    maze[nextTileCoord.y][nextTileCoord.x] = 'o';
                    cout<<"Adding connection: ("<<currentTileCoord.x<<","<<currentTileCoord.y<<") - ("<<nextTileCoord.x<<","<<nextTileCoord.y<<")"<<endl;
                    map[currentTileCoord].addAdjacentTile(direction, &map[nextTileCoord], false, nextTileCoord);
                    map[nextTileCoord].addAdjacentTile(oppositeDirection, &map[currentTileCoord], false, currentTileCoord);
                    //if the tile has not been visited, add it to the queue
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
    //print the map
    cout<<"End of exploration"<<endl;
    string directions[]={{"up"},{"down"},{"left"},{"right"}};
    for(auto it = map.begin(); it != map.end(); it++){
        cout<<"connections of: ("<<it->first.x<<","<<it->first.y<<")"<<endl;
        for(int i=0;i<4;i++){
            if(it->second.adjacentTiles[directions[i]] != NULL && !it->second.walls[directions[i]]){
                //cout<<directions[i]<<": "<<it->second.adjacentTiles[directions[i]]<<endl;
                cout<<directions[i]<<": ("<<it->second.adjacentTiles[directions[i]]->position.x<<","<<it->second.adjacentTiles[directions[i]]->position.y<<")"<<endl;
            }
        }
    }
    //print the maze
    for(int i = 0; i < maze.size(); i++){
        for(int j = 0; j < maze[i].size(); j++){
            cout<<maze[i][j]<<" ";
        }
        cout<<endl;
    }

    dijsktra(coord{1,1},coord{9,7},map);
    return 0;
}

/*TODO:
- visualize map (how tf) done
- weight between nodes (new map?)
- movement comments
- dijsktra's algorithm

(19/12/2023):
- Tile.h
- coord.h
*/