#ifndef Tile_h
#define Tile_h

#include<iostream>
#include<map>
using namespace std;

#include "coord.h"

class Tile{
    public:
        coord position;
        map<string, Tile *> adjacentTiles;
        map<string, bool> walls;
        map <string, int> weights;
        Tile();
        Tile(coord position);
        void addAdjacentTile(string direction, Tile *tile, bool wall, coord position);
        void setPosition(coord position);
};
// inum
Tile::Tile(){
    position = coord{1000,1000};
    adjacentTiles["up"] = NULL;
    adjacentTiles["down"] = NULL;
    adjacentTiles["left"] = NULL;
    adjacentTiles["right"] = NULL;

    walls["up"] = true;
    walls["down"] = true;
    walls["left"] = true;
    walls["right"] = true;

    weights["up"] = 0;
    weights["down"] = 0;
    weights["left"] = 0;
    weights["right"] = 0;
}
Tile::Tile(coord position){
    this->position = position;

    adjacentTiles["up"] = NULL;
    adjacentTiles["down"] = NULL;
    adjacentTiles["left"] = NULL;
    adjacentTiles["right"] = NULL;

    walls["up"] = true;
    walls["down"] = true;
    walls["left"] = true;
    walls["right"] = true;

    weights["up"] = 0;
    weights["down"] = 0;
    weights["left"] = 0;
    weights["right"] = 0;
}
void Tile::addAdjacentTile(string direction, Tile *tile, bool wall, coord position){
    adjacentTiles[direction] = tile;
    weights[direction] = 1;
    walls[direction] = wall;
    tile->setPosition(position);
}
void Tile::setPosition(coord position){
    this->position = position;
}
#endif