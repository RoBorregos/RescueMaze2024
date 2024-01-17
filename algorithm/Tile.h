#ifndef Tile_h
#define Tile_h

#include<iostream>
#include<map>
using namespace std;

#include "coord.h"

enum class TileDirection{
    up = 0,
    down = 1,
    left = 2,
    right = 3,
    none
};

class Tile{
    public:
        coord position_;
        map <TileDirection, Tile *> adjacentTiles_;
        map <TileDirection, bool> walls_;
        map <TileDirection, int> weights_;
        Tile();
        Tile(coord position);
        void addAdjacentTile(const TileDirection direction, Tile *tile, const bool wall, coord position);
        void setPosition(coord position);
};

#endif