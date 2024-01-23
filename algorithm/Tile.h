#ifndef Tile_h
#define Tile_h

#include<iostream>
#include<map>
using namespace std;

#include "coord.h"

enum class TileDirection{
    kUp = 0,
    kDown = 1,
    kLeft = 2,
    kRight = 3,
    kNone
};

constexpr int kVictimBit = 4;
constexpr int kObstacleBit = 5;

class Tile{
    public:
        coord position_;
        map<TileDirection, Tile *> adjacentTiles_;
        char walls_;
        // map<TileDirection, bool> walls_;
        map<TileDirection, int> weights_;
        Tile();
        Tile(const coord& position);
        void addAdjacentTile(const TileDirection direction, Tile *tile, const bool wall, const coord& position);
        void setPosition(const coord& position);
        void setWall(const TileDirection direction, const bool wall);
        bool hasWall(const TileDirection direction);
        bool hasVictim();
        void setVictim();
        bool hasObstacle();
        void setObstacle();
};

#endif