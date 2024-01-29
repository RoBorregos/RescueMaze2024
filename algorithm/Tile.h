#ifndef Tile_h
#define Tile_h

#include<iostream>
#include<map>
using namespace std;

#include "coord.h"

enum class TileDirection {
    kUp = 0,
    kDown = 1,
    kLeft = 2,
    kRight = 3,
    kNone
};

// Bits 0-3 are reserved for the walls. 
constexpr int kVictimBit = 4;
constexpr int kObstacleBit = 5;

constexpr int kMinWeight = 1;

class Tile{
    public:
        coord position_;
        map<TileDirection, Tile *> adjacentTiles_;
        map<TileDirection, int> weights_;
        char data_;
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