#ifndef Tile_h
#define Tile_h

#include<Arduino.h>

#include "coord.h"

enum class TileDirection {
    kUp = 0,
    kLeft = 1,
    kDown = 2,
    kRight = 3,
    kNone
};

// Bits 0-3 are reserved for the walls. 
constexpr int kVictimBit = 4;
constexpr int kObstacleBit = 5;
constexpr int kBlackTileBit = 6;
constexpr int kCheckpointBit = 7;

constexpr int kMinWeight = 1;

constexpr coord kInvalidPosition = coord{1000,1000,1000};

class Tile{
    public:
        // TODO: SAVE RAMP INFORMATION.
        coord position_;
        Tile *adjacentTiles_[4];
        int weights_[4];
        char data_;
        Tile();
        Tile(const coord& position);
        void addAdjacentTile(const TileDirection direction, Tile *tile, const bool wall);
        void setPosition(const coord& position);
        void setWall(const TileDirection direction, const bool wall);
        bool hasWall(const TileDirection direction) const;
        bool hasVictim();
        void setVictim();
        bool hasObstacle();
        void setObstacle();
        bool hasBlackTile();
        void setBlackTile();
        bool hasCheckpoint();
        void setCheckpoint();
};

#endif