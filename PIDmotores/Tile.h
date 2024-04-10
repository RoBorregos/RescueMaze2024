#ifndef Tile_h
#define Tile_h

#include<Arduino.h>

#include "coord.h"
#include "TileDirection.h"

// Bits 0-3 are reserved for the walls. 
constexpr uint8_t kVictimBit = 4;
constexpr uint8_t kObstacleBit = 5;
constexpr uint8_t kBlackTileBit = 6;
constexpr uint8_t kCheckpointBit = 7;

constexpr uint8_t kNumberOfDirections = 4;

constexpr uint8_t kWhiteTileWeight = 1;
constexpr uint8_t kBlueTileWeight = 3; // 5 seconds
constexpr uint8_t kRampWeight = 5;
constexpr uint8_t kObstacleWeight = 4;

constexpr coord kInvalidPosition = coord{1000,1000,1000};

class Tile{
    public:
        // TODO: SAVE RAMP INFORMATION.
        coord position_;
        Tile *adjacentTiles_[kNumberOfDirections];
        int weight_;
        char data_;
        Tile();
        Tile(const coord& position);
        void addAdjacentTile(const TileDirection direction, Tile *tile, const bool wall);
        void setPosition(const coord& position);
        void setWall(const TileDirection direction, const bool wall);
        bool hasWall(const TileDirection direction) const;
        bool hasVictim() const;
        void setVictim();
        bool hasObstacle() const;
        void setObstacle();
        bool hasBlackTile() const;
        void setBlackTile();
        bool hasCheckpoint() const;
        void setCheckpoint();
};

#endif