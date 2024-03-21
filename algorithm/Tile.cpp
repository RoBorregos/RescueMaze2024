#include<iostream>
#include<map>
using namespace std;

#include "Tile.h"

Tile::Tile(){
    this->position_ = kInvalidPosition;

    for (int i = 0; i < kNumberOfDirections; ++i) {
        adjacentTiles_[i] = NULL;
        weights_[i] = 0;
    }
    
    this->data_ = '\0';
}

Tile::Tile(const coord& position) {
    this->position_ = position;

    for (int i = 0; i < kNumberOfDirections; ++i) {
        adjacentTiles_[i] = NULL;
        weights_[i] = 0;
    }

    this->data_ = '\0';
}

void Tile::setWall(const TileDirection direction, const bool wall) {
    if (wall) {
        this->data_ |= (1 << static_cast<int>(direction));
    }
}

bool Tile::hasWall(const TileDirection direction) const {
    return this->data_ & (1 << static_cast<int>(direction));
}

void Tile::setVictim() {
    this->data_ |= (1 << kVictimBit);
}

bool Tile::hasVictim() {
    return this->data_ & (1 << kVictimBit);
}

void Tile::setObstacle() {
    this->data_ |= (1 << kObstacleBit);
}

bool Tile::hasObstacle() {
    return this->data_ & (1 << kObstacleBit);
}

void Tile::setBlackTile() {
    this->data_ |= (1 << kBlackTileBit);
}

bool Tile::hasBlackTile() {
    return this->data_ & (1 << kBlackTileBit);
}

void Tile::setCheckpoint() {
    this->data_ |= (1 << kCheckpointBit);
}

bool Tile::hasCheckpoint() {
    return this->data_ & (1 << kCheckpointBit);
}

void Tile::addAdjacentTile(const TileDirection direction, Tile *tile, const bool wall) {
    adjacentTiles_[static_cast<int>(direction)] = tile;
    weights_[static_cast<int>(direction)] = kMinWeight;
    this->setWall(direction, wall);
}

void Tile::setPosition(const coord& position) {
    this->position_ = position;
}
