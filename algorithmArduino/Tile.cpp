#include "Tile.h"

Tile::Tile(){
    this->position_ = kInvalidPosition;

    adjacentTiles_[0] = NULL;
    adjacentTiles_[1] = NULL;
    adjacentTiles_[2] = NULL;
    adjacentTiles_[3] = NULL;

    weights_[0] = 0;
    weights_[1] = 0;
    weights_[2] = 0;
    weights_[3] = 0;

    this->data_ = '\0';
}

Tile::Tile(const coord& position) {
    this->position_ = position;

    adjacentTiles_[0] = NULL;
    adjacentTiles_[1] = NULL;
    adjacentTiles_[2] = NULL;
    adjacentTiles_[3] = NULL;

    weights_[0] = 0;
    weights_[1] = 0;
    weights_[2] = 0;
    weights_[3] = 0;

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
