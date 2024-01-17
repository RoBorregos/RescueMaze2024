#include<iostream>
#include<map>
using namespace std;

#include "Tile.h"

Tile::Tile(){
    position_ = coord{1000,1000};
    adjacentTiles_[TileDirection::up] = NULL;
    adjacentTiles_[TileDirection::down] = NULL;
    adjacentTiles_[TileDirection::left] = NULL;
    adjacentTiles_[TileDirection::right] = NULL;

    walls_[TileDirection::up] = true;
    walls_[TileDirection::down] = true;
    walls_[TileDirection::left] = true;
    walls_[TileDirection::right] = true;

    weights_[TileDirection::up] = 0;
    weights_[TileDirection::down] = 0;
    weights_[TileDirection::left] = 0;
    weights_[TileDirection::right] = 0;
}
Tile::Tile(coord position){
    this->position_ = position;

    adjacentTiles_[TileDirection::up] = NULL;
    adjacentTiles_[TileDirection::down] = NULL;
    adjacentTiles_[TileDirection::left] = NULL;
    adjacentTiles_[TileDirection::right] = NULL;

    walls_[TileDirection::up] = true;
    walls_[TileDirection::down] = true;
    walls_[TileDirection::left] = true;
    walls_[TileDirection::right] = true;

    weights_[TileDirection::up] = 0;
    weights_[TileDirection::down] = 0;
    weights_[TileDirection::left] = 0;
    weights_[TileDirection::right] = 0;
}
void Tile::addAdjacentTile(const TileDirection direction, Tile *tile, const bool wall, coord position){
    adjacentTiles_[direction] = tile;
    weights_[direction] = 1;
    walls_[direction] = wall;
    tile->setPosition(position);
}
void Tile::setPosition(coord position){
    this->position_ = position;
}