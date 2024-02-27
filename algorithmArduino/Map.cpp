#include "Map.h"

Map::Map() {
    this->tiles.reserve(kMaxMapSize);
}

Map::Map(String type) {
    if (type == "tile") {
        this->tiles.reserve(kMaxMapSize);
    } else if (type == "bool") {
        this->bools.reserve(kMaxMapSize);
    } else if (type == "int") {
        this->ints.reserve(kMaxMapSize);
    } else if (type == "coord") {
        this->coords.reserve(kMaxMapSize);
    }
    this->type = type;
}

void Map::addTile(const Tile& tile, const coord& position) {
    this->tiles.push_back(tile);
    this->indexes.push_back(position);
}

void Map::addBool(const bool& b, const coord& position) {
    this->bools.push_back(b);
    this->indexes.push_back(position);
}

void Map::addInt(const int& i, const coord& position) {
    this->ints.push_back(i);
    this->indexes.push_back(position);
}

void Map::addCoord(const coord& c, const coord& position) {
    this->coords.push_back(c);
    this->indexes.push_back(position);
}

Tile* Map::getTile(const coord& position) {
    // TODO: iterate and return i
    for (auto& i : this->indexes) {
        if (i == position) {
            return &this->tiles[&i - &this->indexes[0]];
        }
    }
    return nullptr;
}

bool Map::getBool(const coord& position) {
    for (auto& i : this->indexes) {
        if (i == position) {
            return this->bools[&i - &this->indexes[0]];
        }
    }
    return false;
}

int Map::getInt(const coord& position) {
    for (auto& i : this->indexes) {
        if (i == position) {
            return this->ints[&i - &this->indexes[0]];
        }
    }
    return 0;
}

coord Map::getCoord(const coord& position) {
    for (auto& i : this->indexes) {
        if (i == position) {
            return this->coords[&i - &this->indexes[0]];
        }
    }
    return kInvalidPosition;
}

void Map::setTile(const Tile& tile, const coord& position) {
    for (auto& i : this->indexes) {
        if (i == position) {
            this->tiles[&i - &this->indexes[0]] = tile;
        }
    }
}

void Map::setBool(const bool& b, const coord& position) {
    for (auto& i : this->indexes) {
        if (i == position) {
            this->bools[&i - &this->indexes[0]] = b;
        }
    }
}

void Map::setInt(const int& in, const coord& position) {
    for (auto& i : this->indexes) {
        if (i == position) {
            this->ints[&i - &this->indexes[0]] = in;
        }
    }
}

void Map::setCoord(const coord& c, const coord& position) {
    for (auto& i : this->indexes) {
        if (i == position) {
            this->coords[&i - &this->indexes[0]] = c;
        }
    }
}