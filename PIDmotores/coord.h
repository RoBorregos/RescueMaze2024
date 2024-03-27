#ifndef coord_h
#define coord_h

struct coord{
    int x;
    int y;
    int z;
    
    bool const operator==(const coord &o) const {
        return x == o.x && y == o.y && z == o.z;
    }

    bool const operator!=(const coord &o) const {
        return x != o.x || y != o.y || z != o.z;
    }

    bool const operator<(const coord &o) const {
        return x < o.x || (x == o.x && y < o.y) || (x == o.x && y == o.y && z < o.z);
    }
};

#endif