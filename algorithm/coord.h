struct coord{
    int x;
    int y;

    bool const operator==(const coord &o) const {
        return x == o.x && y == o.y;
    }
    bool const operator!=(const coord &o) const {
        return x != o.x || y != o.y;
    }
    bool const operator<(const coord &o) const {
        return x < o.x || (x == o.x && y < o.y);
    }
};

namespace std{
    template<>
    struct hash<coord>{
        size_t operator()(const coord& c) const{
            return hash<int>()(c.x)^hash<int>()(c.y);
        }
    };
}