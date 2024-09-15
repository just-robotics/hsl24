#ifndef points_hpp
#define points_hpp


#include <list>


namespace Points {
class Point {
public:
    double x;
    double y;
    double yaw;

    Point(double x, double y, double yaw) {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
    }
};


size_t points_number = 26;

Point pt_array[] = {
    Point(+2.5, 0.5, 0.00),
    Point(+2.5, 1.0, 1.57),
    Point(+1.5, 1.5, 3.14),
    Point(+1.5, 1.5, 3.14),
    Point(+0.5, 2.0, 1.57),
    Point(+0.5, 2.5, 0.79),

    Point(+1.5, 2.5, 0.00),
    Point(+1.5, 2.5, 4.71),
    Point(+1.5, 2.5, 3.14),
    Point(+1.5, 2.5, 1.57),

    Point(+1.5, 4.5, 1.57),
    Point(+1.5, 4.5, 3.14),
    Point(+1.5, 4.5, 1.57),
    Point(+1.5, 4.5, 0.00),

    Point(+2.5, 4.5, 0.00), // new loop from here
    Point(+2.5, 4.5, 0.00),
    Point(+2.5, 4.5, 1.57),
    Point(+2.5, 4.5, 3.14),
    Point(+2.5, 4.5, 3.14),

    Point(+0.0, 4.5, 3.14),

    Point(-2.0, 4.5, 3.14),
    Point(-2.0, 4.5, 3.14),
    Point(-2.0, 4.5, 2.36),
    Point(-2.0, 4.5, 1.57),
    Point(-2.0, 4.5, 0.79),
    Point(-2.0, 4.5, 0.79),

};
}


#endif //points_hpp

