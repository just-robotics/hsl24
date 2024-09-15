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


size_t points_number = 52;

Point pt_array[] = {
    Point(-0.5, 0.5, 1.57),
    Point(-0.5, 0.5, 3.14),
    Point(-0.5, 0.5, 4.71),
    Point(-0.5, 0.5, 0.00),
    Point(-0.5, 0.5, 0.00),
    Point(-0.5, 0.5, 1.57),
    Point(-0.5, 0.5, 3.14),

    Point(-2.5, 0.5, 4.71),
    Point(-2.5, 0.5, 4.71),
    Point(-2.5, 0.5, 0.00),
    Point(-2.5, 0.5, 1.57),
    Point(-2.5, 0.5, 2.36),

    Point(-2.5, 2.0, 3.14),
    Point(-2.5, 2.0, 3.14),
    Point(-2.5, 2.0, 4.71),

    Point(-2.5, 1.0, 5.50),

    Point(-0.5, 2.5, 1.57),
    Point(-0.5, 2.5, 1.57),
    Point(-0.5, 2.5, 3.14),
    Point(-0.5, 2.5, 4.71),
    Point(-0.5, 2.5, 4.71),

    Point(-0.5, 1.0, 4.71),
    Point(-0.5, 1.0, 4.71),

    Point(+2.5, 0.5, 0.00),
    Point(+2.5, 0.5, 0.00),
    Point(+2.5, 0.5, 4.71),
    Point(+2.5, 0.5, 3.14),
    Point(+2.5, 0.5, 1.57),
    Point(+2.5, 0.5, 0.79),

    Point(+2.5, 1.0, 1.57),

    Point(+1.5, 1.5, 3.14),
    Point(+1.5, 1.5, 4.71),
    Point(+1.5, 1.5, 0.00),
    Point(+1.5, 1.5, 1.57),
    Point(+1.5, 1.5, 3.14),
    Point(+1.5, 1.5, 3.93),

    Point(+0.5, 1.5, 4.71),
    Point(+0.5, 1.5, 4.71),
    Point(+0.5, 1.5, 0.00),
    Point(+0.5, 1.5, 1.57),
    Point(+0.5, 1.5, 2.36),

    Point(+0.5, 2.5, 3.14),
    Point(+0.5, 2.5, 3.14),
    Point(+0.5, 2.5, 4.71),
    Point(+0.5, 2.5, 5.50),
    Point(+0.5, 2.5, 1.57),
    Point(+0.5, 2.5, 1.57),

    Point(+0.5, 3.5, 2.36),
    Point(+0.5, 3.5, 2.36),

    Point(-0.5, 3.0, 3.92),

    Point(-0.5, 2.0, 4.71),

    Point(-0.5, 1.0, 4.71),
};
}


#endif //points_hpp

