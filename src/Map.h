#ifndef MAP_H
#define MAP_H
#include <vector>
#include <math.h>

using namespace std;

class Map
{
    public:
        Map();
        virtual ~Map();

        int ClosestWaypoint(double x, double y);
        int NextWaypoint(double x, double y, double theta);

        void add_x(double x);
        void add_y(double y);
        void add_s(double s);
        void add_dx(double dx);
        void add_dy(double dy);

        vector<double> get_x();
        vector<double> get_y();
        vector<double> get_s();
        vector<double> get_dx();
        vector<double> get_dy();
    protected:

    private:
        vector<double> x;
        vector<double> y;
        vector<double> s;
        vector<double> dx;
        vector<double> dy;

        constexpr double pi() { return M_PI; };
        double distance(double x1, double y1, double x2, double y2);
};

#endif // MAP_H
