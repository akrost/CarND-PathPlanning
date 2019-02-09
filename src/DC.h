#ifndef DRIVECONTROLLER_H
#define DRIVECONTROLLER_H
#include "Car.h"
#include "Map.h"
#include <vector>
#include <math.h>

using namespace std;

class DriveController{
    public:
        DriveController(Car* car, Map* map_);
        virtual ~DriveController();

        vector<vector<double>> plan_path(int prev_path_size);
        void update_environment(vector<vector<double>> sensor_fusion);
        Car* car;
    protected:
    private:
        Map* map_;
        double target_speed;
        double ref_speed;
        double points_planned_ahead;
        vector<vector<double>> environment;

        constexpr double pi() { return M_PI; };
        double distance(double x1, double y1, double x2, double y2);

        vector<double> getFrenet(double x, double y, double theta);
        vector<double> getXY(double s, double d);

        bool check_for_car_ahead(int prev_path_size);
        bool check_lane_safe(int lane);

        vector<vector<double>> create_trajectory(int prev_path_size);
};

#endif // DRIVECONTROLLER_H
