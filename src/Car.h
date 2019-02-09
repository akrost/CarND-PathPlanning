#ifndef CAR_h
#define CAR_h
#include <math.h>
#include <vector>

using namespace std;

class Car{
    public:
        Car(double x, double y, double s, double d, double yaw, double speed);
        Car();
        virtual ~Car();

        void setX(double x);
        void setY(double y);
        void setS(double s);
        void setD(double d);
        void setYaw(double yaw);
        void setSpeed(double speed);

        void change_lane_left();
        void change_lane_right();

        void increase_speed(double value);
        void decrease_speed(double value);

        double get_x();
        double get_y();
        double get_s();
        double get_d();
        double get_yaw();
        double get_speed();
        int get_lane();

	vector<double> trajectory_x;
	vector<double> trajectory_y;
    protected:
    private:
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
        int lane = 1;

        constexpr double pi() { return M_PI; };
        double deg2rad(double x) { return x * pi() / 180; };
};

#endif // CAR_h
