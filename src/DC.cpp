#include "DC.h"
#include "Car.h"
#include "Map.h"
#include <iostream>
#include <vector>
#include <math.h>
#include "spline.h"

using namespace std;

DriveController::DriveController(Car* car, Map* map_){
    this->car = car;
    this->map_ = map_;

    this->target_speed = 49.5;
    this->ref_speed = 0.0;
    this->points_planned_ahead = 50;
}

DriveController::~DriveController(){
    //dtor
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> DriveController::getFrenet(double x, double y, double theta){
	int next_wp = this->map_->NextWaypoint(x,y, theta);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = this->map_->get_x().size()-1;
	}

	double n_x = this->map_->get_x()[next_wp] - this->map_->get_x()[prev_wp];
	double n_y = this->map_->get_y()[next_wp] - this->map_->get_y()[prev_wp];
	double x_x = x - this->map_->get_x()[prev_wp];
	double x_y = y - this->map_->get_y()[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000 - this->map_->get_x()[prev_wp];
	double center_y = 2000 - this->map_->get_y()[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef){
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(this->map_->get_x()[i], this->map_->get_y()[i], this->map_->get_x()[i+1],this->map_->get_y()[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> DriveController::getXY(double s, double d){
	int prev_wp = -1;
    double max_s = 6945.554;

    s = fmod(s, max_s);

	while(s > this->map_->get_s()[prev_wp+1] && (prev_wp < (int)(this->map_->get_s().size()-1) )){
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%this->map_->get_x().size();

	double heading = atan2((this->map_->get_y()[wp2] - this->map_->get_y()[prev_wp]),(this->map_->get_x()[wp2] - this->map_->get_x()[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s - this->map_->get_s()[prev_wp]);

	double seg_x = this->map_->get_x()[prev_wp]+seg_s*cos(heading);
	double seg_y = this->map_->get_y()[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);
	return {x,y};

}

double DriveController::distance(double x1, double y1, double x2, double y2){
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Update environment of the car
void DriveController::update_environment(vector<vector<double>> sensor_fusion){
    this->environment = sensor_fusion;
}

// Plan maneuver and create trajectory
vector<vector<double>> DriveController::plan_path(int prev_path_size){
    bool car_in_lane = check_for_car_ahead(prev_path_size);

    if ((this->ref_speed < (this->target_speed - .224)) && !(car_in_lane)){
        this->ref_speed += .224;
    } else if ((this->ref_speed > this->target_speed) || (car_in_lane)){
        this->ref_speed -= .224;
    }

    // Check if lane shift makes sense
    // Lane left
    bool left_lane_safe = check_lane_safe(this->car->get_lane()-1);
    bool right_lane_safe = check_lane_safe(this->car->get_lane()+1);
    cout << "Left safe: " << left_lane_safe << ", Right safe: " << right_lane_safe << endl;

    // Make lane shift
    if (car_in_lane){
        if (left_lane_safe){
            this->car->change_lane_left();
        }else if (right_lane_safe){
            this->car->change_lane_right();
        }
    }

    return create_trajectory(prev_path_size);
}

bool DriveController::check_lane_safe(int target_lane){
    if ((target_lane < 0) || (target_lane > 2)){
        return false;
    }

    for (int i=0; i < this->environment.size(); ++i){
        float env_d = this->environment[i][6];

        if ((env_d < (2+4*target_lane+2)) && (env_d > (2+4*target_lane-2))){
            double vx = this->environment[i][3];
            double vy = this->environment[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = this->environment[i][5];

            double horizon_back = 15 + 3*(this->target_speed - this->car->get_speed());
            double horizon_front = 10 + 1.1 * this->car->get_speed();

            // Car in lane
            if ((check_car_s > this->car->get_s() - horizon_back) && (check_car_s < this->car->get_s() + horizon_front)){
                // Car right next to me
                if ((check_car_s > this->car->get_s() - 10) && (check_car_s < this->car->get_s() + 10)){
                    return false;
                }
                // Car behind me, but faster
                if ((check_car_s < this->car->get_s()) && (check_speed > this->car->get_speed())){
                    return false;
                }
                // Car in front of me, but slower
                if ((check_car_s > this->car->get_s()) && (check_speed < this->car->get_speed())){
                    return false;
                }
            }
        }
    }
    return true;
}

// Check for car ahead in current lane
// Returns true if car in lane, false otherwise
bool DriveController::check_for_car_ahead(int prev_path_size){
    for (int i=0; i < this->environment.size(); ++i){
        float env_d = this->environment[i][6];

        if (env_d < (2+4*this->car->get_lane()+2) && env_d > (2+4*this->car->get_lane()-2)){
            double vx = this->environment[i][3];
            double vy = this->environment[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = this->environment[i][5];

            check_car_s += ((double)prev_path_size*.02*check_speed);

            double check_distance = 1.1 * this->car->get_speed();

            if ((check_car_s > this->car->get_s()) && (check_car_s - this->car->get_s() < check_distance)){
                return true;
            }
        }
    }
    return false;
}

// Create a trajectory based on current state
vector<vector<double>> DriveController::create_trajectory(int prev_size){
    int num_new_points = this->points_planned_ahead - prev_size;

    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    // Later we will interpolate these waypoints with a spline and fill it with more points that control speed
    vector<double> ptsx;
    vector<double> ptsy;

    // reference x, y, yaw states
    // either we will reference the starting point as where the car is or at the previoes paths end point
    double ref_x;
    double ref_y;
    double prev_ref_x;
    double prev_ref_y;
    double ref_yaw;

    if(prev_size < 2){
        prev_ref_x = this->car->get_x() - cos(this->car->get_yaw());
        prev_ref_y = this->car->get_y() - sin(this->car->get_yaw());

        ref_x = this->car->get_x();
        ref_y = this->car->get_y();
        ref_yaw = this->car->get_yaw();
    }else{
        prev_ref_x = this->car->trajectory_x[this->points_planned_ahead-2];
        prev_ref_y = this->car->trajectory_y[this->points_planned_ahead-2];

        ref_x = this->car->trajectory_x[this->points_planned_ahead-1];
        ref_y = this->car->trajectory_y[this->points_planned_ahead-1];
        ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);
    }

    ptsx.push_back(prev_ref_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_ref_y);
    ptsy.push_back(ref_y);

    vector<double> next_wp0 = getXY(this->car->get_s() + 50, (2+4*this->car->get_lane()));
    vector<double> next_wp1 = getXY(this->car->get_s() + 100, (2+4*this->car->get_lane()));
    vector<double> next_wp2 = getXY(this->car->get_s() + 150, (2+4*this->car->get_lane()));

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++ ){
        //shift car reference angle to 0 degrees
        double shift_x = ptsx[i]-ref_x;
        double shift_y = ptsy[i]-ref_y;
        ptsx[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
        ptsy[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
    }


    tk::spline s;
    s.set_points(ptsx,ptsy);

    if(this->car->trajectory_x.size() > 0){
        this->car->trajectory_x.erase(this->car->trajectory_x.begin(), this->car->trajectory_x.begin()+num_new_points);
        this->car->trajectory_y.erase(this->car->trajectory_y.begin(), this->car->trajectory_y.begin()+num_new_points);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

    double x_counter = 0;
    double delta_x = target_x * (.02 * this->ref_speed / 2.24)/target_dist;

    for (int i = 1; i <= num_new_points; i++) {
        x_counter += delta_x;
        double y_counter = s(x_counter);

        double next_x = (x_counter * cos(ref_yaw) - y_counter * sin(ref_yaw));
        double next_y = (x_counter * sin(ref_yaw) + y_counter * cos(ref_yaw));

        next_x += ref_x;
        next_y += ref_y;

        this->car->trajectory_x.push_back(next_x);
        this->car->trajectory_y.push_back(next_y);
    }

    vector<vector<double>> res;

    res.push_back(this->car->trajectory_x);
    res.push_back(this->car->trajectory_y);

    return res;
}


