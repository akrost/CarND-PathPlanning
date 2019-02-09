#include "Map.h"

Map::Map()
{
    //ctor
}

Map::~Map()
{
    //dtor
}

int Map::ClosestWaypoint(double x, double y){

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < this->x.size(); i++)
	{
		double map_x = this->x[i];
		double map_y = this->y[i];
		double dist = distance(x, y, map_x, map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

double Map::distance(double x1, double y1, double x2, double y2){
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int Map::NextWaypoint(double x, double y, double theta){

	int closestWaypoint = ClosestWaypoint(x, y);

	double map_x = this->x[closestWaypoint];
	double map_y = this->y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4){
    closestWaypoint++;
  if (closestWaypoint == this->x.size()){
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

void Map::add_x(double x){
    this->x.push_back(x);
}

void Map::add_y(double y){
    this->y.push_back(y);
}

void Map::add_s(double s){
    this->s.push_back(s);
}

void Map::add_dx(double dx){
    this->dx.push_back(dx);
}

void Map::add_dy(double dy){
    this->dy.push_back(dy);
}

vector<double> Map::get_x(){
    return this->x;
}

vector<double> Map::get_y(){
    return this->y;
}

vector<double> Map::get_s(){
    return this->s;
}

vector<double> Map::get_dx(){
    return this->dx;
}

vector<double> Map::get_dy(){
    return this->dy;
}
