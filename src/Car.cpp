#include "Car.h"
#include <iostream>

Car::Car(double x, double y, double s, double d, double yaw, double speed){
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = this->deg2rad(yaw);
    this->speed = speed;
}

Car::Car(){}

Car::~Car(){

}

void Car::setX(double x){
    this->x = x;
}

void Car::setY(double y){
    this->y = y;
}

void Car::setS(double s){
    this->s = s;
}

void Car::setD(double d){
    this->d = d;
}

void Car::setYaw(double yaw){
    this->yaw = this->deg2rad(yaw);
}

void Car::setSpeed(double speed){
    this->speed = speed;
}

void Car::increase_speed(double value){
    this->speed += value;
}

void Car::decrease_speed(double value){
    this->speed -= value;
}

void Car::change_lane_left(){
    this->lane -= 1;
}

void Car::change_lane_right(){
    this->lane += 1;
}


double Car::get_x(){
    return this->x;
}

double Car::get_y(){
    return this->y;
}

double Car::get_s(){
    return this->s;
}

double Car::get_d(){
    return this->d;
}

double Car::get_yaw(){
    return this->yaw;
}

double Car::get_speed(){
    return this->speed;
}

int Car::get_lane(){
    return this->lane;
}

