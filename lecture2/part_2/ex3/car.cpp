#include <iostream>
#include "car.hpp"

Car::Car(const std::string& brand, int speed)
    : brand_(brand), speed_(speed)
{
    std::cout << "Car Constructor!" << std::endl;
}

Car::~Car()
{
    std::cout << "Car Destructor!" << std::endl;
}

void Car::accelerate()
{
    speed_ += 10;
}

void Car::brake()
{
    speed_ -= 10;

    if (speed_ < 0)
        speed_ = 0;
}

void Car::print() const
{
    std::cout << "Brand: " << brand_ << std::endl;
    std::cout << "Speed: " << speed_ << std::endl;
}