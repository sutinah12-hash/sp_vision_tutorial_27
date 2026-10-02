#ifndef CAR_HPP
#define CAR_HPP

#include <string>

class Car
{
public:
    Car(const std::string& brand, int speed);
    ~Car();

    void accelerate();
    void brake();

    void print() const;

private:
    std::string brand_;
    int speed_;
};

#endif