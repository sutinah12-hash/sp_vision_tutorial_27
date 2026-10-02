#include <iostream>
class Car
{
public:
    Car();
    ~Car();

    void run();

};
Car::Car()
{
    std::cout << "Car 构造完成" << std::endl;
}

Car::~Car()
{
    std::cout << "Car 析构完成" << std::endl;
}

int main()
{
    {
        Car car;
        car.run();
    }

    return 0;
}
void Car::run(){
    std::cout << "car 已经销毁" << std::endl;
}