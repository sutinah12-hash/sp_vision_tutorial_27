#include <iostream>
#include <vector>

class Armor
{
public:
  std::vector<int> brightness;

  Armor() : brightness{20, 60, 100, 60, 20}
  {
    std::cout << "Armor constructed\n";
  }

  ~Armor()
  {
    std::cout << "Armor destructed\n";
  }
};

int main()
{
  std::cout << "before {}\n";

  {
    Armor armor;

    // Help Python Tutor display the heap data managed by the vector.
    int * data = armor.brightness.data();
    std::cout << data[2] << '\n';
  }

  std::cout << "after {}\n";
}
