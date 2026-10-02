#include <iostream>
#include <vector>

int main()
{
  int x = 42;
  std::vector<int> v(10);
  v.push_back(1);

  int * p = v.data();  // a trick for Python Tutor

  std::cout << "&v = " << &v << "\n";
  std::cout << "v.data = " << p << "\n";
  std::cout << "*v.data = " << *p << "\n";
  std::cout << "v.size() = " << v.size() << "\n";
  std::cout << "v.capacity() = " << v.capacity() << "\n";

  return 0;
}