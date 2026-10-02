#include <iostream>

#include "hello.hpp"

int main()
{
  std::cout << "========================================\n";
  std::cout << "       C++ 基础知识演示程序\n";
  std::cout << "========================================\n";

  show_basic_types();

  show_operators();

  show_condition();

  show_loops();

  show_array_and_string();

  show_function();

  show_pointer_and_reference();

  show_class();

  std::cout << "\n========================================\n";
  std::cout << "             演示结束\n";
  std::cout << "========================================\n";

  return 0;
}