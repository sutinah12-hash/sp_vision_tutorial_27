#include "hello.hpp"

#include <iostream>
#include <string>

// ============================================================
// 1. 基础数据类型
// ============================================================

void show_basic_types()
{
  std::cout << "\n========== 1. 基础数据类型 ==========\n";

  int age = 18;
  double height = 175.5;
  float weight = 65.5f;
  char grade = 'A';
  bool is_student = true;

  std::cout << "int age = " << age << std::endl;
  std::cout << "double height = " << height << std::endl;
  std::cout << "float weight = " << weight << std::endl;
  std::cout << "char grade = " << grade << std::endl;
  std::cout << "bool is_student = " << is_student << std::endl;
}

// ============================================================
// 2. 运算符
// ============================================================

void show_operators()
{
  std::cout << "\n========== 2. 运算符 ==========\n";

  int a = 10;
  int b = 3;

  std::cout << "a = " << a << std::endl;
  std::cout << "b = " << b << std::endl;

  std::cout << "a + b = " << a + b << std::endl;
  std::cout << "a - b = " << a - b << std::endl;
  std::cout << "a * b = " << a * b << std::endl;
  std::cout << "a / b = " << a / b << std::endl;
  std::cout << "a % b = " << a % b << std::endl;

  std::cout << "a > b : " << (a > b) << std::endl;
  std::cout << "a == b : " << (a == b) << std::endl;
}

// ============================================================
// 3. 条件判断
// ============================================================

void show_condition()
{
  std::cout << "\n========== 3. 条件判断 ==========\n";

  int score = 85;

  std::cout << "score = " << score << std::endl;

  if (score >= 90) {
    std::cout << "等级：A" << std::endl;
  } else if (score >= 80) {
    std::cout << "等级：B" << std::endl;
  } else if (score >= 60) {
    std::cout << "等级：C" << std::endl;
  } else {
    std::cout << "等级：D" << std::endl;
  }
}

// ============================================================
// 4. 循环
// ============================================================

void show_loops()
{
  std::cout << "\n========== 4. 循环 ==========\n";

  std::cout << "for 循环：";

  for (int i = 0; i < 5; ++i) {
    std::cout << i << " ";
  }

  std::cout << std::endl;

  std::cout << "while 循环：";

  int i = 0;

  while (i < 5) {
    std::cout << i << " ";
    ++i;
  }

  std::cout << std::endl;
}

// ============================================================
// 5. 数组和字符串
// ============================================================

void show_array_and_string()
{
  std::cout << "\n========== 5. 数组和字符串 ==========\n";

  int numbers[5] = {10, 20, 30, 40, 50};

  std::cout << "数组内容：";

  for (int i = 0; i < 5; ++i) {
    std::cout << numbers[i] << " ";
  }

  std::cout << std::endl;

  std::string name = "CMake";

  std::cout << "字符串：" << name << std::endl;
  std::cout << "字符串长度：" << name.length() << std::endl;
}

// ============================================================
// 6. 函数
// ============================================================

int add(int a, int b) { return a + b; }

void show_function()
{
  std::cout << "\n========== 6. 函数 ==========\n";

  int a = 10;
  int b = 20;

  int result = add(a, b);

  std::cout << a << " + " << b << " = " << result << std::endl;
}

// ============================================================
// 7. 指针和引用
// ============================================================

void show_pointer_and_reference()
{
  std::cout << "\n========== 7. 指针和引用 ==========\n";

  int value = 100;

  // 指针
  int * ptr = &value;

  std::cout << "value = " << value << std::endl;
  std::cout << "*ptr = " << *ptr << std::endl;

  // 通过指针修改变量
  *ptr = 200;

  std::cout << "修改后 value = " << value << std::endl;

  // 引用
  int & ref = value;

  ref = 300;

  std::cout << "通过引用修改后 value = " << value << std::endl;
}

// ============================================================
// 8. 类和对象
// ============================================================

Student::Student(const std::string & name, int age) : name_(name), age_(age) {}

void Student::introduce()
{
  std::cout << "大家好，我叫 " << name_ << "，今年 " << age_ << " 岁。" << std::endl;
}

void show_class()
{
  std::cout << "\n========== 8. 类和对象 ==========\n";

  Student student("小明", 18);

  student.introduce();
}