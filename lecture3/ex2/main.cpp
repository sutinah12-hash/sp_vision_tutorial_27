#include <chrono>  // 添加计时头文件
#include <iostream>
#include <utility>
#include <vector>

void copy(std::vector<int> data)
{
  std::cout << "copy: copy new data, size = " << data.size() << '\n';
}

void take(std::vector<int> && data)
{
  std::vector<int> owned = std::move(data);
  std::cout << "take(&&): resource may be transferred, size = " << owned.size() << '\n';
}

void borrow(const std::vector<int> & data)
{
  std::cout << "borrow(const&): caller keeps ownership, size = " << data.size() << '\n';
}

int main()
{
  // 数据量：1000万个int，约40MB
  const int N = 10'000'000;

  // ========== 1. 测试拷贝（最慢） ==========
  {
    std::vector<int> data(N, 42);
    auto start = std::chrono::high_resolution_clock::now();

    copy(data);  // 此处发生深拷贝

    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << "[耗时] 拷贝: " << ms << " ms\n";
    std::cout << "原数据大小(拷贝后): " << data.size() << "\n\n";
  }

  // ========== 2. 测试移动（极快） ==========
  {
    std::vector<int> data(N, 42);
    auto start = std::chrono::high_resolution_clock::now();

    take(std::move(data));  // 触发右值引用版本

    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << "[耗时] 移动: " << ms << " ms\n";
    std::cout << "原数据大小(移动后): " << data.size() << "\n\n";
  }

  // ========== 3. 测试只读借用（零开销） ==========
  {
    std::vector<int> data(N, 42);
    auto start = std::chrono::high_resolution_clock::now();

    borrow(data);  // 匹配 const&，只读不碰数据

    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << "[耗时] 只读借用(const&): " << ms << " ms\n";
    std::cout << "原数据大小(借用后): " << data.size() << "\n\n";
  }

  return 0;
}