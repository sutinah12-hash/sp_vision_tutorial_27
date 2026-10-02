# 现代 C++ 与 OpenCV 入门

代码使用 C++17；`ex1`～`ex4` 是动手实验，`demos` 是演讲中通过 Python Tutor 展示的代码, `hw` 是作业。

## 环境

推荐 Ubuntu 22.04：

```bash
sudo apt update
sudo apt install -y g++ cmake libopencv-dev
```

## 首次编译

克隆仓库后，在仓库根目录执行：

```bash
cmake -S . -B build
cmake --build build -j
```

编译完成后，可执行文件直接生成在 `build` 目录中。

## 练习顺序

| 目录 | 可执行文件 | 主题 |
| --- | --- | --- |
| `ex1` | `ex1` | 对象与其管理的数据可能位于不同地址 |
| `ex2` | `ex2` | 借用、复制与 `std::move` 的调用意图 |
| `ex3` | `ex3` | `cv::Mat` 浅拷贝、`clone()` 深拷贝与 ROI |
| `ex4` | `ex4` | thread、lambda、mutex 与 RAII |

例如：

```bash
./build/ex1
./build/ex2
./build/ex3
./build/ex4
```

