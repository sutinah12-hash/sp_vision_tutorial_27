# ex3：`cv::Mat` 的浅拷贝与深拷贝

运行前预测三件事：

1. `shared.data` 是否等于 `original.data`？
2. 只修改 `shared` 后，`original` 的像素是否变化？
3. `independent.data` 是否等于 `original.data`？

```bash
./build/ex3
```

需要记住：

```cpp
cv::Mat shared = original;              // Mat header 不同，像素 buffer 共享
cv::Mat independent = original.clone(); // 独立复制像素 buffer
```

