# ex2：borrow、copy 与 move

初始代码调用 `take(data)`，因此会匹配只读借用重载。把它改为：

```cpp
take(std::move(data));
```

重新编译并运行：

```bash
cmake --build build -j
./build/ex2
```

观察匹配了哪个重载，以及调用后 `data.size()` 的变化。`std::move()` 本身只是允许后续
操作接管资源；真正的资源转移发生在 `take(&&)` 内的移动构造中。移动后的 `data`
仍是合法对象，但不要依赖它原来的内容。
