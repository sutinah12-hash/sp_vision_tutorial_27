# ex1：对象与数据

运行前先预测：`&values` 和 `values.data()` 是否相同？

```bash
./build/ex1
```

- `&values` 是 `std::vector` 对象本身的地址。
- `values.data()` 是 vector 管理的连续元素区域的地址。

