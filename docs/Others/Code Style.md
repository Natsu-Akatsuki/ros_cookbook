# Code Style

## C++

> [!tip]
> 强烈建议在 ROS2 中遵循面向对象编程，不要使用全局变量来创节点，订阅器，发布器，否则会给你带来笑容：数据可能发不出去；进程结束后有析构错误的问题；运行时有各种奇奇怪怪的报错（需要开 DEBUG 模式来定位）

### Clang

- [ ] 对子类中需要重写的函数使用 `override` 关键字，表明其需要重写

### 命名规范

- [ ] 复合的名字最多不超过三个单词

|       —        |                                 —                                  |        —         |             —             |
|:--------------:|:------------------------------------------------------------------:|:----------------:|:-------------------------:|
|   camelCased   |                      functionName/methodName                       |     onUpdate     | 函数名和方法代表执行某些行为，所以命名一般是动态的 |
|   CamelCased   |                             ClassName                              |      Apple       |           一般是名词           |
|     __XXXX     |                                系统保留                                | __builtin_expect |      一般开发者不需要修改这方面内容      |
|  ALL_CAPITALS  |                              CONSTANT                              |       `PI`       |             —             |
|  ALL_CAPITALS  |                                 宏名                                 | CHECK_CUDA_ERROR |             —             |
|  under_scored  | namespace_name<br />package_name<br />topic_name<br />service_name |        —         |             —             |
| under_scored_  |                          member_varibale                           |        —         |             —             |
| g_under_scored |                         g_global_variable                          |        —         |             —             |

## CMake

## ROS

- [ ] 版本设置为 3.8
- [ ] command 使用小写（find_package, not FIND_PACKAGE）
- [ ] 标识符（identifiers，即 variables, functions, macros) 使用 snake_case
- [ ] 左括号前不加空格
- [ ] 使用两个空格缩进，不适用 Tab
- [ ] Do not use aligned indentation for parameters of multi-line macro invocations. Use two spaces only.

## Custom

TODO

## Launch

1）代码格式化：VSCode 中使用 `XML Formatter` 进行格式化，缩进为 2 空格 \
2）尽量使用 `substitution` 标签 \
3）添加注释和使用 `docs` 属性来描述信息

![](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/xUZKgvoo1W7666ia.png!thumbnail ':size=700')

4）使用条件属性来管理参数

[//]: # (@formatter:off)
```xml
<!-- ROS1 -->
<arg name="launch_dummy_perception" value="false" if="$(arg scenario_simulation)"/>
<arg name="launch_dummy_perception" value="true" unless="$(arg scenario_simulation)"/>

<group if="$(arg foo)">
    <!-- stuff that will only be evaluated if foo is true -->
</group>
```
[//]: # (@formatter:on)

## Reference

- Code style for [ROS2](https://docs.ros.org/en/iron/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)