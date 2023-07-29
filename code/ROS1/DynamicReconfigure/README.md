# 第三方工具

## 动态调节参数

可以在执行节点时动态调节相关配置参数，相关的功能包为[dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure)，但用起来比较麻烦，需要生成cfg文件，在CMakeLists中添加内容。因此，推荐使用[ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure_python)

### 参考教程

* [dynamic_reconfigure官方教程](http://wiki.ros.org/dynamic_reconfigure/Tutorials)
* [ddynamic_reconfigure
    github教程](https://github.com/pal-robotics/ddynamic_reconfigure_python)

### 生成cfg文件

步骤一：以python为例，需要创建一个python文件用于生成cfg文件

```python
#!/usr/bin/env python
PACKAGE = "dynamic_tutorials"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()
#gen.add(参数名称，参数类型，位掩码，参数描述，默认值，最小值，最大值)
gen.add("int_param",    int_t,    0, "An Integer parameter", 50,  0, 100)
gen.add("double_param", , 0, "A double parameter",    .5, 0,   1)
gen.add("str_param",    str_t,    0, "A string parameter",  "Hello World")
gen.add("bool_param",   bool_t,   0, "A Boolean parameter",  True)

# 枚举类型
size_enum = gen.enum([ gen.const("Small",      int_t, 0, "A small constant"),
                       gen.const("Medium",     int_t, 1, "A medium constant"),
                       gen.const("Large",      int_t, 2, "A large constant"),
                       gen.const("ExtraLarge", int_t, 3, "An extra large constant")],
                     "An enum to set size")

gen.add("size", int_t, 0, "A size parameter which is edited via an enum", 1, 0, 3, edit_method=size_enum)

# gen.generate("包名", "节点名", "Tutorials")
# 参数二：仅用于生成文档
# 参数三：生成文件的前缀(e.g.  "<name>Config.h" for c++, or "<name>Config.py" for python
exit(gen.generate(PACKAGE, "dynamic_tutorials", "Tutorials"))
```

步骤二：在 `package.xml` 中，给 `dynamic_reconfigure` 增设 `build_depend` 和 `exec_depend` 标签(label)

步骤三：在 `CMakeLists.txt` 中，增设 `dynamic_reconfigure` package到 `find_package(catkin REQUIRED COMPONENTS ...)` ；以及增设如下宏，以生成cfg文件：

```cmake
generate_dynamic_reconfigure_options(
  cfg/DynReconf1.cfg
  cfg/DynReconf2.cfg
)
```
