# C++ 概念学习笔记

这个文件用来记录学习 C++ 语言本身概念时的问答，和 week1.md（ROS2/MuJoCo 工程实践向）分开，专注在语言层面的东西：指针/引用、RAII、模板、内存模型、STL 容器、动态库加载等。

## 现代 C++ 核心特点总览

现代 C++ 的新特性，从高层看并不是零散语法，而是在解决 C 风格开发中的几类核心问题。

| 核心特点 | 标签 | 代表特性 | 要解决的问题 |
|---|---|---|---|
| **更强的类型表达能力** | `#cpp_更强的类型表达能力` | 引用、`const`、`enum class`、`explicit`、`optional`、`variant`、concepts | 不要靠约定、魔法值和注释表达意图 |
| **资源自动管理** | `#cpp_资源自动管理` | 构造/析构、RAII、`std::string`、容器、智能指针 | 避免忘记释放内存、文件、锁等资源 |
| **所有权明确化** | `#cpp_所有权明确化` | 值、引用、裸指针、`unique_ptr`、`shared_ptr`、移动语义 | 明确"谁拥有对象、谁可以修改、对象活多久" |
| **泛型与抽象增强** | `#cpp_泛型与抽象增强` | 模板、`auto`、lambda、ranges、concepts | 在不牺牲性能的前提下复用算法和数据结构 |
| **更高层的标准库** | `#cpp_更高层的标准库` | `vector`、`string`、`map`、算法、ranges、`optional` 等 | 少重复造轮子，少直接操作裸内存 |
| **更安全的默认写法** | `#cpp_更安全的默认写法` | `{}` 初始化、`nullptr`、范围 `for`、`const`、`enum class` | 将常见错误变成编译期错误，或让代码更难写错 |
| **零成本抽象** | `#cpp_零成本抽象` | 模板、内联、移动语义、RAII、ranges | 写出高层、可读的代码，同时接近手写 C 的性能 |
| **组合式编程** | `#cpp_组合式编程` | lambda、算法、ranges、结构化绑定 | 从"手写循环和状态控制"转向"描述数据处理意图" |
| **并发与内存模型**（未直接对应上表） | `#cpp_并发与内存模型` | `std::atomic`、`std::mutex`、`lock_guard`/`scoped_lock`、memory order | 让"这块内存会被多个线程访问"进入类型系统，而不是靠祈祷 |
| **语言组织机制**（未直接对应上表，但常配合其他标签出现） | `#cpp_语言组织机制` | 命名空间、内部链接、编译单元、类/继承/访问控制 | 控制符号可见性、避免命名冲突、组织代码结构 |
| **设计模式**（未直接对应上表） | `#cpp_设计模式` | 单例（Meyer's Singleton）、RAII 包装、工厂等 | 用语言机制（如 static 局部变量的初始化保证）实现经典设计模式 |

可以把它概括为下面这句话：

> C++ 保留 C 对底层资源和性能的控制力，但试图用类型、对象生命周期和标准库，把大量容易出错的手工管理工作交给编译器和库。

## 格式约定

- 每个问答用 `###` 标题起一个简短描述性标题（不再用生硬的"Q1/Q2"编号），原始提问用 `>` 引用块保留在标题下第一行，方便区分"我问的原话"和"整理后的回答"。
- 每条问答上方用**标签行**标注分类，标签统一带 `#cpp_` 前缀，对应上面表格的"标签"列，方便用编辑器/grep 检索。一条问答涉及多个分类时，标签行里空格分隔列出多个标签。
- 下面的"标签索引"表用超链接把每个标签指向涉及它的问答小节标题，可以按主题横向查找，不用从头往下翻。
- 问答顺序按学习时间正序排列（后面的问答可能引用前面的概念）。

## 标签索引

| 标签 | 涉及的问答 |
|---|---|
| `#cpp_语言组织机制` | [namespace 与匿名 namespace](#namespace-与匿名-namespace)、[class 基础（以 MujocoBridgeNode 为例）](#class-基础以-mujocobridgenode-为例)、[头文件声明 vs .cpp 定义、无命名空间的 C 类型](#头文件声明-vs-cpp-定义无命名空间的-c-类型)、[头文件中的 inline 纯函数](#头文件中的-inline-纯函数以-camera_geometryhpp-为例)、[单例初始化用匿名 lambda 的原因](#单例初始化用匿名-lambda-的原因)、[resolve 模板与 dlopen/dlsym](#resolve-模板与-dlopendlsym)、[虚函数与多态：以 `WaypointSource` 为例](#虚函数与多态以-waypointsource-为例)、[`static constexpr`：类作用域下的编译期常量](#static-constexpr类作用域下的编译期常量)、[GoogleTest fixture、作用域与所有权（以 Stage L 为例）](#googletest-fixture作用域与所有权以-stage-l-为例) |
| `#cpp_更安全的默认写法` | [namespace 与匿名 namespace](#namespace-与匿名-namespace)、[class 基础（以 MujocoBridgeNode 为例）](#class-基础以-mujocobridgenode-为例)、[头文件中的 inline 纯函数](#头文件中的-inline-纯函数以-camera_geometryhpp-为例)、[并发：data race 是 UB，以及为什么不要提前加锁](#并发data-race-是-ub以及为什么不要提前加锁)、[虚函数与多态：以 `WaypointSource` 为例](#虚函数与多态以-waypointsource-为例)、[`static constexpr`：类作用域下的编译期常量](#static-constexpr类作用域下的编译期常量)、[GoogleTest fixture、作用域与所有权（以 Stage L 为例）](#googletest-fixture作用域与所有权以-stage-l-为例) |
| `#cpp_所有权明确化` | [class 基础（以 MujocoBridgeNode 为例）](#class-基础以-mujocobridgenode-为例)、[头文件声明 vs .cpp 定义、无命名空间的 C 类型](#头文件声明-vs-cpp-定义无命名空间的-c-类型)、[GoogleTest fixture、作用域与所有权（以 Stage L 为例）](#googletest-fixture作用域与所有权以-stage-l-为例) |
| `#cpp_设计模式` | [单例初始化用匿名 lambda 的原因](#单例初始化用匿名-lambda-的原因) |
| `#cpp_泛型与抽象增强` | [resolve 模板与 dlopen/dlsym](#resolve-模板与-dlopendlsym)、[虚函数与多态：以 `WaypointSource` 为例](#虚函数与多态以-waypointsource-为例) |
| `#cpp_零成本抽象` | [头文件中的 inline 纯函数](#头文件中的-inline-纯函数以-camera_geometryhpp-为例)、[resolve 模板与 dlopen/dlsym](#resolve-模板与-dlopendlsym) |
| `#cpp_并发与内存模型` | [并发：data race 是 UB，以及为什么不要提前加锁](#并发data-race-是-ub以及为什么不要提前加锁) |
| `#cpp_资源自动管理` | [并发：data race 是 UB，以及为什么不要提前加锁](#并发data-race-是-ub以及为什么不要提前加锁)、[GoogleTest fixture、作用域与所有权（以 Stage L 为例）](#googletest-fixture作用域与所有权以-stage-l-为例) |
| `#cpp_更强的类型表达能力` | [`std::optional`、`mutable` 与接口常量性（以 Stage N 为例）](#stdoptionalmutable-与接口常量性以-stage-n-为例) |

## 目录

  - [namespace 与匿名 namespace](#namespace-与匿名-namespace)
  - [class 基础（以 MujocoBridgeNode 为例）](#class-基础以-mujocobridgenode-为例)
  - [头文件声明 vs .cpp 定义、无命名空间的 C 类型](#头文件声明-vs-cpp-定义无命名空间的-c-类型)
  - [头文件中的 inline 纯函数（以 `camera_geometry.hpp` 为例）](#头文件中的-inline-纯函数以-camera_geometryhpp-为例)
  - [单例初始化用匿名 lambda 的原因](#单例初始化用匿名-lambda-的原因)
  - [resolve 模板与 dlopen/dlsym](#resolve-模板与-dlopendlsym)
  - [虚函数与多态：以 `WaypointSource` 为例](#虚函数与多态以-waypointsource-为例)
  - [`static constexpr`：类作用域下的编译期常量](#static-constexpr类作用域下的编译期常量)
  - [并发：data race 是 UB，以及为什么不要提前加锁](#并发data-race-是-ub以及为什么不要提前加锁)
  - [GoogleTest fixture、作用域与所有权（以 Stage L 为例）](#googletest-fixture作用域与所有权以-stage-l-为例)
    - [GoogleTest、`TEST` 与 `TEST_F`](#googletesttest-与-test_f)
  - [`std::optional`、`mutable` 与接口常量性（以 Stage N 为例）](#stdoptionalmutable-与接口常量性以-stage-n-为例)
    - [`std::optional` 是可选值，不是序列容器](#stdoptional-是可选值不是序列容器)
  - [待补充问答模板](#待补充问答模板)

---

### namespace 与匿名 namespace

`#cpp_语言组织机制` `#cpp_更安全的默认写法`

> Q: `src/mujoco_bridge` 里的 `namespace mujoco_bridge` 是在哪定义、如何作用的？还注意到一个匿名的 namespace，是干什么的？

**`namespace mujoco_bridge` —— 三个文件里是同一个命名空间**

它在三个地方各自"打开"了一次，但都是同一个命名空间，效果是叠加（union）而不是覆盖：

- `mujoco_bridge_node.cpp` — 定义了 `MujocoBridgeNode` 类
- `mujoco_dl.hpp` — 声明了 `MujocoApi` 结构体和 `loadMujocoApi()` 函数
- `mujoco_dl.cpp` — 定义了 `loadMujocoApi()` 的实现

`namespace X { ... }` 不是"声明"一次性的容器，而是每次遇到都往同一个名字下追加内容——就像同一个文件夹被多次打开、往里面放东西。所以 `.hpp` 里声明的 `MujocoApi`/`loadMujocoApi`，和 `.cpp` 里给出的实现，虽然写在不同文件、不同的 `namespace mujoco_bridge { }` 块里，但都属于同一个 `mujoco_bridge::` 作用域，链接器看到的是同一个符号 `mujoco_bridge::loadMujocoApi`。

作用主要是两点：

1. **避免符号冲突**：`MujocoApi`、`loadMujocoApi` 这类名字如果直接放在全局作用域，很容易和 ROS2/MuJoCo 自身的符号撞名；包进 `mujoco_bridge::` 后，外部要用就必须写全 `mujoco_bridge::loadMujocoApi()`（或 `using namespace`/`using`）。
2. **对应包名**：ROS2 生态的常见约定——包名 `mujoco_bridge` 对应同名命名空间，一眼就能看出符号属于哪个包，如 `mujoco_bridge::MujocoBridgeNode`。

**匿名命名空间**（`mujoco_dl.cpp`）

```cpp
namespace mujoco_bridge
{
namespace          // 匿名
{
constexpr const char * kMujocoLibPath = "...";
template<typename FuncPtr>
void resolve(...) { ... }
}  // 匿名 namespace 结束
...
}
```

这是嵌套在 `mujoco_bridge` 里面的第二层命名空间，但没有名字。效果：

- **内部链接（internal linkage）**：`kMujocoLibPath` 和 `resolve()` 只在 `mujoco_dl.cpp` 这个编译单元内可见，其他 `.cpp` 文件即使 `#include` 了同名声明也链接不到它们——编译器给它们生成一个该翻译单元独有、外部不可见的符号。
- 等价于 C 里的 `static` 全局变量/函数，但匿名命名空间是 C++ 推荐的写法（`static` 用在命名空间作用域上是老式写法，C++11 起不推荐）。
- 用途很明确：`kMujocoLibPath`（硬编码库路径）和 `resolve`（dlsym 辅助模板）都是这个文件的实现细节，不应该被其他文件看到或误用，所以特意再包一层匿名的、仅本文件可见的空间。

一句话总结：外层 `mujoco_bridge` 命名空间是"这个包对外/对内共享的名字空间"，内层匿名命名空间是"这个 .cpp 文件私有、别的文件看不到"的名字空间——两者解决的是不同粒度的命名可见性问题。

**如何"使用"一个命名空间里的东西**（上面只讲了怎么往里面加东西，这里补上怎么从外面取）

命名空间里的名字默认不会自动"冒出来"到外层作用域，要拿到里面的东西，有三种方式，安全程度递减：

1. **完全限定名（最安全，项目里实际用的方式）**

   ```cpp
   rclcpp::spin(std::make_shared<mujoco_bridge::MujocoBridgeNode>());
   ```

   `mujoco_bridge_node.cpp` 的 `main()` 里就是这么用的——每次用到 `MujocoBridgeNode` 都写全 `mujoco_bridge::MujocoBridgeNode`。好处是任何地方看到这行代码都能立刻知道这个符号来自哪个命名空间，不会跟别的同名符号混淆。代价是写起来啰嗦。

2. **using 声明（引入单个名字，作用域内有效）**

   ```cpp
   using mujoco_bridge::MujocoBridgeNode;
   // 之后这个作用域内可以直接写 MujocoBridgeNode，不用加前缀
   ```

   只把这一个名字"借"进当前作用域，不会连带把 `mujoco_bridge` 里其他东西也带进来，冲突风险比下一种小得多。适合在函数体内部或某个 `.cpp` 文件顶部用。

3. **using 指令（引入整个命名空间，最危险，尽量别在头文件里写）**

   ```cpp
   using namespace mujoco_bridge;
   // 之后 MujocoApi、loadMujocoApi、MujocoBridgeNode 全部可以直接写，不加前缀
   ```

   把 `mujoco_bridge` 里所有名字都暴露到当前作用域，等于短路了命名空间原本要解决的"避免撞名"问题。尤其**绝对不能**写在头文件的全局作用域——头文件被谁 `#include`，谁的全局作用域就被污染，这是命名空间设计要防止的事故之一。即使要用，也应该限制在某个函数体内部（作用域小、影响范围可控），或者干脆不用，坚持用前两种方式。

**命名空间别名（namespace alias）**——用来给长命名空间起短名字，跟本项目关系不大，但值得提一下：

```cpp
namespace mb = mujoco_bridge;
mb::MujocoBridgeNode node;  // 等价于 mujoco_bridge::MujocoBridgeNode
```

常见于嵌套很深的命名空间（比如某些库的 `foo::bar::detail::impl`），起个别名省得反复打字，同时依然保留限定名的清晰度。

---

### class 基础（以 MujocoBridgeNode 为例）

`#cpp_语言组织机制` `#cpp_所有权明确化` `#cpp_更安全的默认写法`

> Q: 以 `mujoco_bridge_node.cpp` 里的 `MujocoBridgeNode` class 作为样板，教一下 class 的知识。

**1. class 是什么**

`class` 把**数据**（成员变量）和**操作这些数据的函数**（成员函数）捆绑在一起，形成一个新类型。`MujocoBridgeNode` 有 5 个成员变量（`api_`、`model_`、`data_`、`timer_`、`step_count_`），以及构造函数、析构函数、`onTimer()` 三个成员函数。每次构造出一个对象，这些变量就各自有自己的一份存储。

**2. 继承：`: public rclcpp::Node`**

```cpp
class MujocoBridgeNode : public rclcpp::Node
```

表示"`MujocoBridgeNode` 是一种 `rclcpp::Node`"（is-a）。效果：

- 对象内部包含一份完整的 `rclcpp::Node` 子对象，自动获得它所有 public/protected 成员——`get_logger()`、`create_wall_timer()` 都是继承来的，`MujocoBridgeNode` 自己没写却能直接用。
- `public` 继承保持继承接口原有可见性（最常见的继承方式）。
- 任何期望 `rclcpp::Node*`/`rclcpp::Node&` 的地方（如 `rclcpp::spin()`）都能接受 `MujocoBridgeNode` 对象——多态的基础。

**3. 构造函数 + 初始化列表**

```cpp
MujocoBridgeNode()
: Node("mujoco_bridge"), api_(loadMujocoApi())
{ ... }
```

冒号后到 `{` 前是**成员初始化列表**，不是语法糖，是真正的初始化时机：

- `Node("mujoco_bridge")` 调用基类构造——基类必须在派生类构造函数体执行前构造完毕，只能在初始化列表里做。
- `api_(loadMujocoApi())` 初始化引用成员。**引用（`&`）绑定一次后不能重新绑定**，只能在初始化列表里赋值，构造函数体里 `api_ = ...` 是不合法的。
- 没写进列表的成员（`model_`、`data_`、`timer_`、`step_count_`）用各自的**成员默认初始化器**（`= nullptr`、`= 0`，C++11 起的写法），避免"忘记初始化"。
- 初始化顺序永远按**成员声明顺序**（不是初始化列表里写的顺序），是个常见隐藏坑。

**4. 访问控制：`public` / `private`**

- `public:` 是对外接口——外部（main 函数）能调用的只有构造函数和隐式析构。
- `private:` 下的 `onTimer()` 和所有成员变量只有类内部代码能访问，外部拿到引用也读不到、改不了。

这是**封装（encapsulation）**：类自己保证内部状态一致性（`model_` 和 `data_` 必须配对存在），外部只能通过公开接口交互。

**5. 析构函数：`~MujocoBridgeNode() override`**

```cpp
~MujocoBridgeNode() override
{
  if (data_) { api_.deleteData(data_); }
  if (model_) { api_.deleteModel(model_); }
}
```

- 对象销毁时自动调用，负责清理构造函数申请的资源——这里手动调 MuJoCo 的 `deleteData`/`deleteModel`，因为这是 C 风格裸指针资源，没有智能指针管理生命周期（对比之下能看出为什么 RAII/智能指针更好，见表格"资源自动管理"一行）。
- `override` 让编译器检查"这确实覆盖了基类的虚函数"（`rclcpp::Node` 的析构函数是 `virtual`）。写错签名时编译器会报错，而不是悄悄定义出一个不相关的新函数、导致基类指针销毁时不会调到派生类析构逻辑。

**6. 成员函数访问自己的数据**

```cpp
void onTimer()
{
  api_.step(model_, data_);
  ++step_count_;
  ...
}
```

成员函数天然能访问同一对象的所有成员，不需要写 `this->model_`（隐含的 `this` 指针）。

**7. 成员函数当回调传递：`std::bind`**

```cpp
timer_ = create_wall_timer(..., std::bind(&MujocoBridgeNode::onTimer, this));
```

`onTimer` 是非静态成员函数，真实签名隐含一个 `this` 参数。普通函数指针语法接不住它，必须用 `std::bind(&Class::method, 对象指针)` 把"哪个对象、调用哪个函数"打包成无参可调用对象，定时器到时直接 `()` 调用。

---

### 头文件声明 vs .cpp 定义、无命名空间的 C 类型

`#cpp_语言组织机制` `#cpp_所有权明确化`

> Q: 纯语法角度，关于 `mujoco_dl.hpp`：
> 1. `MujocoApi & loadMujocoApi();` 是什么意思？这里的 `&` 是什么？`loadMujocoApi` 不是一个在别处定义好的函数吗？
> 2. bridge 的 cpp 文件只要包含了这个 `dl.hpp`，就可以访问 `dl.cpp` 里定义的函数了吗？
> 3. 为什么 `mjModel` 和 `mjData` 不需要指定它们的命名空间？

**1. `MujocoApi & loadMujocoApi();`**

拆开看：`MujocoApi &` 是**返回类型**——返回一个 `MujocoApi` 的**引用**，不是返回值本身、也不是返回指针。`&` 只是返回类型的一部分，跟"这个函数是不是在别处定义"没有关系，这是两个独立的语法层面：

- **`&` 的语义**：返回引用。
- **声明 vs 定义**：`mujoco_dl.hpp` 里只有一行分号结尾的**声明**（没有函数体），告诉编译器"存在这样一个函数、长这个签名"。真正的**定义**（函数体）在 `mujoco_dl.cpp`：

  ```cpp
  MujocoApi & loadMujocoApi()
  {
    static MujocoApi api = []() { ... }();
    return api;
  }
  ```

  确实是在别处定义好的函数，头文件那行只是"说明书"。这是标准的 C/C++ 模式：头文件放声明给别的文件看接口，`.cpp` 放实现，靠完全一致的签名对应起来。

  为什么返回引用而不是返回值：函数体里 `static MujocoApi api = ...;` 是**静态局部变量**，只在第一次调用时构造一次，之后每次调用复用同一个对象（惰性初始化单例，详见 [单例初始化用匿名 lambda 的原因](#单例初始化用匿名-lambda-的原因)）。返回引用让调用者拿到"这个唯一实例本身的别名"，不发生拷贝，保证所有调用者看到同一份 `MujocoApi`。如果返回值，每次调用都会拷贝一份，浪费且违背"单例"的意图。

**2. `#include` 头文件就能访问 `.cpp` 里定义的函数吗**

分两层：

- **编译期**：`#include` 只是把头文件里的**声明**文本粘贴进当前文件，让编译器知道"有一个这样签名的函数"，语法检查能通过——但编译器不知道函数体在哪，也不关心。
- **链接期**：真正"接起来"靠链接器。`mujoco_bridge_node.cpp` 编译出的 `.o` 里有一处"未解决的引用"，`mujoco_dl.cpp` 编译出的 `.o` 里有完整机器码；只要构建系统（`CMakeLists.txt`）把这两个 `.o` 一起链接进同一个目标，链接器才会把调用点接到真正的函数地址。

  所以准确说法是：`#include` 只解决"编译器认不认识这个名字"，能不能找到函数体要看构建配置有没有把 `mujoco_dl.cpp` 也编译并链接进来。如果 `CMakeLists.txt` 忘了加，即使 `#include` 了头文件，也会在**链接阶段**报 `undefined reference` 错误——`#include` 从不负责链接。

**3. 为什么 `mjModel`、`mjData` 不需要命名空间**

因为它们压根不在任何命名空间里——MuJoCo 是纯 **C 风格 API**，头文件用的是纯 C 语法：

```c
typedef struct mjModel_ mjModel;   // mjmodel.h
typedef struct mjData_  mjData;    // mjdata.h
```

`/opt/mujoco-3.3.7/include/mujoco/` 下的头文件完全没有 `namespace` 关键字。这两个类型是 `typedef` 出来的别名，直接躺在**全局命名空间**（C 语言没有命名空间概念）。

对比项目自己写的 `MujocoApi`——它被包进 `namespace mujoco_bridge { ... }`，外部要用得写 `mujoco_bridge::MujocoApi`。而 `mjModel`/`mjData` 从定义时就没被任何 `namespace` 块包裹，所以任何地方都能直接写，不需要、也没有前缀可加。

一句话：是否需要命名空间前缀，完全取决于这个类型/函数**定义时**有没有被塞进某个 `namespace` 块，跟使用的位置无关。

---

### 头文件中的 inline 纯函数（以 `camera_geometry.hpp` 为例）

`#cpp_语言组织机制` `#cpp_零成本抽象` `#cpp_更安全的默认写法`

> Q: `camera_geometry.hpp` 里的 `metricDepth()` 和 `backproject()` 为什么直接写在 `.hpp` 里，还要加 `inline`？为什么不把它们的实现放到 `.cpp`？

这两个函数分别完成：

```cpp
metricDepth(...)  // OpenGL 深度缓冲值 → 米单位的 z-depth
backproject(...)  // 像素坐标 + z-depth → optical frame 三维点
```

它们有几个共同特点：

- 是无状态的纯函数，只依赖参数，不访问 ROS、MuJoCo 或对象成员；
- 公式很短，接口和实现放在一起更容易直接阅读；
- `rgbd_camera.cpp` 的渲染路径和 `test_camera_geometry.cpp` 的单测都需要调用；
- 它们代表一个需要被多个调用者共享的几何契约，而不是某个 `.cpp` 的私有辅助函数。

因此这里选择一个小型 header-only 几何模块：调用者包含头文件即可使用，单测也可以直接针对公式测试，不需要为了两个公式再增加一个独立的链接边界。

#### `inline` 主要解决什么问题？

`camera_geometry.hpp` 会被多个编译单元包含，例如：

```text
rgbd_camera.cpp
test_camera_geometry.cpp
```

如果头文件中写的是普通外部函数定义：

```cpp
// 错误示例：被多个 .cpp 包含后会有多个外部定义
double metricDepth(double d, double near_m, double far_m)
{
  ...
}
```

每个编译单元都会生成一份同名外部符号。链接器把这些目标文件合在一起时，可能报 `multiple definition`。这违反的是 C++ 的 ODR（One Definition Rule）：一个具有外部链接的普通函数通常只能在整个程序中有一个定义。

加上：

```cpp
inline double metricDepth(...)
{
  ...
}
```

表示这个函数允许在多个翻译单元中出现相同定义，只要这些定义完全一致；它们仍然代表同一个程序实体。`#pragma once` 只能防止同一个编译单元重复包含同一头文件，不能解决不同 `.cpp` 各自包含头文件的问题，所以 `#pragma once` 不能替代 `inline`。

#### `inline` 不等于“强制内联机器码”

`inline` 这个关键字的首要作用是链接和 ODR 规则，不是性能指令。编译器可以：

- 即使没有 `inline`，也把一个小函数优化成内联代码；
- 即使写了 `inline`，也因为调试、优化级别或代码形态而不内联。

因此这里写 `inline` 的理由是“允许安全地在头文件定义”，不能表述成“保证函数调用没有开销”。

#### 为什么不采用 `.hpp` 声明、`.cpp` 定义？

完全可以采用普通的分离式写法：

```cpp
// camera_geometry.hpp
double metricDepth(double, double, double);

// camera_geometry.cpp
double metricDepth(double d, double near_m, double far_m)
{
  ...
}
```

这种写法由 `.cpp` 提供唯一外部定义，头文件只暴露接口，适合实现较大、经常变化、需要隐藏依赖或需要稳定 ABI 的函数。代价是必须把 `camera_geometry.cpp` 加入 CMake 目标并正确链接；如果漏加，就会出现 `undefined reference`。

当前两个函数选择 header-only 是因为它们很短、依赖极少，而且测试和渲染实现都直接共享同一份公式。它不是“函数只能放在 hpp”的规则，而是针对小型纯函数的取舍。

#### 为什么不用命名空间作用域的 `static`？

也可以用 `static` 让每个编译单元各自拥有一份内部链接函数，但那会产生多个独立实体，失去共享同一外部 API 的语义，并可能增加代码体积。这里函数是公共的 `mujoco_bridge::` 几何接口，因此用 `inline` 表示“可以多处定义但仍是同一个接口”更合适。

#### 这两个函数为什么适合单测？

它们不需要启动 ROS 节点、不需要创建 OpenGL context、也不需要加载 MuJoCo 模型：

```cpp
EXPECT_DOUBLE_EQ(mujoco_bridge::metricDepth(0.0, 0.01, 10.0), 0.01);
EXPECT_DOUBLE_EQ(point.z, 0.61);
```

测试可以独立检查：

- 深度缓冲边界和无效值是否返回 NaN；
- 主点反投影是否落在 `x=0,y=0`；
- 像素向右移动时，光学 frame 的 `x` 是否为正。

这体现了“纯公式放在可直接包含的头文件中，ROS/MuJoCo 胶水留在 `.cpp`”的分层：公式容易快速单测，渲染和消息发布再由集成测试验证。

---

### 单例初始化用匿名 lambda 的原因

`#cpp_设计模式` `#cpp_语言组织机制`

> Q: `mujoco_dl.cpp` 的 `loadMujocoApi()` 里为什么要用匿名 lambda（`[]() { ... }()`）？去掉它、把内容直接摊开写在外层函数体里可以吗？这是为了和单例配合吗？

**匿名 lambda 部分**

```cpp
static MujocoApi api = []() {
  void * handle = dlopen(kMujocoLibPath, RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
  if (!handle) { throw ...; }
  MujocoApi api;
  resolve(handle, "mj_loadXML", api.loadXML);
  ...
  return api;
}();
```

不能直接摊开写。原因是 `static` 局部变量的初始化规则：

```cpp
static MujocoApi api = 初始化表达式;
```

C++11 起保证：这个**初始化表达式**只在第一次执行到这行时求值一次（"magic statics"，线程安全），之后每次调用函数都跳过初始化、复用已存在的 `api`。但这个"只跑一次"的保证**只覆盖初始化表达式本身**，不覆盖函数体里其他普通语句。如果去掉 lambda 摊开写：

```cpp
static MujocoApi api;               // 只在首次调用时构造一次
void * handle = dlopen(...);        // 普通语句，不受 static 保护
if (!handle) { throw ...; }
resolve(handle, "mj_loadXML", api.loadXML);
...
```

`static MujocoApi api;` 那行确实只跑一次，但 `dlopen`/`resolve` 这些普通语句**每次调用 `loadMujocoApi()` 都会重新执行**——重新打开一次库、重新查一遍符号表，完全违背"只加载一次、之后复用"的目的。

lambda 的作用是把"`dlopen` → 检查 → 循环 `resolve` → `return api`"这段带分支、多语句的逻辑，打包成**一个表达式**（立即调用 lambda 的返回值），这样才有资格充当 `=` 后面的初始化表达式——C++ 的初始化表达式语法只接受"一个表达式"，不接受"一段代码块"。这个手法叫 IIFE（immediately-invoked function expression）。

（等价的替代写法：抽成具名辅助函数 `static MujocoApi api = buildApi();`，效果完全一样，用匿名 lambda只是因为这段逻辑仅用一次，没必要为它专门起个名字。）

**是否为了配合单例**

是的。这里的写法有专门名字：**Meyer's Singleton**（函数局部 static 单例），是 C++ 里最常见、最推荐的单例实现方式。两者分工不同：

- `static` 局部变量 + 返回引用 → **单例的骨架**，负责"全局只有一份实例，只初始化一次，线程安全"这个语义。这一步跟 lambda 无关——哪怕初始化只是 `static MujocoApi api;`（默认构造），也已经是个单例了。
- lambda IIFE → **单例初始化逻辑的载体**，负责把"`dlopen`+循环 `dlsym`"这种复杂多步逻辑，塞进单例骨架要求的"单一初始化表达式"里。

如果初始化逻辑简单（比如就是 `MujocoApi{}`），根本不需要 lambda。用 lambda 纯粹是因为这次的初始化步骤太复杂，普通表达式语法写不下，又想保住 `static` 带来的单例语义。

---

### resolve 模板与 dlopen/dlsym

`#cpp_泛型与抽象增强` `#cpp_语言组织机制`

> Q: `mujoco_dl.cpp` 里 `resolve` 函数和它用到的 `template` 是什么用法和意义？`dlfcn.h` 相关的 `dlopen`/`dlsym` 分别做什么，`resolve` 调用之后实际发生了什么？本质上是不是就是把 `.so` 共享库里的同名函数接过来了？

```cpp
constexpr const char * kMujocoLibPath = "/opt/mujoco-3.3.7/lib/libmujoco.so.3.3.7";

template<typename FuncPtr>
void resolve(void * handle, const char * symbol, FuncPtr & out)
{
  out = reinterpret_cast<FuncPtr>(dlsym(handle, symbol));
  if (!out) {
    throw std::runtime_error(std::string("dlsym failed for ") + symbol + ": " + dlerror());
  }
}
```

**1. template 在这里的用法和意义**

`resolve` 被调用 7 次，每次第三个参数的类型都不一样（`api.loadXML` 是 `mjModel*(*)(const char*, const mjVFS*, char*, int)`，`api.step` 是 `void(*)(const mjModel*, mjData*)`……）。如果不用模板，得给每种函数指针类型手写一个重载，或者统一用 `void*&` 在每个调用点手写 `reinterpret_cast`。

模板让编译器**按需生成**：每次调用时根据传入 `out` 的真实类型自动推导出 `FuncPtr`（模板参数推导，不需要显式写 `resolve<...>(...)`），为该具体类型生成一份专属实例。好处：

- **类型安全 + 复用**：转换逻辑（`reinterpret_cast` + 判空 + 抛异常）只写一次，对每种函数指针类型都生效，且编译器保证转换目标类型跟 `out` 实际类型一致。
- **零成本**：模板实例化是编译期展开，运行时没有额外分支或虚函数开销，跟手写 7 个重载效果一样，只是省了重复代码。

`FuncPtr & out` 用引用接收（不是返回值），因为要直接修改调用者传进来的成员变量本身（如 `api.loadXML`），不需要额外赋值步骤。

**2. dlfcn 相关：dlopen / dlsym**

`<dlfcn.h>` 是 POSIX 提供的**运行时动态加载**接口，跟平常 `#include` + 链接期解析符号是两条完全不同的路。

- **`dlopen(path, flags)`**：运行时把共享库加载进当前进程地址空间，返回不透明的 `void*` handle（失败返回 `nullptr`）。这里用的三个 flag：
  - `RTLD_NOW`：加载时立刻解析库内所有未定义符号（而非惰性），加载失败会立刻暴露。
  - `RTLD_LOCAL`：这个库导出的符号**不进入进程全局符号表**，别的库看不到、不会误用它内部的符号（隔离 `tinyxml2` 冲突的手段之一，详见 [week1.md 7.2](week1.md#72-调试时踩到的段错误符号冲突与-dlopen-隔离)）。
  - `RTLD_DEEPBIND`：glibc 扩展，让库内部对自己符号的引用优先用自己的定义，不理会进程里其他地方已加载的同名符号——双重保险。
- **`dlsym(handle, symbol)`**：拿一个符号名字符串，去 `handle` 对应库的符号表（ELF `.dynsym`/`.dynstr`）里精确查找，返回该符号在当前进程虚拟地址空间里的实际地址；找不到返回 `nullptr`。
- **`dlerror()`**：返回上一次 `dl*` 调用失败的具体原因，用来给抛出的异常附带详细信息。

**3. resolve 之后实际发生了什么 —— 本质是"把 .so 里同名函数的入口地址接过来"**

一步步拆：

1. `dlsym(handle, "mj_loadXML")` 在 `libmujoco.so` 的符号表里查到 `mj_loadXML`，返回它在内存里的真实地址（已经过动态链接器的重定位校正）。
2. `reinterpret_cast<FuncPtr>(...)` 把这个 `void*` 地址重新解释成具体函数指针类型——纯编译期"贴标签"，不做任何运行时计算，也不校验这个地址背后的函数签名是否真的匹配。
3. `out = ...` 把这个函数指针存进 `api.loadXML` 字段。
4. 之后调 `api_.loadXML(...)`，是通过存好的指针**间接跳转**，直接执行 `libmujoco.so` 里 `mj_loadXML` 的机器码——效果和直接调用 `mj_loadXML(...)` 完全一样。

跟正常链接对比：正常链接下，"查符号表、拿地址"是**链接器在编译期**做的，一次性焊死调用点；这里是把同一个动作挪到了**运行时**，用 `dlopen`+`dlsym` 手动做一遍——查到的还是**同一个符号、同一份机器码**，只是查的时机和方式变了，多了一步"裸地址转带类型函数指针"。所以本质上确实就是"把共享库里的同名函数接过来"，只是这次是运行时手动接线，而不是编译期焊死。

**失败路径**：符号名字打错、或库升级后符号被删/改名，`dlsym` 返回 `nullptr`，`resolve` 立刻 `throw`。因为这发生在 `static MujocoApi api = []() { ... }();` 的 lambda 初始化表达式内部——按 C++ 标准，静态局部变量初始化中途抛异常，这次初始化被视为"没有发生过"，下次调用 `loadMujocoApi()` 会重新尝试整个初始化过程（呼应 [单例初始化用匿名 lambda 的原因](#单例初始化用匿名-lambda-的原因) 的 `static` 单例初始化保证）。

---

### 虚函数与多态：以 `WaypointSource` 为例

`#cpp_泛型与抽象增强` `#cpp_语言组织机制` `#cpp_更安全的默认写法`

> Q: 你在 `WaypointSource` 里使用了 `virtual` 关键字，我不太了解虚函数相关的知识，请你讲解其知识，并且说明为什么这里要用 `virtual`？

**1. 没有 `virtual` 时，调用哪个函数是编译期定死的**

```cpp
struct Base { void f() { /* Base 版本 */ } };
struct Derived : Base { void f() { /* Derived 版本 */ } };

Base * p = new Derived();
p->f();  // 调用哪个 f()？
```

普通（非 `virtual`）成员函数的调用地址在**编译期**就写进了机器码——编译器只看指针 `p` 的**静态类型**（代码里写的类型，`Base *`），完全不管它运行时实际指向什么。上面这段代码会调用 `Base::f()`，即使 `p` 实际指向一个 `Derived` 对象——这通常不是调用方想要的结果，却是没有 `virtual` 时的默认行为，是个常见陷阱。

**2. `virtual` 把这个决定推迟到运行期**

```cpp
struct Base { virtual void f() { /* ... */ } };
```

一个类只要声明了至少一个 `virtual` 函数，它的每个对象在内存里就多带一个隐藏指针（vptr），指向一张按类生成的函数地址表（vtable）。调用 `p->f()` 时，编译器生成的代码变成"先查 `p` 指向对象的 vptr，再从表里取出 `f` 这一槏对应的实际地址，跳过去"——这叫**动态绑定**（运行期才决定调用哪个版本），是"运行期多态"的实现机制。这次是 `p` 实际指向的**动态类型**（`Derived`）说话，不是声明时的静态类型。

**3. `WaypointSource` 用的是纯虚函数（`= 0`），意味着"这是一个接口"**

```cpp
class WaypointSource
{
public:
  virtual ~WaypointSource() = default;
  virtual JointTarget jointTargetFor(Phase phase, const ObjectPose & object_pose) const = 0;
};
```

`= 0` 告诉编译器"这个函数在 `WaypointSource` 这一层没有函数体，必须由派生类提供"——同时把 `WaypointSource` 变成**抽象类**：不能 `new WaypointSource()`（编译器直接拒绝，因为它有未实现的虚函数），只能通过某个真正实现了 `jointTargetFor()` 的派生类（如 `KeyframeWaypointSource`）来用。一个只有纯虚函数、没有数据成员的类，就是 C++ 里写"接口"的标准方式——没有专门的 `interface` 关键字，`class` + 纯虚函数就是等价物。

**4. 为什么 `task_executor` 这里要用这套机制**

调用方（`task_executor_node.cpp`）想表达的是"给我一个能回答 `jointTargetFor()` 的东西，我不关心它具体怎么算的"。这正是虚函数/多态解决的问题——[week2.md 10.5](../my_study/week2.md#105-waypointsource-接口设计为什么现在只有一个查表实现) 已经讲过设计动机（第3周要把 `KeyframeWaypointSource` 换成 diff-IK 实现，`fsm.cpp`/`task_executor_node.cpp` 不用改），这里补语言机制那一半：如果不用虚函数/继承，"换一个实现"就没有对应的语言机制可以表达——要么把两套逻辑都写死在同一个函数里用 `if`/`switch` 分支切换（改动一处就要碰这整个函数），要么每次换实现都得把所有调用点的类型名手改一遍。有了这套接口，调用点看到的类型永远是 `WaypointSource`，具体是哪个派生类只在**构造**那一行决定。

**当前代码的一个诚实的注记**：`task_executor_node.cpp` 目前是 `KeyframeWaypointSource waypoint_source_;`——直接持有具体类型的对象，不是指针/引用/`unique_ptr<WaypointSource>`。这种写法下，编译器在调用点已经知道静态类型就是 `KeyframeWaypointSource`，可以走静态绑定（甚至可能被优化器"去虚化"），虚函数机制此刻并没有真正被用上。它的价值是**面向第3周**：一旦这一行换成 `std::unique_ptr<WaypointSource> waypoint_source_ = std::make_unique<DiffIkWaypointSource>(...);`（具体类型由运行期的某个决定挑选），多态才真正发生。这是"接口先立好、当前只有一个实现"的常见模式（经典说法是**策略模式**/依赖倒置：调用方依赖一个抽象接口，不依赖具体实现），跟 week2.md 10.5 讨论"多一层间接的代价"是同一件事的两个侧面——那里讲的是工程取舍，这里讲的是这个取舍靠哪个语言机制落地。

**5. 析构函数为什么也要 `virtual`**

```cpp
virtual ~WaypointSource() = default;
```

如果不写 `virtual`，通过基类指针 `delete` 一个派生类对象时，只会调用基类自己的析构函数——派生类新增的成员（如果有需要清理的资源）不会被正确销毁，是一个真实的内存/资源泄漏来源。规则很通用：**任何打算被多态使用（会经过基类指针/引用操作）的类，基类析构函数都该是 `virtual`**。[class 基础](#class-基础以-mujocobridgenode-为例) 那节从"派生类怎么覆盖基类析构函数"（`~MujocoBridgeNode() override`，覆盖 `rclcpp::Node` 的虚析构）讲过一次这个机制；这次是从"提供接口的这一侧，为什么必须主动声明这个虚析构"来看同一件事。

**6. `override` 关键字**：`KeyframeWaypointSource::jointTargetFor(...) const override` 末尾的 `override` 不是语法必需（不写也能达到"覆盖"的效果，只要签名完全匹配），但它让编译器额外检查"这确实覆盖了某个基类的虚函数"——手误漏写 `const`、参数类型抄错，都会让编译器把它当成一个**新的、不相关的重载**而不是覆盖，`WaypointSource::jointTargetFor()` 依然是纯虚（未实现），这类错误没有 `override` 时不会在编译期暴露。

---

### `static constexpr`：类作用域下的编译期常量

`#cpp_更安全的默认写法` `#cpp_语言组织机制`

> Q: 解释一下 `keyframe_waypoint_source.hpp` 里使用的 `static constexpr double`，为什么要用这套关键字，我印象中它在 gripper_test 里测试的时候定义过。

`keyframe_waypoint_source.hpp` 的 `private:` 区块里：

```cpp
static constexpr double kOpenWidthM = 0.08;
static constexpr double kClosedWidthM = 0.0;
```

两个关键字各管一件事，叠加起来才是这行代码的完整含义：

**1. `static`（类作用域，不是对象状态）**——不加 `static` 的普通成员变量（`double kOpenWidthM;`）是"每个对象各有一份"：每次 `new KeyframeWaypointSource()`，内存里就多一份 `0.08`，还必须在构造函数（或成员初始化器）里显式赋值。加了 `static`，这个名字就属于**类本身**，不属于任何具体对象——不管创建多少个 `KeyframeWaypointSource` 实例，`kOpenWidthM` 只有一份，甚至不需要任何对象存在就能引用它（虽然这里是 `private`，外部引用不了）。这里恰好合适：`0.08`/`0.0` 是"这个类代表的行为"的固有属性（夹爪开合的两个极值），不是"这一次具体调用"的状态，天然该属于类而不属于对象。

**2. `constexpr`（编译期常量表达式）**——比 `const` 更强的保证：`const` 只承诺"运行后不能再改"，值可以来自运行期计算（比如 `const double x = someFunction();`）；`constexpr` 要求这个值在**编译期**就能求出来，编译器会在用到它的地方直接内联这个数字，效果类似给一个数字standard起了个类型安全的名字。`constexpr` 隐含 `const`（这两个关键字不冲突，但不需要重复写 `const constexpr`）。

**3. 为什么两个叠在一起，而不是用别的写法**：

| 备选写法 | 问题 |
|---|---|
| `#define kOpenWidthM 0.08` 宏 | 没有类型、没有作用域（会污染全局命名空间，任何文件 `#include` 这个头文件后 `kOpenWidthM` 这个名字到处能用、到处能被意外重定义），是纯文本替换，编译器给不出好的错误信息——这正是"更安全的默认写法"这个主题下 C++ 想让你避免的东西 |
| 普通 `private: double kOpenWidthM = 0.08;`（非 static） | 每个对象存一份，纯粹浪费；且不是编译期常量，不能用在要求 `constexpr` 的上下文里（这里恰好没用到，但语义上"这本来就是个常量"没有被表达出来） |
| 函数内部局部 `constexpr double kOpenWidthM = 0.08;` | `kOpenWidthM`/`kClosedWidthM` 在 `jointTargetFor()` 的多个 `case` 分支里重复用到，写成局部变量要么重复声明多次，要么只能在用到的第一个分支里声明、后面分支引用不到（作用域限制在那个 `case` 块内）——类作用域的 `static constexpr` 只需要声明一次，整个类的所有成员函数都能用 |
| 全局命名空间常量 | 泄漏了作用域，容易和其他文件的同名常量冲突；也没有传达"这两个数字是 `KeyframeWaypointSource` 概念上的一部分"这层语义——`private` 修饰符还额外保证了外部代码无法直接读取或依赖这两个数字 |

**4. 和 `fsm.cpp` 里另一种写法的对比**：`fsm.cpp` 的 `kOpen` 分支里也有类似的常量：

```cpp
constexpr double kOpenWidthM = 0.08;
constexpr double kOpenEpsilonM = 0.02;
```

这里**没有** `static`，因为这是在一个自由函数（`step()`）内部的局部变量，不是类成员——局部 `constexpr` 变量本来就没有"每个对象一份"的问题（函数里从来不会有多个对象），`static` 在这里没有必要也不常见（如果写了 `static constexpr` 在函数局部作用域里，语义会变成"这个值只初始化一次，跨多次函数调用共享同一份存储"——对编译期常量而言效果和不加基本一样，只是明确了"只存一份"的意图，不是必须）。这个对比正好说明 `static` 的必要性完全取决于"这个常量归谁管"：类的成员函数之间共享，需要 `static`；单个自由函数内部自己用，不需要。

**5. 你提到 gripper_test.py 里的印象——那是同一个"命名常量"意图，但语言机制完全不同**：

```python
OPEN_WIDTH_M = 0.08
CLOSED_WIDTH_M = 0.0
```

这是 Python 模块顶层的普通变量赋值，不是类成员，也没有编译期/运行期的区分（Python 没有编译期常量这个概念，一切都在运行时求值）。`static`/`constexpr` 这两个关键字在 Python 里根本不存在对应物——Python 靠**全大写命名**这个纯约定来表达"这是个常量，别改它"，语言本身不会在你真的重新赋值时报错（`OPEN_WIDTH_M = 0.5` 完全合法，Python 解释器不会拦你）。C++ 这边 `constexpr` 是**语言强制**的：写 `kOpenWidthM = 0.5;` 在类外或类内都不会编译通过。两者解决的是同一个工程问题（给魔法数字一个名字，别到处抄字面量），但 C++ 能让编译器帮你守住这个约定，Python 只能靠人自律。

---

### 并发：data race 是 UB，以及为什么不要提前加锁

`#cpp_并发与内存模型` `#cpp_资源自动管理` `#cpp_更安全的默认写法`

> Q: 你先前提到线程安全，我想要对 cpp 写 ros2 或者 python 写 ros2 的并发相关的知识，实践中使用频繁吗，是否写代码的时候总是要考虑未来 go parallel 的可能性？

这条只记 **C++ 语言层面**的部分。ROS2 的执行器/回调组模型、`spin_until_future_complete` 死锁、以及"要不要为并行做准备"的工程结论，在 [week1.md 10.8](week1.md#108-并发ros2-的执行器模型以及要不要提前为并行做准备)。

#### 最重要的一条：data race 是 UB，不是"读到旧值"

这是最容易建错的心智模型。很多人以为数据竞争最坏也就是读到过时的数据——**不是**。C++ 标准规定，两个线程在没有同步的情况下访问同一内存位置、且至少一个是写，程序行为**未定义**。

后果不是"值不新鲜"，而是编译器**基于"无竞争"这个前提做优化**。经典例子：

```cpp
bool done = false;                 // 裸 bool，没有同步

// 线程 A
while (!done) { /* 等 */ }         // 编译器：done 在这个循环里没被改过
                                   // → 可以提到循环外读一次 → 变成 while(true)
// 线程 B
done = true;
```

这个循环可能**永远不退出**，而且是在 `-O2` 下才出现、`-O0` 下正常——这类"加了优化才挂"的 bug 极难查。编译器没做错任何事：既然标准说无竞争，它就可以假设 `done` 不会被别人改。

正确写法是让"这个变量会被并发访问"进入类型系统：

```cpp
std::atomic<bool> done{false};     // 现在编译器知道不能这么优化
```

`std::atomic` 的两层含义要分清：

1. **操作不可分割**——`++counter` 对 `int` 是"读-改-写"三步，可以被打断；对 `std::atomic<int>` 是一步。
2. **建立了同步关系**——这是更常被忽略的一层。它约束编译器和 CPU **不许把周围的读写重排到它两侧**，从而让"线程 B 写 `done` 之前做的事，线程 A 看到 `done` 之后也能看到"。

memory order 的细节（`relaxed`/`acquire`/`release`/`seq_cst`）现阶段不需要精通。**默认的 `seq_cst` 最强也最慢，但永远正确**；先用它，等真的测出瓶颈再考虑放松。需要的直觉只有一句：**放松 memory order 是拿正确性换性能，而且错了不会报错。**

#### `std::mutex` 与 RAII：永远不要手动 lock/unlock

```cpp
std::mutex mu_;

// ❌ 别这么写
void bad() {
  mu_.lock();
  if (something) return;        // ☠️ 忘了 unlock，死锁
  mayThrow();                   // ☠️ 抛异常也不会 unlock
  mu_.unlock();
}

// ✅ RAII
void good() {
  std::lock_guard<std::mutex> lk(mu_);   // 构造时 lock
  if (something) return;                 // 析构自动 unlock
  mayThrow();                            // 栈展开也会析构 → 自动 unlock
}
```

这就是 [RAII](#class-基础以-mujocobridgenode-为例) 在并发场景的应用：**把"必须成对出现的操作"绑到对象生命周期上**，让所有退出路径（正常 return、提前 return、异常）都自动走对。和 `unique_ptr` 管内存、`ifstream` 管文件句柄是同一个模式。

三个常用的锁包装：

| 类型 | 用途 |
|---|---|
| `std::lock_guard<M>` | 最简单，构造即锁、析构即解，不能中途解锁。默认选它 |
| `std::unique_lock<M>` | 可以中途 `unlock()`/重新 `lock()`、可延迟加锁、可移动。配合 `condition_variable` 必须用它 |
| `std::scoped_lock<Ms...>` | **一次锁多个 mutex，且内部用避免死锁的算法**。要同时持有两把锁时用它，不要写两个 `lock_guard` |

最后那条值得解释：两个线程分别按 (A,B) 和 (B,A) 的顺序加锁，就可能各持一把、互等另一把——**死锁**。`std::scoped_lock lk(mu_a, mu_b)` 内部处理了这个。这也说明**每多一把锁，复杂度不是线性增长的**。

#### 为什么这些我们一个都没用

`mujoco_bridge` 里没有任何 `atomic`/`mutex`，`mjData` 被两个回调裸着访问。这是对的，因为默认的单线程执行器保证了两个回调不并发（[week1.md 10.8.1](week1.md#1081-并发模型是执行器--回调组不是裸线程)）。

而**提前加锁是纯亏损**：保护 `mjData` 意味着 500Hz × 一个大结构体的加锁开销、要想清楚粒度和顺序、多把锁还要防死锁。更糟的是它给人虚假的安全感——真要并发了，需要的往往不是"每个成员加把锁"，而是重新设计数据流（双缓冲、消息传递），那时原来的锁全要推倒。

替代动作是**把依赖的串行前提写成注释**，让未来改成并发的人看见代价。成本几乎为零，收益是把一个静默的 UB 变成一个有人读过的决定。完整论证在 [week1.md 10.8.3](week1.md#1083-不要提前加锁但要把串行前提写下来)。

#### Python 侧的区别：GIL

`rclpy` 的 `MultiThreadedExecutor` **不能给你 CPU 并行**——GIL 保证同一时刻只有一个线程执行 Python 字节码。它只能解决"阻塞等待"类问题（比如在回调里等另一个服务的响应）。CPU 密集的活要用多进程，或把热点下沉到 C++。

反过来说，Python 里**大部分 data race 不会表现成 UB**：GIL 让单个字节码操作原子化，所以 `self.flag = True` 这种简单赋值是安全的。但**复合操作仍然不安全**（`self.counter += 1` 是读-改-写三个字节码，中间可以切换线程），而且不要指望这个保证——它是 CPython 的实现细节，不是语言规范。

---

### GoogleTest fixture、作用域与所有权（以 Stage L 为例）

`#cpp_语言组织机制` `#cpp_所有权明确化` `#cpp_资源自动管理` `#cpp_更安全的默认写法`

> 我希望针对 `test_model_consistency` 补充一些 C++ 知识。我希望知道这个程序为什么这样设计，背后的思想是什么，希望能够讨论变量和函数的作用域、权限、名称空间等等问题。GoogleTest 是什么，fixture 是什么？`shared_ptr` 可以被不同的 fixture 持有而无需反复初始化吗？“测试不拥有它”，这个是否拥有是如何界定的？还有 `SCOPED_TRACE`，为什么 `cout` 会在这里的函数中使用，运行测试的时候可以看到它们吗？

工程测试的三方数据流见 [week3.md 8.2](week3.md#82-三方测试是怎样工作的)，这里只解释 C++ 与 GoogleTest 机制。

#### GoogleTest、`TEST` 与 `TEST_F`

GoogleTest（gtest）是 C++ 测试框架，负责注册/发现测试、运行测试、提供断言并输出终端与 JUnit XML 结果。无需共享环境的简单测试可以写：

```cpp
TEST(VectorTest, StartsEmpty)
{
  std::vector<int> values;
  EXPECT_TRUE(values.empty());
}
```

多个测试需要相同的复杂准备和清理时，用 fixture（测试夹具）把“实验台”抽出来：

```cpp
class ModelConsistencyTest : public ::testing::Test
{
protected:
  void SetUp() override;
  void TearDown() override;
};

TEST_F(ModelConsistencyTest, FixedConfigurationsMatchAcrossAllThreeModels)
{
  // 测试正文
}
```

`TEST_F` 中的 `F` 就是 fixture。GoogleTest 为每一个 `TEST_F` 生成一个派生测试类，并按“构造 fixture → `SetUp()` → 测试正文 → `TearDown()` → 析构”执行。**每个 `TEST_F` 都有新的 fixture 实例**；本文件两个测试会各自加载和释放模型，因此第一个测试写过的 `qpos` 不会污染第二个。

fixture 使用 `public ::testing::Test`，表示它在类型关系上是一种 GoogleTest 测试基类。辅助函数和成员放在 `protected`：GoogleTest 生成的派生测试类能访问，普通外部代码不能访问。若改为 `private`，生成的测试正文不能直接调用 `setConfiguration()`；改为 `public` 又会无谓扩大接口。

`SetUp()`/`TearDown()` 后面的 `override` 要求编译器确认它们确实覆盖基类虚函数。名字或参数写错会在编译期报错，而不是静默定义一个永远不会被框架调用的新函数。

#### 文件、类与函数的三层作用域

测试文件使用：

```cpp
namespace mujoco_bridge
{
namespace
{
```

外层具名 namespace 表示代码属于项目的 `mujoco_bridge` 名字域。内层匿名 namespace 给常量、`Configuration` 和辅助函数内部链接：它们只在当前 `.cpp` 翻译单元可见，不会成为包的公共 API，也不会与其他 `.cpp` 的同名实现产生链接冲突。

类的 `public/protected/private` 控制“谁能访问成员”，和匿名 namespace 控制的“符号能否跨翻译单元链接”不是同一件事。函数体内的局部变量又是第三层：例如 `jacp`/`jacr` 只活到 `mujocoTcpJacobian()` 返回；模型和索引映射要被多个辅助函数使用，所以保存成 fixture 成员。

成员函数末尾的 `const`，例如 `mujocoBodyTransform(...) const`，承诺不通过该成员函数修改 fixture 的逻辑状态。参数 `const T&` 则表示借用已有对象且不复制、不修改。二者都和所有权不是一回事。

#### `shared_ptr` 不会自动跨 fixture 共享

`shared_ptr` 表示多个智能指针实例可以共同维持**同一个对象**的生命周期，但前提是它们共享同一个 control block，通常来自复制：

```cpp
auto a = std::make_shared<Model>();
auto b = a;  // a/b 共同拥有同一个 Model
```

当前每次 `SetUp()` 都重新调用 `make_shared`，所以每个 fixture 得到不同的 URDF/SRDF/MoveIt 对象。类型写成 `shared_ptr` 不会自动缓存、查找或复用同类型对象。

GoogleTest 可以通过静态 `SetUpTestSuite()` 和静态成员让整个 suite 只加载一次，但当前不采用：测试总耗时只有约 0.4s，而共享可变 `RobotState`/`mjData` 会引入顺序依赖。若以后模型加载成为明显瓶颈，可以只共享只读 `RobotModel`/`mjModel`，每个 fixture 仍创建自己的 `RobotState`/`mjData`。

#### “拥有”由生命周期责任界定，不由指针外观界定

判断所有权的核心问题是：**谁保证对象仍然活着，最后谁负责销毁它？**

| 表达 | 本测试中的例子 | 所有权含义 |
|---|---|---|
| `shared_ptr` | `moveit_model_` | 参与共享所有权；最后一份 `shared_ptr` 销毁时释放对象 |
| `unique_ptr` | `moveit_state_` | fixture 是唯一所有者；不能复制，可移动转交，析构自动释放 |
| 借用裸指针 | `arm_group_`、`tcp_link_` | 指向 `moveit_model_` 内部对象；不单独删除，必须保证 model 先活着 |
| 拥有裸指针 | `mujoco_model_`、`mujoco_data_` | MuJoCo C API 返回新资源，调用者必须用对应 `deleteModel/Data` 释放 |

所以“裸指针 = 不拥有”并不成立。C++ 原始指针本身不编码所有权，必须看 API 契约、创建来源和销毁责任。`const JointModelGroup*` 中的 `const` 只表示不能经它修改对象，不表示是否拥有。

当前 MuJoCo 裸资源由 `TearDown()` 释放，正常测试路径正确；更严格的 RAII 做法是用带自定义 deleter 的 `unique_ptr`，使异常退出作用域时也自动清理。当前没有为这一个测试额外引入包装类型，触发重构的条件是出现第二个需要同类所有权代码的消费者，或异常清理成为真实问题。

#### `ASSERT_*`、`EXPECT_*` 与 `SCOPED_TRACE`

`ASSERT_*` 失败会立即终止当前测试函数；`EXPECT_*` 失败会记录错误后继续。因此模型加载和必要指针检查用 `ASSERT_*`，因为失败后没有条件继续；循环里的数值门禁用 `EXPECT_*`，这样一次运行能收集多个 link 的完整失败分布。

`SCOPED_TRACE(message)` 把诊断上下文压入 GoogleTest 的 trace 栈，离开当前 C++ 作用域时自动弹出。成功时它不制造噪声；作用域内断言失败时，仍生效的 trace 会附在报告中。本测试外层记录构型，辅助比较函数再记录模型对和 link，因此同一行 `EXPECT_LT` 失败也能定位到 `random_b:moveit_vs_mujoco:hand_tcp`。

#### 为什么还使用 `std::cout`

断言回答“有没有越过门槛”，逐项输出回答“离门槛多远、误差集中在哪里”。`cout` 输出 CSV-like 行以及最大误差，适合单次诊断，但不是断言本身。

直接运行 gtest 可执行文件时会看到输出；`colcon test` 会捕获到 `build/mujoco_bridge/ament_cmake_gtest/test_model_consistency.txt`，使用合适的 event handler 时也会显示在终端。`SCOPED_TRACE` 只在失败时出现，`cout` 则确实执行并写出，但成功测试的 stdout 是否立即展示取决于 CTest/colcon 的输出策略。若以后要跨 CI 比较历史趋势，应保存结构化 artifact，不应把 stdout 当长期数据库。

### `std::optional`、`mutable` 与接口常量性（以 Stage N 为例）

`#cpp_更强的类型表达能力` `#cpp_语言组织机制` `#cpp_所有权明确化`

> Q: `optional` 是什么容器？`mutable` 是什么，为什么 `DiffIkWaypointSource` 这里需要它？构造函数在哪里？`const` 是不是只是让编译器检查是否实现了抽象类？

#### `std::optional` 是可选值，不是序列容器

`std::optional<T>` 表示“有一个 `T`”或“没有值”两种状态：

```cpp
std::optional<int> value;  // empty
value = 42;                // engaged
if (value.has_value()) {
  int answer = *value;
}
```

它和 `vector` 的区别是：`optional<T>` 最多容纳一个 `T`，语义是值是否存在，不是元素序列。它通常直接存放对象和一个 engaged 标志，不需要用 `-1`、空字符串或空指针这类魔法值编码“没有结果”。

Stage N 中的几个 `optional` 分别表达：是否已有阶段缓存、是否已经锁定抓取物体位姿、是否已有最近一次 IK 诊断。调用者可以明确区分“没有求解过”和“求解结果的数值恰好为零”。

#### `mutable` 放宽的是成员修改规则

成员函数末尾的 `const` 表示普通成员函数不能修改对象状态；`mutable` 成员是例外：

```cpp
JointTarget jointTargetFor(...) const;
mutable std::optional<Phase> cached_phase_;
```

`WaypointSource` 的接口已经规定 `jointTargetFor()` 是 `const`，但 Stage N 第一次调用时仍要建立阶段缓存、锁定物体位姿并保存诊断结果。因此缓存字段声明为 `mutable`。这表达的是“目标查询的外部语义不变，缓存属于内部实现细节”。严格说它不是数学纯函数；若未来要把接口设计得更诚实，可以把准备缓存拆成非 `const` 的 `preparePhase()`，或去掉方法末尾的 `const`。当前使用 `mutable` 是为了保持已冻结的接口兼容。

#### 构造函数的声明、定义和初始化列表

头文件只声明构造函数：

```cpp
explicit DiffIkWaypointSource(arm_kinematics::ArmModel model);
```

`.cpp` 给出定义：

```cpp
DiffIkWaypointSource::DiffIkWaypointSource(
  arm_kinematics::ArmModel model)
: model_(std::move(model))
{
}
```

`DiffIkWaypointSource::` 表示这是该类的成员函数；构造函数没有返回类型；冒号后的初始化列表在构造函数体之前直接初始化成员。这里用初始化列表比在函数体中先默认构造 `model_` 再赋值更直接，也适用于不能默认构造或不能重新绑定的成员。

普通类的接口放 `.hpp`、实现放 `.cpp` 是分离式编译的常规写法。模板通常需要把实现放在头文件；普通非模板成员函数不需要。派生类通过 `jointTargetFor(...) const override` 实现基类的纯虚函数；`override` 专门让编译器检查函数签名是否真的覆写了基类方法，`const` 本身不是这个检查的关键字。

#### 三种 `const` 要分开看

```cpp
JointTarget jointTargetFor(Phase, const ObjectPose &) const override;
```

- 参数前的 `const`：通过引用避免拷贝，并承诺不修改传入的 `ObjectPose`。
- 方法末尾的 `const`：承诺不修改对象的普通成员状态，也允许对 `const DiffIkWaypointSource` 调用。
- `override`：检查名字、参数、返回类型和方法 `const` 性质是否与基类虚函数匹配。

如果派生类漏写末尾的 `const`，它不会覆写基类函数；有 `override` 时编译器会立刻报错。`const` 表达的是接口和修改权限，`override` 才是覆写检查。

#### 运行时多态和具体类型专属操作

节点同时持有：

```cpp
WaypointSource * waypoint_source_;
std::unique_ptr<DiffIkWaypointSource> diff_ik_source_;
```

前者用于公共操作 `jointTargetFor()`，调用时通过虚函数动态分派到查表或 IK 实现；后者用于 `beginEpisode()`、`setSeed()` 和 `diagnostics()` 这些只有 IK 源才有的操作。当前这样做是为了不把 diff-IK 专属概念塞回所有 waypoint 源的公共接口。若将来有多种 waypoint 源都需要 seed、生命周期钩子和诊断，应重新设计接口，而不是继续增加具体类型分支。

#### 它不是单例

节点只创建一个 `DiffIkWaypointSource`，不等于该类是单例。构造函数是 public，可以同时创建多个独立对象；每个对象拥有自己的模型、seed 和缓存。单例需要私有构造函数、唯一访问入口以及通常删除拷贝/赋值操作。这里使用 `std::unique_ptr` 表示节点拥有这个可选策略对象，并不限制全局只能有一个实例。单例反而会让多个节点或测试共享 episode 缓存，增加状态污染和线程安全问题。

### 待补充问答模板

`#cpp_分类标签`

> Q:

A:
