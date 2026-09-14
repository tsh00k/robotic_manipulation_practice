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
| `#cpp_语言组织机制` | [namespace 与匿名 namespace](#namespace-与匿名-namespace)、[class 基础（以 MujocoBridgeNode 为例）](#class-基础以-mujocobridgenode-为例)、[头文件声明 vs .cpp 定义、无命名空间的 C 类型](#头文件声明-vs-cpp-定义无命名空间的-c-类型)、[单例初始化用匿名 lambda 的原因](#单例初始化用匿名-lambda-的原因)、[resolve 模板与 dlopen/dlsym](#resolve-模板与-dlopendlsym) |
| `#cpp_更安全的默认写法` | [namespace 与匿名 namespace](#namespace-与匿名-namespace)、[class 基础（以 MujocoBridgeNode 为例）](#class-基础以-mujocobridgenode-为例)、[并发：data race 是 UB，以及为什么不要提前加锁](#并发data-race-是-ub以及为什么不要提前加锁) |
| `#cpp_所有权明确化` | [class 基础（以 MujocoBridgeNode 为例）](#class-基础以-mujocobridgenode-为例)、[头文件声明 vs .cpp 定义、无命名空间的 C 类型](#头文件声明-vs-cpp-定义无命名空间的-c-类型) |
| `#cpp_设计模式` | [单例初始化用匿名 lambda 的原因](#单例初始化用匿名-lambda-的原因) |
| `#cpp_泛型与抽象增强` | [resolve 模板与 dlopen/dlsym](#resolve-模板与-dlopendlsym) |
| `#cpp_并发与内存模型` | [并发：data race 是 UB，以及为什么不要提前加锁](#并发data-race-是-ub以及为什么不要提前加锁) |
| `#cpp_资源自动管理` | [并发：data race 是 UB，以及为什么不要提前加锁](#并发data-race-是-ub以及为什么不要提前加锁) |

## 目录

- [namespace 与匿名 namespace](#namespace-与匿名-namespace)
- [class 基础（以 MujocoBridgeNode 为例）](#class-基础以-mujocobridgenode-为例)
- [头文件声明 vs .cpp 定义、无命名空间的 C 类型](#头文件声明-vs-cpp-定义无命名空间的-c-类型)
- [单例初始化用匿名 lambda 的原因](#单例初始化用匿名-lambda-的原因)
- [resolve 模板与 dlopen/dlsym](#resolve-模板与-dlopendlsym)
- [并发：data race 是 UB，以及为什么不要提前加锁](#并发data-race-是-ub以及为什么不要提前加锁)
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

### 待补充问答模板

`#cpp_分类标签`

> Q:

A:
