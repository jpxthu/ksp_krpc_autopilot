# KSP kRPC AutoPilot

B 站视频中 Falcon Heavy 发射、回收，与空间站对接，最后回收载人仓。

## 运行环境

系统环境：Win10 x64

IDE：Visual Studio 2019 Community

游戏版本：1.11.0.3045

## 重要的 Mods

### Module manager

没啥好说的吧，必备的。

### kRPC

和外界程序通讯，关键 Mod。跟 kOS 相比少了很多接口，基于需求自己添加了不少，还修了俩 bug，因此必须使用我更改过的版本。有两种方法安装：

1. 下载代码（[jpxthu/krpc - dev branch](https://github.com/jpxthu/krpc/tree/dev)）自行编译，编译方法参阅 kRPC 文档。我用的是 Win10 内嵌的 Ubuntu 16/18，改了很多编译相关的配置，可以自行阅读和 master 不一样的 commit。不要用 Ubuntu 20，环境配不好。
2. 使用我编译好的文件，Windows 之外的系统我也不知道能不能用。已经上传在 repo 中：[external/kRPC](external/kRPC)。

我提了好几个 PR，作者不更新了，也没有办法，再这么下去可能要自己写插件了。因为常年不更新的原因，游戏内 UI 有问题不会弹窗，server 必须勾选允许自动连接才可以用，不然一辈子都连不上。

### Tundra Exploration

Space X 飞行器，视频中火箭、载人仓都是作者的原版 craft。使用版本 v3.1.1，[官网](https://forum.kerbalspaceprogram.com/index.php?/topic/166915-111x-tundra-exploration-v330-january-24th-restockalike-spacex-falcon-9-crew-dragon-and-starship/)已经更新到 v3.3.0，还没有测试，估计也是可以用的。不能在 CKAN 里下。

该 mod 需要几个必备插件，我不知道有什么用，也不敢问，反正就安了，CKAN 里都有：
- Kerbal Reusability Expansion (For the F9 legs and grid fins)
- B9 Part Switch (For part switching)
- Animated Decouplers (For FH sideboosters decoupler)
- RetractableLiftingSurface (For actuating fins on Gojira Mk3)

还有几个建议的插件，我只写我装了的：
- Kerbal Actuators - 切引擎用的，Space X 在回收的时候只用中间一个梅林，贴近真实。
- Smokescreen - 装了之后尾焰非常真实，非常漂亮。
- Textures Unlimited (For a shiny Gojira Mk4) - 不知道有什么用，也不知道为什么装了，不装也无所谓。

试了很多插件，就这个的功能全、模型精美。不过有几个问题：
- 箭体刚度差。我上传的 craft 里关键部件都加了自动加固（游戏里要开高级模式），不然发射的时候能给自己抖散架。
- 控制力矩弱得令人发指。我的 craft 里把两个小 RCS 换成四个大号的，感觉这样机动性才跟真实的 Space X 差不多。还加了一个动量轮，不然 roll 不动。
- 起落架放在地上的时候会疯狂抖动，视频是掐了，不然上个厕所回来火箭就抖到楼底下了。

### SpaceX Launch Vehicles

这里面 Space X 的火箭做得不怎么样，但是船特别精致，就是我扔海里那个。老视频那个船上的字都是马赛克。

### FAR

**千万不要装！装了卸载！**

**千万不要装！装了卸载！**

**千万不要装！装了卸载！**

重要的事情说三遍！kRPC 计算空气阻力的函数有两部分，都有 bug，我只修了一部分，FAR 那部分我没搞懂怎么回事。

## 使用方法

直接运行 Test project 就行，没注释的是 FH 部分的，Start 函数参数控制跟踪哪部分（载荷/一级火箭/助推器）。如果跟踪载荷的话，紧接着有注释掉的对接函数，自己探索吧。

代码里跟 ship、part 相关的功能都是通过 ship name 和 part tag 实现的，可以自行阅读代码，比如对接的接口：

``` c#
docking.Start("Kerbin空间站", "docking_port_2", "docking_port", 50);
```

如果只玩 FH 发射回收的话，直接用 [我的 craft 文件](external/FH.craft) 可能比较省事。

## 控制思路

总体思路是模拟一条自然下落的轨迹，落点有误差再做修正。

### Simulation

跟上次代码不一样。之前用了类似 Trajectory 的方法，就是纯粹的自由落体。这次在模拟预测轨迹的时候加了动力，所以模拟的是动力减速降落的轨迹，发动机动力对轨迹的影响也会计算，所以精确很多。还顺带算了倾斜下降的升力。

这么做的缺点是需要进行大量通信，因为游戏机制问题会引起掉帧，而且调用频率有限制。虽然加了 cache，但刚开始计算的时候填 cache 很慢，这也是每次分级之后火箭迟迟不点火的原因。

### 落点修正

上次代码直接用 RCS 侧推的原因有两点，一是落点计算不对，上面 simulation 部分提到了。

另一个原因比较麻烦，降落过程中调整横向位置需要倾斜火箭，这时侧向有三个力：
1. 发动机推力的横向分力，这个是可以预测的，simulation 里做了五秒的预测。但提供扭矩的时候最多会转 3°，所以算不是很精确。
2. 升力，这个也是可以预测的，但不同速度和攻角下的升力都需要单独去调 API，只能估计个近似。
3. RCS，有时候火箭侧倾的时候空气会给一个巨大的扭矩，RCS 在纠正的时候会有一个预期之外的侧向力。如果没有这个力的话可以用来提供需要的侧向力，发动机可以抵消这个扭矩，产生的侧向力方向是相同的，就非常妙。

其中升力的变化比较大，大的时候比发动机侧力大好几倍，小的时候基本没影响，而且和发动机侧向力的方向是相反的。必须要做好预测才能比较平滑地处理。

## TODO

### Attitude controller

现在姿态和速度的 controller 都是自己写的，基于模型，但没上优化器，所以临近零点的时候经常有超调和震荡，视频中可以看到。这里需要写一个新的，还没想好用什么。

### Path planning + MPC

实时的轨迹规划，需要一个大优化器。上个视频下面论文地址都贴了，但到现在都没搞定，工作事儿太多了。

轨迹出来之后就需要一个 MPC 来追踪。

我真的厌倦了用简单的反馈方法反复调参和绣花，代码里好些逻辑复杂冗余还不连续。

### Aerodynamic

速度瓶颈，如果上全局优化器的话对这个要求会更高。KSP 里的空气动力学其实是有公式的，摸出来就好。真实世界里做这些事儿也是吹风洞 + CFD 搞得明明白白才做。

## 其它

代码还在早期阶段，包括一些非代码的 binary 不应该这么放在 repo 里。之后可能会频繁地 force push master。
