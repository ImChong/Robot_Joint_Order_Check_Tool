# Robot_Joint_Order_Check_Tool

机器人关节顺序检查工具 / Robot joint ordering cross-check for URDF and MuJoCo MJCF.

[![GitHub Pages](https://img.shields.io/badge/GitHub%20Pages-Live-brightgreen?logo=github)](https://imchong.github.io/Robot_Joint_Order_Check_Tool/)
[![Deploy GitHub Pages](https://github.com/ImChong/Robot_Joint_Order_Check_Tool/actions/workflows/pages.yml/badge.svg)](https://github.com/ImChong/Robot_Joint_Order_Check_Tool/actions/workflows/pages.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![Privacy](https://img.shields.io/badge/URDF_%7C_MJCF-浏览器本地解析-informational?logo=firefoxbrowser&logoColor=white)](#功能)
[![i18n](https://img.shields.io/badge/界面-中文_%7C_English-lightgrey)](#english)

上传一个 **URDF 或 MuJoCo MJCF（.xml）**，把它在 **Isaac Gym / Isaac Sim (Isaac Lab) / MuJoCo / Gazebo / ros2_control** 中的关节顺序并列打印出来。顺序全部一致就显示绿色，任何一个框架的顺序对不上就标红，并指出差在哪里。格式按根元素自动识别（`<robot>` 还是 `<mujoco>`），不用手动选。

**在线使用：** <https://imchong.github.io/Robot_Joint_Order_Check_Tool/>

纯静态页面，文件完全在浏览器本地解析，不上传任何服务器。

---

## 为什么需要这个工具

URDF 本身**不定义关节顺序**——它只是一堆 link 和 joint 的集合。每个下游工具按自己的规则把它们排成一个向量：

MJCF 不一样，它的 body 树把关节顺序写死了；但同一台机器人的 URDF 和 MJCF 未必写成同一个顺序，而且 MJCF 自己的 `<actuator>` 顺序又是另一个向量 —— 所以照样要对一遍。

| 工具 | 顺序规则 |
| --- | --- |
| URDF 文件 | `<joint>` 元素的文档顺序（作为参照基准） |
| MJCF 文件 | body 树里 `<joint>` 的文档顺序 —— 这就是 MuJoCo 编译出来的**关节 id 顺序** |
| Isaac Gym (Preview) | 运动学树**深度优先**（DFS），同层按文档顺序 |
| Isaac Sim / Isaac Lab | 运动学树**广度优先**（BFS，PhysX stage parser） |
| MuJoCo | body 树**深度优先**（DFS），同层按文档顺序 |
| Gazebo (SDF) | **深度优先**（DFS），但**同层按关节名字母序**（sdformat 走 urdfdom 的 `child_links`）；只吃 URDF |
| ros2_control | 取决于配置，默认跟 URDF 文件顺序一致（见下）；只吃 URDF |
| MuJoCo `ctrl` | `<actuator>` 元素的文档顺序 —— 和关节顺序是**两个独立的向量** |

于是一个四足机器人会出现这种经典翻车现场：

```
Isaac Gym / MuJoCo (DFS)   : FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, ...
Isaac Sim / Isaac Lab (BFS): FL_hip, FR_hip,   RL_hip,   RR_hip, FL_thigh, FR_thigh, ...
```

把在 Isaac Gym 里训好的策略直接部署到 Isaac Lab 或实机上，关节向量就会静默错位——不报错，机器人直接抽搐。这个工具就是用来在写代码之前先把这件事看清楚的。

Gazebo 的坑更隐蔽：它也是 DFS，但**同层子关节按关节名字母序**，所以只要某个 link 的多个子关节的书写顺序不等于字母序，Gazebo 就会和 MuJoCo / Isaac Gym 分道扬镳。内置示例「移动机械臂」就是这种情况 —— `base_link` 下先写轮子后写手臂，Gazebo 却把 `arm_shoulder_pan` 排到了两个 `drive_wheel_*` 前面。

## MuJoCo MJCF 支持

除了 URDF，工具也直接读 MuJoCo 的 `.xml`（MJCF）。MJCF 的 XML 嵌套本身就是 body 树，所以「文件顺序」这一列对 MJCF 来说不是参照基准，而是 **MuJoCo 真正的关节 id 顺序**（body id 按 DFS 分配，同一个 body 内的多个关节按书写顺序）。

解析时处理了这些 MJCF 特有的东西：

- `<default>` / `class` / `childclass` 类继承 —— 没写 `type` 的 `<joint>` 要从默认类里取类型
- `<freejoint>` 与 `type="free"`（6 DOF）、`ball`（3）、`slide` / `hinge`（1）
- 没有 `<joint>` 的 body 是**焊死**在父 body 上的（相当于 URDF 的 `fixed`），不产生关节；运动学树里标成 `[weld]`
- 一个 body 上挂多个关节时，按书写顺序展开成一串自由度
- 重复出现的 `<worldbody>` / `<actuator>` 等顶层段（MuJoCo 会合并它们）
- `<actuator>` 顺序单列一列（`data.ctrl` 的顺序），`<equality>` 的关节约束、`<include>`、重名元素都会给出提示

Gazebo 和 ros2_control 不读 MJCF，载入 MJCF 时这两列显示为「不适用」。

### 浮动基座

`<freejoint>`（以及 URDF 里挂在根 link 上的 `floating` 关节）在 MuJoCo 里是一个排在最前面的 free joint，占 7 个 qpos / 6 个 qvel；而 Isaac / Gazebo / ros2_control 把它当作自由浮动的根，**根本不出现在关节列表里**。所以工具只在 MuJoCo 相关的列里保留它，避免其它列整体错位一格。

## 内置示例：Unitree G1

内置了 [unitree_ros](https://github.com/unitreerobotics/unitree_ros) 里 G1 29DOF + 灵巧手的**官方 URDF 和官方 MJCF**（`samples/` 下原样保存，BSD-3-Clause，见 [samples/README.md](samples/README.md)）。两个文件分别载入就能看到：

- 两边的关节顺序是**一致的** —— URDF 按 MuJoCo 规则导入得到的 43 个可动关节顺序，和 MJCF 里的关节顺序完全相同
- 但 MJCF 的 `<actuator>` 顺序和关节顺序在**右手最后四个电机上对不上**：关节顺序是 `… middle_0, middle_1, index_0, index_1`，执行器顺序是 `… index_0, index_1, middle_0, middle_1`（下标 39–42）。也就是说这四个位置上 `data.ctrl[i]` 驱动的不是你按关节顺序数出来的那个关节
- Isaac Sim / Isaac Lab 的 BFS 顺序和这两者都不同（腿、腰、手臂逐层交错）

## 功能

- **拖拽 / 选择文件 / 粘贴文本**三种方式载入 URDF 或 MJCF，或直接点内置示例
- **并列对照表**，逐格标出与参照顺序不同的位置；也可切换成「按关节」视图看每个关节在各框架中的索引和偏移量
- 绿 / 红 / 黄三态结论：顺序一致、顺序不一致、顺序一致但关节集合不同
- **索引重映射代码生成**：选好源框架和目标框架，直接产出可粘贴的 Python / C++ 索引数组
- 导出 CSV 与完整 JSON
- 运动学树可视化、关节类型与 DOF 列表
- 自动提示常见坑：未展开的 xacro、mimic 关节 / MJCF 的 `<equality>`、多自由度关节（floating / planar / ball）、浮动基座、多根链接、成环、只在 joint 里引用却没定义的 link、MJCF 的 `<include>` 与重名元素
- 中英文界面切换、明暗主题

## 关节集合的比较方式

不同框架天然包含不同的关节，直接比长度会误报，所以：

- 顺序比较只在**两边共有的关节**上进行 —— MuJoCo 不会为 URDF 的 `fixed` 关节生成任何 joint，这不算「顺序错」
- 「N 个位置不同」也是在共有关节上数的：少一个关节会让后面所有下标平移一格，但那不是顺序错，红色只留给**相对顺序真的变了**的关节，纯平移标黄
- 关节集合的差异单独用黄色标出（缺少 / 多出哪些关节），不与顺序错误混在一起
- 默认只显示可动关节；勾选「显示 fixed 关节」后，fixed 关节只会出现在真正持有它们的列（URDF / ros2_control）里

## ros2_control 的三种情况

`joint_state_broadcaster` 的发布顺序取决于配置，工具里可以切换前两种：

1. 未设 `joints` 参数、`use_urdf_to_filter=true`（默认）→ **与 URDF 文件里的关节顺序相同**，与 `<ros2_control>` 标签内的顺序无关
2. 未设 `joints` 参数、`use_urdf_to_filter=false` → 按 resource manager 注册顺序，即 **`<ros2_control>` 标签内 `<joint>` 的顺序**
3. 显式设置了 `joints` + `interfaces` 参数 → 按 `joints` 参数的顺序（本工具读不到 YAML，这种情况请以你的配置为准）

注意各控制器（如 `joint_trajectory_controller`）用的是各自 YAML 里 `joints` 参数的顺序，与上面无关。

## 顺序规则的依据

- Isaac Gym (DFS) vs Isaac Sim / Isaac Lab (BFS)：[Isaac Lab — Migrating from IsaacGymEnvs](https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html)
  > Physics simulation in Isaac Sim and Isaac Lab assumes a breadth-first ordering for the joints in a given kinematic tree. However, Isaac Gym Preview Release assumed a depth-first ordering for joints in the kinematic tree.
- MuJoCo（URDF 导入）：[`src/xml/xml_urdf.cc`](https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_urdf.cc) —— 按文档顺序填充 `urChildren`，再从根 body 递归 `AddToTree()`；`fixed` 关节不生成 joint，`planar` 展开成 2 slide + 1 hinge，`<mimic>` 被忽略
- MuJoCo（MJCF 本身）：[MJCF XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html) —— body 树即 XML 嵌套，`<actuator>` 的顺序就是 `data.ctrl` 的顺序
- Gazebo：[sdformat `parser_urdf.cc`](https://github.com/gazebosim/sdformat/blob/sdf14/src/parser_urdf.cc) —— `CreateSDF()` 递归遍历 `_link->child_links`（DFS）；而 `child_links` 由 [urdfdom_headers `model.h`](https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/model.h) 的 `initTree()` 遍历 `std::map` 类型的 `joints_` 填充，因此同层子关节是**按关节名字母序**。sdformat 默认还会吸收 fixed 关节
- ros2_control：[joint_state_broadcaster 文档](https://control.ros.org/rolling/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html)

## 已知限制

- **只接受展开后的 URDF**。检测到 `<xacro:*>` 或 `${}` 会提示先跑 `xacro robot.urdf.xacro > robot.urdf`
- **MJCF 的 `<include>` 不会被跟进**（浏览器里读不到别的文件），结果只反映当前这一个文件；含 `<include>` 时会明确提示
- MJCF 里驱动 tendon / site / body 的执行器不对应任何关节，但一样占 `ctrl` 槽位，所以「MuJoCo ctrl」列的序号在这种模型上会小于真实 `ctrl` 下标（同样会提示）
- 结论是**按公开的导入规则静态推导**的，不运行任何仿真器。生产环境请始终以运行时打印的关节名列表为准：`robot.data.joint_names`（Isaac Lab）、`mj_id2name(m, mjOBJ_JOINT, i)`（MuJoCo）、`gym.get_asset_dof_names(asset)`（Isaac Gym）
- 导入器的选项会改变结果（例如 Isaac 的 `merge_fixed_joints`、MuJoCo 的 `fusestatic`、Isaac Gym 的 `collapse_fixed_joints`、sdformat 的 `disableFixedJointLumping` / `preserveFixedJoint`），工具按各自的默认行为计算
- Gazebo 一列指的是 **URDF→SDF 转换后模型里的关节顺序**（`Model::GetJoints()` 走这个顺序）。`gazebo_ros_joint_state_publisher` 插件按你列的 `<joint_name>` 顺序发布，`gz_ros2_control` 走 `<ros2_control>` 标签，两者都与这一列无关
- 多自由度关节（`floating` / `planar`）在各框架展开成的 DOF 数量和排列不同，关节级顺序一致**不代表** DOF 级一致，工具会单独警告
- `<mimic>` 关节各框架支持程度不同，只作提示，不改变顺序推导

## 本地运行

不需要构建步骤，任何静态服务器都行：

```bash
git clone https://github.com/ImChong/Robot_Joint_Order_Check_Tool.git
cd Robot_Joint_Order_Check_Tool
python3 -m http.server 8000   # 然后打开 http://localhost:8000
```

## 开启 GitHub Pages

仓库 **Settings → Pages → Build and deployment → Source** 选 **GitHub Actions** 即可，`.github/workflows/pages.yml` 会在推送到 `main` 时自动部署。

也可以选 **Deploy from a branch → `main` / `(root)`**，同样能工作（仓库里已有 `.nojekyll`）。

## 项目结构

```
index.html                 页面结构
assets/css/style.css       样式（明暗双主题）
assets/js/i18n.js          中英文文案
assets/js/samples.js       内置示例机器人（唯一数据源）
assets/js/urdf.js          URDF 解析与运动学树构建
assets/js/mjcf.js          MJCF（MuJoCo XML）解析，产出同一套模型结构
assets/js/orderings.js     各框架的顺序规则与一致性分析
assets/js/app.js           界面逻辑
samples/                   示例文件（合成的由 samples.js 导出，真实机型原样收录）
tools/export-samples.mjs   重新生成合成示例，并检查收录的文件还在
```

改过 `assets/js/samples.js` 之后跑 `node tools/export-samples.mjs` 同步示例文件，CI 会检查两者是否一致。收录的第三方模型不会被这个脚本改写，只检查存在性。

---

## English

Upload a **URDF or a MuJoCo MJCF (`.xml`)** — the format is detected from the root element — and this page prints the robot's joint order **side by side** as seen by Isaac Gym, Isaac Sim / Isaac Lab, MuJoCo, Gazebo and ros2_control. Green when every framework agrees, red on the exact cells that don't — plus a generated index-remap array you can paste into your code.

URDF itself defines no joint order, so each downstream tool imposes its own: MuJoCo and Isaac Gym walk the kinematic tree depth-first, Isaac Sim / Isaac Lab walk it breadth-first, Gazebo also walks it depth-first but sorts siblings alphabetically (sdformat recurses over urdfdom's `child_links`, filled from a `std::map`), and ros2_control follows the URDF or the `<ros2_control>` tag depending on configuration. That mismatch is what silently scrambles a joint vector when you move a policy between them.

**MJCF** is a different question: the XML nesting *is* the body tree, so the file order is the joint id order MuJoCo compiles to, and the interesting mismatch moves elsewhere — `data.ctrl` follows the `<actuator>` block, a separate vector, which gets its own column. The bundled Unitree G1 model is a live example: its official URDF and MJCF agree on joint order, but the MJCF's last four right-hand actuators (indices 39–42) are written `index_0, index_1, middle_0, middle_1` while the joints run `middle_0, middle_1, index_0, index_1`. Bodies with no joint (welds), `<default>` class inheritance, free/ball joints and `<equality>` constraints are all handled; `<include>` is not followed and is reported instead.

Everything runs client-side — no upload, no build step. Live at <https://imchong.github.io/Robot_Joint_Order_Check_Tool/>; run locally with `python3 -m http.server 8000`.

See the sections above for the exact ordering rule per framework, the sources they are derived from, and the known limitations (expanded URDF only; static derivation — always confirm against the joint names your runtime prints).

---

## 许可证 / License

[MIT](LICENSE) © 2026 Chong Liu
