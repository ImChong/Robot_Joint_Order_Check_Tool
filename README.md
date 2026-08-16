# Robot_Joint_Order_Check_Tool

机器人关节顺序检查工具 / Robot joint ordering cross-check for URDF and MuJoCo MJCF.

[![GitHub Pages](https://img.shields.io/badge/GitHub%20Pages-Live-brightgreen?logo=github)](https://imchong.github.io/Robot_Joint_Order_Check_Tool/)
[![Deploy GitHub Pages](https://github.com/ImChong/Robot_Joint_Order_Check_Tool/actions/workflows/pages.yml/badge.svg)](https://github.com/ImChong/Robot_Joint_Order_Check_Tool/actions/workflows/pages.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![Privacy](https://img.shields.io/badge/URDF_%7C_MJCF-浏览器本地解析-informational?logo=firefoxbrowser&logoColor=white)](#功能)
[![i18n](https://img.shields.io/badge/界面-中文_%7C_English-lightgrey)](#english)

上传一个 **URDF 或 MuJoCo MJCF（.xml）**，把它在 **Isaac Gym / Isaac Sim (Isaac Lab) / MuJoCo / Genesis / Newton / Gazebo / PyBullet / ros2_control** 中的关节顺序并列打印出来。顺序全部一致就显示绿色，任何一个框架的顺序对不上就标红，并指出差在哪里。格式按根元素自动识别（`<robot>` 还是 `<mujoco>`），不用手动选。

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
| Genesis | 同 MuJoCo —— 结构解析直接交给 MuJoCo 的统一解析器；默认还会合并 fixed link、并在根部插入 `root_joint` |
| Newton (Warp) | **深度优先**拓扑排序（`joint_ordering="dfs"`），同层按文档顺序；fixed 关节**占关节下标**，且最前面还有一个自带的基座关节 |
| Gazebo (SDF) | **深度优先**（DFS），但**同层按关节名字母序**（sdformat 走 urdfdom 的 `child_links`）；只吃 URDF |
| PyBullet | **深度优先**（DFS），同层按文档顺序；`getJointInfo` **包含 fixed 关节** |
| ros2_control | 取决于配置，默认跟 URDF 文件顺序一致（见下）；只吃 URDF |
| MuJoCo `ctrl` | `<actuator>` 元素的文档顺序 —— 和关节顺序是**两个独立的向量** |

于是一个四足机器人会出现这种经典翻车现场：

```
Isaac Gym / MuJoCo (DFS)   : FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, ...
Isaac Sim / Isaac Lab (BFS): FL_hip, FR_hip,   RL_hip,   RR_hip, FL_thigh, FR_thigh, ...
```

把在 Isaac Gym 里训好的策略直接部署到 Isaac Lab 或实机上，关节向量就会静默错位——不报错，机器人直接抽搐。这个工具就是用来在写代码之前先把这件事看清楚的。

Gazebo 的坑更隐蔽：它也是 DFS，但**同层子关节按关节名字母序**，所以只要某个 link 的多个子关节的书写顺序不等于字母序，Gazebo 就会和 MuJoCo / Isaac Gym / PyBullet 分道扬镳。内置示例「移动机械臂」就是这种情况 —— `base_link` 下先写轮子后写手臂，Gazebo 却把 `arm_shoulder_pan` 排到了两个 `drive_wheel_*` 前面。

PyBullet 的 DFS 同层顺序和 Isaac Gym / MuJoCo 一样，但 **`getNumJoints` / `getJointInfo` 默认把 fixed 关节也算进去**（每个 child link 占一个下标，基座是 -1）。对照表默认只显示可动关节；要看与运行时下标一致的完整列表，勾选「显示 fixed 关节」。

Genesis 和 Newton 这两个新引擎的**关节顺序本身**都跟 MuJoCo 一致（DFS，同层按文档顺序），真正容易踩的是它们各自往关节向量里塞的东西：

- **Genesis** 的结构解析直接交给 MuJoCo 的统一解析器（`rigid_entity.py` 里 `l_infos = l_infos_mj`），所以顺序不用另算。但 `gs.morphs.URDF` 默认 `merge_fixed_links=True`，fixed 关节连同子 link 一起被合并掉（要保留用 `links_to_keep=[...]`）；而且默认 `fixed=False` 时它会自动在根部插入一个名为 `root_joint` 的 **free 关节**，哪怕 URDF 里根本没写浮动关节 —— `entity.joints[0]` 往往不是你文件里的第一个关节，后面所有 DOF 下标整体后移 6 位
- **Newton** 默认 `collapse_fixed_joints=False`，fixed 关节保留成 0 自由度的 `JointType.FIXED`，**照样占一个关节下标**（这点像 PyBullet，勾选「显示 fixed 关节」才对得上）；此外它还会在最前面插一个文件里没有的基座关节（`fixed_base` 或 `floating_base`），所以 `model.joint_label[0]` 不是你的关节。Newton 也是唯一一个能直接切换遍历方式的：`add_urdf(..., joint_ordering="bfs")` 就变成 Isaac Sim 那一列的顺序

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

`<freejoint>`（以及 URDF 里挂在根 link 上的 `floating` 关节）在 MuJoCo 里是一个排在最前面的 free joint，占 7 个 qpos / 6 个 qvel；而 Isaac / Gazebo / PyBullet / ros2_control 把它当作自由浮动的根，**根本不出现在关节列表里**。所以工具只在 MuJoCo 相关的列里保留它，避免其它列整体错位一格。

## 内置示例：Unitree G1

内置了 [unitree_ros](https://github.com/unitreerobotics/unitree_ros) 里两套官方描述（`samples/` 下原样保存，BSD-3-Clause，见 [samples/README.md](samples/README.md)）：

**G1 29DOF 机身**（`g1_29dof_rev_1_0`，无灵巧手）是 [ParkourFormer](https://arxiv.org/abs/2605.25782) 等人体跑酷策略实际部署的那一台（[项目页](https://mronaldo-gif.github.io/parkourformer.github.io/)）。策略观测是 96 维：

```
o_t = [ω_a(3), g_p(3), v_c(3), q_p(29), q_v(29), a_{t-1}(29)]
```

动作也是 29 维。MuJoCo（ParkourFormer 走 Project Instinct 管线）按 DFS 排这 29 个关节，Isaac Sim / Isaac Lab 按 BFS 排 —— 从第 2 个关节起就对不上（`left_hip_roll` vs `right_hip_pitch`）。把 MuJoCo 训好的 29 维向量直接喂给 Isaac Lab 会静默错位。这个 29DOF 的 MJCF 里 `<actuator>` 顺序和关节顺序是一致的。

**G1 29DOF + 灵巧手**（`g1_29dof_with_hand_rev_1_0`）分别载入就能看到：

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
- **各框架根节点四元数**的分量顺序（`w,x,y,z` 还是 `x,y,z,w`）与获取命令，逐个列在顺序规则卡片里
- 自动提示常见坑：未展开的 xacro、mimic 关节 / MJCF 的 `<equality>`、多自由度关节（floating / planar / ball）、浮动基座、多根链接、成环、只在 joint 里引用却没定义的 link、MJCF 的 `<include>` 与重名元素
- 中英文界面切换、明暗主题

## 关节集合的比较方式

不同框架天然包含不同的关节，直接比长度会误报，所以：

- 顺序比较只在**两边共有的关节**上进行 —— MuJoCo 不会为 URDF 的 `fixed` 关节生成任何 joint，这不算「顺序错」
- 「N 个位置不同」也是在共有关节上数的：少一个关节会让后面所有下标平移一格，但那不是顺序错，红色只留给**相对顺序真的变了**的关节，纯平移标黄
- 关节集合的差异单独用黄色标出（缺少 / 多出哪些关节），不与顺序错误混在一起
- 默认只显示可动关节；勾选「显示 fixed 关节」后，fixed 关节只会出现在真正持有它们的列（URDF / PyBullet / ros2_control）里

## ros2_control 的三种情况

`joint_state_broadcaster` 的发布顺序取决于配置，工具里可以切换前两种：

1. 未设 `joints` 参数、`use_urdf_to_filter=true`（默认）→ **与 URDF 文件里的关节顺序相同**，与 `<ros2_control>` 标签内的顺序无关
2. 未设 `joints` 参数、`use_urdf_to_filter=false` → 按 resource manager 注册顺序，即 **`<ros2_control>` 标签内 `<joint>` 的顺序**
3. 显式设置了 `joints` + `interfaces` 参数 → 按 `joints` 参数的顺序（本工具读不到 YAML，这种情况请以你的配置为准）

注意各控制器（如 `joint_trajectory_controller`）用的是各自 YAML 里 `joints` 参数的顺序，与上面无关。

## 各框架的根节点四元数

关节顺序只是搬运策略时的一半问题。跟着关节向量一起走的还有**基座姿态**，而它是个四元数 —— 各框架在「标量放前面还是后面」上分成了势均力敌的两派，错了同样不报错。页面「各框架的顺序规则」一节里每张卡片都带一个颜色区分的分量顺序标签（蓝＝标量在前，橙＝标量在后），展开后有获取命令和注意事项：

| 框架 | 分量顺序 | 获取根节点四元数 |
| --- | --- | --- |
| URDF 文件 | **没有四元数** | `<origin rpy="…">`，绕固定轴 X→Y→Z 的欧拉角，**恒为弧度** |
| MJCF 文件 | `w, x, y, z` | `<body quat="w x y z">`（默认 `1 0 0 0`）；也可用 `euler` / `axisangle`，角度单位默认是**度** |
| MuJoCo | `w, x, y, z` | `d.qpos[adr+3 : adr+7]`（`adr = m.jnt_qposadr[jid]`）、`d.body("pelvis").xquat` |
| Isaac Gym (Preview) | `x, y, z, w` | `root_states[:, 3:7]`（每行 13 = 位置 3 + 四元数 4 + 线速度 3 + 角速度 3） |
| Isaac Sim / Isaac Lab | `w, x, y, z`（**3.0 起 `x, y, z, w`**） | `robot.data.root_quat_w`、`view.get_world_poses()[1]` |
| Genesis | `w, x, y, z` | `robot.get_quat()`、`robot.get_qpos()[3:7]` |
| Newton (Warp) | `x, y, z, w` | `state.body_q.numpy()[0][3:7]`（`wp.transform` = `[px py pz qx qy qz qw]`） |
| Gazebo (SDF) | `x, y, z, w`（消息层） | `gz topic -e -t /model/<name>/pose`；但 C++ 的 `gz::math::Quaterniond(w, x, y, z)` 是 **w 在前** |
| PyBullet | `x, y, z, w` | `p.getBasePositionAndOrientation(bid)[1]`，单位四元数 `[0, 0, 0, 1]` |
| ros2_control | `x, y, z, w` | `/joint_states` 里**没有**基座姿态，走 TF：`ros2 run tf2_ros tf2_echo odom base_link` |

三个最容易踩的地方：

- **MuJoCo / Genesis / Newton 的关节顺序完全一样，四元数顺序却不一样** —— Genesis 跟 MuJoCo 都是 `wxyz`，Newton 跟着 Warp 走 `xyzw`。「顺序一样」推不出「约定一样」，两件事各查各的
- **Isaac Lab 3.0 把默认约定从 `wxyz` 改成了 `xyzw`**（对齐 PhysX / Warp / Newton）。这是个破坏性变更，硬编码的 `(1, 0, 0, 0)`、以及没走 `isaaclab.utils.math` 的自定义 MDP 函数升级后全是错的。最快的确认方式是打印一个已知是单位姿态的四元数，看 `1.0` 落在第 0 位还是第 3 位
- **Gazebo 内部两种顺序并存**：消息层（`gz.msgs` / `geometry_msgs`）是 `x, y, z, w`，C++ 数学库（`gz::math::Quaterniond`、`Pose3d`）是 `w` 在前。把消息里的四元数直接塞进构造函数是插件里的经典 bug

另外 URDF 和 SDF 的**文件**里根本没有四元数，`<origin rpy>` / `<pose>` 都是欧拉角（SDFormat 1.9 起可写 `<pose rotation_format="quat_xyzw">`）。而且 **URDF 恒为弧度、MJCF 默认是度** —— 这一条比分量顺序更容易翻车。

四元数约定的出处：

- Isaac Gym vs Isaac Lab：[Migrating from IsaacGymEnvs](https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html)
  > Isaac Lab and Isaac Sim both adopt `wxyz` as the quaternion convention. However, the quaternion convention used in Isaac Gym Preview Release was `xyzw`.
- Isaac Lab 3.0 的变更：[Isaac Lab 3.0 Beta](https://isaac-sim.github.io/IsaacLab/main/source/experimental-features/newton-physics-integration/isaaclab_newton-beta-2.html)
  > we decided to change our default convention to `xyzw`. This means that all our APIs will now return quaternions in the `xyzw` convention.
- Newton / Warp：[Newton — Conventions](https://newton-physics.github.io/newton/stable/concepts/conventions.html)，页面里直接给了对照表和 `newton_quat = (isaac_quat[1], isaac_quat[2], isaac_quat[3], isaac_quat[0])`
- SDFormat 的 `quat_xyzw`：[Specifying pose](http://sdformat.org/tutorials?tut=specify_pose)

## 顺序规则的依据

- Isaac Gym (DFS) vs Isaac Sim / Isaac Lab (BFS)：[Isaac Lab — Migrating from IsaacGymEnvs](https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html)
  > Physics simulation in Isaac Sim and Isaac Lab assumes a breadth-first ordering for the joints in a given kinematic tree. However, Isaac Gym Preview Release assumed a depth-first ordering for joints in the kinematic tree.
- MuJoCo（URDF 导入）：[`src/xml/xml_urdf.cc`](https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_urdf.cc) —— 按文档顺序填充 `urChildren`，再从根 body 递归 `AddToTree()`；`fixed` 关节不生成 joint，`planar` 展开成 2 slide + 1 hinge，`<mimic>` 被忽略
- MuJoCo（MJCF 本身）：[MJCF XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html) —— body 树即 XML 嵌套，`<actuator>` 的顺序就是 `data.ctrl` 的顺序
- Genesis：[`genesis/utils/urdf.py`](https://github.com/Genesis-Embodied-AI/Genesis/blob/main/genesis/utils/urdf.py) 的 `order_links_depth_first()` —— DFS 前序、同层保持原有相对顺序，注释里明写「the result matches MuJoCo's body ordering」；而 [`rigid_entity.py`](https://github.com/Genesis-Embodied-AI/Genesis/blob/main/genesis/engine/entities/rigid_entity/rigid_entity.py) 更进一步，直接用 MuJoCo 统一解析器的结果覆盖 link / joint（`l_infos = l_infos_mj`）。`merge_fixed_links=True`、`fixed=False`（自动补 `root_joint`）是 `gs.morphs.URDF` 的默认值
- Newton：[`import_urdf.py`](https://github.com/newton-physics/newton/blob/main/newton/_src/utils/import_urdf.py) 的 `joint_ordering` 默认 `"dfs"`，交给 [`topology.py`](https://github.com/newton-physics/newton/blob/main/newton/_src/utils/topology.py) 的 `topological_sort(use_dfs=True)`，遍历子边时 `sorted(outgoing[node], key=joint_id)`，即同层按 `<joint>` 文档顺序；`collapse_fixed_joints` 默认 `False`，基座关节由 `_add_base_joint()` 排在最前
- Gazebo：[sdformat `parser_urdf.cc`](https://github.com/gazebosim/sdformat/blob/sdf14/src/parser_urdf.cc) —— `CreateSDF()` 递归遍历 `_link->child_links`（DFS）；而 `child_links` 由 [urdfdom_headers `model.h`](https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/model.h) 的 `initTree()` 遍历 `std::map` 类型的 `joints_` 填充，因此同层子关节是**按关节名字母序**。sdformat 默认还会吸收 fixed 关节
- PyBullet：[bullet3 `URDF2Bullet.cpp`](https://github.com/bulletphysics/bullet3/blob/master/examples/Importers/ImportURDFDemo/URDF2Bullet.cpp) —— 默认 `ConvertURDF2BulletInternal()` 从根递归（DFS）；同层顺序来自 [UrdfParser `initTreeAndRoot()`](https://github.com/bulletphysics/bullet3/blob/master/examples/Importers/ImportURDFDemo/UrdfParser.cpp) 按关节文档顺序填充的 `m_childLinks`。`getJointInfo` 包含 fixed 关节
- ros2_control：[joint_state_broadcaster 文档](https://control.ros.org/rolling/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html)

## 已知限制

- **只接受展开后的 URDF**。检测到 `<xacro:*>` 或 `${}` 会提示先跑 `xacro robot.urdf.xacro > robot.urdf`
- **MJCF 的 `<include>` 不会被跟进**（浏览器里读不到别的文件），结果只反映当前这一个文件；含 `<include>` 时会明确提示
- MJCF 里驱动 tendon / site / body 的执行器不对应任何关节，但一样占 `ctrl` 槽位，所以「MuJoCo ctrl」列的序号在这种模型上会小于真实 `ctrl` 下标（同样会提示）
- 结论是**按公开的导入规则静态推导**的，不运行任何仿真器。生产环境请始终以运行时打印的关节名列表为准：`robot.data.joint_names`（Isaac Lab）、`mj_id2name(m, mjOBJ_JOINT, i)`（MuJoCo）、`gym.get_asset_dof_names(asset)`（Isaac Gym）、`getJointInfo(body, i)[1]`（PyBullet）、`[j.name for j in entity.joints]`（Genesis）、`model.joint_label`（Newton）
- 导入器的选项会改变结果（例如 Isaac 的 `merge_fixed_joints`、MuJoCo 的 `fusestatic`、Isaac Gym 的 `collapse_fixed_joints`、sdformat 的 `disableFixedJointLumping` / `preserveFixedJoint`、PyBullet 的 `URDF_MAINTAIN_LINK_ORDER` / `URDF_MERGE_FIXED_LINKS`、Genesis 的 `merge_fixed_links` / `links_to_keep` / `fixed`、Newton 的 `collapse_fixed_joints` / `joint_ordering`），工具按各自的默认行为计算
- **各框架自己合成的基座关节不会出现在表格里**：Genesis 默认补的 `root_joint`（free，6 DOF）和 Newton 一定会插在最前面的 `fixed_base` / `floating_base`，都不是文件里的关节，工具只列文件里有的。对这两列，运行时的关节下标要在表格序号上再加 1，DOF 下标另按各自基座的自由度平移
- Gazebo 一列指的是 **URDF→SDF 转换后模型里的关节顺序**（`Model::GetJoints()` 走这个顺序）。`gazebo_ros_joint_state_publisher` 插件按你列的 `<joint_name>` 顺序发布，`gz_ros2_control` 走 `<ros2_control>` 标签，两者都与这一列无关
- PyBullet 一列指的是默认 `loadURDF(flags=0)` 之后 `getJointInfo(i)` 的顺序。勾选「显示 fixed 关节」才与运行时下标一致；`setJointMotorControlArray` 如果自己过滤了 fixed，用的是可动关节子序列
- 多自由度关节（`floating` / `planar`）在各框架展开成的 DOF 数量和排列不同，关节级顺序一致**不代表** DOF 级一致，工具会单独警告
- `<mimic>` 关节各框架支持程度不同，只作提示，不改变顺序推导

## 本地运行

不需要构建步骤，任何静态服务器都行：

```bash
git clone https://github.com/ImChong/Robot_Joint_Order_Check_Tool.git
cd Robot_Joint_Order_Check_Tool
python3 -m http.server 8000   # 然后打开 http://localhost:8000
```

顺序规则的无浏览器检查：

```bash
node tools/test-orderings.mjs
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
tools/test-orderings.mjs   各框架顺序规则的无浏览器检查（含 Genesis / Newton / PyBullet 列）
```

改过 `assets/js/samples.js` 之后跑 `node tools/export-samples.mjs` 同步示例文件，CI 会检查两者是否一致。收录的第三方模型不会被这个脚本改写，只检查存在性。改过 `assets/js/orderings.js` 之后跑 `node tools/test-orderings.mjs`。

---

## English

Upload a **URDF or a MuJoCo MJCF (`.xml`)** — the format is detected from the root element — and this page prints the robot's joint order **side by side** as seen by Isaac Gym, Isaac Sim / Isaac Lab, MuJoCo, Genesis, Newton, Gazebo, PyBullet and ros2_control. Green when every framework agrees, red on the exact cells that don't — plus a generated index-remap array you can paste into your code.

URDF itself defines no joint order, so each downstream tool imposes its own: MuJoCo and Isaac Gym walk the kinematic tree depth-first, Isaac Sim / Isaac Lab walk it breadth-first, Gazebo also walks it depth-first but sorts siblings alphabetically (sdformat recurses over urdfdom's `child_links`, filled from a `std::map`), PyBullet is the same DFS as Isaac Gym but **keeps fixed joints in `getJointInfo`**, and ros2_control follows the URDF or the `<ros2_control>` tag depending on configuration. That mismatch is what silently scrambles a joint vector when you move a policy between them.

**Genesis** and **Newton** both land on the same depth-first order as MuJoCo — Genesis because it hands the kinematic structure to MuJoCo's unified parser outright, Newton because `parse_urdf(joint_ordering="dfs")` sorts each node's children by joint id. What differs is what they put *around* your joints: Genesis merges fixed links away by default (`merge_fixed_links=True`) and prepends a synthetic free `root_joint` unless you pass `fixed=True`, while Newton keeps fixed joints as 0-DOF entries that still consume a joint index and always prepends a base joint of its own, so `model.joint_label[0]` is never your first joint. Newton is also the one importer that lets you pick: `joint_ordering="bfs"` reproduces the Isaac Sim column.

**MJCF** is a different question: the XML nesting *is* the body tree, so the file order is the joint id order MuJoCo compiles to, and the interesting mismatch moves elsewhere — `data.ctrl` follows the `<actuator>` block, a separate vector, which gets its own column. Two official Unitree G1 models are bundled. The **29-DOF body** (`g1_29dof_rev_1_0`, no hands) is what humanoid parkour policies such as [ParkourFormer](https://arxiv.org/abs/2605.25782) actually deploy: a 96-D observation `[ang vel 3, gravity 3, cmd 3, q 29, dq 29, last action 29]` and a 29-D action. MuJoCo (ParkourFormer's Instinct pipeline) walks that tree DFS; Isaac Sim / Isaac Lab walk it BFS — they already disagree at the second joint. That 29-DOF MJCF's actuators match the joints 1-to-1. The **29-DOF with-hands** model is the other live example: its official URDF and MJCF agree on joint order, but the MJCF's last four right-hand actuators (indices 39–42) are written `index_0, index_1, middle_0, middle_1` while the joints run `middle_0, middle_1, index_0, index_1`. Bodies with no joint (welds), `<default>` class inheritance, free/ball joints and `<equality>` constraints are all handled; `<include>` is not followed and is reported instead.

**Root quaternions** get the same treatment, because the base orientation travels with the joint vector and the frameworks split almost evenly on component order. Each rule card carries a colour-coded chip — `w, x, y, z` for MuJoCo, Genesis and MJCF files, `x, y, z, w` for Isaac Gym, Newton, Gazebo messages, PyBullet and ROS — plus the call that prints it. Three things bite: MuJoCo, Genesis and Newton agree on joint order yet **not** on quaternion order (Newton follows Warp's `xyzw`); **Isaac Lab 3.0 switched its default from `wxyz` to `xyzw`** to match PhysX / Warp / Newton, a silent breaking change for hard-coded literals; and Gazebo carries both at once — `x, y, z, w` in messages, `w`-first in `gz::math::Quaterniond`. A URDF has no quaternion at all: `<origin rpy>` is Euler, always in radians, while MJCF defaults to degrees.

Everything runs client-side — no upload, no build step. Live at <https://imchong.github.io/Robot_Joint_Order_Check_Tool/>; run locally with `python3 -m http.server 8000`.

See the sections above for the exact ordering rule per framework, the sources they are derived from, and the known limitations (expanded URDF only; static derivation — always confirm against the joint names your runtime prints).

---

## 许可证 / License

[MIT](LICENSE) © 2026 Chong Liu
