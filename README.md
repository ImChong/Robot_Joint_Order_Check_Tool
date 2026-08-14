# Robot_Joint_Order_Check_Tool

机器人关节顺序检查工具 —— 上传一个 URDF，把它在 **Isaac Gym / Isaac Sim (Isaac Lab) / MuJoCo / Gazebo / ros2_control** 中的关节顺序并列打印出来。顺序全部一致就显示绿色，任何一个框架的顺序对不上就标红，并指出差在哪里。

**在线使用：** <https://imchong.github.io/Robot_Joint_Order_Check_Tool/>

纯静态页面，URDF 完全在浏览器本地解析，不上传任何服务器。

---

## 为什么需要这个工具

URDF 本身**不定义关节顺序**——它只是一堆 link 和 joint 的集合。每个下游工具按自己的规则把它们排成一个向量：

| 工具 | 顺序规则 |
| --- | --- |
| URDF 文件 | `<joint>` 元素的文档顺序（作为参照基准） |
| Isaac Gym (Preview) | 运动学树**深度优先**（DFS），同层按 URDF 文档顺序 |
| Isaac Sim / Isaac Lab | 运动学树**广度优先**（BFS，PhysX stage parser） |
| MuJoCo | body 树**深度优先**（DFS），同层按 URDF 文档顺序 |
| Gazebo (SDF) | **深度优先**（DFS），但**同层按关节名字母序**（sdformat 走 urdfdom 的 `child_links`） |
| ros2_control | 取决于配置，默认跟 URDF 文件顺序一致（见下） |

于是一个四足机器人会出现这种经典翻车现场：

```
Isaac Gym / MuJoCo (DFS)   : FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, ...
Isaac Sim / Isaac Lab (BFS): FL_hip, FR_hip,   RL_hip,   RR_hip, FL_thigh, FR_thigh, ...
```

把在 Isaac Gym 里训好的策略直接部署到 Isaac Lab 或实机上，关节向量就会静默错位——不报错，机器人直接抽搐。这个工具就是用来在写代码之前先把这件事看清楚的。

Gazebo 的坑更隐蔽：它也是 DFS，但**同层子关节按关节名字母序**，所以只要某个 link 的多个子关节的书写顺序不等于字母序，Gazebo 就会和 MuJoCo / Isaac Gym 分道扬镳。内置示例「移动机械臂」就是这种情况 —— `base_link` 下先写轮子后写手臂，Gazebo 却把 `arm_shoulder_pan` 排到了两个 `drive_wheel_*` 前面。

## 功能

- **拖拽 / 选择文件 / 粘贴文本**三种方式载入 URDF，或直接点内置示例
- 六列**并列对照表**，逐格标出与参照顺序不同的位置；也可切换成「按关节」视图看每个关节在各框架中的索引和偏移量
- 绿 / 红 / 黄三态结论：顺序一致、顺序不一致、顺序一致但关节集合不同
- **索引重映射代码生成**：选好源框架和目标框架，直接产出可粘贴的 Python / C++ 索引数组
- 导出 CSV 与完整 JSON
- 运动学树可视化、关节类型与 DOF 列表
- 自动提示常见坑：未展开的 xacro、mimic 关节、多自由度关节（floating / planar）、多根链接、成环、只在 joint 里引用却没定义的 link
- 中英文界面切换、明暗主题

## 关节集合的比较方式

不同框架天然包含不同的关节，直接比长度会误报，所以：

- 顺序比较只在**两边共有的关节**上进行 —— MuJoCo 不会为 URDF 的 `fixed` 关节生成任何 joint，这不算「顺序错」
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
- MuJoCo：[`src/xml/xml_urdf.cc`](https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_urdf.cc) —— 按文档顺序填充 `urChildren`，再从根 body 递归 `AddToTree()`；`fixed` 关节不生成 joint，`planar` 展开成 2 slide + 1 hinge，`<mimic>` 被忽略
- Gazebo：[sdformat `parser_urdf.cc`](https://github.com/gazebosim/sdformat/blob/sdf14/src/parser_urdf.cc) —— `CreateSDF()` 递归遍历 `_link->child_links`（DFS）；而 `child_links` 由 [urdfdom_headers `model.h`](https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/model.h) 的 `initTree()` 遍历 `std::map` 类型的 `joints_` 填充，因此同层子关节是**按关节名字母序**。sdformat 默认还会吸收 fixed 关节
- ros2_control：[joint_state_broadcaster 文档](https://control.ros.org/rolling/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html)

## 已知限制

- **只接受展开后的 URDF**。检测到 `<xacro:*>` 或 `${}` 会提示先跑 `xacro robot.urdf.xacro > robot.urdf`
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
assets/js/orderings.js     各框架的顺序规则与一致性分析
assets/js/app.js           界面逻辑
samples/*.urdf             由 samples.js 导出的示例文件
tools/export-samples.mjs   重新生成 samples/*.urdf
```

改过 `assets/js/samples.js` 之后跑 `node tools/export-samples.mjs` 同步示例文件，CI 会检查两者是否一致。

---

## English

Upload a URDF and this page prints the robot's joint order **side by side** as seen by Isaac Gym, Isaac Sim / Isaac Lab, MuJoCo, Gazebo and ros2_control. Green when every framework agrees, red on the exact cells that don't — plus a generated index-remap array you can paste into your code.

URDF itself defines no joint order, so each downstream tool imposes its own: MuJoCo and Isaac Gym walk the kinematic tree depth-first, Isaac Sim / Isaac Lab walk it breadth-first, Gazebo also walks it depth-first but sorts siblings alphabetically (sdformat recurses over urdfdom's `child_links`, filled from a `std::map`), and ros2_control follows the URDF or the `<ros2_control>` tag depending on configuration. That mismatch is what silently scrambles a joint vector when you move a policy between them.

Everything runs client-side — no upload, no build step. Live at <https://imchong.github.io/Robot_Joint_Order_Check_Tool/>; run locally with `python3 -m http.server 8000`.

See the sections above for the exact ordering rule per framework, the sources they are derived from, and the known limitations (expanded URDF only; static derivation — always confirm against the joint names your runtime prints).

---

## 许可证 / License

[MIT](LICENSE) © 2026 Chong Liu
