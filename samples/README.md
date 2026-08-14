# samples/

示例文件 / sample robot descriptions.

## 合成示例 / Synthetic samples

`arm.urdf`、`quadruped.urdf`、`ros2control.urdf` 由 [`assets/js/samples.js`](../assets/js/samples.js) 生成，
改过那个文件之后运行 `node tools/export-samples.mjs` 重新导出（CI 会检查两者是否一致）。

Generated from `assets/js/samples.js`; re-run `node tools/export-samples.mjs` after editing it.

## 收录的第三方模型 / Vendored third-party models

| 文件 / File | 来源 / Upstream |
| --- | --- |
| `g1_29dof_with_hand_rev_1_0.urdf` | [unitree_ros / robots/g1_description](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_with_hand_rev_1_0.urdf) |
| `g1_29dof_with_hand_rev_1_0.xml` | [unitree_ros / robots/g1_description](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_with_hand_rev_1_0.xml) |

两个文件均**原样收录、未作修改**，只是这里不带 `meshes/`（本工具只读 link / joint 拓扑，不需要网格）。
版权归 Unitree Robotics 所有，按 BSD 3-Clause 许可分发，许可证全文见 [`LICENSE.unitree_ros`](LICENSE.unitree_ros)。

Both files are **verbatim, unmodified copies** from [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros);
the referenced `meshes/` are not vendored (this tool only reads the link/joint topology).
Copyright Unitree Robotics, distributed under the BSD 3-Clause License — see
[`LICENSE.unitree_ros`](LICENSE.unitree_ros) for the full text.

这两个示例在页面上是点击时从 `samples/` 读取的，所以用 `file://` 直接打开页面时无法载入 —— 请用
`python3 -m http.server` 之类的本地服务器，或把文件拖进页面。

These two are fetched from `samples/` when their button is clicked, so they do not load when the page
itself is opened over `file://` — serve it locally, or drag the file into the page.
