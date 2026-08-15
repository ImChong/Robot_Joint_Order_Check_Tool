/* Per-framework joint ordering rules + consistency analysis */
(function (global) {
  'use strict';

  var URDF = global.URDF;

  function byName(a, b) { return a.name < b.name ? -1 : a.name > b.name ? 1 : 0; }

  function sortSiblings(joints, siblingOrder) {
    var list = joints.slice();
    if (siblingOrder === 'alphabetical') list.sort(byName);
    return list;
  }

  /* Depth-first pre-order over the kinematic tree. */
  function dfsOrder(tree, siblingOrder) {
    var out = [];
    var visited = {};
    function visit(link) {
      if (visited[link]) return;
      visited[link] = true;
      sortSiblings(tree.childJoints[link] || [], siblingOrder).forEach(function (j) {
        out.push(j);
        visit(j.child);
      });
    }
    tree.roots.forEach(visit);
    return out;
  }

  /* Breadth-first (level-by-level) over the kinematic tree. */
  function bfsOrder(tree, siblingOrder) {
    var out = [];
    var visited = {};
    var queue = tree.roots.slice();
    tree.roots.forEach(function (r) { visited[r] = true; });
    while (queue.length) {
      var link = queue.shift();
      sortSiblings(tree.childJoints[link] || [], siblingOrder).forEach(function (j) {
        out.push(j);
        if (!visited[j.child]) {
          visited[j.child] = true;
          queue.push(j.child);
        }
      });
    }
    return out;
  }

  /* ── Framework definitions ─────────────────────────────────────────
     compute(ctx) -> array of joint objects, or null when not applicable.
     `holdsFixed` marks frameworks whose joint list can contain fixed joints
     at all; Isaac / MuJoCo / Gazebo / Genesis create no joint for a URDF fixed
     joint, while PyBullet's getJointInfo and Newton's joint array keep every
     child link (fixed included).
     `holdsFreeBase` marks the ones that expose a floating base as a joint —
     MuJoCo does (a free joint, first in the vector), and so do Genesis and
     Newton, while Isaac / Gazebo / PyBullet / ros2_control treat it as a
     free-floating root instead.
     `labelByFormat` / `ruleByFormat` override the generic text once the input
     format is known, so the same column can read "URDF" or "MJCF".        */
  var FRAMEWORKS = [
    {
      id: 'file',
      label: { zh: 'URDF / MJCF 文件顺序', en: 'URDF / MJCF file order' },
      labelByFormat: {
        urdf: { zh: 'URDF 文件顺序', en: 'URDF file order' },
        mjcf: { zh: 'MJCF 文件顺序', en: 'MJCF file order' }
      },
      rule: { zh: '<joint> 元素的文档顺序', en: 'document order of <joint>' },
      ruleByFormat: {
        mjcf: { zh: 'body 树中 <joint> 的文档顺序', en: 'document order of <joint> in the body tree' }
      },
      holdsFixed: true,
      holdsFreeBase: true,
      compute: function (ctx) { return ctx.model.joints.slice(); }
    },
    {
      id: 'isaacgym',
      label: { zh: 'Isaac Gym (Preview)', en: 'Isaac Gym (Preview)' },
      rule: { zh: '运动学树深度优先（DFS）', en: 'depth-first over kinematic tree' },
      holdsFixed: false,
      holdsFreeBase: false,
      siblingOrder: 'document',
      compute: function (ctx) { return dfsOrder(ctx.tree, ctx.siblingOrder); }
    },
    {
      id: 'isaacsim',
      label: { zh: 'Isaac Sim / Isaac Lab', en: 'Isaac Sim / Isaac Lab' },
      rule: { zh: '运动学树广度优先（BFS）', en: 'breadth-first over kinematic tree' },
      holdsFixed: false,
      holdsFreeBase: false,
      siblingOrder: 'document',
      compute: function (ctx) { return bfsOrder(ctx.tree, ctx.siblingOrder); }
    },
    {
      id: 'mujoco',
      label: { zh: 'MuJoCo', en: 'MuJoCo' },
      rule: { zh: 'body 树深度优先（DFS）', en: 'depth-first over body tree' },
      ruleByFormat: {
        mjcf: { zh: 'body 树深度优先（= 文件顺序）', en: 'depth-first over body tree (= file order)' }
      },
      holdsFixed: false,
      holdsFreeBase: true,
      siblingOrder: 'document',
      compute: function (ctx) { return dfsOrder(ctx.tree, ctx.siblingOrder); }
    },
    {
      id: 'genesis',
      label: { zh: 'Genesis', en: 'Genesis' },
      // Genesis parses a URDF with its own urdfpy fork for geometry, then lets
      // MuJoCo's unified parser overwrite the kinematic structure outright
      // (`l_infos = l_infos_mj` in rigid_entity.py), and MJCF goes straight
      // through mujoco. Its own fallback, order_links_depth_first(), is the
      // same DFS pre-order with siblings in their original relative order —
      // the comment there says it "matches MuJoCo's body ordering".
      rule: { zh: 'body 树深度优先（同 MuJoCo）', en: 'depth-first over body tree (same as MuJoCo)' },
      // URDF: gs.morphs.URDF defaults to merge_fixed_links=True, so a fixed
      // joint loses its child link entirely. MJCF: a body with no <joint> still
      // becomes a 0-DOF FIXED joint, named after the body.
      holdsFixed: function (ctx) { return ctx.model.format === 'mjcf'; },
      holdsFreeBase: true,
      siblingOrder: 'document',
      compute: function (ctx) { return dfsOrder(ctx.tree, ctx.siblingOrder); }
    },
    {
      id: 'newton',
      label: { zh: 'Newton (Warp)', en: 'Newton (Warp)' },
      // parse_urdf() defaults to joint_ordering="dfs", handing the (parent,
      // child) edges to topological_sort(use_dfs=True), which walks children as
      // `sorted(outgoing[node], key=joint_id)` — document order, like MuJoCo.
      // collapse_fixed_joints defaults to False, so a URDF fixed joint stays in
      // the array as a 0-DOF JointType.FIXED, and a jointless MJCF body gets a
      // fixed joint of its own too. Both occupy a joint index without a DOF.
      rule: { zh: 'DFS 拓扑排序（含 fixed 关节）', en: 'depth-first topological sort, fixed joints included' },
      holdsFixed: true,
      holdsFreeBase: true,
      siblingOrder: 'document',
      compute: function (ctx) { return dfsOrder(ctx.tree, ctx.siblingOrder); }
    },
    {
      id: 'gazebo',
      label: { zh: 'Gazebo (SDF)', en: 'Gazebo (SDF)' },
      // sdformat recurses over urdf::Link::child_links, which urdfdom fills by
      // iterating its alphabetically-sorted joints_ map — so siblings come out
      // alphabetical no matter how the URDF is written.
      rule: { zh: 'DFS，同层按关节名字母序', en: 'depth-first, siblings alphabetical' },
      holdsFixed: false,
      holdsFreeBase: false,
      siblingOrder: 'alphabetical',
      compute: function (ctx) {
        if (ctx.model.format === 'mjcf') return null;   // sdformat reads no MJCF
        return dfsOrder(ctx.tree, ctx.siblingOrder);
      },
      naReason: 'na.urdfOnly'
    },
    {
      id: 'pybullet',
      label: { zh: 'PyBullet', en: 'PyBullet' },
      // Default loadURDF (flags=0): ConvertURDF2BulletInternal recurses over
      // getLinkChildIndices(), which follows m_childLinks filled in joint
      // document order — DFS with document-order siblings. Unlike Isaac /
      // MuJoCo / Gazebo, getJointInfo includes every child link's joint,
      // fixed ones included (base is index -1, not a joint).
      rule: { zh: 'DFS（含 fixed 关节）', en: 'depth-first, including fixed joints' },
      holdsFixed: true,
      holdsFreeBase: false,
      siblingOrder: 'document',
      compute: function (ctx) {
        // MJCF weld edges keep the body tree connected but are not joints
        // PyBullet would name; drop them so the column stays joint-valued.
        return dfsOrder(ctx.tree, ctx.siblingOrder).filter(function (j) {
          return j.type !== 'weld';
        });
      }
    },
    {
      id: 'ros2control',
      label: { zh: 'ros2_control', en: 'ros2_control' },
      rule: {
        zh: 'joint_state_broadcaster 发布顺序',
        en: 'joint_state_broadcaster publish order'
      },
      holdsFixed: true,
      holdsFreeBase: false,
      compute: function (ctx) {
        if (!ctx.model.ros2ControlJoints.length) return null;
        var wanted = {};
        ctx.model.ros2ControlJoints.forEach(function (n) { wanted[n] = true; });

        if (ctx.ros2cMode === 'tag') {
          // Resource-manager order: the <joint> elements inside <ros2_control>.
          return ctx.model.ros2ControlJoints
            .map(function (n) { return ctx.model.jointByName[n]; })
            .filter(Boolean);
        }
        // use_urdf_to_filter = true (default): URDF order, filtered to the tag's joints.
        return ctx.model.joints.filter(function (j) { return wanted[j.name]; });
      },
      naReason: function (ctx) { return ctx.model.format === 'mjcf' ? 'na.urdfOnly' : 'na.noTag'; }
    },
    {
      id: 'mjcfctrl',
      label: { zh: 'MuJoCo ctrl（执行器）', en: 'MuJoCo ctrl (actuators)' },
      rule: { zh: '<actuator> 元素的文档顺序', en: 'document order of <actuator>' },
      holdsFixed: false,
      holdsFreeBase: true,
      // data.ctrl is indexed by actuator, not by joint — a separate vector that
      // only looks like the joint one until someone reorders the <actuator> block.
      compute: function (ctx) {
        var acts = ctx.model.actuators;
        if (!acts || !acts.length) return null;
        return acts
          .map(function (a) { return a.joint ? ctx.model.jointByName[a.joint] : null; })
          .filter(Boolean);
      },
      naReason: 'na.noActuator'
    }
  ];

  function pick(map, byFormat, lang, format) {
    var src = (format && byFormat && byFormat[format]) || map;
    return src[lang] || src.en;
  }

  function labelOf(fw, lang, format) { return pick(fw.label, fw.labelByFormat, lang, format); }
  function ruleOf(fw, lang, format) { return pick(fw.rule, fw.ruleByFormat, lang, format); }

  /* ── Root-node quaternion ──────────────────────────────────────────
     Joint order is only half of the porting problem: the base pose that
     travels alongside the joint vector is a quaternion, and the frameworks
     split almost evenly between scalar-first and scalar-last. `order` drives
     the chip's wording and colour; a column whose answer does not fit in one
     word (URDF has no quaternion at all, Isaac changed its mind in 3.0)
     carries an explicit `chip` instead.                                   */
  var QUAT_TEXT = { wxyz: 'w, x, y, z', xyzw: 'x, y, z, w' };

  function quatChipOf(doc, lang) {
    var q = doc && doc.quat;
    if (!q) return '';
    if (q.chip) return q.chip[lang] || q.chip.en;
    return QUAT_TEXT[q.order] || '';
  }

  /**
   * Joints that carry a floating base: 6-DOF and hanging off a root link.
   * MuJoCo turns these into a free joint at the head of the vector; the other
   * importers drop them and let the articulation root float instead.
   */
  function freeBaseJoints(model, tree) {
    var rootSet = {};
    tree.roots.forEach(function (r) { rootSet[r] = true; });
    return model.joints.filter(function (j) {
      return URDF.dofOf(j.type) === 6 && rootSet[j.parent];
    });
  }

  /**
   * Compare a sequence against the reference on the intersection of their
   * joint sets, so a framework that legitimately omits joints (e.g. fixed
   * joints in MuJoCo) is not reported as an ordering bug.
   *
   * Everything is measured on that intersection: one missing joint shifts
   * every later index by one without a single joint being out of order, and
   * counting those as differences would drown the joints that really moved.
   * `misordered` names the ones that did.
   */
  function compareOrders(refNames, names) {
    var refSet = {}, set = {};
    refNames.forEach(function (n) { refSet[n] = true; });
    names.forEach(function (n) { set[n] = true; });

    var a = refNames.filter(function (n) { return set[n]; });
    var b = names.filter(function (n) { return refSet[n]; });

    // Null-prototype: joint names are arbitrary strings, and "constructor" or
    // "toString" would otherwise resolve against Object.prototype.
    var rankRef = Object.create(null), rankSeq = Object.create(null);
    a.forEach(function (n, i) { if (!(n in rankRef)) rankRef[n] = i; });
    b.forEach(function (n, i) { if (!(n in rankSeq)) rankSeq[n] = i; });

    var misordered = Object.create(null);
    a.forEach(function (n) { if (rankRef[n] !== rankSeq[n]) misordered[n] = true; });

    var diffPositions = [];
    var n = Math.max(a.length, b.length);
    for (var k = 0; k < n; k++) {
      if (a[k] !== b[k]) diffPositions.push(k);
    }

    return {
      orderMatch: diffPositions.length === 0,
      common: a,
      missing: refNames.filter(function (nm) { return !set[nm]; }),
      extra: names.filter(function (nm) { return !refSet[nm]; }),
      misordered: misordered,
      diffPositions: diffPositions
    };
  }

  /**
   * Run every framework rule and compare against the reference column.
   * @param opts {refId, siblingOrder, ros2cMode, showFixed}
   */
  function analyze(model, tree, opts) {
    opts = opts || {};
    var showFixed = !!opts.showFixed;
    // 'auto' honours each framework's own sibling rule; the other values force
    // one rule across every column so the effect can be inspected directly.
    var override = opts.siblingOrder || 'auto';

    var freeBase = {};
    freeBaseJoints(model, tree).forEach(function (j) { freeBase[j.name] = true; });

    var columns = FRAMEWORKS.map(function (fw) {
      var ctx = {
        model: model,
        tree: tree,
        ros2cMode: opts.ros2cMode || 'urdf',
        siblingOrder: override === 'auto' ? (fw.siblingOrder || 'document') : override
      };
      var seq = null;
      try {
        seq = fw.compute(ctx);
      } catch (e) {
        seq = null;
      }
      // Most simulators create no joint at all for a URDF fixed joint, so it
      // can never appear in those columns. PyBullet / Newton (and the URDF /
      // ros2_control lists) do keep them; the toggle only reveals fixed joints
      // in columns whose joint list can actually contain them — which for
      // Genesis depends on whether the input is a URDF or an MJCF.
      var holdsFixed = typeof fw.holdsFixed === 'function' ? fw.holdsFixed(ctx) : fw.holdsFixed;
      if (seq && (!showFixed || !holdsFixed)) {
        seq = seq.filter(function (j) { return URDF.isMovable(j); });
      }
      // Same idea for the floating base: it is a joint in MuJoCo and a free
      // root everywhere else, so it must not shift the other vectors.
      if (seq && !fw.holdsFreeBase) {
        seq = seq.filter(function (j) { return !freeBase[j.name]; });
      }
      var names = seq ? seq.map(function (j) { return j.name; }) : null;

      // DOF index at which each joint starts, within this framework's vector.
      var dofStart = {}, acc = 0;
      if (seq) {
        seq.forEach(function (j) {
          dofStart[j.name] = acc;
          acc += URDF.dofOf(j.type);
        });
      }

      return {
        id: fw.id,
        label: fw,
        seq: seq,
        names: names,
        dofStart: dofStart,
        totalDof: acc,
        naReasonKey: typeof fw.naReason === 'function' ? fw.naReason(ctx) : (fw.naReason || null),
        holdsFixed: holdsFixed
      };
    });

    var available = columns.filter(function (c) { return c.names !== null; });

    var refId = opts.refId;
    var ref = available.filter(function (c) { return c.id === refId; })[0] || available[0];

    columns.forEach(function (c) {
      if (!c.names || !ref) { c.status = 'na'; return; }
      if (ref && c.id === ref.id) { c.status = 'ref'; c.cmp = null; return; }
      c.cmp = compareOrders(ref.names, c.names);
      if (!c.cmp.orderMatch) c.status = 'bad';
      else if (c.cmp.missing.length || c.cmp.extra.length) c.status = 'warn';
      else c.status = 'ok';
    });

    var comparable = columns.filter(function (c) { return c.status === 'ok' || c.status === 'bad' || c.status === 'warn'; });
    var bad = comparable.filter(function (c) { return c.status === 'bad'; });
    var warn = comparable.filter(function (c) { return c.status === 'warn'; });

    var verdict;
    if (!comparable.length) verdict = 'single';
    else if (bad.length) verdict = 'bad';
    else if (warn.length) verdict = 'warn';
    else verdict = 'ok';

    /* Union of all joints in any column, reference order first. */
    var allNames = [];
    var seen = {};
    var pool = (ref ? ref.names.slice() : []);
    columns.forEach(function (c) { pool = pool.concat(c.names || []); });
    pool.forEach(function (n) { if (!seen[n]) { seen[n] = true; allNames.push(n); } });

    return {
      columns: columns,
      available: available,
      reference: ref,
      verdict: verdict,
      badColumns: bad,
      warnColumns: warn,
      allNames: allNames,
      maxLen: columns.reduce(function (m, c) { return Math.max(m, c.names ? c.names.length : 0); }, 0)
    };
  }

  /* Reference notes rendered in the "ordering rules" section. */
  var RULE_DOCS = [
    {
      id: 'file',
      verify: '# URDF\nfrom urdf_parser_py.urdf import URDF\nprint([j.name for j in URDF.from_xml_file("robot.urdf").joints])\n\n# MJCF\nimport mujoco\nm = mujoco.MjModel.from_xml_path("robot.xml")\nprint([m.joint(i).name for i in range(m.njnt)])',
      quat: {
        order: 'mixed',
        chip: { zh: 'URDF rpy · MJCF w, x, y, z', en: 'URDF rpy · MJCF w, x, y, z' },
        code: '<!-- URDF: no quaternion anywhere. The root pose is fixed-axis RPY, in radians -->\n<joint name="floating_base" type="floating">\n  <origin xyz="0 0 0.8" rpy="0 0 0"/>              <!-- roll, pitch, yaw -->\n</joint>\n\n<!-- MJCF: an explicit quaternion, scalar first -->\n<body name="pelvis" pos="0 0 0.8" quat="1 0 0 0">  <!-- w x y z, identity -->\n  <freejoint name="root"/>                         <!-- qpos[0:3] = xyz, qpos[3:7] = w x y z -->\n</body>',
        note: {
          zh: '<p>URDF <b>没有四元数</b>：<code>&lt;origin&gt;</code> 只有 <code>rpy</code>，是绕固定轴依次 X→Y→Z 旋转的欧拉角，单位<b>恒为弧度</b>。所以「URDF 的四元数顺序」这个问题本身不成立 —— 四元数是各框架把文件读进去之后才有的东西，顺序也就随各框架而变。</p><p>MJCF 则直接写四元数：<code>&lt;body quat="w x y z"&gt;</code> <b>标量在前</b>，默认 <code>"1 0 0 0"</code>。同一个 body 也可以改用 <code>euler</code> / <code>axisangle</code>（<code>x y z 角度</code>）/ <code>xyaxes</code> / <code>zaxis</code>；欧拉角的轴序由 <code>&lt;compiler eulerseq&gt;</code> 决定（默认 <code>xyz</code>），角度单位由 <code>&lt;compiler angle&gt;</code> 决定 —— <b>MJCF 默认是「度」，URDF 恒为弧度</b>，这一条比四元数顺序更容易翻车。</p>',
          en: '<p>A URDF has <b>no quaternion at all</b>: <code>&lt;origin&gt;</code> carries only <code>rpy</code>, fixed-axis X→Y→Z Euler angles, <b>always in radians</b>. So "the URDF quaternion order" is not a well-formed question — the quaternion only comes into existence once a framework loads the file, which is why the answer varies by column.</p><p>MJCF writes one out explicitly: <code>&lt;body quat="w x y z"&gt;</code> is <b>scalar first</b> and defaults to <code>"1 0 0 0"</code>. The same body may instead use <code>euler</code> / <code>axisangle</code> (<code>x y z angle</code>) / <code>xyaxes</code> / <code>zaxis</code>; the Euler axis sequence comes from <code>&lt;compiler eulerseq&gt;</code> (default <code>xyz</code>) and the unit from <code>&lt;compiler angle&gt;</code> — <b>degrees by default in MJCF, always radians in URDF</b>, which bites more often than the component order does.</p>'
        }
      },
      body: {
        zh: '<p>URDF 本身不定义关节顺序，只是一个 link/joint 的集合。这里取 <code>&lt;joint&gt;</code> 元素在 XML 中出现的先后顺序，作为最直观的参照基准。</p><p><b>注意：</b>URDF 规范并不要求关节按树的顺序书写，父关节写在子关节后面也是合法的。</p><p><b>MJCF：</b>MuJoCo 的 XML 嵌套本身就是 body 树，所以载入 MJCF 时这一列不只是「参照基准」，它<b>就是 MuJoCo 编译出来的关节 id 顺序</b> —— body id 按深度优先分配，同一个 body 内的多个关节按书写顺序排。没有 <code>&lt;joint&gt;</code> 的 body 是焊死在父 body 上的（相当于 URDF 的 <code>fixed</code>），不产生任何关节；本工具把这种边在运动学树里标成 <code>[weld]</code>。</p><p><b>注意：</b>关节序号不等于 qpos 下标 —— <code>free</code> 占 7 个 qpos / 6 个 qvel，<code>ball</code> 占 4 个 qpos / 3 个 qvel，其余各占 1 个。<code>&lt;include&gt;</code> 引入的部分本工具读不到。</p>',
        en: '<p>URDF itself defines no joint order — it is just a bag of links and joints. This column is the order the <code>&lt;joint&gt;</code> elements appear in the XML, used as the most intuitive baseline.</p><p><b>Note:</b> the spec does not require joints to be written in tree order; declaring a parent joint after its child is perfectly legal.</p><p><b>MJCF:</b> in MuJoCo the XML nesting <i>is</i> the body tree, so for an MJCF input this column is not merely a baseline — it <b>is the joint id order MuJoCo compiles to</b>: body ids are assigned depth-first, and several joints on one body keep their written order. A body with no <code>&lt;joint&gt;</code> is welded to its parent (the MJCF equivalent of a URDF <code>fixed</code> joint) and creates no joint at all; this tool marks those edges <code>[weld]</code> in the kinematic tree.</p><p><b>Note:</b> a joint index is not a qpos index — <code>free</code> spans 7 qpos / 6 qvel, <code>ball</code> 4 qpos / 3 qvel, everything else 1. Anything pulled in through <code>&lt;include&gt;</code> is invisible to this tool.</p>'
      }
    },
    {
      id: 'mujoco',
      verify: 'print([m.joint(i).name for i in range(m.njnt)])\n\n# C API: mj_id2name(m, mjOBJ_JOINT, i)',
      quat: {
        order: 'wxyz',
        code: 'import mujoco\nm = mujoco.MjModel.from_xml_path("robot.xml")\nd = mujoco.MjData(m)\nmujoco.mj_forward(m, d)\n\nadr = m.jnt_qposadr[m.joint("root").id]   # never hard-code 3:7 — find the free joint\nprint(d.qpos[adr + 3 : adr + 7])          # w, x, y, z\n\nprint(d.body("pelvis").xquat)             # world orientation of any body, w x y z\n# C API: d->qpos + m->jnt_qposadr[jid],  d->xquat + 4*bid',
        note: {
          zh: '<p>MuJoCo <b>全线标量在前</b>（<code>w, x, y, z</code>）：<code>qpos</code> 里 free 关节的姿态、<code>mjData.xquat</code> / <code>xiquat</code>、<code>mju_mulQuat</code> / <code>mju_negQuat</code> 等工具函数、<code>&lt;body quat&gt;</code> 属性，全是同一个顺序。</p><p><b>注意：</b><code>qpos[3:7]</code> 只在「根关节是 free 且排在最前」时才对。通用写法是先取 <code>m.jnt_qposadr[jid]</code>，因为关节 id 顺序（本表的内容）和 qpos 下标不是一回事 —— free 占 7 个 qpos、ball 占 4 个，其余各 1 个。</p><p><b>注意：</b>速度侧不对称：free 关节 7 个 qpos 对应 <b>6 个 qvel</b>，而且 <code>qvel</code> 的角速度是在<b>本体坐标系</b>下的，线速度在世界坐标系下 —— 换算基座姿态时这一条比四元数顺序更容易被漏掉。</p>',
          en: '<p>MuJoCo is <b>scalar-first everywhere</b> (<code>w, x, y, z</code>): the free-joint orientation inside <code>qpos</code>, <code>mjData.xquat</code> / <code>xiquat</code>, helpers such as <code>mju_mulQuat</code> / <code>mju_negQuat</code>, and the <code>&lt;body quat&gt;</code> attribute all use the same order.</p><p><b>Note:</b> <code>qpos[3:7]</code> is only correct when the free joint is the root and comes first. The general form reads <code>m.jnt_qposadr[jid]</code>, because a joint id (what this table lists) is not a qpos index — free spans 7 qpos, ball 4, everything else 1.</p><p><b>Note:</b> the velocity side is not symmetric: those 7 qpos map to <b>6 qvel</b>, and in <code>qvel</code> the angular part is expressed in the <b>body frame</b> while the linear part is in the world frame — a detail that trips people up more often than the component order.</p>'
        }
      },
      body: {
        zh: '<p>MuJoCo 的 URDF 导入器先把所有 link 读进来，再按 <code>&lt;joint&gt;</code> 的文档顺序填充 <code>urChildren[parent].push_back(child)</code>，然后从根 body 递归 <code>AddToTree()</code>。结果就是 <b>body 树的深度优先前序遍历，同层子 body 按 URDF 里关节出现的先后排列</b>；关节 id 随 body 创建顺序递增。</p><p><b>注意：</b>URDF 的 <code>fixed</code> 关节在 MJCF 里不会生成任何 joint（只是 body 嵌套），所以不占 qpos；<code>planar</code> 会被展开成 2 个 slide + 1 个 hinge；<code>&lt;mimic&gt;</code> 被完全忽略。运行时用 <code>mj_id2name(m, mjOBJ_JOINT, i)</code> 核对。</p><p><b>直接载入 MJCF 时</b>这一列没有任何推导成分：XML 的 body 嵌套就是 body 树，DFS 的结果必然等于文件顺序，两列一致只是印证解析没跑偏。真正要小心的是同一台机器人的 URDF 和 MJCF 未必写成同一个顺序 —— 把两个文件分别载入本工具，对比「文件顺序」这一列即可。</p><p>来源：<a href="https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_urdf.cc" target="_blank" rel="noopener">mujoco <code>xml_urdf.cc</code></a>、<a href="https://mujoco.readthedocs.io/en/stable/XMLreference.html" target="_blank" rel="noopener">MJCF XML reference</a></p>',
        en: '<p>MuJoCo\'s URDF importer reads all links first, then fills <code>urChildren[parent].push_back(child)</code> in <code>&lt;joint&gt;</code> document order, and finally recurses with <code>AddToTree()</code> from the root body. The result is a <b>depth-first pre-order walk of the body tree with siblings in URDF joint order</b>; joint ids increase with body creation order.</p><p><b>Note:</b> a URDF <code>fixed</code> joint produces no MJCF joint at all (just nested bodies), so it occupies no qpos; <code>planar</code> expands into 2 slides + 1 hinge; <code>&lt;mimic&gt;</code> is ignored outright. Verify at runtime with <code>mj_id2name(m, mjOBJ_JOINT, i)</code>.</p><p><b>When an MJCF is loaded directly</b> nothing is inferred here: the XML nesting is the body tree, so the depth-first walk necessarily equals the file order — the two columns agreeing only confirms the parse. What does bite is that the URDF and the MJCF of the same robot need not be written in the same order: load both files and compare the file-order column.</p><p>Sources: <a href="https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_urdf.cc" target="_blank" rel="noopener">mujoco <code>xml_urdf.cc</code></a>, <a href="https://mujoco.readthedocs.io/en/stable/XMLreference.html" target="_blank" rel="noopener">MJCF XML reference</a></p>'
      }
    },
    {
      id: 'isaacgym',
      verify: 'print(gym.get_asset_dof_names(asset))',
      quat: {
        order: 'xyzw',
        code: 'root = gymtorch.wrap_tensor(gym.acquire_actor_root_state_tensor(sim))\nroot = root.view(num_envs, num_actors, 13)   # 13 = pos3 + quat4 + linvel3 + angvel3\nbase_quat = root[:, 0, 3:7]                  # x, y, z, w\n\n# non-tensor API, same convention:\ngymapi.Transform().r        # Quat(x=0, y=0, z=0, w=1)\ngym.get_actor_rigid_body_states(env, actor, gymapi.STATE_POS)["pose"]["r"]',
        note: {
          zh: '<p>Isaac Gym Preview 用 <b>标量在后</b>（<code>x, y, z, w</code>），单位四元数是 <code>(0, 0, 0, 1)</code>。root state 张量每行 13 个数：位置 3 + 四元数 4 + 线速度 3 + 角速度 3，所以基座姿态永远是 <code>[3:7]</code>。</p><p><b>注意：</b>从 IsaacGymEnvs 迁到 Isaac Lab 时，<b>关节顺序（DFS→BFS）和四元数顺序（xyzw→wxyz）是两个互相独立的坑</b>，官方迁移文档把两件事写在同一页上：「Isaac Lab and Isaac Sim both adopt <code>wxyz</code> as the quaternion convention. However, the quaternion convention used in Isaac Gym Preview Release was <code>xyzw</code>.」IsaacGymEnvs 里的 <code>quat_rotate_inverse(base_quat, gravity_vec)</code> 这类函数全是按 xyzw 写的，照搬到 Isaac Lab 会得到一个静默错误的重力投影 —— 观测量看着「像」正常值，策略却学不出来。</p><p>来源：<a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a></p>',
          en: '<p>Isaac Gym Preview is <b>scalar-last</b> (<code>x, y, z, w</code>), with identity <code>(0, 0, 0, 1)</code>. Each row of the root-state tensor holds 13 floats — 3 position, 4 quaternion, 3 linear velocity, 3 angular velocity — so the base orientation is always <code>[3:7]</code>.</p><p><b>Note:</b> porting from IsaacGymEnvs to Isaac Lab means <b>two independent traps at once</b> — joint order (DFS→BFS) and quaternion order (xyzw→wxyz). The migration guide states both on one page: "Isaac Lab and Isaac Sim both adopt <code>wxyz</code> as the quaternion convention. However, the quaternion convention used in Isaac Gym Preview Release was <code>xyzw</code>." Helpers like <code>quat_rotate_inverse(base_quat, gravity_vec)</code> in IsaacGymEnvs assume xyzw; copied over unchanged they yield a silently wrong projected gravity — the observation still <i>looks</i> plausible while the policy fails to learn.</p><p>Source: <a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a></p>'
        }
      },
      body: {
        zh: '<p>Isaac Gym Preview Release 对运动学树采用<b>深度优先</b>排列 DOF。因此一个四足机器人会得到 <code>FL_hip, FL_thigh, FL_calf, FR_hip, …</code> 这样“一条腿走到底”的顺序。</p><p><b>注意：</b>这正是从 IsaacGymEnvs 迁移到 Isaac Lab 时最容易踩的坑 —— 同一个 URDF 在两边的 DOF 索引不一样。运行时用 <code>gym.get_asset_dof_names(asset)</code> 核对。</p><p>来源：<a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a></p>',
        en: '<p>Isaac Gym Preview Release orders DOFs <b>depth-first</b> over the kinematic tree. A quadruped therefore comes out as <code>FL_hip, FL_thigh, FL_calf, FR_hip, …</code> — one whole leg at a time.</p><p><b>Note:</b> this is exactly the trap when porting from IsaacGymEnvs to Isaac Lab — the same URDF yields different DOF indices on each side. Verify at runtime with <code>gym.get_asset_dof_names(asset)</code>.</p><p>Source: <a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a></p>'
      }
    },
    {
      id: 'isaacsim',
      verify: '# Isaac Lab\nprint(robot.data.joint_names)\n\n# Isaac Sim (ArticulationView)\nprint(view.joint_names)',
      quat: {
        order: 'mixed',
        chip: { zh: 'w, x, y, z（3.0 起 x, y, z, w）', en: 'w, x, y, z (3.0+: x, y, z, w)' },
        code: '# Isaac Lab 2.x — scalar first\nprint(robot.data.root_quat_w)            # (num_envs, 4)  w, x, y, z\nprint(robot.data.root_state_w[:, 3:7])   # the same four numbers\n\n# Isaac Sim core API — scalar first\npos, quat = view.get_world_poses()       # quat: w, x, y, z\n\n# Isaac Lab 3.0+ — every API returns x, y, z, w instead\n\n# Never guess — print one you know is identity and look at which slot holds the 1:\nprint(robot.data.root_quat_w[0])',
        note: {
          zh: '<p><b>这一列是唯一一个换过约定的。</b>Isaac Sim 的 core API 与 Isaac Lab 2.x 都是<b>标量在前</b>（<code>w, x, y, z</code>）：文档里 <code>root_quat_w</code> 明写「asset root orientation in <code>(w, x, y, z)</code>」，<code>get_world_poses()</code> 也标注 quaternion scalar-first。</p><p><b>但 Isaac Lab 3.0 把默认约定改成了 <code>xyzw</code></b>，原话是「we decided to change our default convention to <code>xyzw</code>. This means that all our APIs will now return quaternions in the <code>xyzw</code> convention.」理由是对齐 PhysX / Warp / Newton，省掉来回转换。这是个<b>破坏性变更且不会报错</b>：代码里硬编码的 <code>(1, 0, 0, 0)</code>、以及自己写的、没走 <code>isaaclab.utils.math</code> 的 MDP 函数，升级后全都是错的。</p><p><b>所以先确认你的版本</b>：打印一个已知是单位姿态的四元数，看 <code>1.0</code> 落在第 0 位还是第 3 位，比查文档快也更可靠。</p><p>来源：<a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a>、<a href="https://isaac-sim.github.io/IsaacLab/main/source/experimental-features/newton-physics-integration/isaaclab_newton-beta-2.html" target="_blank" rel="noopener">Isaac Lab 3.0 Beta — quaternion convention</a></p>',
          en: '<p><b>This is the one column that changed its mind.</b> The Isaac Sim core API and Isaac Lab 2.x are <b>scalar-first</b> (<code>w, x, y, z</code>): the docs describe <code>root_quat_w</code> as "asset root orientation in <code>(w, x, y, z)</code>", and <code>get_world_poses()</code> is likewise documented as scalar-first.</p><p><b>Isaac Lab 3.0 flipped the default to <code>xyzw</code></b> — "we decided to change our default convention to <code>xyzw</code>. This means that all our APIs will now return quaternions in the <code>xyzw</code> convention" — to line up with PhysX, Warp and Newton and drop the conversions in between. It is a <b>breaking change that raises nothing</b>: hard-coded <code>(1, 0, 0, 0)</code> literals and any custom MDP term that does not go through <code>isaaclab.utils.math</code> are simply wrong after the upgrade.</p><p><b>So pin down your version first</b>: print a quaternion you know to be identity and see whether the <code>1.0</code> lands in slot 0 or slot 3 — quicker and more reliable than reading the docs for the version you think you have.</p><p>Sources: <a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a>, <a href="https://isaac-sim.github.io/IsaacLab/main/source/experimental-features/newton-physics-integration/isaaclab_newton-beta-2.html" target="_blank" rel="noopener">Isaac Lab 3.0 Beta — quaternion convention</a></p>'
        }
      },
      body: {
        zh: '<p>官方文档原文：“Physics simulation in Isaac Sim and Isaac Lab assumes a <b>breadth-first</b> ordering for the joints in a given kinematic tree. However, Isaac Gym Preview Release assumed a <b>depth-first</b> ordering.” 也就是 PhysX stage parser 按<b>广度优先</b>（逐层）排列关节。</p><p>同一个四足机器人在这里是 <code>FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, …</code> —— 先排完所有髋关节，再排所有大腿。</p><p><b>注意：</b>运行时用 <code>robot.data.joint_names</code> 核对，用 <code>ArticulationCfg</code> 的 <code>joint_names_expr</code> 或 <code>find_joints()</code> 做重映射，不要硬编码索引。</p><p>来源：<a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a></p>',
        en: '<p>Straight from the docs: “Physics simulation in Isaac Sim and Isaac Lab assumes a <b>breadth-first</b> ordering for the joints in a given kinematic tree. However, Isaac Gym Preview Release assumed a <b>depth-first</b> ordering.” The PhysX stage parser walks the tree level by level.</p><p>The same quadruped comes out as <code>FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, …</code> — all hips first, then all thighs.</p><p><b>Note:</b> verify at runtime with <code>robot.data.joint_names</code>, and remap with <code>joint_names_expr</code> / <code>find_joints()</code> instead of hard-coding indices.</p><p>Source: <a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a></p>'
      }
    },
    {
      id: 'genesis',
      verify: 'import genesis as gs\ngs.init()\nscene = gs.Scene()\nrobot = scene.add_entity(gs.morphs.URDF(file="robot.urdf"))   # or gs.morphs.MJCF(file="robot.xml")\nscene.build()\nprint([j.name for j in robot.joints])\nprint([(j.name, j.dofs_idx_local) for j in robot.joints])',
      quat: {
        order: 'wxyz',
        code: 'robot = scene.add_entity(gs.morphs.URDF(file="robot.urdf", quat=(1, 0, 0, 0)))  # w x y z\nscene.build()\n\nprint(robot.get_quat())        # base-link orientation, (4,) or (n_envs, 4) — w, x, y, z\nprint(robot.get_qpos()[:7])    # the free root_joint: xyz + w x y z\n\nprint(robot.base_link.get_quat())   # per-link, same convention',
        note: {
          zh: '<p>Genesis 跟着 MuJoCo 走，<b>标量在前</b>（<code>w, x, y, z</code>）：<code>get_quat()</code>、<code>set_quat()</code>、<code>gs.morphs.*</code> 的 <code>quat=</code> 参数、IK 的目标姿态，全是这个顺序，单位四元数写作 <code>(1, 0, 0, 0)</code>。</p><p><b>注意：</b>基座姿态在 <code>get_qpos()</code> 的前 7 个数里 —— 是 Genesis 自己补的那个 <code>root_joint</code>（<code>fixed=False</code> 时默认插入，见上）占的，<code>[0:3]</code> 是平移、<code>[3:7]</code> 才是四元数。也就是说<b>文件里的第一个关节从 qpos[7] 开始</b>，不是 qpos[0]。</p><p><b>注意：</b>Genesis 和 Newton 的<b>关节顺序完全一致</b>（都跟 MuJoCo 的 DFS），<b>四元数顺序却正好相反</b>（Genesis wxyz、Newton xyzw）。这两件事各查各的，别用「顺序一样」推断「约定一样」。</p>',
          en: '<p>Genesis follows MuJoCo and is <b>scalar-first</b> (<code>w, x, y, z</code>): <code>get_quat()</code>, <code>set_quat()</code>, the <code>quat=</code> argument on <code>gs.morphs.*</code> and IK target orientations all share it, with identity written <code>(1, 0, 0, 0)</code>.</p><p><b>Note:</b> the base pose occupies the first 7 numbers of <code>get_qpos()</code> — that is the synthetic <code>root_joint</code> Genesis inserts when <code>fixed=False</code> (see above); <code>[0:3]</code> is translation and <code>[3:7]</code> the quaternion. Which means <b>the first joint from your file starts at qpos[7]</b>, not qpos[0].</p><p><b>Note:</b> Genesis and Newton produce the <b>same joint order</b> (both follow MuJoCo\'s DFS) yet the <b>opposite quaternion order</b> — Genesis wxyz, Newton xyzw. Check the two independently; agreeing on order says nothing about agreeing on convention.</p>'
        }
      },
      body: {
        zh: '<p>Genesis 既读 URDF 也读 MJCF，两边的关节顺序都跟着 MuJoCo 走。</p><p><b>URDF：</b><code>parse_urdf()</code> 先用自带的 urdfpy 分支读几何和 <code>&lt;equality&gt;</code>，随后 <code>rigid_entity.py</code> 用 <b>MuJoCo 的统一解析器</b>把 link / joint 结构整个覆盖掉（<code>l_infos = l_infos_mj</code>），所以关节顺序就是 MuJoCo 的 URDF 导入顺序。就算退回它自己的 legacy 路径，<code>order_links_depth_first()</code> 也是深度优先前序、同层保持原有相对顺序，源码注释里明写「the result matches MuJoCo’s body ordering」。<b>MJCF</b> 则交给 mujoco 编译，再按 <code>body_parentid</code> 读回来，同样是 body 树 DFS。</p><p><b>注意：</b><code>gs.morphs.URDF</code> 默认 <code>merge_fixed_links=True</code> —— fixed 关节连同子 link 一起被合并进父 link，比其它导入器「不生成关节」更进一步（link 本身也没了）。要留住某个 link 用 <code>links_to_keep=[...]</code>，或整体关掉 <code>merge_fixed_links=False</code>；此时被保留的 link 会得到一个 0 自由度的 <code>FIXED</code> 关节，占一个关节下标。</p><p><b>注意：</b><code>fixed</code> 默认是 <code>False</code>，Genesis 会自动在根部插入一个名叫 <code>root_joint</code> 的 <b>free 关节</b>（7 qpos / 6 DOF），哪怕 URDF 里根本没写浮动关节。所以 <code>entity.joints[0]</code> 常常是个文件里不存在的关节，后面每个关节的 DOF 下标整体后移 6 位。</p><p><b>注意：</b><code>entity.joints</code> 里混着 0 自由度的 <code>FIXED</code> 关节（MJCF 中没有 <code>&lt;joint&gt;</code> 的 body 会得到一个<b>以 body 名命名</b>的 fixed 关节），所以关节序号不等于 DOF 序号 —— 要下标就读 <code>joint.dofs_idx_local</code>，别拿关节序号去索引 <code>qpos</code>。</p><p>来源：<a href="https://github.com/Genesis-Embodied-AI/Genesis/blob/main/genesis/utils/urdf.py" target="_blank" rel="noopener">Genesis <code>utils/urdf.py</code></a>、<a href="https://github.com/Genesis-Embodied-AI/Genesis/blob/main/genesis/engine/entities/rigid_entity/rigid_entity.py" target="_blank" rel="noopener">Genesis <code>rigid_entity.py</code></a></p>',
        en: '<p>Genesis reads both URDF and MJCF, and both follow MuJoCo.</p><p><b>URDF:</b> <code>parse_urdf()</code> reads geometry and <code>&lt;equality&gt;</code> with its own urdfpy fork, then <code>rigid_entity.py</code> overwrites the whole link / joint structure with <b>MuJoCo’s unified parser</b> (<code>l_infos = l_infos_mj</code>) — so the joint order <i>is</i> MuJoCo’s URDF import order. Even on the legacy fallback path, <code>order_links_depth_first()</code> is a depth-first pre-order that keeps siblings in their original relative order, and its comment states outright that "the result matches MuJoCo’s body ordering". <b>MJCF</b> is compiled by mujoco and read back through <code>body_parentid</code> — the same body-tree DFS.</p><p><b>Note:</b> <code>gs.morphs.URDF</code> defaults to <code>merge_fixed_links=True</code>, which merges a fixed joint’s child link into its parent — one step beyond the other importers, which merely create no joint. Keep a link with <code>links_to_keep=[...]</code>, or switch the whole thing off with <code>merge_fixed_links=False</code>; a link kept that way carries a 0-DOF <code>FIXED</code> joint that does occupy a joint index.</p><p><b>Note:</b> <code>fixed</code> defaults to <code>False</code>, so Genesis inserts a <b>free joint named <code>root_joint</code></b> (7 qpos / 6 DOF) at the root even when the URDF declares no floating joint. <code>entity.joints[0]</code> is therefore usually a joint that is not in your file, and every DOF index behind it is shifted by 6.</p><p><b>Note:</b> <code>entity.joints</code> also carries 0-DOF <code>FIXED</code> joints (a jointless MJCF body yields one <b>named after the body</b>), so a joint index is not a DOF index — read <code>joint.dofs_idx_local</code> instead of indexing <code>qpos</code> by joint number.</p><p>Sources: <a href="https://github.com/Genesis-Embodied-AI/Genesis/blob/main/genesis/utils/urdf.py" target="_blank" rel="noopener">Genesis <code>utils/urdf.py</code></a>, <a href="https://github.com/Genesis-Embodied-AI/Genesis/blob/main/genesis/engine/entities/rigid_entity/rigid_entity.py" target="_blank" rel="noopener">Genesis <code>rigid_entity.py</code></a></p>'
      }
    },
    {
      id: 'newton',
      verify: 'import newton\nbuilder = newton.ModelBuilder()\nbuilder.add_urdf("robot.urdf")        # or builder.add_mjcf("robot.xml")\nmodel = builder.finalize()\nprint(model.joint_label)             # joint_label[0] is Newton\'s own base joint',
      quat: {
        order: 'xyzw',
        code: 'state = model.state()\nnewton.eval_fk(model, model.joint_q, model.joint_qd, state)\n\nxform = state.body_q.numpy()[0]      # wp.transform: [px py pz  qx qy qz qw]\nprint(xform[3:7])                    # x, y, z, w\n\nprint(model.joint_q.numpy()[:7])     # floating-base joint coords, same layout\n\n# wp.quat is (x, y, z, w) — wp.quat_identity() == (0, 0, 0, 1)',
        note: {
          zh: '<p>Newton 建在 NVIDIA Warp 上，四元数就是 Warp 的 <code>wp.quat</code> —— <b>标量在后</b>（<code>x, y, z, w</code>），单位四元数是 <code>(0, 0, 0, 1)</code>。<code>state.body_q</code> 是一个 <code>wp.transform</code> 数组，每项 7 个数：<code>[px, py, pz, qx, qy, qz, qw]</code>。</p><p>官方 conventions 页直接给了对照表和换算式：Newton / Warp 是 <code>(x, y, z, w)</code>，Isaac Lab（2.x）与 MuJoCo 是 <code>(w, x, y, z)</code>，从 Isaac 转过来要写 <code>newton_quat = (isaac_quat[1], isaac_quat[2], isaac_quat[3], isaac_quat[0])</code>。</p><p><b>注意：</b>本工具里 MuJoCo / Genesis / Newton 三列的<b>关节顺序是同一个</b>（都是 body 树 DFS），但四元数上 Newton 和另外两个<b>反着来</b>。用 Newton 复现一个 MuJoCo 环境时，关节向量可以直接搬，基座姿态必须换序 —— 而且换错了不会报错，只会让机器人以一个镜像的姿态起步。</p><p>来源：<a href="https://newton-physics.github.io/newton/stable/concepts/conventions.html" target="_blank" rel="noopener">Newton — Conventions</a></p>',
          en: '<p>Newton is built on NVIDIA Warp, so its quaternion is Warp\'s <code>wp.quat</code>: <b>scalar-last</b> (<code>x, y, z, w</code>), identity <code>(0, 0, 0, 1)</code>. <code>state.body_q</code> is an array of <code>wp.transform</code>, 7 numbers each — <code>[px, py, pz, qx, qy, qz, qw]</code>.</p><p>The official conventions page spells out both the table and the conversion: Newton / Warp use <code>(x, y, z, w)</code> while Isaac Lab (2.x) and MuJoCo use <code>(w, x, y, z)</code>, so coming from Isaac you need <code>newton_quat = (isaac_quat[1], isaac_quat[2], isaac_quat[3], isaac_quat[0])</code>.</p><p><b>Note:</b> the MuJoCo, Genesis and Newton columns of this tool share <b>one joint order</b> (body-tree DFS), yet Newton\'s quaternion runs <b>the other way</b> from the other two. Reproducing a MuJoCo setup in Newton, the joint vector carries over untouched while the base orientation must be reordered — and getting it wrong raises nothing, it just starts the robot in a mirrored pose.</p><p>Source: <a href="https://newton-physics.github.io/newton/stable/concepts/conventions.html" target="_blank" rel="noopener">Newton — Conventions</a></p>'
        }
      },
      body: {
        zh: '<p>Newton（NVIDIA / Google DeepMind / Disney Research，构建在 NVIDIA Warp 之上）用 <code>ModelBuilder.add_urdf()</code> / <code>add_mjcf()</code> 导入模型。</p><p><b>URDF：</b><code>parse_urdf()</code> 的 <code>joint_ordering</code> 默认是 <code>"dfs"</code>，它把所有 (parent, child) 边交给 <code>topological_sort(use_dfs=True)</code>；递归子节点时取的是 <code>sorted(outgoing[node], key=joint_id)</code>，也就是<b>同层按 <code>&lt;joint&gt;</code> 的文档顺序</b> —— 和 MuJoCo / Isaac Gym / PyBullet 的 DFS 一致，不是 Gazebo 的字母序。改成 <code>joint_ordering="bfs"</code> 会走 Kahn 广度优先（≈ Isaac Sim 那一列），传 <code>None</code> 则原样保留文件顺序。<b>MJCF</b> 由 <code>parse_body()</code> 递归 body 树，同样是 DFS。</p><p><b>和 MuJoCo / Genesis 的关键差别：</b><code>collapse_fixed_joints</code> 默认是 <code>False</code>，URDF 的 fixed 关节会保留成 0 自由度的 <code>JointType.FIXED</code>，<b>照样占一个关节下标</b>；MJCF 里没有 <code>&lt;joint&gt;</code> 的 body 也会拿到一个 fixed 关节（标签形如 <code>&lt;body&gt;/&lt;body&gt;_joint</code>，本工具这一列用 body 名显示）。所以勾选「显示 fixed 关节」才对得上运行时的 <code>model.joint_label</code>。</p><p><b>注意：</b>Newton 还会在这些关节<b>前面</b>插一个文件里没有的基座关节 —— URDF 默认 <code>floating=None</code> 给 <code>fixed_base</code>（0 DOF），<code>floating=True</code> 给 <code>floating_base</code>（7 qpos / 6 DOF）。所以 <code>model.joint_label[0]</code> 是这个基座关节，文件里的关节整体后移一位。</p><p><b>注意：</b>同一个 MJCF body 上挂多个 <code>&lt;joint&gt;</code> 时，Newton 会把它们合并成<b>一个</b> D6 关节（自由度仍按书写顺序排开，但关节个数对不上 MuJoCo 的 <code>njnt</code>）。模型有多个根时，遍历起点按根 link 名<b>字母序</b>（<code>roots = sorted(roots)</code>），而本工具按文档顺序。</p><p>来源：<a href="https://github.com/newton-physics/newton/blob/main/newton/_src/utils/import_urdf.py" target="_blank" rel="noopener">newton <code>import_urdf.py</code></a>、<a href="https://github.com/newton-physics/newton/blob/main/newton/_src/utils/topology.py" target="_blank" rel="noopener">newton <code>topology.py</code></a></p>',
        en: '<p>Newton (NVIDIA / Google DeepMind / Disney Research, built on NVIDIA Warp) imports models through <code>ModelBuilder.add_urdf()</code> / <code>add_mjcf()</code>.</p><p><b>URDF:</b> <code>parse_urdf()</code> defaults to <code>joint_ordering="dfs"</code> and hands every (parent, child) edge to <code>topological_sort(use_dfs=True)</code>, which recurses over <code>sorted(outgoing[node], key=joint_id)</code> — i.e. <b>siblings in <code>&lt;joint&gt;</code> document order</b>, matching the MuJoCo / Isaac Gym / PyBullet depth-first walk rather than Gazebo’s alphabetical one. Pass <code>joint_ordering="bfs"</code> for Kahn’s breadth-first order (≈ the Isaac Sim column), or <code>None</code> to keep the file order verbatim. <b>MJCF</b> is walked recursively by <code>parse_body()</code> — the same DFS.</p><p><b>The difference versus MuJoCo / Genesis:</b> <code>collapse_fixed_joints</code> defaults to <code>False</code>, so a URDF fixed joint survives as a 0-DOF <code>JointType.FIXED</code> and <b>still occupies a joint index</b>; a jointless MJCF body likewise gets a fixed joint of its own (labelled <code>&lt;body&gt;/&lt;body&gt;_joint</code>, shown here under the body name). Tick “show fixed joints” to get the list that matches <code>model.joint_label</code> at runtime.</p><p><b>Note:</b> Newton also prepends a base joint that exists nowhere in your file — <code>fixed_base</code> (0 DOF) for the URDF default <code>floating=None</code>, or <code>floating_base</code> (7 qpos / 6 DOF) for <code>floating=True</code>. <code>model.joint_label[0]</code> is that base joint, and every joint from the file shifts one slot later.</p><p><b>Note:</b> several <code>&lt;joint&gt;</code> elements on one MJCF body are merged into <b>a single</b> D6 joint (the DOFs stay in written order, but the joint count no longer matches MuJoCo’s <code>njnt</code>). With multiple roots the traversal starts in <b>alphabetical</b> root-link order (<code>roots = sorted(roots)</code>), where this tool uses document order.</p><p>Sources: <a href="https://github.com/newton-physics/newton/blob/main/newton/_src/utils/import_urdf.py" target="_blank" rel="noopener">newton <code>import_urdf.py</code></a>, <a href="https://github.com/newton-physics/newton/blob/main/newton/_src/utils/topology.py" target="_blank" rel="noopener">newton <code>topology.py</code></a></p>'
      }
    },
    {
      id: 'gazebo',
      verify: '# print the SDF Gazebo actually loads, then read the <joint> order\ngz sdf -p robot.urdf        # ign sdf -p on Ignition',
      quat: {
        order: 'xyzw',
        code: '# runtime pose of the model (gz.msgs.Pose), fields in x y z w order\ngz topic -e -t /model/<model_name>/pose -n 1\n#   orientation { x: 0  y: 0  z: 0  w: 1 }\n\n# in the SDF file <pose> is NOT a quaternion by default:\n#   <pose>0 0 0.8  0 0 0</pose>                        xyz + roll pitch yaw (radians)\n#   <pose rotation_format="quat_xyzw">0 0 0.8  0 0 0 1</pose>    SDFormat >= 1.9\n\n# C++ is the other way round:\n#   gz::math::Quaterniond(w, x, y, z)\n#   gz::math::Pose3d(x, y, z, qw, qx, qy, qz)',
        note: {
          zh: '<p><b>Gazebo 里两种顺序同时存在，这是本表最容易出错的一列。</b></p><ul><li><b>消息层是 <code>x, y, z, w</code></b>：<code>gz.msgs.Quaternion</code> 的字段顺序就是 x/y/z/w，ROS 侧的 <code>geometry_msgs/Quaternion</code> 也一样，<code>gz topic -e</code> 打印出来就是这个顺序。</li><li><b>C++ 数学库是 <code>w</code> 在前</b>：<code>gz::math::Quaterniond(w, x, y, z)</code>、<code>Pose3d(x, y, z, qw, qx, qy, qz)</code> 的构造函数都是标量在前。</li></ul><p>也就是说把一个消息里的四元数塞进 <code>Quaterniond</code> 构造函数（或反过来）而不换序，是 Gazebo 插件里的经典 bug —— 编译通过、运行不报错、物体姿态离谱。</p><p><b>注意：</b>SDF / URDF <b>文件</b>里根本没有四元数：<code>&lt;pose&gt;</code> 默认是 <code>x y z roll pitch yaw</code>（弧度）。SDFormat 1.9 起可以显式写 <code>&lt;pose rotation_format="quat_xyzw"&gt;</code>，属性名本身就把顺序标出来了；还有 <code>&lt;pose degrees="true"&gt;</code> 改角度单位。</p>',
          en: '<p><b>Gazebo carries both orders at once — the most error-prone column here.</b></p><ul><li><b>The message layer is <code>x, y, z, w</code></b>: <code>gz.msgs.Quaternion</code> declares its fields x/y/z/w, ROS\'s <code>geometry_msgs/Quaternion</code> matches, and that is what <code>gz topic -e</code> prints.</li><li><b>The C++ math library puts <code>w</code> first</b>: <code>gz::math::Quaterniond(w, x, y, z)</code> and <code>Pose3d(x, y, z, qw, qx, qy, qz)</code> are both scalar-first constructors.</li></ul><p>Feeding a quaternion straight from a message into that constructor (or the reverse) without reordering is the classic Gazebo-plugin bug: it compiles, it runs, and the object sits at a nonsense orientation.</p><p><b>Note:</b> the SDF / URDF <b>file</b> holds no quaternion at all — <code>&lt;pose&gt;</code> defaults to <code>x y z roll pitch yaw</code> in radians. SDFormat 1.9 added the explicit <code>&lt;pose rotation_format="quat_xyzw"&gt;</code>, whose attribute name states the order outright, alongside <code>&lt;pose degrees="true"&gt;</code> for the unit.</p>'
        }
      },
      body: {
        zh: '<p>只适用于 URDF：sdformat 不认 MJCF，载入 MJCF 时这一列显示为不适用。</p><p>Gazebo 不直接吃 URDF —— 它先用 sdformat 把 URDF 转成 SDF，模型里的关节顺序就是这次转换的产物。<code>parser_urdf.cc</code> 里的 <code>CreateSDF()</code> 递归遍历 <code>_link-&gt;child_links</code>，是<b>深度优先</b>。</p><p>关键在于 <code>child_links</code> 是谁填的：urdfdom 的 <code>initTree()</code> 遍历 <code>std::map</code> 类型的 <code>joints_</code> 来填充它，而 <code>std::map</code> 按 key 排序。所以 <b>Gazebo 的同层子关节是按关节名字母序排的，跟你在 URDF 里的书写顺序无关</b> —— 这一点和 MuJoCo / Isaac Gym 的 DFS（同层按文档顺序）不同，也是这一列存在的意义。</p><p><b>注意：</b>sdformat <b>默认会把 fixed 关节吸收掉</b>（把子 link 合并进父 link），除非加 <code>&lt;disableFixedJointLumping&gt;</code> 或 <code>&lt;preserveFixedJoint&gt;</code>（两者同时存在时后者优先）。另外 <code>gazebo_ros_joint_state_publisher</code> 插件发布的顺序取决于你在插件里列 <code>&lt;joint_name&gt;</code> 的顺序，<code>gz_ros2_control</code> 则走 <code>&lt;ros2_control&gt;</code> 标签，都与这一列无关。</p><p>来源：<a href="https://github.com/gazebosim/sdformat/blob/sdf14/src/parser_urdf.cc" target="_blank" rel="noopener">sdformat <code>parser_urdf.cc</code></a>、<a href="https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/model.h" target="_blank" rel="noopener">urdfdom_headers <code>model.h</code></a></p>',
        en: '<p>URDF only: sdformat does not read MJCF, so this column is n/a for an MJCF input.</p><p>Gazebo does not consume URDF directly — sdformat converts it to SDF first, and the model\'s joint order is a product of that conversion. <code>CreateSDF()</code> in <code>parser_urdf.cc</code> recurses over <code>_link-&gt;child_links</code>, so the walk is <b>depth-first</b>.</p><p>What matters is who fills <code>child_links</code>: urdfdom\'s <code>initTree()</code> populates it by iterating <code>joints_</code>, a <code>std::map</code>, which is sorted by key. So <b>Gazebo orders sibling joints alphabetically by joint name, regardless of how the URDF is written</b> — unlike MuJoCo / Isaac Gym depth-first, whose siblings follow document order. That difference is why this column exists.</p><p><b>Note:</b> sdformat <b>lumps fixed joints away by default</b> (merging the child link into its parent) unless <code>&lt;disableFixedJointLumping&gt;</code> or <code>&lt;preserveFixedJoint&gt;</code> is set (the latter wins when both appear). Separately, the <code>gazebo_ros_joint_state_publisher</code> plugin publishes in the order you list <code>&lt;joint_name&gt;</code> in the plugin, and <code>gz_ros2_control</code> follows the <code>&lt;ros2_control&gt;</code> tag — neither matches this column.</p><p>Sources: <a href="https://github.com/gazebosim/sdformat/blob/sdf14/src/parser_urdf.cc" target="_blank" rel="noopener">sdformat <code>parser_urdf.cc</code></a>, <a href="https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/model.h" target="_blank" rel="noopener">urdfdom_headers <code>model.h</code></a></p>'
      }
    },
    {
      id: 'pybullet',
      verify: 'import pybullet as p\np.connect(p.DIRECT)\nbid = p.loadURDF("robot.urdf")\nprint([p.getJointInfo(bid, i)[1].decode() for i in range(p.getNumJoints(bid))])',
      quat: {
        order: 'xyzw',
        code: 'pos, orn = p.getBasePositionAndOrientation(bid)\nprint(orn)                                   # x, y, z, w\n\np.resetBasePositionAndOrientation(bid, pos, [0, 0, 0, 1])   # identity = (0, 0, 0, 1)\nprint(p.getQuaternionFromEuler([0, 0, 0]))                  # also x, y, z, w\n\nprint(p.getLinkState(bid, i)[1])             # link frame orientation, x y z w',
        note: {
          zh: '<p>Bullet <b>全线标量在后</b>（<code>x, y, z, w</code>），单位四元数是 <code>[0, 0, 0, 1]</code>：<code>getBasePositionAndOrientation</code>、<code>getLinkState</code>、<code>getQuaternionFromEuler</code>、<code>multiplyTransforms</code> 都一致，没有例外。</p><p><b>注意：</b>基座姿态<b>不在</b>关节列表里 —— 基座 link 的 index 是 <code>-1</code>，<code>getJointInfo</code> 从 0 开始只数关节。所以左边那张顺序表和这里的基座四元数是两个独立的东西，重映射关节向量时别把基座的 7 个数一起算进去。</p><p><b>注意：</b><code>getEulerFromQuaternion</code> 返回的是 URDF 那套 rpy（弧度），可以直接写回 <code>&lt;origin rpy&gt;</code>；但 <code>getQuaternionFromEuler</code> 的输入也是同一套 rpy，两者互逆，不要和 MJCF 默认的「度」混起来。</p>',
          en: '<p>Bullet is <b>scalar-last throughout</b> (<code>x, y, z, w</code>) with identity <code>[0, 0, 0, 1]</code>: <code>getBasePositionAndOrientation</code>, <code>getLinkState</code>, <code>getQuaternionFromEuler</code> and <code>multiplyTransforms</code> all agree, with no exceptions.</p><p><b>Note:</b> the base orientation is <b>not</b> part of the joint list — the base link is index <code>-1</code>, and <code>getJointInfo</code> counts joints from 0. The order table on the left and this base quaternion are therefore separate things; don\'t fold the base\'s 7 numbers into a joint-vector remap.</p><p><b>Note:</b> <code>getEulerFromQuaternion</code> returns URDF-style rpy in radians, so it can be written straight back into <code>&lt;origin rpy&gt;</code>, and <code>getQuaternionFromEuler</code> is its exact inverse — just don\'t mix either with MJCF\'s default of degrees.</p>'
        }
      },
      body: {
        zh: '<p>PyBullet 的 <code>loadURDF</code> 默认（<code>flags=0</code>）从根 link 递归调用 <code>ConvertURDF2BulletInternal()</code>，按 <code>getLinkChildIndices()</code> 处理每个子 link，是<b>深度优先</b>。</p><p>子 link 数组来自 <code>UrdfParser::initTreeAndRoot()</code>：它按 <code>m_joints</code> 的插入顺序（即 <code>&lt;joint&gt;</code> 的文档顺序）<code>push_back</code> 到父 link 的 <code>m_childLinks</code>。所以 <b>同层子关节按 URDF 书写顺序</b>，和 Isaac Gym / MuJoCo 的 DFS 一样，和 Gazebo 的字母序不同。</p><p><b>和 Isaac / MuJoCo / Gazebo 的关键差别：</b>每个 child link 对应一个关节下标，<code>getNumJoints</code> / <code>getJointInfo</code> <b>默认包含 fixed 关节</b>（基座 link 的 index 是 -1，不是关节）。fixed 占一个关节下标，但不占自由度。勾选「显示 fixed 关节」才能看到与运行时下标一致的完整列表；默认对照表只列出可动关节。</p><p><b>注意：</b><code>URDF_MAINTAIN_LINK_ORDER</code> 会改成按 <code>&lt;link&gt;</code> 声明顺序编号（要求父 link 写在子 link 前面）；<code>URDF_MERGE_FIXED_LINKS</code> 会把 fixed 关节合并掉。本列按默认 flags 计算。<code>useFixedBase=False</code>（默认）时根 link 是自由浮动的基座，不出现在关节列表里。</p><p>PyBullet 也能 <code>loadMJCF</code>，走同一套 DFS 转换；Bullet 的 MJCF 导入器并不完整，焊死的 body 有时会多出 unnamed fixed 关节，请以运行时打印为准。</p><p>来源：<a href="https://github.com/bulletphysics/bullet3/blob/master/examples/Importers/ImportURDFDemo/URDF2Bullet.cpp" target="_blank" rel="noopener">bullet3 <code>URDF2Bullet.cpp</code></a>、<a href="https://github.com/bulletphysics/bullet3/blob/master/examples/Importers/ImportURDFDemo/UrdfParser.cpp" target="_blank" rel="noopener">bullet3 <code>UrdfParser.cpp</code></a></p>',
        en: '<p>PyBullet\'s <code>loadURDF</code> (default <code>flags=0</code>) starts at the root link and recurses with <code>ConvertURDF2BulletInternal()</code>, walking <code>getLinkChildIndices()</code> — a <b>depth-first</b> walk.</p><p>The child-link array comes from <code>UrdfParser::initTreeAndRoot()</code>, which <code>push_back</code>s onto the parent\'s <code>m_childLinks</code> in <code>m_joints</code> insertion order, i.e. <code>&lt;joint&gt;</code> document order. So <b>siblings follow URDF writing order</b>, matching Isaac Gym / MuJoCo depth-first and unlike Gazebo\'s alphabetical siblings.</p><p><b>The difference versus Isaac / MuJoCo / Gazebo:</b> every child link occupies a joint index, so <code>getNumJoints</code> / <code>getJointInfo</code> <b>include fixed joints by default</b> (the base link is index -1, not a joint). A fixed joint takes a slot but no DOF. Tick “show fixed joints” to see the full list that matches runtime indices; the default table lists movable joints only.</p><p><b>Note:</b> <code>URDF_MAINTAIN_LINK_ORDER</code> numbers links in <code>&lt;link&gt;</code> declaration order (parents must appear before children); <code>URDF_MERGE_FIXED_LINKS</code> folds fixed joints away. This column uses the default flags. With <code>useFixedBase=False</code> (the default) the root link is a free-floating base and never appears in the joint list.</p><p>PyBullet can also <code>loadMJCF</code>, using the same DFS conversion. Bullet\'s MJCF importer is incomplete and may emit extra unnamed fixed joints for welded bodies — trust the names your runtime prints.</p><p>Sources: <a href="https://github.com/bulletphysics/bullet3/blob/master/examples/Importers/ImportURDFDemo/URDF2Bullet.cpp" target="_blank" rel="noopener">bullet3 <code>URDF2Bullet.cpp</code></a>, <a href="https://github.com/bulletphysics/bullet3/blob/master/examples/Importers/ImportURDFDemo/UrdfParser.cpp" target="_blank" rel="noopener">bullet3 <code>UrdfParser.cpp</code></a></p>'
      }
    },
    {
      id: 'ros2control',
      verify: 'ros2 control list_hardware_interfaces\nros2 topic echo /joint_states --field name --once',
      quat: {
        order: 'xyzw',
        code: '# /joint_states carries no base pose at all — the root pose lives in TF\nros2 run tf2_ros tf2_echo odom base_link\n#   - Rotation: in Quaternion [x, y, z, w]\n\nros2 topic echo /odom --field pose.pose.orientation --once   # x y z w\n\n# Python: geometry_msgs/Quaternion has fields .x .y .z .w — index-free, so\n# read them by name and the order stops mattering:\nq = msg.pose.pose.orientation\nnp.array([q.x, q.y, q.z, q.w])',
        note: {
          zh: '<p>ROS <b>全线标量在后</b>（<code>x, y, z, w</code>）：<code>geometry_msgs/Quaternion</code> 的字段顺序、<code>tf2::Quaternion(x, y, z, w)</code>、<code>tf2_echo</code> 的输出都一致。</p><p><b>但这一列本身没有根节点四元数</b> —— <code>joint_state_broadcaster</code> 只发关节的 position / velocity / effort，浮动基座的姿态不在 <code>/joint_states</code> 里。基座姿态由 <code>odom → base_link</code> 的 TF 提供，来源通常是 <code>robot_localization</code> 之类的状态估计、腿式机器人的里程计，或仿真器的 ground truth 插件，和 URDF 的关节顺序完全无关。</p><p><b>建议：</b>ROS 消息的四元数是<b>具名字段</b>而不是数组，这是它相对其它框架的一个实际优势 —— 按 <code>q.x / q.y / q.z / q.w</code> 取值，顺序问题在这一层根本不会发生。真正的风险在你把它转成 numpy 数组、送进一个按别的约定写的函数的那一行。</p>',
          en: '<p>ROS is <b>scalar-last throughout</b> (<code>x, y, z, w</code>): the field order of <code>geometry_msgs/Quaternion</code>, <code>tf2::Quaternion(x, y, z, w)</code> and the output of <code>tf2_echo</code> all agree.</p><p><b>But this column has no root quaternion of its own</b> — <code>joint_state_broadcaster</code> publishes only joint position / velocity / effort, and a floating base\'s orientation is not in <code>/joint_states</code>. It comes from the <code>odom → base_link</code> TF instead, produced by a state estimator such as <code>robot_localization</code>, by legged odometry, or by a simulator ground-truth plugin — unrelated to the URDF joint order.</p><p><b>Tip:</b> a ROS quaternion is <b>named fields</b>, not an array, which is a genuine advantage over every other framework here: read <code>q.x / q.y / q.z / q.w</code> and the ordering question cannot arise at this layer. The risk lives on the line where you flatten it into a numpy array and hand it to a function written for a different convention.</p>'
        }
      },
      body: {
        zh: '<p>只适用于 URDF：MJCF 里没有 <code>&lt;ros2_control&gt;</code> 标签，载入 MJCF 时这一列显示为不适用。</p><p><code>joint_state_broadcaster</code> 发布的顺序取决于配置，文档给出三种情况：</p><ul><li>未设 <code>joints</code> 参数、<code>use_urdf_to_filter=true</code>（默认）→ <b>与 URDF 文件里的关节顺序相同，和 &lt;ros2_control&gt; 标签内的顺序无关</b>。</li><li>未设 <code>joints</code> 参数、<code>use_urdf_to_filter=false</code> → 按 resource manager 里注册 state interface 的顺序，也就是 <b>&lt;ros2_control&gt; 标签内 &lt;joint&gt; 的顺序</b>（硬件组件加载顺序）。</li><li>显式设置了 <code>joints</code> + <code>interfaces</code> 参数 → <b>按 joints 参数的顺序</b>（本工具无法读取 YAML，这种情况请以你的配置为准）。</li></ul><p><b>注意：</b>控制器（如 <code>joint_trajectory_controller</code>）用的是各自 YAML 里 <code>joints</code> 参数的顺序，与这一列无关。<code>extra_joints</code> 会追加到末尾。</p><p>来源：<a href="https://control.ros.org/rolling/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html" target="_blank" rel="noopener">joint_state_broadcaster 文档</a></p>',
        en: '<p>URDF only: MJCF has no <code>&lt;ros2_control&gt;</code> tag, so this column reads n/a for an MJCF input.</p><p>The order <code>joint_state_broadcaster</code> publishes depends on configuration; the docs give three cases:</p><ul><li>no <code>joints</code> param, <code>use_urdf_to_filter=true</code> (default) → <b>same as the joint order in the URDF file, independent of the order in the &lt;ros2_control&gt; tag</b>.</li><li>no <code>joints</code> param, <code>use_urdf_to_filter=false</code> → the order state interfaces were registered in the resource manager, i.e. <b>the order of &lt;joint&gt; inside the &lt;ros2_control&gt; tag</b> (hardware component load order).</li><li><code>joints</code> + <code>interfaces</code> params set explicitly → <b>the order of the joints param</b> (this tool cannot read your YAML — trust your config in that case).</li></ul><p><b>Note:</b> controllers such as <code>joint_trajectory_controller</code> use their own YAML <code>joints</code> parameter order, unrelated to this column. <code>extra_joints</code> are appended at the end.</p><p>Source: <a href="https://control.ros.org/rolling/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html" target="_blank" rel="noopener">joint_state_broadcaster docs</a></p>'
      }
    },
    {
      id: 'mjcfctrl',
      verify: 'print([m.actuator(i).name for i in range(m.nu)])\n\n# joint each actuator drives (joint transmissions only):\nprint([m.joint(m.actuator_trnid[i, 0]).name for i in range(m.nu)])',
      quat: {
        order: 'wxyz',
        code: '# data.ctrl holds no pose — the base quaternion is still MuJoCo\'s, w x y z\nadr = m.jnt_qposadr[m.joint("root").id]\nprint(d.qpos[adr + 3 : adr + 7])     # w, x, y, z\n\nprint(m.nu, m.nq, m.njnt)            # actuators / qpos width / joints — three lengths',
        note: {
          zh: '<p><code>data.ctrl</code> 里<b>没有基座姿态</b>：执行器只驱动关节 / tendon / site，浮动基座是不被驱动的。根节点四元数还是走 MuJoCo 那一列的 <code>qpos</code>，<b>标量在前</b>（<code>w, x, y, z</code>）。</p><p><b>注意：</b>一个 MuJoCo 模型里至少有三个长度不同的向量 —— <code>nu</code>（执行器 / <code>ctrl</code>）、<code>nq</code>（<code>qpos</code>）、<code>njnt</code>（关节数），再加上 <code>nv</code>（<code>qvel</code>）。本表比较的是关节顺序，<code>ctrl</code> 这一列是另一个索引空间，而基座的 7 个 qpos 又只存在于第三个。任何「用一个下标同时索引三者」的代码都是错的。</p>',
          en: '<p><code>data.ctrl</code> holds <b>no base pose</b>: actuators drive joints, tendons or sites, and a floating base is not actuated. The root quaternion still comes from <code>qpos</code> as in the MuJoCo column — <b>scalar first</b> (<code>w, x, y, z</code>).</p><p><b>Note:</b> a MuJoCo model carries at least three vectors of different length — <code>nu</code> (actuators / <code>ctrl</code>), <code>nq</code> (<code>qpos</code>) and <code>njnt</code> (joints), plus <code>nv</code> for <code>qvel</code>. This table compares joint order, the ctrl column is a separate index space, and the base\'s 7 qpos live only in the third. Any code that indexes all three with one integer is wrong.</p>'
        }
      },
      body: {
        zh: '<p>只适用于 MJCF。<code>data.ctrl</code> 的下标是<b>执行器</b>下标，不是关节下标 —— 它按 <code>&lt;actuator&gt;</code> 段里元素出现的先后排列，和 <code>qpos</code> / 关节 id 是两个独立的向量。两者平时看起来一样，只要有人在 actuator 段里调换了两行，<code>ctrl[i]</code> 就不再对应你以为的那个关节，而且没有任何报错。</p><p>内置的 <b>Unitree G1 MJCF 示例正是这种情况</b>：body 树里右手是 <code>middle_0, middle_1, index_0, index_1</code>，而 <code>&lt;actuator&gt;</code> 里写的是 <code>index_0, index_1, middle_0, middle_1</code> —— 最后四个电机与关节顺序对不上。</p><p><b>注意：</b>驱动 tendon / site / body（吸附）的执行器不对应任何关节，但一样占 <code>ctrl</code> 槽位；本列只列出驱动关节的执行器，出现这类执行器时列里的序号会小于真实 <code>ctrl</code> 下标。同一个关节也可以挂多个执行器（位置 + 速度），那样它会在这一列里出现多次。</p><p>来源：<a href="https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator" target="_blank" rel="noopener">MJCF XML reference — actuator</a></p>',
        en: '<p>MJCF only. <code>data.ctrl</code> is indexed by <b>actuator</b>, not by joint: it follows the document order of the <code>&lt;actuator&gt;</code> section, an entirely separate vector from <code>qpos</code> and the joint ids. The two usually look identical — until someone swaps two lines in the actuator block, after which <code>ctrl[i]</code> silently drives a different joint than you think.</p><p>The bundled <b>Unitree G1 MJCF sample is exactly this case</b>: the body tree has the right hand as <code>middle_0, middle_1, index_0, index_1</code>, while <code>&lt;actuator&gt;</code> lists <code>index_0, index_1, middle_0, middle_1</code> — the last four motors do not line up with the joint order.</p><p><b>Note:</b> actuators driving a tendon / site / body (adhesion) map to no joint at all yet still occupy a <code>ctrl</code> slot; this column lists only joint actuators, so its indices run lower than the real <code>ctrl</code> indices when such actuators are present. One joint may also carry several actuators (position + velocity), in which case it appears more than once here.</p><p>Source: <a href="https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator" target="_blank" rel="noopener">MJCF XML reference — actuator</a></p>'
      }
    }
  ];

  global.Orderings = {
    FRAMEWORKS: FRAMEWORKS,
    RULE_DOCS: RULE_DOCS,
    QUAT_TEXT: QUAT_TEXT,
    quatChipOf: quatChipOf,
    analyze: analyze,
    compareOrders: compareOrders,
    dfsOrder: dfsOrder,
    bfsOrder: bfsOrder,
    freeBaseJoints: freeBaseJoints,
    labelOf: labelOf,
    ruleOf: ruleOf
  };
})(window);
