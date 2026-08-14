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
     at all; simulators create no DOF for a URDF fixed joint.
     `holdsFreeBase` marks the ones that expose a floating base as a joint —
     MuJoCo does (a free joint, first in the vector), Isaac / Gazebo /
     ros2_control treat it as a free-floating root instead.
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
      // A simulator creates no joint at all for a URDF fixed joint, so it can
      // never appear in those columns — the toggle only reveals fixed joints
      // in the frameworks whose joint list can actually contain them.
      if (seq && (!showFixed || !fw.holdsFixed)) {
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
        holdsFixed: fw.holdsFixed
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
      body: {
        zh: '<p>URDF 本身不定义关节顺序，只是一个 link/joint 的集合。这里取 <code>&lt;joint&gt;</code> 元素在 XML 中出现的先后顺序，作为最直观的参照基准。</p><p><b>注意：</b>URDF 规范并不要求关节按树的顺序书写，父关节写在子关节后面也是合法的。</p><p><b>MJCF：</b>MuJoCo 的 XML 嵌套本身就是 body 树，所以载入 MJCF 时这一列不只是「参照基准」，它<b>就是 MuJoCo 编译出来的关节 id 顺序</b> —— body id 按深度优先分配，同一个 body 内的多个关节按书写顺序排。没有 <code>&lt;joint&gt;</code> 的 body 是焊死在父 body 上的（相当于 URDF 的 <code>fixed</code>），不产生任何关节；本工具把这种边在运动学树里标成 <code>[weld]</code>。</p><p><b>注意：</b>关节序号不等于 qpos 下标 —— <code>free</code> 占 7 个 qpos / 6 个 qvel，<code>ball</code> 占 4 个 qpos / 3 个 qvel，其余各占 1 个。<code>&lt;include&gt;</code> 引入的部分本工具读不到。</p>',
        en: '<p>URDF itself defines no joint order — it is just a bag of links and joints. This column is the order the <code>&lt;joint&gt;</code> elements appear in the XML, used as the most intuitive baseline.</p><p><b>Note:</b> the spec does not require joints to be written in tree order; declaring a parent joint after its child is perfectly legal.</p><p><b>MJCF:</b> in MuJoCo the XML nesting <i>is</i> the body tree, so for an MJCF input this column is not merely a baseline — it <b>is the joint id order MuJoCo compiles to</b>: body ids are assigned depth-first, and several joints on one body keep their written order. A body with no <code>&lt;joint&gt;</code> is welded to its parent (the MJCF equivalent of a URDF <code>fixed</code> joint) and creates no joint at all; this tool marks those edges <code>[weld]</code> in the kinematic tree.</p><p><b>Note:</b> a joint index is not a qpos index — <code>free</code> spans 7 qpos / 6 qvel, <code>ball</code> 4 qpos / 3 qvel, everything else 1. Anything pulled in through <code>&lt;include&gt;</code> is invisible to this tool.</p>'
      }
    },
    {
      id: 'mujoco',
      verify: 'print([m.joint(i).name for i in range(m.njnt)])\n\n# C API: mj_id2name(m, mjOBJ_JOINT, i)',
      body: {
        zh: '<p>MuJoCo 的 URDF 导入器先把所有 link 读进来，再按 <code>&lt;joint&gt;</code> 的文档顺序填充 <code>urChildren[parent].push_back(child)</code>，然后从根 body 递归 <code>AddToTree()</code>。结果就是 <b>body 树的深度优先前序遍历，同层子 body 按 URDF 里关节出现的先后排列</b>；关节 id 随 body 创建顺序递增。</p><p><b>注意：</b>URDF 的 <code>fixed</code> 关节在 MJCF 里不会生成任何 joint（只是 body 嵌套），所以不占 qpos；<code>planar</code> 会被展开成 2 个 slide + 1 个 hinge；<code>&lt;mimic&gt;</code> 被完全忽略。运行时用 <code>mj_id2name(m, mjOBJ_JOINT, i)</code> 核对。</p><p><b>直接载入 MJCF 时</b>这一列没有任何推导成分：XML 的 body 嵌套就是 body 树，DFS 的结果必然等于文件顺序，两列一致只是印证解析没跑偏。真正要小心的是同一台机器人的 URDF 和 MJCF 未必写成同一个顺序 —— 把两个文件分别载入本工具，对比「文件顺序」这一列即可。</p><p>来源：<a href="https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_urdf.cc" target="_blank" rel="noopener">mujoco <code>xml_urdf.cc</code></a>、<a href="https://mujoco.readthedocs.io/en/stable/XMLreference.html" target="_blank" rel="noopener">MJCF XML reference</a></p>',
        en: '<p>MuJoCo\'s URDF importer reads all links first, then fills <code>urChildren[parent].push_back(child)</code> in <code>&lt;joint&gt;</code> document order, and finally recurses with <code>AddToTree()</code> from the root body. The result is a <b>depth-first pre-order walk of the body tree with siblings in URDF joint order</b>; joint ids increase with body creation order.</p><p><b>Note:</b> a URDF <code>fixed</code> joint produces no MJCF joint at all (just nested bodies), so it occupies no qpos; <code>planar</code> expands into 2 slides + 1 hinge; <code>&lt;mimic&gt;</code> is ignored outright. Verify at runtime with <code>mj_id2name(m, mjOBJ_JOINT, i)</code>.</p><p><b>When an MJCF is loaded directly</b> nothing is inferred here: the XML nesting is the body tree, so the depth-first walk necessarily equals the file order — the two columns agreeing only confirms the parse. What does bite is that the URDF and the MJCF of the same robot need not be written in the same order: load both files and compare the file-order column.</p><p>Sources: <a href="https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_urdf.cc" target="_blank" rel="noopener">mujoco <code>xml_urdf.cc</code></a>, <a href="https://mujoco.readthedocs.io/en/stable/XMLreference.html" target="_blank" rel="noopener">MJCF XML reference</a></p>'
      }
    },
    {
      id: 'isaacgym',
      verify: 'print(gym.get_asset_dof_names(asset))',
      body: {
        zh: '<p>Isaac Gym Preview Release 对运动学树采用<b>深度优先</b>排列 DOF。因此一个四足机器人会得到 <code>FL_hip, FL_thigh, FL_calf, FR_hip, …</code> 这样“一条腿走到底”的顺序。</p><p><b>注意：</b>这正是从 IsaacGymEnvs 迁移到 Isaac Lab 时最容易踩的坑 —— 同一个 URDF 在两边的 DOF 索引不一样。运行时用 <code>gym.get_asset_dof_names(asset)</code> 核对。</p><p>来源：<a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a></p>',
        en: '<p>Isaac Gym Preview Release orders DOFs <b>depth-first</b> over the kinematic tree. A quadruped therefore comes out as <code>FL_hip, FL_thigh, FL_calf, FR_hip, …</code> — one whole leg at a time.</p><p><b>Note:</b> this is exactly the trap when porting from IsaacGymEnvs to Isaac Lab — the same URDF yields different DOF indices on each side. Verify at runtime with <code>gym.get_asset_dof_names(asset)</code>.</p><p>Source: <a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a></p>'
      }
    },
    {
      id: 'isaacsim',
      verify: '# Isaac Lab\nprint(robot.data.joint_names)\n\n# Isaac Sim (ArticulationView)\nprint(view.joint_names)',
      body: {
        zh: '<p>官方文档原文：“Physics simulation in Isaac Sim and Isaac Lab assumes a <b>breadth-first</b> ordering for the joints in a given kinematic tree. However, Isaac Gym Preview Release assumed a <b>depth-first</b> ordering.” 也就是 PhysX stage parser 按<b>广度优先</b>（逐层）排列关节。</p><p>同一个四足机器人在这里是 <code>FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, …</code> —— 先排完所有髋关节，再排所有大腿。</p><p><b>注意：</b>运行时用 <code>robot.data.joint_names</code> 核对，用 <code>ArticulationCfg</code> 的 <code>joint_names_expr</code> 或 <code>find_joints()</code> 做重映射，不要硬编码索引。</p><p>来源：<a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a></p>',
        en: '<p>Straight from the docs: “Physics simulation in Isaac Sim and Isaac Lab assumes a <b>breadth-first</b> ordering for the joints in a given kinematic tree. However, Isaac Gym Preview Release assumed a <b>depth-first</b> ordering.” The PhysX stage parser walks the tree level by level.</p><p>The same quadruped comes out as <code>FL_hip, FR_hip, RL_hip, RR_hip, FL_thigh, …</code> — all hips first, then all thighs.</p><p><b>Note:</b> verify at runtime with <code>robot.data.joint_names</code>, and remap with <code>joint_names_expr</code> / <code>find_joints()</code> instead of hard-coding indices.</p><p>Source: <a href="https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html" target="_blank" rel="noopener">Isaac Lab — Migrating from IsaacGymEnvs</a></p>'
      }
    },
    {
      id: 'gazebo',
      verify: '# print the SDF Gazebo actually loads, then read the <joint> order\ngz sdf -p robot.urdf        # ign sdf -p on Ignition',
      body: {
        zh: '<p>只适用于 URDF：sdformat 不认 MJCF，载入 MJCF 时这一列显示为不适用。</p><p>Gazebo 不直接吃 URDF —— 它先用 sdformat 把 URDF 转成 SDF，模型里的关节顺序就是这次转换的产物。<code>parser_urdf.cc</code> 里的 <code>CreateSDF()</code> 递归遍历 <code>_link-&gt;child_links</code>，是<b>深度优先</b>。</p><p>关键在于 <code>child_links</code> 是谁填的：urdfdom 的 <code>initTree()</code> 遍历 <code>std::map</code> 类型的 <code>joints_</code> 来填充它，而 <code>std::map</code> 按 key 排序。所以 <b>Gazebo 的同层子关节是按关节名字母序排的，跟你在 URDF 里的书写顺序无关</b> —— 这一点和 MuJoCo / Isaac Gym 的 DFS（同层按文档顺序）不同，也是这一列存在的意义。</p><p><b>注意：</b>sdformat <b>默认会把 fixed 关节吸收掉</b>（把子 link 合并进父 link），除非加 <code>&lt;disableFixedJointLumping&gt;</code> 或 <code>&lt;preserveFixedJoint&gt;</code>（两者同时存在时后者优先）。另外 <code>gazebo_ros_joint_state_publisher</code> 插件发布的顺序取决于你在插件里列 <code>&lt;joint_name&gt;</code> 的顺序，<code>gz_ros2_control</code> 则走 <code>&lt;ros2_control&gt;</code> 标签，都与这一列无关。</p><p>来源：<a href="https://github.com/gazebosim/sdformat/blob/sdf14/src/parser_urdf.cc" target="_blank" rel="noopener">sdformat <code>parser_urdf.cc</code></a>、<a href="https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/model.h" target="_blank" rel="noopener">urdfdom_headers <code>model.h</code></a></p>',
        en: '<p>URDF only: sdformat does not read MJCF, so this column is n/a for an MJCF input.</p><p>Gazebo does not consume URDF directly — sdformat converts it to SDF first, and the model\'s joint order is a product of that conversion. <code>CreateSDF()</code> in <code>parser_urdf.cc</code> recurses over <code>_link-&gt;child_links</code>, so the walk is <b>depth-first</b>.</p><p>What matters is who fills <code>child_links</code>: urdfdom\'s <code>initTree()</code> populates it by iterating <code>joints_</code>, a <code>std::map</code>, which is sorted by key. So <b>Gazebo orders sibling joints alphabetically by joint name, regardless of how the URDF is written</b> — unlike MuJoCo / Isaac Gym depth-first, whose siblings follow document order. That difference is why this column exists.</p><p><b>Note:</b> sdformat <b>lumps fixed joints away by default</b> (merging the child link into its parent) unless <code>&lt;disableFixedJointLumping&gt;</code> or <code>&lt;preserveFixedJoint&gt;</code> is set (the latter wins when both appear). Separately, the <code>gazebo_ros_joint_state_publisher</code> plugin publishes in the order you list <code>&lt;joint_name&gt;</code> in the plugin, and <code>gz_ros2_control</code> follows the <code>&lt;ros2_control&gt;</code> tag — neither matches this column.</p><p>Sources: <a href="https://github.com/gazebosim/sdformat/blob/sdf14/src/parser_urdf.cc" target="_blank" rel="noopener">sdformat <code>parser_urdf.cc</code></a>, <a href="https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/model.h" target="_blank" rel="noopener">urdfdom_headers <code>model.h</code></a></p>'
      }
    },
    {
      id: 'ros2control',
      verify: 'ros2 control list_hardware_interfaces\nros2 topic echo /joint_states --field name --once',
      body: {
        zh: '<p>只适用于 URDF：MJCF 里没有 <code>&lt;ros2_control&gt;</code> 标签，载入 MJCF 时这一列显示为不适用。</p><p><code>joint_state_broadcaster</code> 发布的顺序取决于配置，文档给出三种情况：</p><ul><li>未设 <code>joints</code> 参数、<code>use_urdf_to_filter=true</code>（默认）→ <b>与 URDF 文件里的关节顺序相同，和 &lt;ros2_control&gt; 标签内的顺序无关</b>。</li><li>未设 <code>joints</code> 参数、<code>use_urdf_to_filter=false</code> → 按 resource manager 里注册 state interface 的顺序，也就是 <b>&lt;ros2_control&gt; 标签内 &lt;joint&gt; 的顺序</b>（硬件组件加载顺序）。</li><li>显式设置了 <code>joints</code> + <code>interfaces</code> 参数 → <b>按 joints 参数的顺序</b>（本工具无法读取 YAML，这种情况请以你的配置为准）。</li></ul><p><b>注意：</b>控制器（如 <code>joint_trajectory_controller</code>）用的是各自 YAML 里 <code>joints</code> 参数的顺序，与这一列无关。<code>extra_joints</code> 会追加到末尾。</p><p>来源：<a href="https://control.ros.org/rolling/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html" target="_blank" rel="noopener">joint_state_broadcaster 文档</a></p>',
        en: '<p>URDF only: MJCF has no <code>&lt;ros2_control&gt;</code> tag, so this column reads n/a for an MJCF input.</p><p>The order <code>joint_state_broadcaster</code> publishes depends on configuration; the docs give three cases:</p><ul><li>no <code>joints</code> param, <code>use_urdf_to_filter=true</code> (default) → <b>same as the joint order in the URDF file, independent of the order in the &lt;ros2_control&gt; tag</b>.</li><li>no <code>joints</code> param, <code>use_urdf_to_filter=false</code> → the order state interfaces were registered in the resource manager, i.e. <b>the order of &lt;joint&gt; inside the &lt;ros2_control&gt; tag</b> (hardware component load order).</li><li><code>joints</code> + <code>interfaces</code> params set explicitly → <b>the order of the joints param</b> (this tool cannot read your YAML — trust your config in that case).</li></ul><p><b>Note:</b> controllers such as <code>joint_trajectory_controller</code> use their own YAML <code>joints</code> parameter order, unrelated to this column. <code>extra_joints</code> are appended at the end.</p><p>Source: <a href="https://control.ros.org/rolling/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html" target="_blank" rel="noopener">joint_state_broadcaster docs</a></p>'
      }
    },
    {
      id: 'mjcfctrl',
      verify: 'print([m.actuator(i).name for i in range(m.nu)])\n\n# joint each actuator drives (joint transmissions only):\nprint([m.joint(m.actuator_trnid[i, 0]).name for i in range(m.nu)])',
      body: {
        zh: '<p>只适用于 MJCF。<code>data.ctrl</code> 的下标是<b>执行器</b>下标，不是关节下标 —— 它按 <code>&lt;actuator&gt;</code> 段里元素出现的先后排列，和 <code>qpos</code> / 关节 id 是两个独立的向量。两者平时看起来一样，只要有人在 actuator 段里调换了两行，<code>ctrl[i]</code> 就不再对应你以为的那个关节，而且没有任何报错。</p><p>内置的 <b>Unitree G1 MJCF 示例正是这种情况</b>：body 树里右手是 <code>middle_0, middle_1, index_0, index_1</code>，而 <code>&lt;actuator&gt;</code> 里写的是 <code>index_0, index_1, middle_0, middle_1</code> —— 最后四个电机与关节顺序对不上。</p><p><b>注意：</b>驱动 tendon / site / body（吸附）的执行器不对应任何关节，但一样占 <code>ctrl</code> 槽位；本列只列出驱动关节的执行器，出现这类执行器时列里的序号会小于真实 <code>ctrl</code> 下标。同一个关节也可以挂多个执行器（位置 + 速度），那样它会在这一列里出现多次。</p><p>来源：<a href="https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator" target="_blank" rel="noopener">MJCF XML reference — actuator</a></p>',
        en: '<p>MJCF only. <code>data.ctrl</code> is indexed by <b>actuator</b>, not by joint: it follows the document order of the <code>&lt;actuator&gt;</code> section, an entirely separate vector from <code>qpos</code> and the joint ids. The two usually look identical — until someone swaps two lines in the actuator block, after which <code>ctrl[i]</code> silently drives a different joint than you think.</p><p>The bundled <b>Unitree G1 MJCF sample is exactly this case</b>: the body tree has the right hand as <code>middle_0, middle_1, index_0, index_1</code>, while <code>&lt;actuator&gt;</code> lists <code>index_0, index_1, middle_0, middle_1</code> — the last four motors do not line up with the joint order.</p><p><b>Note:</b> actuators driving a tendon / site / body (adhesion) map to no joint at all yet still occupy a <code>ctrl</code> slot; this column lists only joint actuators, so its indices run lower than the real <code>ctrl</code> indices when such actuators are present. One joint may also carry several actuators (position + velocity), in which case it appears more than once here.</p><p>Source: <a href="https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator" target="_blank" rel="noopener">MJCF XML reference — actuator</a></p>'
      }
    }
  ];

  global.Orderings = {
    FRAMEWORKS: FRAMEWORKS,
    RULE_DOCS: RULE_DOCS,
    analyze: analyze,
    compareOrders: compareOrders,
    dfsOrder: dfsOrder,
    bfsOrder: bfsOrder,
    freeBaseJoints: freeBaseJoints,
    labelOf: labelOf,
    ruleOf: ruleOf
  };
})(window);
