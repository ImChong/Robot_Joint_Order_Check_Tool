/* i18n — zh (default) / en */
(function (global) {
  'use strict';

  var DICT = {
    zh: {
      'app.title': '机器人关节顺序检查工具',
      'app.subtitle': '比较 URDF / MJCF 在各仿真器 / 框架中的关节顺序',
      'ui.theme': '切换主题',

      'intro.title': '同一个模型，六个框架，未必是同一个关节顺序',
      'intro.lead': 'URDF 只是一堆 link 和 joint 的集合，<b>它本身不定义关节顺序</b>。每个仿真器和框架都按自己的规则把这些关节排成一个向量：Isaac Gym 沿运动学树深度优先，Isaac Sim / Isaac Lab 广度优先，Gazebo 也是深度优先、但同层子关节按名字的字母序，PyBullet 则把 fixed 关节一起编进下标。',
      'intro.demoLabel': '同一个四足 URDF，两种导入顺序',
      'intro.note': '于是把 Isaac Gym 里训好的策略搬到 Isaac Lab 或实机上，关节向量会<b>静默错位</b>：不报错、不崩溃，机器人直接抽搐。MJCF 也躲不掉 —— body 树虽然定死了关节顺序，但 <code>&lt;actuator&gt;</code>（也就是 <code>data.ctrl</code>）的顺序是另一个独立的向量。',
      'intro.does': '这个工具就是在写代码之前把这件事看清楚：载入一个 URDF 或 MJCF（按根元素自动识别），把它在各框架里的关节顺序并列排开 —— 全部一致显示绿色，对不上就精确标红到是哪一格。',
      'intro.p1.title': '六个框架并列对照',
      'intro.p1.body': 'Isaac Gym、Isaac Sim / Isaac Lab、MuJoCo、Gazebo、PyBullet、ros2_control 一次算完；载入 MJCF 时还会单列 <actuator>（data.ctrl）的顺序。',
      'intro.p2.title': '直接给出对齐用的代码',
      'intro.p2.body': '选好源框架和目标框架，生成可粘贴的 Python / C++ / JSON 索引数组，把手上的关节向量重排成目标顺序；也可导出 CSV 与完整 JSON。',
      'intro.p3.title': '自动提示常见的坑',
      'intro.p3.body': '未展开的 xacro、mimic 关节、浮动基座、多自由度关节、MJCF 的 <include> 与重名元素，解析时都会单独提示。',
      'intro.p4.title': '本地解析，不上传',
      'intro.p4.body': '纯静态页面，文件只在你的浏览器里解析。结论按各导入器公开的默认规则静态推导，上线前请以运行时打印的关节名为准。',
      'intro.linkRules': '看看各框架的顺序规则和它们的出处 ↓',

      'input.title': '1. 载入 URDF / MJCF',
      'input.hint': '文件完全在你的浏览器本地解析，不会上传到任何服务器。',
      'input.drop': '拖拽 .urdf / .xml 文件到这里，或点击选择文件',
      'input.dropSmall': 'URDF（.urdf）或 MuJoCo MJCF（.xml），自动识别；xacro 需先展开',
      'input.samples': '示例：',
      'input.paste': '或粘贴 URDF / XML 文本 ▾',
      'input.pasteHide': '收起粘贴框 ▴',
      'input.placeholder': '在此粘贴 URDF 或 MJCF XML …',
      'input.parse': '解析',
      'input.pasted': '粘贴的文本',
      'input.upstream': '上游原始文件',

      'opts.title': '2. 选项',
      'opts.reference': '参照顺序',
      'opts.ros2c': 'ros2_control 顺序来源',
      'opts.ros2c.urdf': 'URDF 顺序 (use_urdf_to_filter=true，默认)',
      'opts.ros2c.tag': '<ros2_control> 标签顺序 (resource manager)',
      'opts.sibling': '同层子关节排序',
      'opts.sibling.auto': '按各框架自身规则（默认）',
      'opts.sibling.doc': '全部按 URDF 文档顺序',
      'opts.sibling.alpha': '全部按关节名字母序',
      'opts.showFixed': '显示 fixed 关节',

      'table.title': '3. 顺序对照表',
      'table.byIndex': '按序号',
      'table.byJoint': '按关节',
      'table.legendOk': '与参照一致',
      'table.legendBad': '与参照不一致',
      'table.legendShift': '顺序一致但因关节集合不同而错位',
      'table.legendNa': '该框架不含此关节',
      'table.idx': '序号',
      'table.joint': '关节',
      'table.type': '类型',
      'table.dof': 'DOF',
      'table.chain': '父 → 子',

      'export.title': '4. 导出与索引重映射',
      'export.hint': '生成把一个框架的关节向量重排到另一个框架的索引数组，直接贴进代码里用。',
      'export.from': '源顺序（你手上的数据）',
      'export.to': '目标顺序（你要的顺序）',
      'export.copy': '复制',
      'export.copied': '已复制 ✓',
      'export.csv': '下载 CSV',
      'export.json': '下载完整 JSON',

      'tree.title': '5. 运动学树与关节详情',
      'tree.sub': '运动学树',
      'tree.jointsSub': '关节列表',

      'rules.title': '各框架的顺序规则',
      'rules.hint': '这些规则决定了上面的计算结果。点击展开查看依据、注意事项和运行时核对命令。',
      'rules.verify': '运行时核对',

      'verdict.ok.title': '顺序一致',
      'verdict.ok.desc': '在共有的 {n} 个关节上，所有被比较的框架顺序完全相同。',
      'verdict.bad.title': '顺序不一致',
      'verdict.bad.desc': '有 {n} 个框架的关节顺序与参照（{ref}）不同，直接把向量传过去会错位：',
      'verdict.warn.title': '顺序一致，但关节集合不同',
      'verdict.warn.desc': '各框架在共有关节上的相对顺序一致，但包含的关节不完全相同：',
      'verdict.diffCount': '{n} 个位置不同',
      'verdict.missing': '缺少 {n} 个关节：{list}',
      'verdict.extra': '多出 {n} 个关节：{list}',
      'verdict.single': '只有一个可比较的顺序，无法进行比较。',

      'msg.xacro': '这看起来是未展开的 xacro 文件（含 &lt;xacro:*&gt; 标签或 ${} 表达式）。请先运行 <code>xacro robot.urdf.xacro &gt; robot.urdf</code>，再上传展开后的 URDF。',
      'msg.xmlError': 'XML 解析失败：',
      'msg.noRobot': '根元素不是 &lt;robot&gt;（URDF）也不是 &lt;mujoco&gt;（MJCF），无法识别这个文件。',
      'msg.noRoot': '根元素不是 &lt;mujoco&gt;，这不是一个有效的 MJCF 文件。',
      'msg.noJoints': '文件里没有找到 &lt;joint&gt; 元素。',
      'msg.multiRoot': '发现 {n} 个根链接（{list}）。标准 URDF 应当只有一个根，这里按文档顺序依次遍历每棵树。',
      'msg.cycle': '运动学树中存在环或未连接的关节（{n} 个关节没有被遍历到）：{list}',
      'msg.undeclaredLink': '{n} 个链接只在 &lt;joint&gt; 里被引用、没有 &lt;link&gt; 定义：{list}',
      'msg.mimic': '{n} 个 mimic 关节：{list}。MuJoCo 的 URDF 导入会忽略 &lt;mimic&gt;，Isaac 各版本支持程度也不同，务必单独确认。',
      'msg.noRos2Control': '未找到 &lt;ros2_control&gt; 标签，因此不参与比较。',
      'msg.multiDof': '存在多自由度关节（floating / planar / ball）：{list}。各框架展开成的 DOF 数量和排列方式不同，关节级顺序一致不代表 DOF 级一致。',
      'msg.parsed': '已解析 <b>{name}</b> — {links} 个 link，{joints} 个 joint（{movable} 个可动）。',
      'msg.parsedMjcf': '已解析 MJCF <b>{name}</b> — {links} 个 body，{joints} 个 joint，{actuators} 个 actuator。',
      'msg.freeBase': '检测到浮动基座关节：{list}。MuJoCo 会为它生成一个 free joint 排在关节向量最前面（占 7 个 qpos / 6 个 qvel），而 Isaac / Gazebo / PyBullet / ros2_control 把它当作自由浮动的根、根本不放进关节列表 —— 所以本工具只在 MuJoCo 相关的列里保留它。',
      'msg.mjcfInclude': '文件里有 {n} 处 &lt;include&gt;（{list}）。被引入的 body 和 actuator 本工具读不到，下面的顺序只反映当前这个文件。',
      'msg.mjcfEquality': '{n} 个 joint 等式约束（MuJoCo 版的 mimic）：{list}。它们不改变关节顺序，但被约束的关节不是独立自由度。',
      'msg.mjcfActuatorSkipped': '{n} 个执行器没有对应到本文件里的关节（驱动 tendon / site / body，或引用了 &lt;include&gt; 里的关节）：{list}。它们同样占用 ctrl 槽位，所以「MuJoCo ctrl」一列的序号会小于真实的 ctrl 下标。',
      'msg.duplicateName': '{n} 个重名的 body / joint：{list}。MuJoCo 编译时会直接报错，这里给重复项加了 <code>#n</code> 后缀以便继续分析。',
      'msg.sampleFetch': '载入示例失败（{file}：{detail}）。这个示例存放在 <code>samples/</code> 目录里、点击时才读取，用 <code>file://</code> 直接打开页面时会被浏览器拦截 —— 请改用本地服务器（<code>python3 -m http.server</code>），或下载 <a href="{url}" target="_blank" rel="noopener">上游原始文件</a> 后拖进来。',

      'na.noTag': '无 <ros2_control> 标签',
      'na.noActuator': '无 <actuator>',
      'na.urdfOnly': '不支持 MJCF',
      'chip.ok': '一致',
      'chip.bad': '不一致',
      'chip.warn': '集合不同',
      'chip.ref': '参照',
      'chip.na': '不适用',

      'footer.disclaimer': '静态推导结果仅供排查，请以运行时打印的关节名列表为准。',
      'footer.author': '刘冲'
    },

    en: {
      'app.title': 'Robot Joint Order Check Tool',
      'app.subtitle': 'Compare URDF / MJCF joint ordering across simulators and frameworks',
      'ui.theme': 'Toggle theme',

      'intro.title': 'One model, six frameworks — not necessarily one joint order',
      'intro.lead': 'A URDF is just a bag of links and joints — <b>it defines no joint order of its own</b>. Every simulator and framework imposes one by its own rule: Isaac Gym walks the kinematic tree depth-first, Isaac Sim / Isaac Lab walk it breadth-first, Gazebo is depth-first too but sorts sibling joints alphabetically by name, and PyBullet numbers fixed joints right along with the movable ones.',
      'intro.demoLabel': 'the same quadruped URDF, two import orders',
      'intro.note': 'So moving a policy trained in Isaac Gym to Isaac Lab or to real hardware makes the joint vector <b>silently mismatch</b>: nothing errors, nothing crashes, the robot just convulses. MJCF is not exempt — its body tree does pin the joint order down, but <code>&lt;actuator&gt;</code> order (that is, <code>data.ctrl</code>) is a separate vector of its own.',
      'intro.does': 'This tool is how you see all of that before writing the code: load a URDF or MJCF (the format is detected from the root element) and it prints the joint order side by side across the frameworks — green when they all agree, red on the exact cells that do not.',
      'intro.p1.title': 'Six frameworks side by side',
      'intro.p1.body': 'Isaac Gym, Isaac Sim / Isaac Lab, MuJoCo, Gazebo, PyBullet and ros2_control in one pass; MJCF input adds a column for <actuator> (data.ctrl) order.',
      'intro.p2.title': 'The code that realigns them',
      'intro.p2.body': 'Pick a source and a target framework to get a paste-ready Python / C++ / JSON index array that reorders your joint vector — plus CSV and full JSON export.',
      'intro.p3.title': 'Common traps flagged for you',
      'intro.p3.body': 'Unexpanded xacro, mimic joints, floating bases, multi-DOF joints, MJCF <include> and duplicate names each get their own warning at parse time.',
      'intro.p4.title': 'Parsed locally, never uploaded',
      'intro.p4.body': 'A purely static page — your file is parsed in your own browser. Results are derived statically from each importer\'s documented defaults, so confirm against the joint names your runtime prints.',
      'intro.linkRules': 'See the ordering rule for each framework and where it comes from ↓',

      'input.title': '1. Load a URDF or MJCF',
      'input.hint': 'The file is parsed entirely in your browser — nothing is uploaded to any server.',
      'input.drop': 'Drop a .urdf / .xml file here, or click to choose one',
      'input.dropSmall': 'URDF (.urdf) or MuJoCo MJCF (.xml), detected automatically; expand xacro first',
      'input.samples': 'Samples:',
      'input.paste': 'or paste URDF / XML text ▾',
      'input.pasteHide': 'hide paste box ▴',
      'input.placeholder': 'Paste URDF or MJCF XML here …',
      'input.parse': 'Parse',
      'input.pasted': 'pasted text',
      'input.upstream': 'upstream file',

      'opts.title': '2. Options',
      'opts.reference': 'Reference order',
      'opts.ros2c': 'ros2_control order source',
      'opts.ros2c.urdf': 'URDF order (use_urdf_to_filter=true, default)',
      'opts.ros2c.tag': '<ros2_control> tag order (resource manager)',
      'opts.sibling': 'Sibling joint order',
      'opts.sibling.auto': "Each framework's own rule (default)",
      'opts.sibling.doc': 'Force URDF document order',
      'opts.sibling.alpha': 'Force alphabetical by joint name',
      'opts.showFixed': 'Show fixed joints',

      'table.title': '3. Side-by-side order',
      'table.byIndex': 'By index',
      'table.byJoint': 'By joint',
      'table.legendOk': 'matches reference',
      'table.legendBad': 'differs from reference',
      'table.legendShift': 'same relative order, offset by a differing joint set',
      'table.legendNa': 'joint absent in this framework',
      'table.idx': 'Index',
      'table.joint': 'Joint',
      'table.type': 'Type',
      'table.dof': 'DOF',
      'table.chain': 'parent → child',

      'export.title': '4. Export & index remapping',
      'export.hint': 'Generate the index array that reorders a joint vector from one framework into another — paste it straight into your code.',
      'export.from': 'Source order (the data you have)',
      'export.to': 'Target order (the order you want)',
      'export.copy': 'Copy',
      'export.copied': 'Copied ✓',
      'export.csv': 'Download CSV',
      'export.json': 'Download full JSON',

      'tree.title': '5. Kinematic tree & joint details',
      'tree.sub': 'Kinematic tree',
      'tree.jointsSub': 'Joints',

      'rules.title': 'Ordering rule for each framework',
      'rules.hint': 'These rules drive the results above. Expand for the reasoning, caveats and the command to verify at runtime.',
      'rules.verify': 'Verify at runtime',

      'verdict.ok.title': 'Orders are consistent',
      'verdict.ok.desc': 'All compared frameworks agree on the order of the {n} shared joints.',
      'verdict.bad.title': 'Orders are inconsistent',
      'verdict.bad.desc': '{n} framework(s) order joints differently from the reference ({ref}) — passing a vector across will silently mismatch:',
      'verdict.warn.title': 'Order consistent, joint sets differ',
      'verdict.warn.desc': 'Relative order agrees on the shared joints, but the frameworks do not contain the same joints:',
      'verdict.diffCount': '{n} position(s) differ',
      'verdict.missing': 'missing {n} joint(s): {list}',
      'verdict.extra': 'extra {n} joint(s): {list}',
      'verdict.single': 'Only one comparable order — nothing to compare against.',

      'msg.xacro': 'This looks like an unexpanded xacro file (it contains &lt;xacro:*&gt; tags or ${} expressions). Run <code>xacro robot.urdf.xacro &gt; robot.urdf</code> first and upload the expanded URDF.',
      'msg.xmlError': 'XML parse error: ',
      'msg.noRobot': 'The root element is neither &lt;robot&gt; (URDF) nor &lt;mujoco&gt; (MJCF) — this file is not recognised.',
      'msg.noRoot': 'The root element is not &lt;mujoco&gt; — this is not a valid MJCF file.',
      'msg.noJoints': 'No &lt;joint&gt; elements found in this file.',
      'msg.multiRoot': 'Found {n} root links ({list}). A standard URDF has exactly one root; each tree is traversed in document order here.',
      'msg.cycle': 'The kinematic tree contains a cycle or disconnected joints ({n} joint(s) never reached): {list}',
      'msg.undeclaredLink': '{n} link(s) are referenced by joints but never declared with &lt;link&gt;: {list}',
      'msg.mimic': '{n} mimic joint(s): {list}. MuJoCo\'s URDF importer ignores &lt;mimic&gt;, and Isaac support varies by version — verify these separately.',
      'msg.noRos2Control': 'No &lt;ros2_control&gt; tag found, so it is excluded from the comparison.',
      'msg.multiDof': 'Multi-DOF joints present (floating / planar / ball): {list}. Frameworks expand these into different numbers of DOFs, so matching joint order does not imply matching DOF order.',
      'msg.parsed': 'Parsed <b>{name}</b> — {links} links, {joints} joints ({movable} movable).',
      'msg.parsedMjcf': 'Parsed MJCF <b>{name}</b> — {links} bodies, {joints} joints, {actuators} actuators.',
      'msg.freeBase': 'Floating-base joint detected: {list}. MuJoCo turns it into a free joint at the head of the vector (7 qpos / 6 qvel), while Isaac / Gazebo / PyBullet / ros2_control treat it as a free-floating root and never list it as a joint — so this tool keeps it in the MuJoCo columns only.',
      'msg.mjcfInclude': '{n} &lt;include&gt; element(s) in this file ({list}). The included bodies and actuators cannot be read here, so the orders below reflect this file alone.',
      'msg.mjcfEquality': '{n} joint equality constraint(s) — MuJoCo\'s answer to &lt;mimic&gt;: {list}. They do not change joint order, but the constrained joints are not independent DOFs.',
      'msg.mjcfActuatorSkipped': '{n} actuator(s) map to no joint in this file (they drive a tendon / site / body, or a joint from an &lt;include&gt;): {list}. They still occupy a ctrl slot, so the indices in the "MuJoCo ctrl" column run lower than the real ctrl indices.',
      'msg.duplicateName': '{n} duplicate body / joint name(s): {list}. MuJoCo rejects these at compile time; a <code>#n</code> suffix was added here so the rest of the analysis can continue.',
      'msg.sampleFetch': 'Could not load the sample ({file}: {detail}). This one lives in <code>samples/</code> and is fetched on click, which the browser blocks when the page is opened over <code>file://</code> — serve it locally (<code>python3 -m http.server</code>) or download the <a href="{url}" target="_blank" rel="noopener">upstream file</a> and drop it in.',

      'na.noTag': 'no <ros2_control> tag',
      'na.noActuator': 'no <actuator>',
      'na.urdfOnly': 'no MJCF support',
      'chip.ok': 'match',
      'chip.bad': 'differs',
      'chip.warn': 'set differs',
      'chip.ref': 'reference',
      'chip.na': 'n/a',

      'footer.disclaimer': 'Statically derived — for triage only. Trust the joint names your runtime prints.',
      'footer.author': 'Chong Liu'
    }
  };

  var lang = 'zh';

  function t(key, vars) {
    var s = (DICT[lang] && DICT[lang][key]) || (DICT.zh && DICT.zh[key]) || key;
    if (vars) {
      Object.keys(vars).forEach(function (k) {
        s = s.split('{' + k + '}').join(vars[k]);
      });
    }
    return s;
  }

  function setLang(next) {
    lang = DICT[next] ? next : 'zh';
    document.documentElement.lang = lang === 'zh' ? 'zh-CN' : 'en';
    applyStatic();
  }

  function applyStatic(root) {
    var scope = root || document;
    scope.querySelectorAll('[data-i18n]').forEach(function (el) {
      el.textContent = t(el.getAttribute('data-i18n'));
    });
    // Only for strings that carry markup of their own (<b>, <code>) — the
    // source is this dictionary, never user input.
    scope.querySelectorAll('[data-i18n-html]').forEach(function (el) {
      el.innerHTML = t(el.getAttribute('data-i18n-html'));
    });
    scope.querySelectorAll('[data-i18n-title]').forEach(function (el) {
      el.title = t(el.getAttribute('data-i18n-title'));
    });
    scope.querySelectorAll('[data-i18n-placeholder]').forEach(function (el) {
      el.placeholder = t(el.getAttribute('data-i18n-placeholder'));
    });
  }

  global.I18N = {
    t: t,
    setLang: setLang,
    getLang: function () { return lang; },
    applyStatic: applyStatic
  };
})(window);
