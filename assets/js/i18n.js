/* i18n — zh (default) / en */
(function (global) {
  'use strict';

  var DICT = {
    zh: {
      'app.title': '机器人关节顺序检查工具',
      'app.subtitle': '比较 URDF 在各仿真器 / 框架中的关节顺序',
      'ui.theme': '切换主题',

      'input.title': '1. 载入 URDF',
      'input.hint': '文件完全在你的浏览器本地解析，不会上传到任何服务器。',
      'input.drop': '拖拽 .urdf 文件到这里，或点击选择文件',
      'input.dropSmall': '支持 .urdf / .xml（xacro 需先展开）',
      'input.samples': '示例：',
      'input.paste': '或粘贴 URDF 文本 ▾',
      'input.pasteHide': '收起粘贴框 ▴',
      'input.placeholder': '在此粘贴 URDF XML …',
      'input.parse': '解析',
      'input.pasted': '粘贴的文本',

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
      'msg.noRobot': '根元素不是 &lt;robot&gt;，这不是一个有效的 URDF 文件。',
      'msg.noJoints': '文件里没有找到 &lt;joint&gt; 元素。',
      'msg.multiRoot': '发现 {n} 个根链接（{list}）。标准 URDF 应当只有一个根，这里按文档顺序依次遍历每棵树。',
      'msg.cycle': '运动学树中存在环或未连接的关节（{n} 个关节没有被遍历到）：{list}',
      'msg.undeclaredLink': '{n} 个链接只在 &lt;joint&gt; 里被引用、没有 &lt;link&gt; 定义：{list}',
      'msg.mimic': '{n} 个 mimic 关节：{list}。MuJoCo 的 URDF 导入会忽略 &lt;mimic&gt;，Isaac 各版本支持程度也不同，务必单独确认。',
      'msg.noRos2Control': '未找到 &lt;ros2_control&gt; 标签，因此不参与比较。',
      'msg.multiDof': '存在多自由度关节（floating / planar / ball）：{list}。各框架展开成的 DOF 数量和排列方式不同，关节级顺序一致不代表 DOF 级一致。',
      'msg.parsed': '已解析 <b>{name}</b> — {links} 个 link，{joints} 个 joint（{movable} 个可动）。',

      'na.noTag': '无 <ros2_control> 标签',
      'chip.ok': '一致',
      'chip.bad': '不一致',
      'chip.warn': '集合不同',
      'chip.ref': '参照',
      'chip.na': '不适用',

      'footer.disclaimer': '静态推导结果仅供排查，请以运行时打印的关节名列表为准。'
    },

    en: {
      'app.title': 'Robot Joint Order Check Tool',
      'app.subtitle': 'Compare URDF joint ordering across simulators and frameworks',
      'ui.theme': 'Toggle theme',

      'input.title': '1. Load a URDF',
      'input.hint': 'The file is parsed entirely in your browser — nothing is uploaded to any server.',
      'input.drop': 'Drop a .urdf file here, or click to choose one',
      'input.dropSmall': 'Accepts .urdf / .xml (expand xacro first)',
      'input.samples': 'Samples:',
      'input.paste': 'or paste URDF text ▾',
      'input.pasteHide': 'hide paste box ▴',
      'input.placeholder': 'Paste URDF XML here …',
      'input.parse': 'Parse',
      'input.pasted': 'pasted text',

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
      'msg.noRobot': 'The root element is not &lt;robot&gt; — this is not a valid URDF.',
      'msg.noJoints': 'No &lt;joint&gt; elements found in this file.',
      'msg.multiRoot': 'Found {n} root links ({list}). A standard URDF has exactly one root; each tree is traversed in document order here.',
      'msg.cycle': 'The kinematic tree contains a cycle or disconnected joints ({n} joint(s) never reached): {list}',
      'msg.undeclaredLink': '{n} link(s) are referenced by joints but never declared with &lt;link&gt;: {list}',
      'msg.mimic': '{n} mimic joint(s): {list}. MuJoCo\'s URDF importer ignores &lt;mimic&gt;, and Isaac support varies by version — verify these separately.',
      'msg.noRos2Control': 'No &lt;ros2_control&gt; tag found, so it is excluded from the comparison.',
      'msg.multiDof': 'Multi-DOF joints present (floating / planar / ball): {list}. Frameworks expand these into different numbers of DOFs, so matching joint order does not imply matching DOF order.',
      'msg.parsed': 'Parsed <b>{name}</b> — {links} links, {joints} joints ({movable} movable).',

      'na.noTag': 'no <ros2_control> tag',
      'chip.ok': 'match',
      'chip.bad': 'differs',
      'chip.warn': 'set differs',
      'chip.ref': 'reference',
      'chip.na': 'n/a',

      'footer.disclaimer': 'Statically derived — for triage only. Trust the joint names your runtime prints.'
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
