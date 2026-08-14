/* UI wiring for the Robot Joint Order Check Tool */
(function (global) {
  'use strict';

  var t = global.I18N.t;
  var URDF = global.URDF;
  var Orderings = global.Orderings;

  var state = {
    text: '',
    source: { kind: 'file', name: '' },
    model: null,
    tree: null,
    analysis: null,
    view: 'byindex',
    codeLang: 'python',
    opts: { refId: 'urdf', siblingOrder: 'auto', ros2cMode: 'urdf', showFixed: false }
  };

  var $ = function (sel) { return document.querySelector(sel); };

  function esc(s) {
    return String(s).replace(/[&<>"']/g, function (c) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
    });
  }

  function lang() { return global.I18N.getLang(); }

  function fwLabel(col) { return Orderings.labelOf(col.label, lang()); }
  function fwRule(col) { return Orderings.ruleOf(col.label, lang()); }

  /* ── Theme ─────────────────────────────────────────────────────── */
  var THEME_KEY = 'rjoct.theme';

  function applyTheme(theme) {
    document.documentElement.setAttribute('data-theme', theme);
    $('#themeIcon').textContent = theme === 'dark' ? '☾' : '☀';
  }

  function initTheme() {
    var saved = null;
    try { saved = localStorage.getItem(THEME_KEY); } catch (e) { /* private mode */ }
    var theme = saved || (global.matchMedia && global.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
    applyTheme(theme);
    $('#themeToggle').addEventListener('click', function () {
      var next = document.documentElement.getAttribute('data-theme') === 'dark' ? 'light' : 'dark';
      applyTheme(next);
      try { localStorage.setItem(THEME_KEY, next); } catch (e) { /* ignore */ }
    });
  }

  /* ── Language ──────────────────────────────────────────────────── */
  var LANG_KEY = 'rjoct.lang';

  function initLang() {
    var saved = null;
    try { saved = localStorage.getItem(LANG_KEY); } catch (e) { /* ignore */ }
    var initial = saved || (/^zh\b/i.test(navigator.language || '') ? 'zh' : 'zh');
    setLang(initial, true);
    $('#langToggle').addEventListener('click', function () {
      setLang(lang() === 'zh' ? 'en' : 'zh');
    });
  }

  function setLang(next, silent) {
    global.I18N.setLang(next);
    $('#langToggle').textContent = next === 'zh' ? 'EN' : '中文';
    try { localStorage.setItem(LANG_KEY, next); } catch (e) { /* ignore */ }
    renderSamples();
    renderRules();
    syncPasteToggleLabel();
    if (!silent && state.model) renderAll();
  }

  /* ── Input ─────────────────────────────────────────────────────── */
  function renderSamples() {
    var host = $('#sampleButtons');
    host.innerHTML = '';
    (global.SAMPLES || []).forEach(function (s) {
      var b = document.createElement('button');
      b.type = 'button';
      b.className = 'btn-chip';
      b.textContent = s.label[lang()] || s.label.en;
      b.addEventListener('click', function () { load(s.text, { kind: 'sample', id: s.id }); });
      host.appendChild(b);
    });
  }

  function syncPasteToggleLabel() {
    var btn = $('#togglePaste');
    var hidden = $('#pasteArea').hidden;
    var key = hidden ? 'input.paste' : 'input.pasteHide';
    btn.setAttribute('data-i18n', key);
    btn.textContent = t(key);
  }

  function initInput() {
    var dz = $('#dropzone');
    var input = $('#fileInput');

    dz.addEventListener('click', function () { input.click(); });
    dz.addEventListener('keydown', function (e) {
      if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); input.click(); }
    });
    input.addEventListener('change', function () {
      if (input.files && input.files[0]) readFile(input.files[0]);
      input.value = '';
    });

    ['dragenter', 'dragover'].forEach(function (ev) {
      dz.addEventListener(ev, function (e) { e.preventDefault(); dz.classList.add('is-over'); });
    });
    ['dragleave', 'drop'].forEach(function (ev) {
      dz.addEventListener(ev, function (e) { e.preventDefault(); dz.classList.remove('is-over'); });
    });
    dz.addEventListener('drop', function (e) {
      var f = e.dataTransfer && e.dataTransfer.files && e.dataTransfer.files[0];
      if (f) readFile(f);
    });

    $('#togglePaste').addEventListener('click', function () {
      var area = $('#pasteArea');
      area.hidden = !area.hidden;
      syncPasteToggleLabel();
      if (!area.hidden) $('#urdfText').focus();
    });
    $('#parsePasted').addEventListener('click', function () {
      var v = $('#urdfText').value;
      if (v.trim()) load(v, { kind: 'paste' });
    });
  }

  function readFile(file) {
    var reader = new FileReader();
    reader.onload = function () { load(String(reader.result), { kind: 'file', name: file.name }); };
    reader.onerror = function () {
      showMessages([{ level: 'error', html: esc(String(reader.error || 'read error')) }]);
    };
    reader.readAsText(file);
  }

  /* ── Load & analyse ────────────────────────────────────────────── */
  function load(text, source) {
    state.text = text;
    state.source = source || { kind: 'file', name: 'urdf' };
    state.model = URDF.parseUrdf(text);
    state.tree = state.model.errors.length ? null : URDF.buildTree(state.model);

    // With no root link every tree walk comes back empty, so any order we
    // derived would be meaningless — report the broken tree instead.
    var rootless = state.tree && state.tree.roots.length === 0;

    if (state.model.errors.length || !state.tree || rootless) {
      $('#results').hidden = true;
      $('#fileMeta').hidden = true;
      renderMessages();
      return;
    }
    state.opts.refId = 'urdf';
    $('#results').hidden = false;
    renderAll();
    $('#results').scrollIntoView({ behavior: 'smooth', block: 'start' });
  }

  function recompute() {
    state.analysis = Orderings.analyze(state.model, state.tree, state.opts);
  }

  function renderAll() {
    recompute();
    renderFileMeta();
    renderMessages();
    renderOptions();
    renderVerdict();
    renderTable();
    renderExport();
    renderTreeView();
    renderJointsTable();
    global.I18N.applyStatic();
    syncPasteToggleLabel();
  }

  /* ── Messages ──────────────────────────────────────────────────── */
  function joinNames(list, max) {
    max = max || 6;
    var shown = list.slice(0, max).map(esc).join(', ');
    return list.length > max ? shown + ' … (+' + (list.length - max) + ')' : shown;
  }

  function renderMessages() {
    var msgs = [];
    var m = state.model;

    m.errors.forEach(function (e) {
      if (e.code === 'xmlError') msgs.push({ level: 'error', html: t('msg.xmlError') + esc(e.detail || '') });
      else msgs.push({ level: 'error', html: t('msg.' + e.code) });
    });

    if (m.looksLikeXacro) msgs.push({ level: 'warn', html: t('msg.xacro') });

    if (state.tree) {
      state.tree.warnings.forEach(function (w) {
        if (w.code === 'multiRoot') {
          msgs.push({ level: 'warn', html: t('msg.multiRoot', { n: w.links.length, list: joinNames(w.links) }) });
        } else if (w.code === 'undeclaredLink') {
          msgs.push({ level: 'warn', html: t('msg.undeclaredLink', { n: w.links.length, list: joinNames(w.links) }) });
        } else if (w.code === 'cycle') {
          msgs.push({ level: 'error', html: t('msg.cycle', { n: w.joints.length, list: joinNames(w.joints) }) });
        }
      });

      var mimics = m.joints.filter(function (j) { return j.mimic; });
      if (mimics.length) {
        msgs.push({
          level: 'warn',
          html: t('msg.mimic', { n: mimics.length, list: joinNames(mimics.map(function (j) { return j.name; })) })
        });
      }

      var multi = m.joints.filter(function (j) { return URDF.dofOf(j.type) > 1; });
      if (multi.length) {
        msgs.push({
          level: 'warn',
          html: t('msg.multiDof', {
            list: joinNames(multi.map(function (j) { return j.name + ' (' + j.type + ')'; }))
          })
        });
      }

      var fatal = msgs.some(function (x) { return x.level === 'error'; });
      if (!fatal && !m.ros2ControlJoints.length) msgs.push({ level: 'info', html: t('msg.noRos2Control') });
    }

    showMessages(msgs);
  }

  function showMessages(msgs) {
    var host = $('#messages');
    host.innerHTML = msgs.map(function (msg) {
      var icon = msg.level === 'error' ? '✕' : msg.level === 'warn' ? '!' : 'i';
      return '<div class="msg msg-' + msg.level + '"><span class="msg-icon">' + icon + '</span><span>' + msg.html + '</span></div>';
    }).join('');
  }

  /* Resolved at render time so it follows the language toggle. */
  function sourceLabel() {
    var s = state.source || {};
    if (s.kind === 'paste') return t('input.pasted');
    if (s.kind === 'sample') {
      var found = (global.SAMPLES || []).filter(function (x) { return x.id === s.id; })[0];
      if (found) return found.label[lang()] || found.label.en;
    }
    return s.name || 'urdf';
  }

  function renderFileMeta() {
    var m = state.model;
    var movable = m.joints.filter(URDF.isMovable).length;
    var meta = $('#fileMeta');
    meta.hidden = false;
    meta.innerHTML = esc(sourceLabel()) + ' · ' + t('msg.parsed', {
      name: esc(m.robotName),
      links: m.links.length,
      joints: m.joints.length,
      movable: movable
    });
  }

  /* ── Options ───────────────────────────────────────────────────── */
  var optionsBound = false;

  function renderOptions() {
    var sel = $('#refSelect');
    sel.innerHTML = state.analysis.available.map(function (c) {
      return '<option value="' + c.id + '"' + (state.analysis.reference && c.id === state.analysis.reference.id ? ' selected' : '') + '>' +
             esc(fwLabel(c)) + '</option>';
    }).join('');

    $('#ros2cMode').value = state.opts.ros2cMode;
    $('#siblingMode').value = state.opts.siblingOrder;
    $('#showFixed').checked = state.opts.showFixed;

    if (optionsBound) return;
    optionsBound = true;

    sel.addEventListener('change', function () { state.opts.refId = sel.value; renderAll(); });
    $('#ros2cMode').addEventListener('change', function () { state.opts.ros2cMode = this.value; renderAll(); });
    $('#siblingMode').addEventListener('change', function () { state.opts.siblingOrder = this.value; renderAll(); });
    $('#showFixed').addEventListener('change', function () { state.opts.showFixed = this.checked; renderAll(); });

    document.querySelectorAll('.seg-btn[data-view]').forEach(function (b) {
      b.addEventListener('click', function () {
        state.view = b.getAttribute('data-view');
        document.querySelectorAll('.seg-btn[data-view]').forEach(function (x) {
          x.classList.toggle('is-active', x === b);
        });
        renderTable();
      });
    });

    document.querySelectorAll('.seg-btn[data-lang]').forEach(function (b) {
      b.addEventListener('click', function () {
        state.codeLang = b.getAttribute('data-lang');
        document.querySelectorAll('.seg-btn[data-lang]').forEach(function (x) {
          x.classList.toggle('is-active', x === b);
        });
        renderCode();
      });
    });
  }

  /* ── Verdict ───────────────────────────────────────────────────── */
  function renderVerdict() {
    var a = state.analysis;
    var host = $('#verdict');
    var refLabel = a.reference ? fwLabel(a.reference) : '';
    var cls, mark, title, desc, items = [];

    if (a.verdict === 'ok') {
      cls = 'verdict-ok'; mark = '✓';
      title = t('verdict.ok.title');
      desc = t('verdict.ok.desc', { n: a.reference ? a.reference.names.length : 0 });
    } else if (a.verdict === 'bad') {
      cls = 'verdict-bad'; mark = '✕';
      title = t('verdict.bad.title');
      desc = t('verdict.bad.desc', { n: a.badColumns.length, ref: refLabel });
      items = a.badColumns.concat(a.warnColumns).map(describeColumn);
    } else if (a.verdict === 'warn') {
      cls = 'verdict-warn'; mark = '!';
      title = t('verdict.warn.title');
      desc = t('verdict.warn.desc');
      items = a.warnColumns.map(describeColumn);
    } else {
      cls = 'verdict-warn'; mark = 'i';
      title = t('verdict.warn.title');
      desc = t('verdict.single');
    }

    host.className = 'verdict ' + cls;
    host.innerHTML =
      '<span class="verdict-mark">' + mark + '</span>' +
      '<div><h2>' + esc(title) + '</h2><p>' + desc + '</p>' +
      (items.length ? '<ul class="verdict-list">' + items.map(function (x) { return '<li>' + x + '</li>'; }).join('') + '</ul>' : '') +
      '</div>';
  }

  function describeColumn(col) {
    var parts = [];
    if (!col.cmp.orderMatch) parts.push(t('verdict.diffCount', { n: col.cmp.diffPositions.length }));
    if (col.cmp.missing.length) parts.push(t('verdict.missing', { n: col.cmp.missing.length, list: joinNames(col.cmp.missing, 4) }));
    if (col.cmp.extra.length) parts.push(t('verdict.extra', { n: col.cmp.extra.length, list: joinNames(col.cmp.extra, 4) }));
    return '<b>' + esc(fwLabel(col)) + '</b> — ' + parts.join('；');
  }

  /* ── Comparison table ──────────────────────────────────────────── */
  function badgeFor(col) {
    if (col.status === 'ref') return '<span class="th-badge badge-ok">' + t('chip.ref') + '</span>';
    if (col.status === 'na') {
      return '<span class="th-badge badge-na">' + t('chip.na') +
             (col.naReasonKey ? ' · ' + esc(t(col.naReasonKey)) : '') + '</span>';
    }
    if (col.status === 'ok') return '<span class="th-badge badge-ok">' + t('chip.ok') + '</span>';
    if (col.status === 'warn') return '<span class="th-badge badge-warn">' + t('chip.warn') + '</span>';
    return '<span class="th-badge badge-bad">' + t('chip.bad') + '</span>';
  }

  function headerCells() {
    return state.analysis.columns.map(function (col) {
      var isRef = col.status === 'ref';
      return '<th class="' + (isRef ? 'col-ref' : '') + '">' +
             '<span class="th-name">' + esc(fwLabel(col)) + '</span>' +
             '<span class="th-rule">' + esc(fwRule(col)) + '</span>' +
             badgeFor(col) + '</th>';
    }).join('');
  }

  function renderTable() {
    if (!state.analysis) return;
    $('#tableWrap').innerHTML = state.view === 'byindex' ? tableByIndex() : tableByJoint();
  }

  function tableByIndex() {
    var a = state.analysis;
    var ref = a.reference;
    var rows = '';

    for (var i = 0; i < a.maxLen; i++) {
      var cells = a.columns.map(function (col) {
        if (!col.names) return '<td class="cell cell-na">—</td>';
        var name = col.names[i];
        if (name === undefined) return '<td class="cell cell-na"></td>';

        var joint = col.seq[i];
        var fixedCls = URDF.isMovable(joint) ? '' : ' cell-fixed';
        var cls;
        if (col.status === 'ref') cls = 'cell';
        else if (!ref || ref.names[i] === name) cls = 'cell cell-ok';
        else if (col.status === 'bad') cls = 'cell cell-bad';
        else cls = 'cell cell-shift';

        return '<td class="' + cls + fixedCls + '">' + esc(name) + '</td>';
      }).join('');
      rows += '<tr><td class="col-idx">' + i + '</td>' + cells + '</tr>';
    }

    return '<table class="tbl"><thead><tr><th class="col-idx">' + t('table.idx') + '</th>' +
           headerCells() + '</tr></thead><tbody>' + rows + '</tbody></table>';
  }

  function tableByJoint() {
    var a = state.analysis;
    var m = state.model;
    var rows = a.allNames.map(function (name) {
      var joint = m.jointByName[name];
      var cells = a.columns.map(function (col) {
        if (!col.names) return '<td class="cell cell-na">—</td>';
        var idx = col.names.indexOf(name);
        if (idx === -1) return '<td class="cell cell-na">—</td>';
        var refIdx = a.reference ? a.reference.names.indexOf(name) : -1;
        var cls = 'cell cell-idx';
        if (col.status !== 'ref' && refIdx !== -1 && refIdx !== idx) {
          cls += col.status === 'bad' ? ' cell-bad' : ' cell-shift';
        } else if (col.status !== 'ref') {
          cls += ' cell-ok';
        }
        var delta = (col.status !== 'ref' && refIdx !== -1 && refIdx !== idx)
          ? '<span class="delta">(' + (idx > refIdx ? '+' : '') + (idx - refIdx) + ')</span>' : '';
        return '<td class="' + cls + '">' + idx + delta + '</td>';
      }).join('');
      return '<tr><td class="cell">' + esc(name) + '</td>' +
             '<td class="cell">' + esc(joint ? joint.type : '?') + '</td>' +
             '<td class="col-idx">' + (joint ? URDF.dofOf(joint.type) : '?') + '</td>' + cells + '</tr>';
    }).join('');

    return '<table class="tbl"><thead><tr><th>' + t('table.joint') + '</th><th>' + t('table.type') +
           '</th><th class="col-idx">' + t('table.dof') + '</th>' + headerCells() +
           '</tr></thead><tbody>' + rows + '</tbody></table>';
  }

  /* ── Export / remap ────────────────────────────────────────────── */
  var exportBound = false;

  function renderExport() {
    var a = state.analysis;
    var opts = a.available.map(function (c) {
      return '<option value="' + c.id + '">' + esc(fwLabel(c)) + '</option>';
    }).join('');

    var from = $('#mapFrom'), to = $('#mapTo');
    var prevFrom = from.value, prevTo = to.value;
    from.innerHTML = opts;
    to.innerHTML = opts;

    var ids = a.available.map(function (c) { return c.id; });
    from.value = ids.indexOf(prevFrom) !== -1 ? prevFrom : (ids.indexOf('isaacsim') !== -1 ? 'isaacsim' : ids[0]);
    to.value = ids.indexOf(prevTo) !== -1 ? prevTo : (ids.indexOf('mujoco') !== -1 ? 'mujoco' : ids[ids.length - 1]);

    if (!exportBound) {
      exportBound = true;
      from.addEventListener('change', renderCode);
      to.addEventListener('change', renderCode);
      $('#copyCode').addEventListener('click', copyCode);
      $('#dlCsv').addEventListener('click', downloadCsv);
      $('#dlJson').addEventListener('click', downloadJson);
    }
    renderCode();
  }

  function colById(id) {
    return state.analysis.columns.filter(function (c) { return c.id === id; })[0] || null;
  }

  function buildRemap() {
    var src = colById($('#mapFrom').value);
    var dst = colById($('#mapTo').value);
    if (!src || !dst || !src.names || !dst.names) return null;

    var srcSet = {};
    src.names.forEach(function (n, i) { srcSet[n] = i; });
    var targets = dst.names.filter(function (n) { return srcSet[n] !== undefined; });
    var idx = targets.map(function (n) { return srcSet[n]; });
    var dropped = dst.names.filter(function (n) { return srcSet[n] === undefined; });

    return {
      src: src, dst: dst, targets: targets, idx: idx, dropped: dropped,
      complete: targets.length === src.names.length && targets.length === dst.names.length
    };
  }

  function renderCode() {
    var r = buildRemap();
    var out = $('#codeOut').querySelector('code');
    if (!r) { out.textContent = ''; return; }

    var srcName = fwLabel(r.src), dstName = fwLabel(r.dst);
    var robot = state.model.robotName;
    var txt;

    if (state.codeLang === 'json') {
      var dump = {
        robot: robot,
        generated_by: 'Robot_Joint_Order_Check_Tool',
        options: state.opts,
        orders: {},
        remap: { from: r.src.id, to: r.dst.id, target_joints: r.targets, target_from_source: r.idx }
      };
      state.analysis.columns.forEach(function (c) {
        dump.orders[c.id] = c.names;
      });
      txt = JSON.stringify(dump, null, 2);
    } else if (state.codeLang === 'cpp') {
      txt = '// ' + robot + ':  ' + srcName + '  ->  ' + dstName + '\n' +
            '// dst[i] = src[kTargetFromSource[i]]\n' +
            (r.dropped.length ? '// skipped (absent in source): ' + r.dropped.join(', ') + '\n' : '') +
            'constexpr std::size_t kNumJoints = ' + r.idx.length + ';\n' +
            'constexpr std::array<int, kNumJoints> kTargetFromSource = {\n' +
            wrapList(r.idx.map(String), '    ') + '\n};\n\n' +
            'const std::array<const char*, kNumJoints> kTargetJointNames = {\n' +
            wrapList(r.targets.map(function (n) { return '"' + n + '"'; }), '    ') + '\n};';
    } else {
      txt = '# ' + robot + ':  ' + srcName + '  ->  ' + dstName + '\n' +
            '# q_target = q_source[..., TARGET_FROM_SOURCE]\n' +
            (r.dropped.length ? '# skipped (absent in source): ' + r.dropped.join(', ') + '\n' : '') +
            'SOURCE_JOINTS = [\n' + wrapList(r.src.names.map(quote), '    ') + '\n]\n\n' +
            'TARGET_JOINTS = [\n' + wrapList(r.targets.map(quote), '    ') + '\n]\n\n' +
            'TARGET_FROM_SOURCE = [\n' + wrapList(r.idx.map(String), '    ') + '\n]\n\n' +
            '# numpy / torch:\n' +
            '#   q_target = q_source[..., TARGET_FROM_SOURCE]\n' +
            '# safer, order-independent version:\n' +
            '#   TARGET_FROM_SOURCE = [SOURCE_JOINTS.index(n) for n in TARGET_JOINTS]';
    }
    out.textContent = txt;
  }

  function quote(s) { return "'" + s + "'"; }

  function wrapList(items, indent) {
    var lines = [], cur = indent;
    items.forEach(function (it, i) {
      var piece = it + (i === items.length - 1 ? '' : ', ');
      if (cur.length + piece.length > 92 && cur.trim()) { lines.push(cur.replace(/\s+$/, '')); cur = indent; }
      cur += piece;
    });
    if (cur.trim()) lines.push(cur.replace(/\s+$/, ''));
    return lines.join('\n');
  }

  function copyCode() {
    var txt = $('#codeOut').textContent;
    var btn = $('#copyCode');
    var done = function () {
      btn.textContent = t('export.copied');
      setTimeout(function () { btn.textContent = t('export.copy'); }, 1400);
    };
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(txt).then(done, fallbackCopy);
    } else {
      fallbackCopy();
    }
    function fallbackCopy() {
      var ta = document.createElement('textarea');
      ta.value = txt;
      ta.style.position = 'fixed';
      ta.style.opacity = '0';
      document.body.appendChild(ta);
      ta.select();
      try { document.execCommand('copy'); done(); } catch (e) { /* ignore */ }
      document.body.removeChild(ta);
    }
  }

  function download(name, mime, content) {
    var blob = new Blob([content], { type: mime });
    var url = URL.createObjectURL(blob);
    var a = document.createElement('a');
    a.href = url;
    a.download = name;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    setTimeout(function () { URL.revokeObjectURL(url); }, 0);
  }

  function csvCell(s) {
    s = String(s === undefined || s === null ? '' : s);
    return /[",\n]/.test(s) ? '"' + s.replace(/"/g, '""') + '"' : s;
  }

  function downloadCsv() {
    var a = state.analysis;
    var cols = a.columns.filter(function (c) { return c.names; });
    var lines = [['index'].concat(cols.map(function (c) { return fwLabel(c); })).map(csvCell).join(',')];
    for (var i = 0; i < a.maxLen; i++) {
      lines.push([i].concat(cols.map(function (c) { return c.names[i] || ''; })).map(csvCell).join(','));
    }
    download(state.model.robotName + '_joint_order.csv', 'text/csv;charset=utf-8', '﻿' + lines.join('\n'));
  }

  function downloadJson() {
    var a = state.analysis;
    var dump = {
      robot: state.model.robotName,
      source: sourceLabel(),
      generated_by: 'Robot_Joint_Order_Check_Tool',
      options: state.opts,
      verdict: a.verdict,
      reference: a.reference ? a.reference.id : null,
      joints: state.model.joints.map(function (j) {
        return { name: j.name, type: j.type, dof: URDF.dofOf(j.type), parent: j.parent, child: j.child, mimic: j.mimic };
      }),
      orders: {},
      comparison: {}
    };
    a.columns.forEach(function (c) {
      dump.orders[c.id] = c.names;
      dump.comparison[c.id] = c.cmp
        ? { status: c.status, order_match: c.cmp.orderMatch, missing: c.cmp.missing, extra: c.cmp.extra, diff_positions: c.cmp.diffPositions }
        : { status: c.status };
    });
    download(state.model.robotName + '_joint_order.json', 'application/json', JSON.stringify(dump, null, 2));
  }

  /* ── Tree & joint list ─────────────────────────────────────────── */
  function renderTreeView() {
    var lines = URDF.renderTree(state.model, state.tree, { siblingOrder: state.opts.siblingOrder });
    $('#treeOut').innerHTML = lines.map(function (l) {
      if (l.kind === 'gap') return '';
      if (l.kind === 'link') return '<b>' + esc(l.text) + '</b>';
      var cls = URDF.dofOf(l.type) > 0 ? 't-joint' : 't-fixed';
      return esc(l.text) + '<span class="' + cls + '">' + esc(l.joint) + '</span>' +
             '<span class="t-type"> [' + esc(l.type) + ']</span> → ' + esc(l.child);
    }).join('\n');
  }

  function renderJointsTable() {
    var m = state.model;
    var head = '<thead><tr><th>' + t('table.joint') + '</th><th>' + t('table.type') +
               '</th><th class="col-idx">' + t('table.dof') + '</th><th>' + t('table.chain') + '</th></tr></thead>';
    var body = m.joints.map(function (j) {
      return '<tr><td class="cell">' + esc(j.name) + (j.mimic ? ' <span class="delta">mimic→' + esc(j.mimic) + '</span>' : '') + '</td>' +
             '<td>' + esc(j.type) + '</td>' +
             '<td class="col-idx">' + URDF.dofOf(j.type) + '</td>' +
             '<td class="cell">' + esc(j.parent) + ' → ' + esc(j.child) + '</td></tr>';
    }).join('');
    $('#jointsTable').innerHTML = head + '<tbody>' + body + '</tbody>';
  }

  /* ── Rules reference ───────────────────────────────────────────── */
  function renderRules() {
    var host = $('#rulesList');
    // Driven by FRAMEWORKS so the rule cards always match the table's column order.
    host.innerHTML = Orderings.FRAMEWORKS.map(function (fw) {
      var doc = Orderings.RULE_DOCS.filter(function (d) { return d.id === fw.id; })[0];
      if (!doc) return '';
      var verify = doc.verify
        ? '<div class="rule-verify"><span class="rule-verify-label">' + t('rules.verify') + '</span>' +
          '<pre><code>' + esc(doc.verify) + '</code></pre></div>'
        : '';
      return '<details class="rule"><summary>' + esc(Orderings.labelOf(fw, lang())) +
             ' <span class="rule-key">' + esc(Orderings.ruleOf(fw, lang())) + '</span></summary>' +
             '<div class="rule-body">' + (doc.body[lang()] || doc.body.en) + verify + '</div></details>';
    }).join('');
  }

  /* ── Boot ──────────────────────────────────────────────────────── */
  function init() {
    initTheme();
    initInput();
    initLang();
    global.I18N.applyStatic();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})(window);
