/* MJCF (MuJoCo XML) parsing — produces the same model shape as urdf.js */
(function (global) {
  'use strict';

  var URDF = global.URDF;

  /* The name given to MuJoCo's <worldbody>, so the body tree has a root link
     the shared tree walkers can start from. */
  var WORLD = 'world';

  function directChildren(parent, name) {
    var out = [];
    for (var i = 0; i < parent.children.length; i++) {
      var el = parent.children[i];
      if (el.nodeName === name || el.localName === name) out.push(el);
    }
    return out;
  }

  function tagOf(el) { return el.localName || el.nodeName; }

  function textAttr(el, name) {
    var v = el.getAttribute(name);
    return v === null ? '' : v.trim();
  }

  /**
   * Default classes: <default class="x"><joint type="slide"/></default>.
   * A joint with no explicit type="" inherits it from its class, so the class
   * tree has to be resolved before joint types are known.
   * Returns className -> { parent, jointType }.
   */
  function collectDefaults(root) {
    var classes = Object.create(null);

    function entry(name) {
      if (!classes[name]) classes[name] = { parent: null, jointType: null };
      return classes[name];
    }

    function walk(el, parentName) {
      // The top-level <default> without a class attribute is the "main" class;
      // a top-level <default class="x"> is a child of main.
      var explicit = textAttr(el, 'class');
      var name = explicit || (parentName === null ? 'main' : parentName);
      var e = entry(name);
      if (name !== 'main') e.parent = parentName === null ? 'main' : parentName;

      var jointEl = directChildren(el, 'joint')[0];
      if (jointEl && textAttr(jointEl, 'type')) e.jointType = textAttr(jointEl, 'type');

      directChildren(el, 'default').forEach(function (d) { walk(d, name); });
    }

    directChildren(root, 'default').forEach(function (d) { walk(d, null); });
    return classes;
  }

  /** Walk a class up its parents until one of them sets a joint type. */
  function typeFromClass(classes, name) {
    var guard = Object.create(null);
    var cur = name;
    while (cur && classes[cur] && !guard[cur]) {
      guard[cur] = true;
      if (classes[cur].jointType) return classes[cur].jointType;
      cur = classes[cur].parent;
    }
    return null;
  }

  /**
   * Parse MJCF XML text.
   * Model shape matches parseUrdf(), plus: format, edges, actuators,
   * equalities, includes. `edges` carries the tree edges in document order —
   * both real joints and the weld edges of jointless bodies — while `joints`
   * holds only the joints MuJoCo actually creates, in joint-id order.
   */
  function parseMjcf(text) {
    var model = {
      format: 'mjcf',
      robotName: '',
      links: [],
      joints: [],
      edges: [],
      jointByName: {},
      actuators: [],
      equalities: [],
      includes: [],
      duplicateNames: [],
      ros2ControlJoints: [],
      ros2ControlBlocks: [],
      warnings: [],
      errors: [],
      looksLikeXacro: false
    };

    if (!text || !text.trim()) {
      model.errors.push({ code: 'xmlError', detail: 'empty input' });
      return model;
    }

    var doc;
    try {
      doc = new DOMParser().parseFromString(text, 'application/xml');
    } catch (e) {
      model.errors.push({ code: 'xmlError', detail: String(e && e.message || e) });
      return model;
    }

    var perr = doc.getElementsByTagName('parsererror')[0];
    if (perr) {
      model.errors.push({ code: 'xmlError', detail: perr.textContent.replace(/\s+/g, ' ').trim().slice(0, 240) });
      return model;
    }

    var root = doc.documentElement;
    if (!root || (root.nodeName !== 'mujoco' && root.localName !== 'mujoco')) {
      model.errors.push({ code: 'noRoot' });
      return model;
    }
    model.robotName = textAttr(root, 'model') || 'mujoco';

    /* <include> pulls in bodies and actuators this page cannot fetch. */
    var includes = doc.getElementsByTagName('include');
    for (var ii = 0; ii < includes.length; ii++) {
      model.includes.push(textAttr(includes[ii], 'file') || '?');
    }

    var classes = collectDefaults(root);

    /* MuJoCo rejects duplicate names; if one slips in, keep both entries
       distinguishable rather than silently merging two branches of the tree. */
    var used = Object.create(null);
    function unique(name) {
      if (!used[name]) { used[name] = 1; return name; }
      model.duplicateNames.push(name);
      return name + '#' + (++used[name]);
    }

    var anonBody = 0, anonJoint = 0;

    function pushEdge(edge) {
      edge.mimic = null;
      edge.dof = URDF.dofOf(edge.type);
      model.edges.push(edge);
      if (!edge.weld) {
        edge.docIndex = model.joints.length;
        model.joints.push(edge);
        model.jointByName[edge.name] = edge;
      }
    }

    model.links.push({ name: WORLD, declared: true, world: true });

    function walkBody(el, parentLink, inheritedClass) {
      var childClass = textAttr(el, 'childclass') || inheritedClass;
      var bodyName = unique(textAttr(el, 'name') || ('body_' + (++anonBody)));
      model.links.push({ name: bodyName, declared: true });

      /* <joint> and <freejoint> in a single pass so their document order,
         which is the order MuJoCo numbers them in, is preserved. */
      var jointEls = [];
      for (var i = 0; i < el.children.length; i++) {
        var tag = tagOf(el.children[i]);
        if (tag === 'joint' || tag === 'freejoint') jointEls.push(el.children[i]);
      }

      if (!jointEls.length) {
        // No joint means the body is welded to its parent — MuJoCo creates no
        // joint here, the same way a URDF fixed joint creates none.
        pushEdge({
          name: bodyName, type: 'weld', parent: parentLink, child: bodyName,
          body: bodyName, weld: true
        });
      } else {
        var prev = parentLink;
        jointEls.forEach(function (je, k) {
          var isFree = tagOf(je) === 'freejoint';
          var jname = unique(textAttr(je, 'name') || ('<unnamed_joint_' + (++anonJoint) + '>'));
          var type = isFree
            ? 'free'
            : (textAttr(je, 'type') ||
               typeFromClass(classes, textAttr(je, 'class') || childClass || 'main') ||
               'hinge').toLowerCase();

          // Several joints on one body form a chain of DOFs between the parent
          // and the body; give each but the last a synthetic link so the tree
          // walkers see one joint per edge, as they do for URDF.
          var last = k === jointEls.length - 1;
          var childLink = last ? bodyName : unique(bodyName + '~' + jname);
          if (!last) model.links.push({ name: childLink, declared: true, synthetic: true });

          pushEdge({
            name: jname, type: type, parent: prev, child: childLink,
            body: bodyName, weld: false
          });
          prev = childLink;
        });
      }

      directChildren(el, 'body').forEach(function (b) { walkBody(b, bodyName, childClass); });
    }

    // Every top-level section may appear more than once; MuJoCo merges them in
    // document order (the G1 model ships a second <worldbody> for the scene).
    directChildren(root, 'worldbody').forEach(function (wb) {
      directChildren(wb, 'body').forEach(function (b) { walkBody(b, WORLD, ''); });
    });

    /* <actuator> order is the order of data.ctrl — a vector of its own. */
    directChildren(root, 'actuator').forEach(function (sec) {
      for (var i = 0; i < sec.children.length; i++) {
        var a = sec.children[i];
        var joint = textAttr(a, 'joint') || textAttr(a, 'jointinparent');
        model.actuators.push({
          name: textAttr(a, 'name') || '',
          kind: tagOf(a),
          joint: joint || null
        });
      }
    });

    /* MuJoCo's answer to <mimic>. */
    directChildren(root, 'equality').forEach(function (sec) {
      directChildren(sec, 'joint').forEach(function (e) {
        model.equalities.push({ joint1: textAttr(e, 'joint1'), joint2: textAttr(e, 'joint2') });
      });
    });

    if (!model.joints.length) model.errors.push({ code: 'noJoints' });

    return model;
  }

  global.MJCF = {
    parseMjcf: parseMjcf,
    WORLD: WORLD
  };
})(window);
