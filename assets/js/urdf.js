/* URDF parsing + kinematic tree construction (no dependencies) */
(function (global) {
  'use strict';

  /* Degrees of freedom contributed by each joint type, URDF names first, then
     the MJCF ones (mjcf.js builds the same model shape).
     planar = 2 translations + 1 rotation in the plane; floating/free = 6-DOF
     (a free joint spans 7 qpos but 6 qvel — this is the qvel/DOF count).
     `weld` is not a real element: mjcf.js uses it for a body with no joint,
     the MJCF equivalent of a URDF fixed joint. */
  var DOF_BY_TYPE = {
    revolute: 1,
    continuous: 1,
    prismatic: 1,
    fixed: 0,
    floating: 6,
    planar: 3,
    hinge: 1,
    slide: 1,
    ball: 3,
    free: 6,
    weld: 0
  };

  function dofOf(type) {
    return Object.prototype.hasOwnProperty.call(DOF_BY_TYPE, type) ? DOF_BY_TYPE[type] : 1;
  }

  function isMovable(joint) {
    return dofOf(joint.type) > 0;
  }

  /* Direct element children of `parent` whose tag name matches `name`.
     Must stay direct-only: <ros2_control> and <transmission> also contain <joint>. */
  function directChildren(parent, name) {
    var out = [];
    for (var i = 0; i < parent.children.length; i++) {
      var el = parent.children[i];
      if (el.nodeName === name || el.localName === name) out.push(el);
    }
    return out;
  }

  function textAttr(el, name) {
    var v = el.getAttribute(name);
    return v === null ? '' : v.trim();
  }

  /**
   * Parse URDF XML text.
   * @returns {{robotName, links, joints, jointByName, ros2ControlJoints,
   *            ros2ControlBlocks, warnings, errors, looksLikeXacro}}
   */
  function parseUrdf(text) {
    var model = {
      format: 'urdf',
      robotName: '',
      links: [],
      joints: [],
      jointByName: {},
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

    model.looksLikeXacro = /<\s*xacro:/i.test(text) || /\$\{[^}]*\}/.test(text);

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

    var robot = doc.documentElement;
    if (!robot || (robot.nodeName !== 'robot' && robot.localName !== 'robot')) {
      model.errors.push({ code: 'noRobot' });
      return model;
    }
    model.robotName = textAttr(robot, 'name') || 'robot';

    /* ── links ── */
    directChildren(robot, 'link').forEach(function (el) {
      var name = textAttr(el, 'name');
      if (name) model.links.push({ name: name, declared: true });
    });

    /* ── joints ── */
    directChildren(robot, 'joint').forEach(function (el, i) {
      var name = textAttr(el, 'name');
      if (!name) name = '<unnamed_joint_' + i + '>';
      var type = (textAttr(el, 'type') || 'revolute').toLowerCase();
      var parentEl = directChildren(el, 'parent')[0];
      var childEl = directChildren(el, 'child')[0];
      var mimicEl = directChildren(el, 'mimic')[0];

      var joint = {
        name: name,
        type: type,
        parent: parentEl ? textAttr(parentEl, 'link') : '',
        child: childEl ? textAttr(childEl, 'link') : '',
        docIndex: i,
        dof: dofOf(type),
        mimic: mimicEl ? textAttr(mimicEl, 'joint') : null
      };
      model.joints.push(joint);
      model.jointByName[name] = joint;
    });

    /* ── <ros2_control> blocks (joints listed inside, in document order) ── */
    directChildren(robot, 'ros2_control').forEach(function (block) {
      var entry = { name: textAttr(block, 'name') || 'ros2_control', type: textAttr(block, 'type'), joints: [] };
      directChildren(block, 'joint').forEach(function (j) {
        var n = textAttr(j, 'name');
        if (!n) return;
        entry.joints.push(n);
        if (model.ros2ControlJoints.indexOf(n) === -1) model.ros2ControlJoints.push(n);
      });
      model.ros2ControlBlocks.push(entry);
    });

    if (!model.joints.length) model.errors.push({ code: 'noJoints' });

    return model;
  }

  /**
   * Build the kinematic tree.
   * childJoints maps a link name -> array of outgoing joints in document order.
   * Walks model.edges when present (MJCF adds weld edges for jointless bodies,
   * which carry no DOF but still hold the body tree together); for URDF the
   * edges are exactly the joints.
   */
  function buildTree(model) {
    var edges = model.edges || model.joints;

    var declared = {};
    model.links.forEach(function (l) { declared[l.name] = true; });

    var childJoints = {};   // parent link -> [joint]
    var parentJoint = {};   // child link  -> joint
    var seenLinks = [];     // link names in first-appearance order
    var undeclared = [];

    function touch(name) {
      if (!name) return;
      if (seenLinks.indexOf(name) === -1) seenLinks.push(name);
      if (!declared[name] && undeclared.indexOf(name) === -1) undeclared.push(name);
    }

    model.links.forEach(function (l) { touch(l.name); });

    edges.forEach(function (j) {
      touch(j.parent);
      touch(j.child);
      if (!childJoints[j.parent]) childJoints[j.parent] = [];
      childJoints[j.parent].push(j);
      // A link with two parent joints is a closed loop; keep the first.
      if (j.child && !parentJoint[j.child]) parentJoint[j.child] = j;
    });

    var roots = seenLinks.filter(function (n) { return !parentJoint[n]; });

    var depth = {};
    roots.forEach(function (r) { depth[r] = 0; });

    var tree = {
      roots: roots,
      childJoints: childJoints,
      parentJoint: parentJoint,
      links: seenLinks,
      undeclaredLinks: undeclared,
      depth: depth,
      warnings: []
    };

    if (roots.length === 0) {
      tree.warnings.push({ code: 'cycle', joints: model.joints.map(function (j) { return j.name; }) });
      return tree;
    }
    if (roots.length > 1) {
      tree.warnings.push({ code: 'multiRoot', links: roots.slice() });
    }
    if (undeclared.length) {
      tree.warnings.push({ code: 'undeclaredLink', links: undeclared.slice() });
    }

    /* depth via BFS, and detection of joints unreachable from any root */
    var reached = {};
    var queue = roots.slice();
    roots.forEach(function (r) { reached[r] = true; });
    var reachedJoints = {};
    while (queue.length) {
      var link = queue.shift();
      (childJoints[link] || []).forEach(function (j) {
        reachedJoints[j.name] = true;
        if (!reached[j.child]) {
          reached[j.child] = true;
          depth[j.child] = (depth[link] || 0) + 1;
          queue.push(j.child);
        }
      });
    }
    var orphans = model.joints.filter(function (j) { return !reachedJoints[j.name]; });
    if (orphans.length) {
      tree.warnings.push({ code: 'cycle', joints: orphans.map(function (j) { return j.name; }) });
    }

    return tree;
  }

  /** ASCII rendering of the kinematic tree. */
  function renderTree(model, tree, opts) {
    opts = opts || {};
    var lines = [];
    var visited = {};

    function walk(link, prefix, isLast, isRoot) {
      if (isRoot) {
        lines.push({ text: link, kind: 'link', depth: 0 });
      }
      if (visited[link]) return;
      visited[link] = true;

      var kids = (tree.childJoints[link] || []).slice();
      if (opts.siblingOrder === 'alphabetical') {
        kids.sort(function (a, b) { return a.name < b.name ? -1 : a.name > b.name ? 1 : 0; });
      }
      kids.forEach(function (j, i) {
        var last = i === kids.length - 1;
        var branch = prefix + (last ? '└─ ' : '├─ ');
        lines.push({
          text: branch,
          joint: j.name,
          type: j.type,
          child: j.child,
          kind: 'joint'
        });
        walk(j.child, prefix + (last ? '   ' : '│  '), last, false);
      });
    }

    tree.roots.forEach(function (r, i) {
      if (i > 0) lines.push({ text: '', kind: 'gap' });
      walk(r, '', true, true);
    });

    return lines;
  }

  global.URDF = {
    parseUrdf: parseUrdf,
    buildTree: buildTree,
    renderTree: renderTree,
    dofOf: dofOf,
    isMovable: isMovable,
    DOF_BY_TYPE: DOF_BY_TYPE
  };
})(window);
