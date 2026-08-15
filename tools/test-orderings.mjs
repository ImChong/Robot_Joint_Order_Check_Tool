/**
 * Headless checks for every framework column and its ordering rule.
 * No extra packages: the IIFE scripts are eval'd against a stub `window`,
 * and the kinematic trees are built by hand so we do not need a DOMParser.
 *
 *   node tools/test-orderings.mjs
 */
import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const window = {};
for (const file of ['assets/js/urdf.js', 'assets/js/orderings.js']) {
  new Function('window', readFileSync(resolve(root, file), 'utf8'))(window);
}

const { URDF, Orderings } = window;
const out = [];
let failed = 0;

function eq(actual, expected, msg) {
  const a = JSON.stringify(actual);
  const e = JSON.stringify(expected);
  if (a === e) {
    out.push('ok   ' + msg);
  } else {
    failed++;
    out.push('FAIL ' + msg);
    out.push('     expected: ' + e);
    out.push('     actual:   ' + a);
  }
}

function namesOf(col) { return col.names; }

function col(analysis, id) {
  const found = analysis.columns.find((c) => c.id === id);
  if (!found) throw new Error('missing column ' + id);
  return found;
}

function J(name, type, parent, child) {
  return { name, type, parent, child, mimic: null };
}

function modelFromJoints(joints, extra) {
  const m = {
    format: 'urdf',
    robotName: 'test',
    links: [],
    joints,
    jointByName: {},
    ros2ControlJoints: [],
    ros2ControlBlocks: [],
    warnings: [],
    errors: [],
    looksLikeXacro: false
  };
  const links = new Set();
  joints.forEach((j) => {
    m.jointByName[j.name] = j;
    links.add(j.parent);
    links.add(j.child);
  });
  m.links = [...links].map((name) => ({ name, declared: true }));
  return Object.assign(m, extra || {});
}

/* Quadruped: trunk → four legs hip/thigh/calf/foot, joints in FL,FR,RL,RR order. */
function quadruped() {
  const legs = ['FL', 'FR', 'RL', 'RR'];
  const joints = [];
  legs.forEach((p) => {
    joints.push(J(p + '_hip_joint', 'revolute', 'trunk', p + '_hip'));
    joints.push(J(p + '_thigh_joint', 'revolute', p + '_hip', p + '_thigh'));
    joints.push(J(p + '_calf_joint', 'revolute', p + '_thigh', p + '_calf'));
    joints.push(J(p + '_foot_fixed', 'fixed', p + '_calf', p + '_foot'));
  });
  return modelFromJoints(joints);
}

/* Mobile manipulator: wheels declared before the arm; arm_* sorts first alphabetically. */
function mobile() {
  const joints = [
    J('drive_wheel_left', 'continuous', 'base_link', 'wheel_left'),
    J('drive_wheel_right', 'continuous', 'base_link', 'wheel_right'),
    J('arm_shoulder_pan', 'revolute', 'base_link', 'shoulder'),
    J('arm_shoulder_lift', 'revolute', 'shoulder', 'upper_arm'),
    J('arm_elbow', 'revolute', 'upper_arm', 'forearm'),
    J('arm_wrist', 'revolute', 'forearm', 'wrist'),
    J('gripper_mount', 'fixed', 'wrist', 'gripper_base'),
    J('gripper_finger_left', 'prismatic', 'gripper_base', 'finger_left'),
    J('gripper_finger_right', 'prismatic', 'gripper_base', 'finger_right')
  ];
  return modelFromJoints(joints, {
    ros2ControlJoints: [
      'arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow', 'arm_wrist',
      'gripper_finger_left', 'drive_wheel_left', 'drive_wheel_right'
    ]
  });
}

function serialArm() {
  const joints = [];
  let prev = 'base_link';
  for (let i = 1; i <= 6; i++) {
    joints.push(J('joint_' + i, i === 3 ? 'prismatic' : 'revolute', prev, 'link_' + i));
    prev = 'link_' + i;
  }
  joints.push(J('tool_fixed_joint', 'fixed', 'link_6', 'tool0'));
  return modelFromJoints(joints);
}

function run(model, opts) {
  return Orderings.analyze(model, URDF.buildTree(model), opts || {});
}

const ids = Orderings.FRAMEWORKS.map((fw) => fw.id);
eq(ids.indexOf('genesis'), ids.indexOf('mujoco') + 1, 'Genesis sits immediately after MuJoCo');
eq(ids.indexOf('newton'), ids.indexOf('genesis') + 1, 'Newton sits immediately after Genesis');
eq(ids.indexOf('gazebo'), ids.indexOf('newton') + 1, 'Gazebo sits immediately after Newton');
eq(ids.indexOf('pybullet'), ids.indexOf('gazebo') + 1, 'PyBullet sits immediately after Gazebo');
eq(ids.indexOf('ros2control'), ids.indexOf('pybullet') + 1, 'ros2_control sits immediately after PyBullet');

const py = Orderings.FRAMEWORKS.find((fw) => fw.id === 'pybullet');
eq(!!py.holdsFixed, true, 'PyBullet holds fixed joints');
eq(!!py.holdsFreeBase, false, 'PyBullet does not list the floating base as a joint');
eq(py.siblingOrder, 'document', 'PyBullet siblings follow document order');

const gen = Orderings.FRAMEWORKS.find((fw) => fw.id === 'genesis');
eq(gen.holdsFixed({ model: { format: 'urdf' } }), false,
  'Genesis merges fixed links away for a URDF (merge_fixed_links=True)');
eq(gen.holdsFixed({ model: { format: 'mjcf' } }), true,
  'Genesis keeps a jointless MJCF body as a fixed joint');
eq(!!gen.holdsFreeBase, true, 'Genesis lists the floating base as a joint');
eq(gen.siblingOrder, 'document', 'Genesis siblings follow document order');

const nt = Orderings.FRAMEWORKS.find((fw) => fw.id === 'newton');
eq(!!nt.holdsFixed, true, 'Newton holds fixed joints (collapse_fixed_joints=False)');
eq(!!nt.holdsFreeBase, true, 'Newton lists the floating base as a joint');
eq(nt.siblingOrder, 'document', 'Newton siblings follow document order');

const quad = run(quadruped(), { showFixed: false });
eq(namesOf(col(quad, 'pybullet')), namesOf(col(quad, 'isaacgym')),
  'quadruped: PyBullet movable order matches Isaac Gym DFS');
eq(namesOf(col(quad, 'pybullet')), [
  'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
  'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
  'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
  'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
], 'quadruped: PyBullet DFS walks each leg to the end');
eq(
  namesOf(col(quad, 'pybullet'))[0] === namesOf(col(quad, 'isaacsim'))[0] &&
  namesOf(col(quad, 'pybullet'))[1] !== namesOf(col(quad, 'isaacsim'))[1],
  true,
  'quadruped: PyBullet DFS differs from Isaac Sim BFS after the first hip'
);

eq(namesOf(col(quad, 'genesis')), namesOf(col(quad, 'mujoco')),
  'quadruped: Genesis follows the MuJoCo body-tree DFS');
eq(namesOf(col(quad, 'newton')), namesOf(col(quad, 'mujoco')),
  'quadruped: Newton DFS matches MuJoCo on movable joints');

const quadFixed = run(quadruped(), { showFixed: true });
eq(namesOf(col(quadFixed, 'newton')), namesOf(col(quadFixed, 'pybullet')),
  'quadruped: Newton keeps fixed joints in the vector, like PyBullet');
eq(namesOf(col(quadFixed, 'genesis')).indexOf('FL_foot_fixed'), -1,
  'quadruped: Genesis drops URDF fixed joints even when the toggle is on');
eq(namesOf(col(quadFixed, 'pybullet')), [
  'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'FL_foot_fixed',
  'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'FR_foot_fixed',
  'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', 'RL_foot_fixed',
  'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', 'RR_foot_fixed'
], 'quadruped: showing fixed joints inserts foot_fixed into the PyBullet vector');
eq(namesOf(col(quadFixed, 'isaacgym')).indexOf('FL_foot_fixed'), -1,
  'quadruped: Isaac Gym still omits fixed joints when the toggle is on');

const arm = run(serialArm(), { showFixed: false });
eq(namesOf(col(arm, 'pybullet')), namesOf(col(arm, 'gazebo')),
  'serial arm: PyBullet matches Gazebo (no sibling branching)');
const armFixed = run(serialArm(), { showFixed: true });
eq(
  namesOf(col(armFixed, 'pybullet'))[namesOf(col(armFixed, 'pybullet')).length - 1],
  'tool_fixed_joint',
  'serial arm: PyBullet keeps tool_fixed_joint at the end when showing fixed'
);

const mob = run(mobile(), { showFixed: false });
eq(namesOf(col(mob, 'pybullet')), [
  'drive_wheel_left', 'drive_wheel_right',
  'arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow', 'arm_wrist',
  'gripper_finger_left', 'gripper_finger_right'
], 'mobile arm: PyBullet DFS keeps URDF sibling order (wheels then arm)');
eq(namesOf(col(mob, 'gazebo'))[0], 'arm_shoulder_pan',
  'mobile arm: Gazebo alphabetical siblings put the arm first');
eq(namesOf(col(mob, 'pybullet'))[0] !== namesOf(col(mob, 'gazebo'))[0], true,
  'mobile arm: PyBullet order differs from Gazebo');
eq(namesOf(col(mob, 'genesis')), namesOf(col(mob, 'pybullet')),
  'mobile arm: Genesis keeps URDF sibling order, unlike Gazebo');
eq(namesOf(col(mob, 'newton')), namesOf(col(mob, 'pybullet')),
  'mobile arm: Newton sorts siblings by joint id, i.e. document order');

const mobFixed = run(mobile(), { showFixed: true });
eq(namesOf(col(mobFixed, 'pybullet')).includes('gripper_mount'), true,
  'mobile arm: PyBullet lists gripper_mount when showing fixed joints');

eq(Orderings.FRAMEWORKS.every((fw) => Orderings.RULE_DOCS.some((d) => d.id === fw.id)), true,
  'every framework column has a RULE_DOCS entry');

/* Root-node quaternion: every card documents one, in both languages. */
const QUAT_ORDERS = ['wxyz', 'xyzw', 'mixed'];
eq(Orderings.RULE_DOCS.filter((d) => !d.quat).map((d) => d.id), [],
  'every rule card documents the root-node quaternion');
eq(Orderings.RULE_DOCS.filter((d) => !QUAT_ORDERS.includes(d.quat.order)).map((d) => d.id), [],
  'every quaternion order is one of ' + QUAT_ORDERS.join(' / '));
eq(Orderings.RULE_DOCS.filter((d) => !d.quat.code || !d.quat.note.zh || !d.quat.note.en).map((d) => d.id), [],
  'every quaternion entry carries a retrieval command and a bilingual note');
/* 'mixed' is for the columns whose answer needs more than a component list —
   those must spell the chip out themselves, the other two must not. */
eq(Orderings.RULE_DOCS.filter((d) => (d.quat.order === 'mixed') !== !!d.quat.chip).map((d) => d.id), [],
  'only the mixed columns override the chip text');
eq(['zh', 'en'].every((l) => Orderings.RULE_DOCS.every((d) => Orderings.quatChipOf(d, l))), true,
  'every quaternion chip resolves to text in both languages');

eq(Orderings.RULE_DOCS.filter((d) => d.quat.order === 'wxyz').map((d) => d.id),
  ['mujoco', 'genesis', 'mjcfctrl'], 'scalar-first (w, x, y, z) columns');
eq(Orderings.RULE_DOCS.filter((d) => d.quat.order === 'xyzw').map((d) => d.id),
  ['isaacgym', 'newton', 'gazebo', 'pybullet', 'ros2control'], 'scalar-last (x, y, z, w) columns');
eq(Orderings.RULE_DOCS.filter((d) => d.quat.order === 'mixed').map((d) => d.id),
  ['file', 'isaacsim'], 'columns whose quaternion order needs a caveat');
/* MuJoCo, Genesis and Newton share one joint order but not one quaternion —
   the pairing this section exists to make visible. */
const quatOf = (id) => Orderings.RULE_DOCS.find((d) => d.id === id).quat.order;
eq(quatOf('genesis') === quatOf('mujoco') && quatOf('newton') !== quatOf('mujoco'), true,
  'Newton breaks with MuJoCo / Genesis on quaternion order despite matching joint order');
eq(Orderings.quatChipOf({ quat: { order: 'xyzw' } }, 'zh'), 'x, y, z, w',
  'a chip with no override falls back to the component list');

/* MJCF weld edges must not leak into the PyBullet column. */
const mjcfJoints = [
  J('root', 'free', 'world', 'torso'),
  J('FL_hip', 'hinge', 'torso', 'FL_hip'),
  J('FL_thigh', 'hinge', 'FL_hip', 'FL_thigh'),
  J('FR_hip', 'hinge', 'torso', 'FR_hip')
];
const weld = { name: 'scene', type: 'weld', parent: 'world', child: 'floor', weld: true };
const mjcfModel = modelFromJoints(mjcfJoints, {
  format: 'mjcf',
  edges: mjcfJoints.concat([weld]),
  links: [
    { name: 'world', world: true },
    { name: 'torso' }, { name: 'FL_hip' }, { name: 'FL_thigh' },
    { name: 'FR_hip' }, { name: 'floor' }
  ]
});
const mjcfA = run(mjcfModel, { showFixed: true });
eq(col(mjcfA, 'pybullet').names !== null, true, 'MJCF: PyBullet column is applicable');
eq(col(mjcfA, 'gazebo').names, null, 'MJCF: Gazebo column stays n/a');
eq(namesOf(col(mjcfA, 'pybullet')).includes('scene'), false,
  'MJCF: PyBullet drops weld edges even when showing fixed joints');
eq(namesOf(col(mjcfA, 'pybullet')).includes('root'), false,
  'MJCF: PyBullet drops the floating-base free joint');
eq(namesOf(col(mjcfA, 'pybullet')), ['FL_hip', 'FL_thigh', 'FR_hip'],
  'MJCF: PyBullet DFS matches the body tree without the free base');

/* Genesis / Newton keep the MJCF free base and the welded bodies. */
eq(namesOf(col(mjcfA, 'newton')).includes('root'), true,
  'MJCF: Newton lists the free base joint');
eq(namesOf(col(mjcfA, 'newton')).includes('scene'), true,
  'MJCF: Newton turns a jointless body into a fixed joint of its own');
eq(namesOf(col(mjcfA, 'genesis')).includes('scene'), true,
  'MJCF: Genesis does the same, naming that fixed joint after the body');

const mjcfMovable = run(mjcfModel, { showFixed: false });
eq(namesOf(col(mjcfMovable, 'newton')).includes('scene'), false,
  'MJCF: the weld disappears from Newton once fixed joints are hidden');
eq(namesOf(col(mjcfMovable, 'genesis')), ['root', 'FL_hip', 'FL_thigh', 'FR_hip'],
  'MJCF: Genesis movable order keeps the free base at the head');
eq(namesOf(col(mjcfMovable, 'genesis')), namesOf(col(mjcfMovable, 'mujoco')),
  'MJCF: Genesis matches the MuJoCo joint order');
eq(namesOf(col(mjcfMovable, 'newton')), namesOf(col(mjcfMovable, 'mujoco')),
  'MJCF: Newton matches it too once the welds are hidden');

out.push('');
out.push(failed ? failed + ' failed' : 'ALL PASSED');
process.stdout.write(out.join('\n') + '\n');
if (failed) process.exit(1);
