/* Built-in sample URDFs (embedded so the page also works from file://) */
(function (global) {
  'use strict';

  function link(name) {
    return '  <link name="' + name + '">\n' +
           '    <inertial><mass value="1.0"/><origin xyz="0 0 0"/>' +
           '<inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>\n' +
           '  </link>\n';
    }

  function joint(name, type, parent, child, xyz, axis) {
    var s = '  <joint name="' + name + '" type="' + type + '">\n' +
            '    <parent link="' + parent + '"/>\n' +
            '    <child link="' + child + '"/>\n' +
            '    <origin xyz="' + (xyz || '0 0 0.1') + '" rpy="0 0 0"/>\n';
    if (type !== 'fixed') {
      s += '    <axis xyz="' + (axis || '0 0 1') + '"/>\n' +
           '    <limit lower="-3.14" upper="3.14" effort="100" velocity="3.0"/>\n';
    }
    return s + '  </joint>\n';
  }

  /* ── 1. 6-DOF serial arm: every framework agrees ── */
  function buildArm() {
    var s = '<?xml version="1.0"?>\n<robot name="simple_arm_6dof">\n' +
            '  <!-- Serial chain, joints declared in tree order and named in\n' +
            '       ascending alphabetical order: every framework agrees. -->\n';
    s += link('base_link');
    for (var i = 1; i <= 6; i++) s += link('link_' + i);
    s += link('tool0');
    var prev = 'base_link';
    for (var j = 1; j <= 6; j++) {
      s += joint('joint_' + j, j === 3 ? 'prismatic' : 'revolute', prev, 'link_' + j,
                 '0 0 0.2', j % 2 ? '0 0 1' : '0 1 0');
      prev = 'link_' + j;
    }
    s += joint('tool_fixed_joint', 'fixed', 'link_6', 'tool0', '0 0 0.05');
    return s + '</robot>\n';
  }

  /* ── 2. Quadruped: the canonical DFS vs BFS mismatch ── */
  function buildQuadruped() {
    var legs = ['FL', 'FR', 'RL', 'RR'];
    var s = '<?xml version="1.0"?>\n<robot name="quadruped_12dof">\n' +
            '  <!-- Four identical legs branching off one trunk.\n' +
            '       MuJoCo / Isaac Gym walk depth-first  -> FL_hip, FL_thigh, FL_calf, FR_hip, ...\n' +
            '       Isaac Sim / Isaac Lab walk breadth-first -> FL_hip, FR_hip, RL_hip, RR_hip, ...\n' +
            '       urdf::Model sorts alphabetically      -> FL_calf, FL_hip, FL_thigh, FR_calf, ... -->\n';
    s += link('trunk');
    legs.forEach(function (p) {
      s += link(p + '_hip') + link(p + '_thigh') + link(p + '_calf') + link(p + '_foot');
    });
    legs.forEach(function (p) {
      var x = (p[0] === 'F' ? '0.18' : '-0.18');
      var y = (p[1] === 'L' ? '0.05' : '-0.05');
      s += joint(p + '_hip_joint', 'revolute', 'trunk', p + '_hip', x + ' ' + y + ' 0', '1 0 0');
      s += joint(p + '_thigh_joint', 'revolute', p + '_hip', p + '_thigh', '0 ' + y + ' 0', '0 1 0');
      s += joint(p + '_calf_joint', 'revolute', p + '_thigh', p + '_calf', '0 0 -0.21', '0 1 0');
      s += joint(p + '_foot_fixed', 'fixed', p + '_calf', p + '_foot', '0 0 -0.21');
    });
    return s + '</robot>\n';
  }

  /* ── 3. Mobile manipulator with a <ros2_control> tag in a different order ── */
  function buildRos2Control() {
    var s = '<?xml version="1.0"?>\n<robot name="mobile_manipulator">\n' +
            '  <!-- The <ros2_control> tag lists joints in a different order than the\n' +
            '       <joint> elements, and the arm joints are named so that alphabetical\n' +
            '       sorting reshuffles them too. Also contains a mimic gripper. -->\n';
    ['base_link', 'wheel_left', 'wheel_right', 'shoulder', 'upper_arm', 'forearm',
     'wrist', 'gripper_base', 'finger_left', 'finger_right'].forEach(function (n) {
      s += link(n);
    });

    s += joint('drive_wheel_left', 'continuous', 'base_link', 'wheel_left', '0 0.2 0.05', '0 1 0');
    s += joint('drive_wheel_right', 'continuous', 'base_link', 'wheel_right', '0 -0.2 0.05', '0 1 0');
    s += joint('arm_shoulder_pan', 'revolute', 'base_link', 'shoulder', '0 0 0.3', '0 0 1');
    s += joint('arm_shoulder_lift', 'revolute', 'shoulder', 'upper_arm', '0 0 0.1', '0 1 0');
    s += joint('arm_elbow', 'revolute', 'upper_arm', 'forearm', '0 0 0.35', '0 1 0');
    s += joint('arm_wrist', 'revolute', 'forearm', 'wrist', '0 0 0.3', '0 1 0');
    s += joint('gripper_mount', 'fixed', 'wrist', 'gripper_base', '0 0 0.06');
    s += joint('gripper_finger_left', 'prismatic', 'gripper_base', 'finger_left', '0 0.02 0.02', '0 1 0');
    s += '  <joint name="gripper_finger_right" type="prismatic">\n' +
         '    <parent link="gripper_base"/>\n' +
         '    <child link="finger_right"/>\n' +
         '    <origin xyz="0 -0.02 0.02" rpy="0 0 0"/>\n' +
         '    <axis xyz="0 1 0"/>\n' +
         '    <limit lower="-0.04" upper="0" effort="20" velocity="0.2"/>\n' +
         '    <mimic joint="gripper_finger_left" multiplier="-1" offset="0"/>\n' +
         '  </joint>\n';

    s += '\n  <ros2_control name="MobileManipulatorSystem" type="system">\n' +
         '    <hardware>\n' +
         '      <plugin>mock_components/GenericSystem</plugin>\n' +
         '    </hardware>\n';
    ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow', 'arm_wrist',
     'gripper_finger_left', 'drive_wheel_left', 'drive_wheel_right'].forEach(function (n) {
      s += '    <joint name="' + n + '">\n' +
           '      <command_interface name="position"/>\n' +
           '      <state_interface name="position"/>\n' +
           '      <state_interface name="velocity"/>\n' +
           '    </joint>\n';
    });
    s += '  </ros2_control>\n';
    return s + '</robot>\n';
  }

  global.SAMPLES = [
    {
      id: 'arm',
      label: { zh: '6 轴机械臂（全部一致）', en: '6-DOF arm (all consistent)' },
      text: buildArm()
    },
    {
      id: 'quadruped',
      label: { zh: '四足机器人（DFS / BFS 冲突）', en: 'Quadruped (DFS vs BFS)' },
      text: buildQuadruped()
    },
    {
      id: 'ros2control',
      label: { zh: '移动机械臂（含 ros2_control）', en: 'Mobile manipulator (ros2_control)' },
      text: buildRos2Control()
    }
  ];
})(window);
