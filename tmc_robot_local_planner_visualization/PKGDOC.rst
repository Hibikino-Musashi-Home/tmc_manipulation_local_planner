=====================
Overview
=====================

提供機能
---------
* tmc_planning_msgs::msg::Constraintsを可視化するノード
* 生成された軌道候補群を可視化するrvizプラグイン
* Robot Local Plannerのrosbagを読み込み，再生するスクリプト


================================
link_constraints_visualization
================================

概要
++++++++++++++
- 購読した拘束条件をマーカーに変換して発行する
- goal拘束条件はXYZをRGBで表す座標軸+原点にCube，path拘束条件は前記座標軸として表現される

ROS Interface
++++++++++++++

Subscribed Topics
^^^^^^^^^^^^^^^^^
- **~/constraints** (:ros:msg:`tmc_planning_msgs/msg/Constraints`) 可視化対象の拘束条件
- **~/robot_local_goal** (:ros:msg:`tmc_planning_msgs/msg/RobotLocalGoal`) 可視化対象のRobotLocalPlannerの目標拘束条件

Published Topics
^^^^^^^^^^^^^^^^
- **~/link_constraints_marker_array** (:ros:msg:`visualization_msgs/msg/MarkerArray`) 手先位置/姿勢拘束条件を可視化したマーカー

Parameter
^^^^^^^^^
hardは，満たさなければならない条件，softは，満たしていたほうが良いとみなすされる条件を表している

- **marker.hard.line_length** (double: 0.2) hard条件の，手先位置/姿勢を示す座標軸マーカーの長さ[m]
- **marker.hard.line_width** (double: 0.02) hard条件の，手先位置/姿勢を示す座標軸マーカーの太さ[m]
- **marker.hard.alpha** (double: 1.0) hard条件の，手先位置/姿勢を示す座標軸マーカーのアルファ値
- **marker.soft.line_length** (double: 0.2) soft条件の，手先位置/姿勢を示す座標軸マーカーの長さ[m]
- **marker.soft.line_width** (double: 0.04) soft条件の，手先位置/姿勢を示す座標軸マーカーの太さ[m]
- **marker.soft.alpha** (double: 0.5) soft条件の，手先位置/姿勢を示す座標軸マーカーのアルファ値
- **marker.life_time** (int: 0) マーカーの生存期間[sec]，0は無限に生存を意味する


====================================================================
tmc_robot_local_planner_visualization/GeneratedTrajectoriesDisplay
====================================================================

概要
++++++++++++++
- 軌道候補群を可視化するrvizプラグイン

制約
++++++++++++++
- 軌道候補群のロボット位置/姿勢軌道は，rvizのFixed frameと同じでなければならない
- rvizがrobot_descriptionをパラメータとして持っていなければならない

使い方
++++++++++++++
- Robot Local Plannerノードのパラメータpublish_generated_trajectoriesをtrueにして起動する
- rvizを起動し，DisplaysのAddボタンで追加する
- Topic欄でトピック名を指定する
- Robot Local Plannerに指令値を投げる


====================================================================
rosbag_player.py
====================================================================

概要
++++++++++++++
- Robot Local Plannerのrosbagを読み込み，再生する

rosbagが収集すべきtopic
+++++++++++++++++++++++
.. code-block:: bash

   $ ros2 bag record /attached_object_publisher/marker_array /collision_environment_server/marker_array /hsrb_robot_local_planner/constraints /hsrb_robot_local_planner/generated_trajectories /omni_base_controller/state /joint_states

使い方
++++++++++++++
可視化用のlaunchファイルを起動する．ロボットを変更したい場合は，適当に書き換えること．

.. code-block:: bash

   $ ros2 run tmc_robot_local_planner_visualization rosbag_player.py ARGS


ARGSは次の通り．

- **--start-offset START_OFFSET** rosbag開始時刻を基準とする，再生開始時刻[sec]
- **--rate RATE** 再生速度，1.0で等速，2.0で倍速
- **--tmc-planning-msgs-share-dir TMC_PLANNING_MSGS_SHARE_DIR** tmc_planning_msgsのshareのディレクトリのパス，--symlink-installの場合，install/tmc_planning_msgs/share/
- **--publish-generated-trajectories** 軌道候補群の読み込みと可視化を行うフラグ，軌道候補群は重いデータで読み込み時間が長くなるので注意
- **--start-paused** 読み込み完了後，自動で再生を行わないフラグ
- **--rosbag2_dir** rosbagのディレクトリのパス

.. code-block:: bash

   usage: rosbag_player.py [-h] [--start-offset START_OFFSET] [--rate RATE] [--tmc-planning-msgs-share-dir TMC_PLANNING_MSGS_SHARE_DIR]
                           [--publish-generated-trajectories] [--start-paused]
                           rosbag2_dir

再生中のキーボード割り込みは次の通り．

- s: 一時停止
- r: 一時停止の解除
- q: 再生終了
- g: 最新の軌道候補群を再発行 (一時停止中 かつ **--publish-generated-trajectories** のときのみ動作)
