# tmc_simple_path_generator

軌道候補を生成するgenerator

提供するアクション
-----------------

- ~/generate(tmc_planning_msgs/action/GenerateRobotTrajectories)

利用するパラメータ
-----------------

- robot_description_kinematics(string):
  ロボットモデル
  FK/IK計算，関節角度リミット読み込み等で利用される
  デフォルトは空文字列

- robot_description(string):
  ロボットモデル
  robot_description_kinematicsが空の場合，こちらのロボットモデルを利用する
  デフォルトは空文字列

- kinematics_type(string):
  運動学クラスIRobotKinematicsModelのプラグイン名
  デフォルトは空文字列で，空文字列の場合は"tmc_robot_kinematics_model::PinocchioWrapper"が読み込まれる

- solver_type(string):
  IKSolverのプラグイン名
  デフォルトは"tmc_robot_kinematics_model::NumericIKSolver"

- goal_sampler_type(string):
  SampleGoalGeneratorBaseのプラグイン名
  デフォルトは"tmc_simple_path_generator/SampleGoalGenerator"

- interpolation_type(string):
  ITrajectoryInterpolatorのプラグイン名
  デフォルトは空文字列で，空文字列の場合は"tmc_simple_path_generator/PlanarTrajectoryInterpolator"が読み込まれる
  補間を行わない(Generatorの次にOptimizerを実行する)場合は，"tmc_simple_path_generator/PlanarTrajectoryInterpolatorPlain"を指定する

- joint_names(string[]):
  制御対象の関節名
  デフォルトは空配列

- joint_weights(double[]):
  制御対象の関節重み
  joint_namesとサイズを揃えること
  デフォルトは空配列

- base_names(string[]):
  制御対象の台車関節名
  デフォルトは空配列だが，HSR系であれば[world_joint]を入れれば良い

- base_weights(double[]):
  台車関節の自由度ごとの重み
  選択したbase_movement_typeの自由度と要素数が合う必要がある
  base_namesとは要素数が同じである必要はない
  デフォルトは空配列

- base_movement_type(int):
  台車の移動タイプ
  tmc_manipulation_types::BaseMovementTypeから選択
  デフォルトは-1，xyYawの平面移動台車であれば1を入れる

- origin_frame(string):
  座標原点フレーム名
  アクションサーバーノードを起動した際にのみ使用される
  デフォルトは"odom"

- timeout(double):
  座標変換のタイムアウト[sec]
  アクションサーバーノードを起動した際にのみ使用される
  デフォルトは0.05


動的なパラメータ
-----------------

- generator_timeout(double):
  軌道生成のタイムアウト[sec]
  デフォルトは0.03

- max_simple_trajectory_num(int):
  現在の関節角度，速度とゴール関節角度を単純に結んだ軌道を生成する数
  姿勢拘束の場合、ゴール姿勢にランダム要素が含まれるためデフォルトで５軌道生成する

- max_trajectory_num(int):
  軌道候補の生成数の上限
  デフォルトは0で，0の場合は上限なしでタイムアウトまで生成を続ける

- middle_state_base_position_range(double):
  3点軌道を生成する際の，中間点に加算されるランダムな台車位置のサンプリング範囲[m]
  デフォルトは0.5

- middle_state_base_rotation_range(double):
  3点軌道を生成する際の，中間点に加算されるランダムな台車yaw方向のサンプリング範囲[rad]
  デフォルトは0.05

- max_sampling_trajectory(int):
  Generatorで補間を行う際の，軌道の最大点数
  "tmc_simple_path_generator/PlanarTrajectoryInterpolator"で利用される
  デフォルトは30

- time_stamp(double):
  Generatorで補間を行う際の，軌道点間の最小の時間幅[sec]
  "tmc_simple_path_generator/PlanarTrajectoryInterpolator"で利用される
  デフォルトは0.1

- ik_initial_range(double):
  IKを解く際の初期値のランダム化のサンプリング範囲，単位は関節の単位による
  デフォルトは0.5
