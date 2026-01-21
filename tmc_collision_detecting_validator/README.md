# tmc_collision_detecting_validator

軌道群から順に軌道の干渉チェックを行い，干渉しなかった軌道を返すvalidator


提供するアクション
----------------

- ~/validate(tmc_planning_msgs/action/ValidateRobotTrajectories)


購読するトピック
----------------

- attached_object_topicsで指定されたトピック(moveit_msgs/msg/RobotState):
  ロボット状態のうち把持物情報のみが利用される

- obstacle_map_topic_namesで指定されたトピック(nav_msgs/msg/OccupancyGrid):
  2D障害物地図

- collision_environment_server/transformed_environment(moveit_msgs/msg/PlanningSceneWorld):
  障害物環境情報

- ~/collisions_disabled_object_pairs(tmc_planning_msgs/msg/ObjectPairsList):
  干渉チェックから除外される物体名のペア情報


利用するパラメータ
----------------

- kinematics_type(string):
  運動学クラスIRobotKinematicsModelのプラグイン名
  デフォルトは空文字列で，空文字列の場合は"tmc_robot_kinematics_model::PinocchioWrapper"が読み込まれる

- robot_description_kinematics(string):
  ロボットモデル
  FK計算で利用される
  デフォルトは空文字列

- robot_description(string):
  ロボットモデル
  robot_description_kinematicsが空の場合，こちらのロボットモデルを利用する
  デフォルトは空文字列

- robot_collision_pait(string):
  ロボットの干渉チェックの設定情報
  デフォルトは空文字列

- collision_engine(string):
  干渉チェックの内部実装
  デフォルトはODE

- attached_object_topics(string[]):
  把持物情報のトピック名群
  デフォルトは"attached_object_publisher/attached_object"

- obstacle_map_topic_names(string[]):
  干渉チェックに用いる2D障害物地図のトピック名群
  デフォルトは空配列

- trajectory_points_collision_check_order_type(string):
  軌道点のチェック順の種類
  "TrajectoryPointsCollisionCheckBothEnds"を指定すると，両端のみを全点チェックし，中間は飛ばしながらチェックする
  デフォルトは"TrajectoryPointsCollisionCheckSkippingOrder"

- time_from_start(double):
  軌道点のチェックで全点をチェックする開始からの時刻[sec]
  "TrajectoryPointsCollisionCheckBothEnds"でのみ利用される
  デフォルトは1.0

- time_from_end(double):
  軌道点のチェックで全点をチェックする終了からの時刻[sec]
  "TrajectoryPointsCollisionCheckBothEnds"でのみ利用される
  デフォルトは1.0

- interval_middle(double):
  軌道点のチェックでの中間の飛ばし幅[sec]
  "TrajectoryPointsCollisionCheckBothEnds"でのみ利用される
  デフォルトは0.2


動的なパラメータ
-----------------

- validate_timeout(double):
  干渉チェックのタイムアウト[sec]
  デフォルトは0.15

- save_request(bool):
  デバッグのために入力された軌道群等をファイルに書き出す機能の有効/無効
  デフォルトはtrue

- print_debug_info(bool):
  デバッグのために衝突情報のログ出力の有効/無効
  デフォルトはtrue
