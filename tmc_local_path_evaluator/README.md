# tmc_local_path_evaluator

軌道を評価するクラスでスコアを算出し、スコアの小さい順にソートするevaluator

提供するアクション
-----------------

- ~/evaluate(tmc_planning_msgs/action/EvaluateRobotTrajectories)

利用するパラメータ
-----------------

- thread_pool_size(int):
  軌道ごとのスコア計算のスレッドプールのスレッド数
  デフォルトは8

- score_calculations.names(string[]):
  スコア計算の名前リスト
  デフォルトは空配列

- score_calculations.{name}.type(string):
  スコア計算プラグイン
  利用可能なプラグインはscore_evaluation_plugins.xmlを参照すること
  デフォルトは空文字列

- score_calculations.{name}.score_weight(double):
  スコアの重み
  デフォルトは1.0

- score_calculations.{name}.base_movement_type(int):
  台車の移動タイプ
  tmc_manipulation_types::BaseMovementTypeから選択
  typeが"tmc_local_path_evaluator/LengthBaseScoreCalculation"の場合に使用される
  デフォルトは-1，xyYawの平面移動台車であれば1を入れる

- score_calculations.{name}.joint_names(string[]):
  制御対象の関節名
  typeが"tmc_local_path_evaluator/LengthBaseScoreCalculation"の場合に使用される
  デフォルトは空配列

- score_calculations.{name}.joint_weights(double[]):
  制御対象の関節重み
  joint_namesとサイズを揃えること
  typeが"tmc_local_path_evaluator/LengthBaseScoreCalculation"の場合に使用される
  デフォルトは空配列

- score_calculations.{name}.base_weights(double[]):
  台車関節の自由度ごとの重み
  選択したbase_movement_typeの自由度と要素数が合う必要がある
  typeが"tmc_local_path_evaluator/LengthBaseScoreCalculation"の場合に使用される
  デフォルトは空配列

- kinematics_type(string):
  運動学クラスIRobotKinematicsModelのプラグイン名
  typeが"tmc_local_path_evaluator/SoftLinkConstraintScoreCalculation"の場合に使用される
  デフォルトは空文字列で，空文字列の場合は"tmc_robot_kinematics_model::PinocchioWrapper"が読み込まれる

- robot_description_kinematics(string):
  ロボットモデル
  FK計算で利用される
  typeが"tmc_local_path_evaluator/SoftLinkConstraintScoreCalculation"の場合に使用される
  デフォルトは空文字列

- robot_description(string):
  ロボットモデル
  robot_description_kinematicsが空の場合，こちらのロボットモデルを利用する
  typeが"tmc_local_path_evaluator/SoftLinkConstraintScoreCalculation"の場合に使用される
  デフォルトは空文字列