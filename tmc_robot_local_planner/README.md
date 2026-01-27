# tmc_robot_local_planner

constraintを満たすローカルな経路を生成する

利用するパラメータ
----------------
- optimize_second(bool):
  optimizeを2番目に行うか否か
  デフォルトはfalseで，gen -> eval -> val -> optの順に実行される
  trueの場合，gen -> opt -> eval -> valの順に実行される


動的なパラメータ
----------------
- publish_generated_trajectories(bool):
  生成された時間軌道候補群を，可視化やデバッグのために発行するか否か
  デフォルトはfalse


クラス
==================

RobotLocalPlanner
------------------
ロボットローカルプランナの抽象クラス
constraintを満たす経路を提供する

以下の情報が必要
* generator: 経路生成部のアクションサーバ
* evaluator: 経路評価部のアクションサーバ
* validator: 経路検証部のアクションサーバ
* optimizer: 経路最適化部のアクションサーバ

ILinkConstraint
------------------
link constraintインターフェースクラス

TsrLinkConstraint
------------------
TSRとリンク制約クラス
以下の情報が必要
*tsr TSR
*priority 優先度
*seed 乱数生成シード

IJointConstraint
------------------
joint constraintインターフェースクラス

RangeJointConstraint
------------------
距離ジョイント制約クラス
以下の情報が必要
*min ロボット最小姿勢
*max ロボット最大姿勢
*priority 優先度
*seed 乱数生成シード
