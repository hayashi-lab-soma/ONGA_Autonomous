# onga_description
機体の座標リンクを定義したファイルに関するパッケージ \
シミュレーションや実機テストを行う際に起動する。 \ 
各launchの引数についてを以下に示します。

## description.launch
- mode ... 使用するモード名
- robot_namespace ... プログラム内でのロボット名
- laser_enabled ... 2次元LiDARを使用するかどうか
- realsense_enabled ... Realsenseを使用するかどうか
- urdf_extras ... 

## URDFについて
URDF (Unified Robot Description Format)とは、ロボットの構造を記述するためのXMLのフォーマットです。URDFにおける記述はLinkとJointから構成されます。
Linkはロボットの駆動しない1ブロックです。
### Linkは以下のような要素を持ちます。
- 見た目の形、色
- 衝突判定の形
- 重さとイナーシャ
### JointはLinkとLinkの接続を表します。
- jointの種類（固定、回転、直動....）
- jointの詳細オプション
- このlinkとjointによって、木構造となります。

またurdfに詳細なオプションを書くことで、これからsimulation用モデルを生成することができます。

<p align="center"><img src="https://camo.qiitausercontent.com/a5e42a203d2303454a0f789640533f3fd7902187/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e616d617a6f6e6177732e636f6d2f302f3235343434322f35323533396638612d363764322d313662332d383035372d6632326337376431303832372e706e67"></p>

