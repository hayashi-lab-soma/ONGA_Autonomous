# onga_perception
センサ処理等のロボット知覚に関するパッケージ \
Arucoマーカーの検出やホームポジション座標のpublish機能を含む \
各launchの引数についてを以下に示します。

## cloud2laserscan.launch
- cloud_in ... 入力する３次元点群のTopic名
- scan ... 出力する２次元スキャンのTopic名

## rtabmap.launch
- camera1 ... 使用するカメラ１つ目
- camera2 ... 使用するカメラ２つ目
- use_rviz ... rvizを使うかどうか
- use_rtabmapviz ... rtabmapvizを使うかどうか
- rviz_cfg ... rvizを使用する際に読み込む設定ファイル