# onga_exp
実験用パッケージ \
bringupやテスト用の各種実験用スクリプトを含みます。 \
各launchの引数についてを以下に示します。

## aruco_bringup.launch

## bringup.launch
ロボットの制御、リモコン信号管理、Realsense起動、点群の２Dスキャン変換。自律ナビゲーション起動、Aruduino接続開始、RViz起動を行う
- mode ... 使用するモード名

## realsense.launch
Realsense(T265,D435)を起動し、二つのセンサの各種パラメータをセットする。また、二つのセンサの座標関係を補正する。
- device_type_camera1 ... １つ目のカメラタイプ（T265）
- device_type_camera2 ... ２つ目のカメラタイプ（D400シリーズ）
- camera1 ... １つ目のカメラの便宜上の名前
- camera2 ... ２つ目のカメラの便宜上の名前
- initial_reset ... 起動時に初期化をするかどうか
- enable_fisheye ... 魚眼カメラのモードを使うかどうか（今回は使わない）
- color_width ... 
- color_height ... 
- depth_width ... 
- depth_height ... 
- clip_distance ... 深度画像から、指定された値（メートル）を超えるすべての値を削除する。負の値を指定して無効にする。（距離フィルタのイメージ）
- topic_odom_in ... 
- calib_odom_file ... 

## record.launch
バグファイル（ログ）を記録するためのスクリプトファイル。指定したトピックを録画できる。 \
launch開始と同時に録画スタート。launch終了と同時に録画終了。 \
### バグファイルの再生方法
```
# terminal 1
roscore

# terminal 2 (バグファイルがあるディレクトリに移動して)
rosparam set /use_sim_time true
rosbag play ***.bag --clock 　　(***のところにファイル名を入れてください)
```

## return_home.launch

## test_rtabmap.launch

