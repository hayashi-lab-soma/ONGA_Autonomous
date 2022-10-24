# onga_control
機体制御用パッケージ \
ロボットのコントロール起動用スクリプト、コントロール用設定ファイル、リモコン制御等を含む。 \
各launchの引数についてを以下に示します。

## control.launch
ロボットのURDFを読み込み、
- mode ... 使用するモード名
- enable_ekf ... 事故位置推定にEKF(拡張カルマンフィルタ)を使用するかどうか
- laser_enabled ... 2DLiDARを使用するかどうか
- realsense_enabled ... Realsenseを使用するかどうか
- urdf_extras ...

## teleop.launch
- joy_dev ... 
- joystick ... 
