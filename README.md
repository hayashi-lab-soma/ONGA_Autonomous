# ONGA_Autonomou
壁面吸引ロボット自律化に関するROSパッケージ

## 使用するセンサ
* Intel Realsense D435i
* Intel Realsense T265

## 要求要件
* Ubuntu18.04
* ROS melodic
* RealsenseSDK 2.0 

## セットアップ方法
2021/07/07現在

* 本パッケージのダウンロード
```
git clone https://github.com/hayashi-lab-soma/ONGA_Autonomous.git
```
* Husky ROSパッケージのインストール
```
sudo apt install ros-melodic-husky-gazebo ros-melodic-husky-viz ros-melodic-husky-control
```
* RTAB-MApのインストール
```
sudo apt install ros-melodic-rtabmap-ros
```

## パッケージ内容
### onga_arduino
AruduinoによるPWM出力制御パッケージ

### onga_control
機体制御用パッケージ

### onga_description
Gazeboシミュレーション用の機体仕様パッケージ

### onga_exp
実験用パッケージ

### onga_gazebo
Gazeboシミュレーション制御パッケージ

### onga_navigation
ナビゲーションシステムパッケージ

### onga_perception
センサ処理等のロボット知覚に関するパッケージ

### onga_viz
Rvizによる描画パッケージ

## パッケージの使い方
### Gazeboシミュレーション

### 実機による検証