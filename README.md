# ONGA_Autonomous
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
* Husky ROSパッケージのインストール
    ```
    sudo apt install ros-melodic-husky-gazebo ros-melodic-husky-viz ros-melodic-husky-control
    ```
* RTAB-MApのインストール
    ```
    sudo apt install ros-melodic-rtabmap-ros
    ```
* 本パッケージのダウンロード
    ```
    ##onga_wsの作成
    mkdir ~/onga_ws/src
    cd ~/onga_ws/src
    catkin_init_workspace
    cd ..
    catkin build
    cd
    echo "source ~/onga_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

    ##ONGA_Autonomousのインストール
    cd ~onga_ws/src
    git clone https://github.com/hayashi-lab-soma/ONGA_Autonomous.git
    catkin build
    ```
* catkin buildを未インストールの場合
    ```
    sudo apt update
    sudo apt install python-catkin-tools
    ```


## パッケージ内容
### ・onga_arduino
AruduinoによるPWM出力制御パッケージ

### ・onga_control
機体制御用パッケージ

### ・onga_description
Gazeboシミュレーション用の機体仕様パッケージ

### ・onga_exp
実験用パッケージ

### ・onga_gazebo
Gazeboシミュレーション制御パッケージ

### ・onga_navigation
ナビゲーションシステムパッケージ

### ・onga_perception
センサ処理等のロボット知覚に関するパッケージ

### ・onga_viz
Rvizによる描画パッケージ

## パッケージの使い方
### Gazeboシミュレーション
1. シミュレーションをとりあえず動かすとき(大まかには以下の３機能が開始される) \
    ・Gazebo空間生成 \
    ・RVizにて可視化 \
    ・機体のコントロール \
    ```
    roslaunch onga_control product_sim.launch
    ```

2. 機能ごとにそれぞれを立ち上げるとき \
    ・Gazebo空間生成 
    ```
    ##terminalその１で##
    roslaunch onga_gazebo onga_playpen.launch
    ```
    ・Rvizにて可視化 
    ```
    ##terminalその２で##
    roslaunch onga_viz view_robot.launch
    ```
    ・機体のコントロール 
    ```
    ##terminalその１で##
    roslaunch onga_navigation move_base_mapless_demo.launch
    ```


### 実機による検証