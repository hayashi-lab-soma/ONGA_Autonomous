# ONGA_Autonomous
壁面吸引ロボット自律化に関するROSパッケージ \
最終更新日：2021/09/06

## 使用するセンサ
* Intel Realsense D435i
* Intel Realsense T265

## 要求要件
* Ubuntu18.04
* ROS melodic
* RealsenseSDK 2.0 

## 外部パッケージ、必要なツールのインストール
* catkin buildのインストール
    ```
    sudo apt update
    sudo apt install python-catkin-tools
    ```
    
* Husky ROSパッケージのインストール
    ```
    sudo apt install ros-melodic-husky-gazebo ros-melodic-husky-viz ros-melodic-husky-navigation
    ```
* RTAB-MApのインストール
    ```
    sudo apt install ros-melodic-rtabmap-ros
    ```
* rosseral
    ```
    sudo apt install ros-melodic-rosserial
    sudo apt install ros-melodic-rosserial-arduino
    ```
* Aruduino セットアップ \
    本パッケージでは統合開発環境ArduinoIDEを使用しない。\
    Platformioを使用してコンパイル、Aruduinoへの書き込みを行う
    ```
    ##platformioのインストール##
    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
    python3 get-platformio.py

    ##pipのアップグレード
    pip3 install --upgrade pip
    
    ##platformioをインストール
    pip3 install platformio
    
    ##インストール完了の確認
    pio --version
    #PlatformIO Core, version 5.1.1
    
    sudo usermod -a -G dialout <username>
    sudo chmod a+rw /dev/ttyACM0
    ```
    
## 本パッケージのインストール
ワークスペースのディレクトリを作成し、githubからソースをダウンロードした後、ビルドします。
また、Aruduinoにソースを書き込みます。
    ```
    ##onga_wsの作成
    mkdir -p ~/onga_ws/src
    cd ~/onga_ws/src
    catkin_init_workspace
    cd ..
    catkin build
    cd
    echo "source ~/onga_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

    ##ONGA_Autonomousのインストール
    cd ~/onga_ws/src
    git clone https://github.com/hayashi-lab-soma/ONGA_Autonomous.git
    catkin build
    source ~/.bashrc
    
    ##Arduinoへのソース書き込み##
    rosrun onga_arduino write_cmd2pwm.sh
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
Huskyの機体のみ対応しています。 \
・Gazebo空間生成 \
・RVizにて可視化 \
・機体のコントロール 
```
roslaunch onga_control sim_full.launch
```

### Osoyooロボットを動かす
RealsenseとArduinoをPCに接続してないと動作しません。
```
roslaunch onga_exp bringup.launch
```
