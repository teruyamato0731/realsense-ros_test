# realsense-ros_test
[![Open in Dev Containers](https://img.shields.io/static/v1?label=Dev%20Containers&message=Open&color=blue&logo=visualstudiocode)](https://vscode.dev/redirect?url=vscode://ms-vscode-remote.remote-containers/cloneInVolume?url=https://github.com/teruyamato0731/realsense-ros_test)
[![license](https://img.shields.io/github/license/teruyamato0731/realsense-ros_test)](https://github.com/teruyamato0731/realsense-ros_test/blob/main/LICENSE)
[![CI](https://github.com/teruyamato0731/realsense-ros_test/actions/workflows/main.yml/badge.svg)](https://github.com/teruyamato0731/realsense-ros_test/actions/workflows/main.yml)

ROS2 humbleのdev container開発環境。
[realsense-ros](https://github.com/IntelRealSense/realsense-ros)を使用してROS2でRealSenseの動作確認とPCLによる点群のフィルタリング・クラスタリングを行う。
X11をマウントしてGUIアプリの使用ができるようにしている。
Ubuntu 22.04, RealSense D455で動作確認済み。

# Quick Start
あなたがすでにVS CodeとDockerをインストールしている場合は、上記のバッジまたは[こちら](https://vscode.dev/redirect?url=vscode://ms-vscode-remote.remote-containers/cloneInVolume?url=https://github.com/teruyamato0731/realsense-ros_test)をクリックすることで使用することができる。<br>
これらのリンクをクリックすると、vscodeが必要に応じてdev container拡張機能を自動的にインストールし、ソースコードをコンテナボリュームにクローンし、使用するためのdev containerを起動する。

# How to use
1. Docker, vscode, devcontainer拡張機能をインストールする。
1. X11のアクセスをローカルに対して許可する。
    ```bash
    xhost +local:
    # non-network local connections being added to access control list
    ```
1. ホストにudev ruleをインストール。下記コマンドのうちいずれかをどちらかを実行する。
    ```bash
    (cd /usr/lib/udev/rules.d/ && sudo curl -O https://raw.githubusercontent.com/IntelRealSense/librealsense/8ffb17b027e100c2a14fa21f01f97a1921ec1e1b/config/99-realsense-libusb.rules)
    ```
    ```bash
    (cd /usr/lib/udev/rules.d/ && sudo wget https://raw.githubusercontent.com/IntelRealSense/librealsense/8ffb17b027e100c2a14fa21f01f97a1921ec1e1b/config/99-realsense-libusb.rules)
    ```
    ```bash
    # アンインストールしたい場合
    sudo rm /usr/lib/udev/rules.d/99-realsense-libusb.rules
    ```
1. リポジトリをcloneしvscodeで開く。
    ```bash
    git clone https://github.com/teruyamato0731/realsense-ros_test.git
    code realsense-ros_test
    ```
1. 「Reopen in Container」でdevcontainerを開く
1. 下記コマンドを各ターミナルで実行。
    ```bash
    sudo apt update && rosdep update && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
    colcon build
    . install/local_setup.bash
    ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 pointcloud.enable:=true decimation_filter.enable:=true
    ```
    ```bash
    . install/local_setup.bash
    ros2 run realsense-pcl filter
    ```
    ```bash
    rviz2
    ```
