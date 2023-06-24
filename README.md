# realsense-ros_test
[![Open in Dev Containers](https://img.shields.io/static/v1?label=Dev%20Containers&message=Open&color=blue&logo=visualstudiocode)](https://vscode.dev/redirect?url=vscode://ms-vscode-remote.remote-containers/cloneInVolume?url=https://github.com/teruyamato0731/realsense-ros_test)
[![license](https://img.shields.io/github/license/teruyamato0731/realsense-ros_test)](https://github.com/teruyamato0731/realsense-ros_test/blob/main/LICENSE)
[![CI](https://github.com/teruyamato0731/realsense-ros_test/actions/workflows/main.yml/badge.svg)](https://github.com/teruyamato0731/realsense-ros_test/actions/workflows/main.yml)

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
1. リポジトリをcloneしvscodeで開く。
1. 「Reopen in Container」でdevcontainerを開く
