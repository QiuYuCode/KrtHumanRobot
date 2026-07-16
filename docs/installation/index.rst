.. _installation:

获取与安装
==========

系统要求
--------

* Ubuntu 22.04；
* ROS 2 Humble；
* Python 3.10+；
* Colcon 与 ``uv``；
* 支持目标硬件的 CAN、V4L2、PulseAudio 和 RealSense 运行环境。

RealSense ROS 子模块必须使用与 Jetson ``librealsense2 2.56.4`` 匹配的 ``4.56.4``
分支，不要直接升级到上游最新版本。

获取源码与依赖
--------------

.. code-block:: bash

   cd /home/nvidia/WorkSpace/KrtHumanRobot
   git submodule update --init --recursive
   ./scripts/import_3dloc_deps.sh

语音栈使用独立的 ``uv`` 环境和本地模型：

.. code-block:: bash

   cd src/voice_assistant
   bash scripts/setup_uv.sh
   bash scripts/download_voice_models.sh
   cd ../..

环境变量和云服务凭据写入 ``src/voice_assistant/.env``，不得提交到 Git。

构建工作区
----------

.. code-block:: bash

   cd /home/nvidia/WorkSpace/KrtHumanRobot
   colcon build --symlink-install
   source install/setup.bash

Python 包开发使用 ``--symlink-install``，避免每次修改后复制源码。构建单个包示例：

.. code-block:: bash

   colcon build --packages-select krt_human_robot --symlink-install
   colcon build --packages-select hand_camera_driver --symlink-install
   colcon build --packages-select ranger_nav --symlink-install

构建文档
--------

.. code-block:: bash

   uv tool run --from sphinx \
     --with-requirements docs/requirements.txt \
     sphinx-build -M html docs build/docs -n -W

HTML 首页生成在 ``build/docs/html/index.html``。
