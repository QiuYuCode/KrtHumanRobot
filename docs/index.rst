KrtHumanRobot
=============

KrtHumanRobot 是一个基于 ROS 2 Humble 的人形机器人工作区。整机核心以行为树为
调度入口，将语音、视觉、导航建图、机械臂、灵巧手和动作编排组合为统一能力。

本文档同时面向部署操作和二次开发。第一次使用建议依次阅读“获取与安装”和
“快速开始”；修改行为、接口或接入硬件时阅读“接口说明”和“二次开发边界与
安全声明”。

快速开始
--------

.. code-block:: bash

   cd /home/nvidia/WorkSpace/KrtHumanRobot
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch krt_human_robot robot.launch.py

构建本文档
----------

仓库使用已有的 ``uv`` 工具隔离运行 Sphinx：

.. code-block:: bash

   cd /home/nvidia/WorkSpace/KrtHumanRobot
   uv tool run --from sphinx \
     --with-requirements docs/requirements.txt \
     sphinx-build -M html docs build/docs

生成首页位于 ``build/docs/html/index.html``。

.. toctree::
   :maxdepth: 4
   :numbered: 3

   about/index
   operation/index
   installation/index
   quickstart/index
   interfaces/index
   examples/index
   faq/index
   boundaries/index
   changelog/index
