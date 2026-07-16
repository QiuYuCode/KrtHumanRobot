.. _changelog:

版本更新记录
============

0.1.0
-----

当前 ``krt_human_robot`` 包版本为 0.1.0，已包含：

* 整机行为树入口和关键词/LLM 两种对话路径；
* 语音、头部及双手相机、导航建图、机械臂、灵巧手和任务流程集成；
* 动作组示教、流程编辑和 Web 控制台；
* Sphinx 中文操作与开发文档。

维护规则
--------

发布版本时同步更新 ``src/krt_human_robot/package.xml`` 和本页。新增功能、行为变化、
修复和不兼容变更分别记录；删除 package、ROS 接口、topic、launch 或 executable 时，
必须明确标注 ``BREAKING CHANGE`` 和迁移方式。
