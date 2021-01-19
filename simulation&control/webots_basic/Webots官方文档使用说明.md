# 0. 说明

Webots仿真过程，需要我们时刻查询手册，所以熟悉手册的内容至关重要。

> 在线文档访问不畅，需科学上网；如果使用本地离线文档，请先cd到安装目录下的docs下执行如下命令：
>
> ```bash
> python3 -m http.server 8000
> ```
> 推荐使用本地离线文档！

# 1. 用户手册

离线文档链接：[http://localhost:8000/?url=&book=guide](http://localhost:8000/?url=&book=guide)

官方在线文档链接：[https://www.cyberbotics.com/doc/guide/index](https://www.cyberbotics.com/doc/guide/index)

- Webots的安装
  - [离线文档](http://localhost:8000/?url=&book=guide&page=installing-webots)
  - [在线文档](https://www.cyberbotics.com/doc/guide/installing-webots.md)
- Webots界面介绍
  - [离线文档](http://localhost:8000/?url=&book=guide&page=getting-started-with-webots)
  - [在线文档](https://www.cyberbotics.com/doc/guide/getting-started-with-webots.md)
- Webots示例
  - [离线文档](http://localhost:8000/?url=&book=guide&page=sample-webots-applications)
  - [在线文档](https://www.cyberbotics.com/doc/guide/sample-webots-applications.md)
- 编程语言设置(★★★★★)
  - [离线文档](http://localhost:8000/?url=&book=guide&page=language-setup)
  - [在线文档](https://www.cyberbotics.com/doc/guide/language-setup.md)
- 开发环境搭建(★★★★★)
  - [离线文档](http://localhost:8000/?url=&book=guide&page=development-environments)
  - [在线文档](https://www.cyberbotics.com/doc/guide/development-environments.md)
- 编程基础(★★★★★)
  - [离线文档](http://localhost:8000/?url=&book=guide&page=programming-fundamentals)
  - [在线文档](https://www.cyberbotics.com/doc/guide/programming-fundamentals.md)
- 网页界面
  - [离线文档](http://localhost:8000/?url=&book=guide&page=web-interface)
  - [在线文档](https://www.cyberbotics.com/doc/guide/web-interface.md)
- 官方入门视频教程
  - [离线文档](http://localhost:8000/?url=&book=guide&page=tutorials)
  - [在线文档](https://www.cyberbotics.com/doc/guide/tutorials.md)
- 现有的机器人模型
  - [离线文档](http://localhost:8000/?url=&book=guide&page=robots)
  - [在线文档](https://www.cyberbotics.com/doc/guide/robots.md)
- 现有的执行器
  - [离线文档](http://localhost:8000/?url=&book=guide&page=actuators)
  - [在线文档](https://www.cyberbotics.com/doc/guide/actuators.md)
- 现有的传感器
  - [离线文档](http://localhost:8000/?url=&book=guide&page=sensors)
  - [在线文档](https://www.cyberbotics.com/doc/guide/sensors.md)
- 现有的其他场景物体
  - [离线文档](http://localhost:8000/?url=&book=guide&page=objects)
  - [在线文档](https://www.cyberbotics.com/doc/guide/objects.md)
- 现有的纹理
  - [离线文档](http://localhost:8000/?url=&book=guide&page=appearances)
  - [在线文档](https://www.cyberbotics.com/doc/guide/appearances.md)
- Webots常见问题
  - [离线文档](http://localhost:8000/?url=&book=guide&page=webots-faq)
  - [在线文档](https://www.cyberbotics.com/doc/guide/webots-faq.md)
- 已知错误
  - [离线文档](http://localhost:8000/?url=&book=guide&page=known-bugs)
  - [在线文档](https://www.cyberbotics.com/doc/guide/known-bugs.md)

# 2. API文档

离线文档链接：[http://localhost:8000/?url=&book=reference](http://localhost:8000/?url=&book=reference)

官方在线文档链接：[https://www.cyberbotics.com/doc/reference/index](https://www.cyberbotics.com/doc/reference/index)

- 节点关系图(★★★)
  - [离线文档](http://localhost:8000/?url=&book=reference&page=node-chart)
  - [在线文档](https://www.cyberbotics.com/doc/reference/node-chart.md)
- 节点属性说明及其对应的API函数(★★★★★)
  - [离线文档](http://localhost:8000/?url=&book=reference&page=nodes-and-api-functions)
  - [官方文档](https://www.cyberbotics.com/doc/reference/nodes-and-api-functions.md)
- Motion函数(类似于ROS中的rosbag)
  - [离线文档](http://localhost:8000/?url=&book=reference&page=motion-functions)
  - [官方文档](https://www.cyberbotics.com/doc/reference/motion-functions.md)
- PROTO扩展说明
  - [离线文档](http://localhost:8000/?url=&book=reference&page=proto)
  - [在线文档](https://www.cyberbotics.com/doc/reference/proto.md)
- 物理插件(高阶用法)
  - [离线文档](http://localhost:8000/?url=&book=reference&page=physics-plugin)
  - [在线文档](https://www.cyberbotics.com/doc/reference/physics-plugin.md)
- 其他API（ROS API）
  - [离线文档](http://localhost:8000/?url=&book=reference&page=ros-api)
  - [在线文档](https://www.cyberbotics.com/doc/reference/other-apis.md)
  
# 3. 软件例程

- 编程语言的例程：`$WEBOTS_HOME\projects\languages\python\worlds`
- 其他案例：`$WEBOTS_HOME\Webots\projects\samples`
# 4. 后记

对于Webots来说，官方提供的文档很详细，软件中也附带了大量demo，其中包括入门教程等资料，美中不足的是缺少中文资料，且大多数案例编程语言为C，如果需要使用其他语言编程，相关的参考资料较少，上手较麻烦，针对这个问题，后期我们会推出相关教程。