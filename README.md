# 一个将ros2与godot连接起来的godot插件

非常简单的使用方式

Ros2Node这个节点代表了一个ros2里面的node

继承Ros2Communication的节点代表了ros2节点内部的通讯机制

目前实现了通用的自定义topic的publisher和subscription

以及发布tf2数据的Tf2Publisher和专门发布图像与相机内参的ImagePublisher

使用时只需将Ros2Node拖进场景，再将你需要的通讯节点设置为它的子节点，设置好名称等信息，就可以用了

目前初步完工，还有一些bug没修，欢迎提交issue
