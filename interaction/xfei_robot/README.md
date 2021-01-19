## 注意
xfei_robot/src目录下的文件中含有APPID的部分需要修改为自己的迅飞APPID

xfei_robot/VoiceOut目录内容为`src/iat_publish_speak.cpp`文件151行，绝对路径需要修改为你的目录

若出现不能调用APPID的报错，需要从官网下载自己的SDK，替换掉src中的内容，注意：`src/iat_publish_speak.cpp`以及`tts_subscribe_speak.cpp`需要修改相应内容，将自己SDK的两个相应文件内容添加上述两个文件的内容。