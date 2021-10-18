function faceDetection()%#codegen

% 和树莓派建立通信
r = raspi;
disp('connection')

% 获取树莓派上接着的USB摄像头
w = webcam(r,2);

% 从摄像头中捕获连续的200帧，逐帧处理
for k = 1:200
    
img = snapshot(w); % 从摄像头获取图像画面

fD = vision.CascadeObjectDetector(); % 级联人脸检测器
bbox = step(fD,img); % 使用级联人脸检测器检测图像中的人脸
img_out = insertObjectAnnotation(img,'rectangle',bbox,'Face'); % 可视化人脸框

imshow(img_out); % 展示检测效果
title('Face Detection')
end
disp('finish');
end
