function edgeDetection()%#codegen

% 和树莓派建立通信
r = raspi;
disp('connection')

% 获取树莓派上接着的USB摄像头
w = webcam(r,2);

% 定义边缘检测的卷积核（sobel算子）
kern = [1 2 1; 0 0 0; -1 -2 -1];

% 从摄像头中捕获连续的200帧，边缘检测
for k = 1:200
img = snapshot(w); % 从摄像头获取图像画面

% 获得水平和竖直方向的梯度，求总梯度
h = conv2(img(:,:,2),kern,'same');
v = conv2(img(:,:,2),kern','same');
e = sqrt(h.*h + v.*v);
edgeImg = uint8((e > 100) * 240);
% 获得梯度图像
newImg = cat(3,edgeImg,edgeImg,edgeImg);
% 展示梯度图像
% imshow([img newImg]);
displayImage(r,[img newImg], 'Title', 'Edge Detection')
end
disp('finish');
end