import torch.utils.data.dataset
from torchvision import transforms
import os
from PIL import Image
import torch
import torch.nn as nn
#==========================================#
#                                          #
#              作者:小白                    #
#                                          #
#      --搭建数据集以及CNN神经网络结构--      #
#                    ————2020.6.26         #
#==========================================#

class MyDataset(torch.utils.data.Dataset):
    def __init__(self, label_file_path):
        super().__init__()
        with open(label_file_path, 'r', encoding='UTF-8') as f:
            # (image_path(str), image_label(str))
            self.imgs = list(map(lambda line: line.strip().split(' '), f))

    def __getitem__(self, index):
        path, label = self.imgs[index]
        img = Image.open(path).convert('RGB')
        transform = transforms.ToTensor()
        img = transform(img)
        label = int(label)
        return img, label

    def __len__(self):  # 返回数据集的长度，即多少张图片
        return len(self.imgs)

class CNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Sequential(
            nn.Conv2d(           #(3,60,40)
                in_channels=3,   #输入图片的通道rgb三通道
                out_channels=16,
                kernel_size=5,   #卷积核
                stride=1,
                padding=2    #padding = (kernel_size-1)/2
            ),
            nn.ReLU(),       #维度变换(3,60,40) --> (16,60,40)
            nn.MaxPool2d(kernel_size=2)   #维度变换(16,60,40) --> (16,30,20)
        )
        self.conv2 = nn.Sequential(   #(16,30,20)
            nn.Conv2d(
                in_channels=16,
                out_channels=32,       # (32,30,20)
                kernel_size=5,
                stride=1,
                padding=2
            ),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2)      #维度变换(32,30,20) --> (32,15,10)
        )
        self.output = nn.Linear(32*15*10, 24)

    def forward(self, x):
        out = self.conv1(x)
        out = self.conv2(out)                #维度变换(Batch,32,25,50)
        out = out.view(out.size(0), -1)       #(Batch,32*25*50)将其展平
        out = self.output(out)
        return out

def predicted_data(Path):
    test_data = []
    path_num = []
    path_str = []

    def del_file(filepath):
        """
        删除某一目录下的所有文件或文件夹
        :param filepath: 路径
        :return:
        """
        del_list = os.listdir(filepath)
        for f in del_list:
            file_path = os.path.join(filepath, f)
            if os.path.isfile(file_path):
                os.remove(file_path)

    for root, dirs, files in os.walk(Path):
        path = files
        root = root

    for j in range(len(path)):
        path[j] = path[j].strip('.jpg')
        num = int(path[j])
        path_num.append(num)
        path_num.sort()
    for k in range(len(path_num)):
        num = str(path_num[k])
        path_str.append(num + '.jpg')
    for i in range(len(path)):
        real_path = root+'/'+path_str[i]
        img = Image.open(real_path).convert('RGB')
        transform = transforms.ToTensor()
        img = transform(img)
        img = img.unsqueeze(0)
        test_data.append(img)
    del_file(root+'/')
    return test_data


def decode_output(input = []):
    output = [' ' for i in range(len(input))]
    for i in range(len(input)):
        if input[i] == 10:
            output[i] = 'A'
        elif input[i] == 11:
            output[i] = 'V'
        elif input[i] == 12:
            output[i] = 'K'
        elif input[i] == 13:
            output[i] = 'L'
        elif input[i] == 14:
            output[i] = 'B'
        elif input[i] == 15:
            output[i] = 'N'
        elif input[i] == 16:
            output[i] = 'C'
        elif input[i] == 17:
            output[i] = 'G'
        elif input[i] == 18:
            output[i] = 'P'
        elif input[i] == 19:
            output[i] = 'J'
        elif input[i] == 20:
            output[i] = '苏'
        elif input[i] == 21:
            output[i] = '豫'
        elif input[i] == 22:
            output[i] = '浙'
        elif input[i] == 23:
            output[i] = '粤'
        elif input[i]>=0 and input[i]<=9:
            output[i] = str(input[i])
    result = ''
    for i in range(len(output)):
        result += output[i]
    return result


