import cv2
import numpy as np
from PIL import Image


#=========================================#
#                                         #
#              作者:小白                   #
#                                         #
#       --将车牌各个字符分割成独立图片--     #
#                    ————2020.6.22        #
#=========================================#


class Carlicense_distinguish:
    def __init__(self):
        self.content = 'D:/PycharmProjects/Num_distinguish/image/'
        self.after = '.jpg'
        self.choice = input('Please choose one from \'car1\',\'car2\',\'car3\',\'car4\'.\n')
        if self.choice == 'car1' or self.choice == 'car2' or self.choice == 'car3' or self.choice == 'car4' or self.choice == 'car5':
            self.carlicense_content = self.content + self.choice + self.after
            print('Identify success !')
        else:
            self.carlicense_content = 'D:/PycharmProjects/Num_distinguish/image/car1.jpg'
            print('Input wrong !\nInitialize to \'car1\'')
        im = Image.open(self.carlicense_content)
        im.show()

    def carlicense_distinguish(self):
        lower_limi = np.array([80, 30, 0])  # 设置色域上下界
        upper_limi = np.array([230, 110, 70])
        img_init = cv2.imread(self.carlicense_content)  # 加载原始图片
        img_resize = cv2.resize(img_init, dsize=(1000, 600), fx=0, fy=0)  # 重置图片尺寸
        img_blur = cv2.blur(img_resize, (2, 2))  # 对图片进行降噪
        img_limi = cv2.inRange(img_blur, lower_limi, upper_limi)  # 选出指定色域(二值化，黑色：0，白色：255)
        cont = cv2.findContours(img_limi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # 找出色域外边界
        c = max(cont, key=cv2.contourArea)  # 在边界中找最大面积的区域
        rect = cv2.minAreaRect(c)  # 绘制出该区域最小外接矩形
        box = cv2.boxPoints(rect)  # 记录上述矩形四点坐标
        '''cv2.drawContours(img_resize, [np.int0(box)], -1, (0, 255, 255), 2)             #绘制出矩形轮廓'''
        img_final = img_resize[int(min(box[1][1], box[2][1]))
                               :int(max(box[0][1], box[3][1]))
        , int(min(box[0][0], box[1][0]))
          :int(max(box[2][0], box[3][0]))]  # 圈出车牌
        img_final_resize = cv2.resize(img_final, dsize=(200, 50), fx=0, fy=0)
        lower_limi2 = np.array([100, 100, 100])  # 设置色域上下界
        upper_limi2 = np.array([255, 255, 255])
        img_final_bin = cv2.inRange(img_final_resize, lower_limi2, upper_limi2)

        #cv2.imshow('IMG', img_final_bin)
######分割字符#######
        black_count = 0
        char_spli = [0 for i in range(200)]
        for i in range(200):
            for j in range (4,45):
                if img_final_bin[j,i] == 0:
                    black_count += 1
            if black_count >= 40:
                char_spli[i] = 1
                #cv2.line(img_final_resize, (i,0), (i,49), (0, 255, 255), 2)
            black_count = 0
##以上是绘制出所有字符间隔的蓝色块，从第4行至第45行，超过四十个点为蓝色即画线
        yellow_count = 0
        start_position = 0
        end_position = 0
        split_line = [0 for h in range(10)]
        x = 0
        for k in range(1, 200):
            if char_spli[k] == 1:
                yellow_count += 1
                if (char_spli[k] - char_spli[k - 1] == 1):
                    start_position = k
            else:
                if (char_spli[k] - char_spli[k - 1] == -1):
                    end_position = k - 1
                if (yellow_count >= 4 and end_position > start_position):
                    cv2.line(img_final_resize, (start_position+(end_position-start_position)//2, 0),
                                               (start_position+(end_position-start_position)//2, 49),
                                               (0, 255, 255), 2)
                    split_line[x] = start_position+(end_position-start_position) #记录所有分割线横坐标
                    x += 1
                yellow_count = 0
        #print(split_line)
##以上是把绘制出的黄色块区域取中线并绘制，以此来分割各个字符
        for f in range(10):
            if split_line[f] == 0:
                flag_posi = f
                break
        #print(flag_posi)
        flag_step = 0
        for g in range(flag_posi+1):
            if flag_step == 0 and g < flag_posi and split_line[g] - 0 >=18:
                split0 = img_final_resize[1:49, 1:split_line[g]]
                split0_resize = cv2.resize(split0, dsize=(40, 60), fx=0, fy=0)
                # cv2.namedWindow('split0_resize',0)
                # cv2.resizeWindow('split0_resize', 100, 200)
                # cv2.moveWindow('split0_resize',400,600)
                # cv2.imshow('split0_resize', split0_resize)
                cv2.imwrite('D:/PycharmProjects/Num_distinguish/test_data/0.jpg',split0_resize)
            elif flag_step == 1 and g < flag_posi and split_line[1] - split_line[0] >=18:
                split1 = img_final_resize[1:49, split_line[0]:split_line[1]]
                split1_resize = cv2.resize(split1, dsize=(40, 60), fx=0, fy=0)
                # cv2.namedWindow('split1_resize', 0)
                # cv2.resizeWindow('split1_resize', 100, 200)
                # cv2.moveWindow('split1_resize', 550, 600)
                # cv2.imshow('split1_resize', split1_resize)
                cv2.imwrite('D:/PycharmProjects/Num_distinguish/test_data/1.jpg', split1_resize)

            elif flag_step == 2 and g < flag_posi and split_line[2] - split_line[1] >=18:
                split2 = img_final_resize[1:49, split_line[1]:split_line[2]]
                split2_resize = cv2.resize(split2, dsize=(40, 60), fx=0, fy=0)
                # cv2.namedWindow('split2_resize', 0)
                # cv2.resizeWindow('split2_resize', 100, 200)
                # cv2.moveWindow('split2_resize', 700, 600)
                # cv2.imshow('split2_resize', split2_resize)
                cv2.imwrite('D:/PycharmProjects/Num_distinguish/test_data/2.jpg', split2_resize)

            elif flag_step == 3 and g < flag_posi and split_line[3] - split_line[2] >=18:
                split3 = img_final_resize[1:49, split_line[2]:split_line[3]]
                split3_resize = cv2.resize(split3, dsize=(40, 60), fx=0, fy=0)
                # cv2.namedWindow('split3_resize', 0)
                # cv2.resizeWindow('split3_resize', 100, 200)
                # cv2.moveWindow('split3_resize', 850, 600)
                # cv2.imshow('split3_resize', split3_resize)
                cv2.imwrite('D:/PycharmProjects/Num_distinguish/test_data/3.jpg', split3_resize)

            elif flag_step == 4 and g < flag_posi and split_line[4] - split_line[3] >=18:
                split4 = img_final_resize[1:49, split_line[3]:split_line[4]]
                split4_resize = cv2.resize(split4, dsize=(40, 60), fx=0, fy=0)
                # cv2.namedWindow('split4_resize', 0)
                # cv2.resizeWindow('split4_resize', 100, 200)
                # cv2.moveWindow('split4_resize', 1000, 600)
                # cv2.imshow('split4_resize', split4_resize)
                cv2.imwrite('D:/PycharmProjects/Num_distinguish/test_data/4.jpg', split4_resize)

            elif flag_step == 5 and g < flag_posi and split_line[5] - split_line[4] >=18:
                split5 = img_final_resize[1:49, split_line[4]:split_line[5]]
                split5_resize = cv2.resize(split5, dsize=(40, 60), fx=0, fy=0)
                # cv2.namedWindow('split5_resize', 0)
                # cv2.resizeWindow('split5_resize', 100, 200)
                # cv2.moveWindow('split5_resize', 1150, 600)
                # cv2.imshow('split5_resize', split5_resize)
                cv2.imwrite('D:/PycharmProjects/Num_distinguish/test_data/5.jpg', split5_resize)

            elif flag_step == 6 and g < flag_posi and split_line[6] - split_line[5] >=18:
                split6 = img_final_resize[1:49, split_line[5]:split_line[6]]
                split6_resize = cv2.resize(split6, dsize=(40, 60), fx=0, fy=0)
                # cv2.namedWindow('split6_resize', 0)
                # cv2.resizeWindow('split6_resize', 100, 200)
                # cv2.moveWindow('split6_resize', 1300, 600)
                # cv2.imshow('split6_resize', split6_resize)
                cv2.imwrite('D:/PycharmProjects/Num_distinguish/test_data/6.jpg', split6_resize)

            elif flag_step == 7 and g < flag_posi and split_line[7] - split_line[6] >=18:
                split7 = img_final_resize[1:49, split_line[6]:split_line[7]]
                split7_resize = cv2.resize(split7, dsize=(40, 60), fx=0, fy=0)
                # cv2.namedWindow('split7_resize', 0)
                # cv2.resizeWindow('split7_resize', 100, 200)
                # cv2.moveWindow('split7_resize', 1450, 600)
                # cv2.imshow('split7_resize', split7_resize)
                cv2.imwrite('D:/PycharmProjects/Num_distinguish/test_data/7.jpg', split7_resize)

            elif flag_step == 8 and g < flag_posi and split_line[8] - split_line[7] >=18:
                split8 = img_final_resize[1:49, split_line[7]:split_line[8]]
                split8_resize = cv2.resize(split8, dsize=(40, 60), fx=0, fy=0)
                # cv2.namedWindow('split8_resize', 0)
                # cv2.resizeWindow('split8_resize', 100, 200)
                # cv2.moveWindow('split8_resize', 1600, 600)
                # cv2.imshow('split8_resize', split8_resize)
                cv2.imwrite('D:/PycharmProjects/Num_distinguish/test_data/8.jpg', split8_resize)

            elif flag_step == 9 and g < flag_posi and split_line[9] - split_line[8] >=18:
                split9 = img_final_resize[1:49, split_line[8]:split_line[9]]
                split9_resize = cv2.resize(split9, dsize=(40, 60), fx=0, fy=0)
                # cv2.namedWindow('split9_resize', 0)
                # cv2.resizeWindow('split9_resize', 100, 200)
                # cv2.moveWindow('split9_resize', 1750, 600)
                # cv2.imshow('split9_resize', split9_resize)
                cv2.imwrite('D:/PycharmProjects/Num_distinguish/test_data/9.jpg', split9_resize)
            elif g == flag_posi and 199 - split_line[flag_posi-1] >=15:
                split10 = img_final_resize[1:49, split_line[flag_posi-1]:199]
                split10_resize = cv2.resize(split10, dsize=(40, 60), fx=0, fy=0)
                # cv2.namedWindow('split10_resize', 0)
                # cv2.resizeWindow('split10_resize', 100, 200)
                # cv2.moveWindow('split10_resize', 1600, 600)
                # cv2.imshow('split10_resize', split10_resize)
                cv2.imwrite('D:/PycharmProjects/Num_distinguish/test_data/10.jpg', split10_resize)
            flag_step += 1
#以上为显示被分割出的子字符

        # def mouse_callback(event, x, y, flags, param):
        #     if event == cv2.EVENT_LBUTTONDBLCLK:
        #         print('BGR:', img_resize[y, x], 'x:', x, 'y:', y)

        #cv2.namedWindow('IMG', cv2.WINDOW_NORMAL)
        #cv2.imshow('IMG', img_resize)
        #cv2.imshow('IMG2', img_final_resize)
        #cv2.imshow('IMG3', img_final_bin)
        #cv2.setMouseCallback('IMG', mouse_callback)
        cv2.waitKey(0)

# img_dist = Carlicense_distinguish()
# img_dist.carlicense_distinguish()

def img_resize():
    with open('D:/PycharmProjects/Num_distinguish/train_data/labels.txt', 'r', encoding='UTF-8') as f:
        # (image_path(str), image_label(str))
        imgs = list(map(lambda line: line.strip().split(' '), f))
    ex_img_path = []
    for i in range(len(imgs)):
        ex_img_path.append(imgs[i][0])

    for i in range(len(ex_img_path)):
        ex_img = cv2.imread(ex_img_path[i])
        img = cv2.resize(ex_img, dsize=(40, 60), fx=0, fy=0)
        cv2.imwrite(ex_img_path[i], img)
