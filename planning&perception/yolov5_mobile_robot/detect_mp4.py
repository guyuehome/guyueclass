#!/usr/bin/python3
# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Run inference on images, videos, directories, streams, etc.

Usage:
    $ python path/to/detect.py --source path/to/img.jpg --weights yolov5s.pt --img 640
"""
import rospy
from std_msgs.msg import Int8

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.experimental import attempt_load
from utils.datasets import LoadImages, LoadStreams
from utils.general import apply_classifier, check_img_size, check_imshow, check_requirements, check_suffix, colorstr, \
    increment_path, non_max_suppression, print_args, save_one_box, scale_coords, set_logging, \
    strip_optimizer, xyxy2xywh
from utils.plots import Annotator, colors
from utils.torch_utils import load_classifier, select_device, time_sync


#被torch.no_grad()包住的代码,不需要计算梯度,也不会进行反向传播 
@torch.no_grad()


#检测函数
def run(weights=ROOT / 'yolov5s.pt',  # model.pt path(s)
        source='/home/ljj/Desktop/windows/abc.mp4',  # file/dir/URL/glob, 0 for webcam
        project=ROOT / 'runs/detect/exp',  # save results to project/name
        imgsz=640,  # inference size (pixels)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        max_det=1000,  # maximum detections per image
        device='cpu',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        ):

    #初始化ROS节点
    rospy.init_node('yolo', anonymous=True)
    # 创建话题发布者
    pub = rospy.Publisher('/person', Int8, queue_size=10)
    

    # Directories
    save_dir = increment_path(Path(project), exist_ok=False, mkdir=True)  # increment and make dir
    print(save_dir)

    # Initialize
    set_logging()
    device = select_device(device)


    ##################################首先加载模型#########################################
    model = attempt_load(weights, map_location=device)
    stride = int(model.stride.max())  # model stride
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names



    ################################然后加载识别对象#######################################
    dataset = LoadImages(str(source), img_size=imgsz, stride=stride, auto=True)
    a = ""

    ####################################开始识别##########################################
    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.parameters())))  # run once


    dt, seen = [0.0, 0.0, 0.0], 0

    
    #注意这个循环是，每有一张图片循环一次
    for path, img, im0s, vid_cap in dataset:
        #首先将图片转为张量
        t1 = time_sync()
        img = torch.from_numpy(img).to(device)
        img = img / 255.0  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim
        t2 = time_sync()
        dt[0] += t2 - t1

        #开始推理，也就是检测
        pred = model(img)[0]
        t3 = time_sync()
        dt[1] += t3 - t2

        #NMS非极大值抑制，过滤、去除框选
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        dt[2] += time_sync() - t3


        #注意这个循环是，每检测到一种特征循环一次
        for i, det in enumerate(pred):  # per image
            
            im0 = im0s.copy()
            person = 0
            

            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # 注意这个循环是，某种特征每检测到一个循环一次
                # 其中xyxy[0]-xyxy[3]对应目标特征的x0\y0\x1\y1
                # int(cls)对应获得识别到的特征种类的编号，names[int(cls)]是识别到的种类
                # config对应此次特征的置信度
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)
                    if c == 0 :
                        person += 1 # 特征计数
                    
                        # 画框
                        label = f'{names[int(cls)]} {conf:.2f}'
                        annotator = Annotator(im0, line_width=3 , example=str(names))
                        annotator.box_label(xyxy, label, color=colors(c, True))
                    
        # 保存识别结果、
        if a != path:  # new video
            a = path
            vid_writer = None
            if isinstance(vid_writer, cv2.VideoWriter):
                vid_writer.release()  # release previous video writer
            fps, w, h = 30, im0.shape[1], im0.shape[0]
            vid_writer = cv2.VideoWriter(str(save_dir / Path(path).name), cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        vid_writer.write(im0)

        # 发布识别信息话题
        pub.publish(person)

        # 显示识别结果
        cv2.imshow('', im0)
        cv2.waitKey(1)  # 1 millisecond

        # 确保关闭正常
        if rospy.is_shutdown():
            exit()



if __name__ == "__main__":
    run()


