import Augmentor
# https://blog.csdn.net/qq_33270279/article/details/103505589
# 1. 指定图片所在目录
p = Augmentor.Pipeline("data/all1/")
# 2. 增强操作
# 旋转 概率0.7，向左最大旋转角度10，向右最大旋转角度10
# p.rotate(probability=1,max_left_rotation=20, max_right_rotation=20)
# 放大 概率0.3，最小为1.1倍，最大为1.6倍；1不做变换
# p.zoom(probability=1, min_factor=0.8, max_factor=1.6)
# 曝光变换，
p.random_brightness(probability=1, min_factor=0.2, max_factor=1.5)
# 颜色变换，
p.random_color(probability=1, min_factor=0, max_factor=2)
# 3. 指定增强后图片数目总量
p.sample(500)