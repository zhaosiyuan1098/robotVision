# robotVision_homework
记录2022春季学期机器人视觉课程大作业
***
## 1 改进的张正友标定法
### 说明
solution文件夹为解法源码，word/pdf为提交论文，pptx为文中插图原图。
### 使用方法
下载代码至本地，将需要标定的图片放入/solution1文件夹中的/traditional_solve/image内，运行main.m  
quick_solve调用matlab提供函数求解，具体使用方法同上。
### 作用
完成相机标定，获取相机内参矩阵A，畸变系数k1、k2、k3、k4,以及每张图片对应的外参矩阵RT、内外参矩阵之积H

## 2 机器人视觉全自动测量

### 说明
solution文件夹为解法源码，word/pdf为提交论文，pptx为文中插图原图。
### 使用方法
下载代码至本地，/homework2/toMeasure.bmp为待测量图片，运行/solution2文件夹中的main.m  
运行结果中sub开头变量为亚像素结果，true开头变量为真实尺寸结果。
### 作用
测量图片中小孔的直径、圆心坐标、各小孔圆心间半径。

## 3 视频人脸检测与跟踪
### 说明
solution文件夹为解法源码，word/pdf为提交论文，pptx为文中插图原图。
### 使用方法
下载代码至本地，标定信息为mat文件，运行/solution2文件夹中的main.m 
### 作用
识别并跟踪视频中最大的人脸，计算其运动速度。
