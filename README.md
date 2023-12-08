### __Report__



__图像预处理__

亮度调整：
乘以系数对图像进行亮度调整-通过调整 alpha 变量实现。 降低亮度以更好地以颜色作为特征识别灯带。

通道分离和颜色相减：
cv::split() 用于将图像分割为三个通道（红、绿、蓝）。
cv::subtract() 用于对红色和蓝色通道的图像进行相减操作，产生差异图像，从而突出了蓝色的部分。

严格阈值化：
使用 cv::threshold() 对差异图像进行阈值化处理，将目标颜色区域二值化为白色（255）。

__轮廓检测与矩形合并__

轮廓检测：
cv::findContours() 用于寻找图像中的轮廓信息，利用检测到的轮廓进一步处理。

外接矩形提取：
cv::boundingRect() 获取每个轮廓的外接矩形信息。

矩形聚类与合并：
为了使识别效果更好，减少密集的杂点等，使用clusterRects() 函数用于将检测到的矩形根据位置和大小进行聚类和合并。

__位置和角度的参数输出__

灯带位置信息提取：
对合并后的矩形提取位置信息，例如矩形的 x 和 y 坐标，并输出到控制台或其他输出流中。
旋转矩形和角度计算：

使用 cv::minAreaRect() 计算每个灯带外接矩形的最小旋转矩形信息，包括角度信息。
将旋转矩形的角度提取出来，并输出到控制台或其他输出流中。

## 视频处理
视频帧读取：
使用 cv::VideoCapture 对象读取视频帧。
图像处理：

处理视频的每一帧图像，对每一帧执行前述的预处理和灯带识别操作。
ROS 环境集成：

__显示处理后的视频__

原视频是30帧的，所以用cv::waitKey(33.3)；在每帧之间停留33.3毫秒。





