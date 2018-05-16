用于比赛的目标检测包
====================
医药包->sift, 运行ＧＵＩ测试:
订阅objects将其转换为objectboxs
# objects format: 
# [ObjectId1 x,y,w,h]
# 求出四个顶点，然后再求这四个点的最大矩形．
x = h31
y = h32
w = 
# [ObjectId1, objectWidth, objectHeight, h11, h12, h13, h21, h22, h23, h31, h32, h33, ObjectId2...] 
# where h## is a 3x3 homography matrix (h31 = dx and h32 = dy, see QTransform)
std_msgs/Float32MultiArray objects 

小人->颜色

objects float32MultiArrary, 占１00%cpu,7HZ,sift