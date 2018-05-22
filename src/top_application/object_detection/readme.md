用于比赛的目标检测包
====================
## 医药包->sift, 运行ＧＵＩ测试:

### 原理

find-object发布objects，然后再将其转换为objectboxs

### 订阅topic

std_msgs/Float32MultiArray objects 

[ObjectId1, objectWidth, objectHeight, h11, h12, h13, h21, h22, h23, h31, h32, h33, ObjectId2...] 

### 发布topic

### Test

objects float32MultiArrary, 占１00%cpu,7HZ,sift

## 小人->颜色