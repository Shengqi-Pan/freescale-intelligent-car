飞思卡尔全国大学生智能车竞赛 直立节能组

# 项目日志

## 6.24

1. 测定陀螺仪和加速度计bias和比例

2. 调通了卡尔曼滤波

3. 测定小车过轴线时的倾斜角，设定其为平衡角，23.8°

4. pd参数调节，最终角度确定为23.87°，角度环参数P=2000 D=50

## 6.25, 6.26

1. 调了下电磁，实现基础的转向

2. 控速有点迷，一直没找到bug，进展不大

## 6.27

1. 找到控速bug，因为限幅使得控速的角度附加全部处于死区，调节不过来

其他可能导致bug的原因：

- P太大

- 角度梯度太小

- angle_bias太小

- 限幅值太小

- 设定速度太大