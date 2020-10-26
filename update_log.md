## Update Log

   ***Harbin Engineering University***

**Author: Zhang Zheng **

**Last version: v 2.0**

**Update date: 2020年10月26日**

---

### v 1.0 	2020年10月12日

```
最初版本，内容是严恭敏老师《捷联惯导算法与组合导航原理》第八章中的程序。
```

#### v 1.1	2020年10月15日
    . 增加了OBA法进行初始对准的程序
    . 增加了用量测矢量计算最优四元数的QUEST算法程序
    . 增加了设置imu误差的程序
    . 增加了规范化姿态误差角的程序
    . 更新了README.md

---

### V 2.0	2020年10月26日

```
. 新增了第二版OBA算法 optimal_based_alignment_vn_aid_v2.m，另外修改了第一版的名称
. 原 imuerror.m 函数更名为 imuerrorset.m
. 修改了轨迹发生器 trajectory_simulator.m 现在该程序运行后会自动生成.mat文件
. 新增了角运动发生器  angle_motion_simulator_v1.m ，目前能生成多组正弦运动叠加的角运动，caculate_theta 是专门为其编写的子函数。
. 新增了验证姿态更新算法有效性的attitude_update_test.m
. 微调了一些程序和注释
. 更新了README.md
```

