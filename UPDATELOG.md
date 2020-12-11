## Update Log

   ***Harbin Engineering University***

**Author: Zhang Zheng**

**Last version: v 2.0**

**Update date: 2020年12月11日**

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

#### v 2.1 2020年12月11日

```
. 删除以下文件：
angle_motion_simulator_v1.m; atterrnorml.m; caculate_theta.m; 
. 重命名：update_log.md --> UPDATELOG.md
. 以下文件可能发生过改动（未对程序本身造成大的影响）：
attitude_update_test.m; cnscl.m; earth.m; gvar_earth; imuadderr.m; imuerrorset.m; insupdate.m; kfft15.m; kfupdate.m; optimal_based_alignment_vn_aid_v1.m; optimal_based_alignment_vn_aid_v1.m; trajectory_simulator.m
. 新增 Runge_Kutta_att_update.m 龙格库塔法更新姿态
. 新增 addImuError.m 添加imu误差到一段数据中
. 新增 angle_motion_simulator.m 生成角运动轨迹和imu数据
. 新增 demo_SINS.m SINS演示程序
. 新增 demo_generate_gyroscope_error.m 生成陀螺误差演示程序
. 新增 demo_generate_imu_Error.m 生成imu误差的演示程序
. 新增 demo_static_SINS 静态情况下SINS导航的演示程序
. 新增 ecef_to_geodetic.m 将ECEF系下的坐标转化为大地坐标（纬度，经度，高度）
. 新增 generateAccError.m 生成加计误差
. 新增 generateGyrError.m 生成陀螺误差
. 新增 geodetic_to_ecef.m 将大地坐标转化为ECEF系下的坐标
. 新增 myequal.m 判断浮点数是否近似相等
. 新增 progressTip.m 提示程序运行进度的小程序
. 新增 setGl_all.m; setGl_unit.m; setGl_earth.m setGl_navInfo.m 设置全局变量
```

