# Inertial navigation basic

**这个仓库的主要内容是基本惯导解算程序。包含严恭敏老师 捷联惯导算法与组合导航原理 上的matlab示例程序，以及我自己写的姿态解算和对准等程序**

---

#1.  遵循“先解释，后执行”的原则，注释总是写在对应程序的前面（或上方）。

#2.  注意：在matlab程序中表示同时具有上标和下标的变量时

​	书上的程序案例按照“上标在前，下标在后”的原则，

​	我们实验室习惯上按照 “下标在前，上标在后”的原则。

因此，书中对于 "从b系到n系的过渡矩阵" Cbn 的写法为Cnb。出于习惯原因我选择后者，因此程序中这些变量的名称与书中有差异，这种差异存在最初几个版本之中。

#3.  总是按照如下规则来表示位置、速度变量：

​	pos : [纬度, 经度, 高度] 单位为rad, rad, m

​	vn : [vE, vN, vU] 单位为m/s

另外，若无特殊说明，表示角度变量的单位总是rad，其他物理量的单位总是相应的国际单位。

#4. 以**_ref**结尾的量，注释中称其为“参考值”，“标准值”。二者均表示“真实”、“没有误差”之意。其中，本人更愿把实测数据中的高精度参考称为“参考值”，把仿真程序生成的绝对准确的结果称为“标准值”或“真实值”。不过，真的写到程序里时也没区分那么细。

#5. 

#6.  除非变量名**k**不得不另作它用，则脚标均用小写**k**表示； **k_init** 特指初值数据或计算中第一个时刻数据的编号

#7. 变量名 **imuError**特指**惯性器件误差**。如需设置多种imu误差，应记为**imuError_1**,**imuError_phins**...

#8. 从程序的准确性和可读性角度来看，在计算/定义/赋值时，加上相应的单位都是值得的。但考虑到大多数变量的单位都是国际单位，出于简洁性考虑，在不引起歧义的前提下，省略国际单位

---

**一些缩写的含义:**
**ref**: reference    **msr**: measurement    **stg**: storage    **err**: error    **prev**: previous    **opt**: optimal  **crt**: current    **init**: initial    **num**: number     **calc**: calculate    **Lp**: loop of ..    **idx**: index    **simData**: simulation data     **fig:** figure    **info:** information    **ts:** time of step

**导航专用缩写:**

**avp**: attitude velocity attitude    **pos**: positon    **att**: attitude    **vn**: ground velocity     **acc**: accelerometer    **gyr**: gyroscope    **ang**: angle    **imu.fb:** specific force in b frame    **imu.wb:** angular velocity in b frame    **imu.tb:** angular increment     **imu.vb:** velocity increment    **navinfo:** navigation information

***

 ***更新时间：2020年12月5日***

