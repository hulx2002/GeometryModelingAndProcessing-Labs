# 作业 5（曲线细分）  

## 一、逼近型细分  

Chaikin 方法（二次 B 样条）  

- 输入点集  
- 选择细分方法 `approach chaikin`  
- 选择闭合曲线 `closed curve`  
- 迭代次数 $iterations=0$  

![chaikin_0](images/chaikin_0.jpg)

- 迭代次数 $iterations=1$  

![chaikin_1](images/chaikin_1.jpg)

- 迭代次数 $iterations=2$  

![chaikin_2](images/chaikin_2.jpg)

- 迭代次数 $iterations=3$  

![chaikin_3](images/chaikin_3.jpg)

三次 B 样条细分方法  

- 输入点集  
- 选择细分方法 `approach cubicbspline`  
- 选择闭合曲线 `closed curve`  
- 迭代次数 $iterations=0$  

![cubicbspline_0](images/cubicbspline_0.jpg)

- 迭代次数 $iterations=1$  

![cubicbspline_1](images/cubicbspline_1.jpg)

- 迭代次数 $iterations=2$  

![cubicbspline_2](images/cubicbspline_2.jpg)

- 迭代次数 $iterations=3$  

![cubicbspline_3](images/cubicbspline_3.jpg)

## 二、插值型细分  

4 点细分方法  

- 输入点集  
- 选择细分方法 `interpolation 4points`  
- 选择闭合曲线 `closed curve`  
- 输入参数 $\alpha=0.125$  
- 迭代次数 $iterations=0$  

![4points_0](images/4points_0.jpg)

- 迭代次数 $iterations=1$  

![4points_1](images/4points_1.jpg)

- 迭代次数 $iterations=2$  

![4points_2](images/4points_2.jpg)

- 迭代次数 $iterations=3$  

![4points_3](images/4points_3.jpg)