# 作业 1  

## 一、插值型拟合方法  

1. 使用多项式函数（幂基函数的线性组合）  

- 输入点集（点的数量 $n=7$）  
- 选择拟合方法为 `interpolation polynomial`  

![polynomial](images/polynomial.jpg)

2. 使用 Gauss 基函数的线性组合  

- 输入点集（点的数量 $n=7$）  
- 选择拟合方法为 `interpolation gauss`  
- 输入参数 $\sigma = 1$  

![gauss_0](images/gauss_0.jpg)

- 修改参数 $\sigma \approx 5$  

![gauss_1](images/gauss_1.jpg)

- 修改参数 $\sigma \approx 10$  

![gauss_2](images/gauss_2.jpg)

- 修改参数 $\sigma=20$  

![gauss_3](images/gauss_3.jpg)

## 二、逼近型拟合方法  

1. 最小二乘法  

- 输入点集（点的数量 $n=7$）  
- 选择拟合方法为 `approach leastsquare`  
- 输入幂基函数的最高次数 $m=4$  

![leastsquare](images/leastsquare.jpg)

2. 岭回归  

- 输入点集（点的数量 $n=7$）  
- 选择拟合方法为 `approach ridgeregression`  
- 输入幂基函数的最高次数 $m=6$  
- 输入参数 $\lambda = 0$  

![ridgeregression_0](images/ridgeregression_0.jpg)

- 修改参数 $\lambda \approx 0.1$  

![ridgeregression_1](images/ridgeregression_1.jpg)

- 修改参数 $\lambda=0.5$  

![ridgeregression_2](images/ridgeregression_2.jpg)

- 修改参数 $\lambda=1$  

![ridgeregression_3](images/ridgeregression_3.jpg)