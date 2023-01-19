[TOC]

### Code Parser

0. data&code
   IMU数据是从euroc上找的。

​		代码中的一部分是从[okvis](https://github.com/ethz-asl/okvis)中拿到的。至于为什么是okvis，可能是借鉴了很多ethz-asl组里面的工作。

#### Test从测试看代码如何使用

1. [csv.h](https://github.com/ben-strasser/fast-cpp-csv-parser)
A small, easy-to-use and fast header-only library for reading comma separated value (CSV) files. 
2. PoseTest.cpp主要是pose相关的函数。
3. SplineTest.cpp测试spline相关的函数。
4. 

#### PoseSpline核心

1. Pose.hpp & Pose_imp.hpp

​		是从okvis中的*kinematics/Transformation.hpp*代码继承过来的，使用了四元数来表示旋转是从*Quaternion* 类借鉴过来的。其中需要注意的就是位姿的表示方式，是**先平移再旋转**，下面的位姿更新为了兼容ceres，添加了不少的重载`liftJacobian`。

```c++
/// \brief Apply a small update with delta being 6x1.
/// \tparam Derived_delta Deducible matrix type.
/// @param[in] delta The 6x1 minimal update.
/// \return True on success.
template<typename Derived_delta>
bool oplus(const Eigen::MatrixBase<Derived_delta> & delta);
/// \brief Get the Jacobian of the oplus operation (a 7 by 6 matrix).
/// @param[out] jacobian The output Jacobian.
/// \return True on success.
template<typename Derived_jacobian>
bool oplusJacobian(const Eigen::MatrixBase<Derived_jacobian> & jacobian) const;
/// \brief Apply a small update with delta being 6x1 --
///        the Jacobian is a 7 by 6 matrix.
/// @param[in] delta The 6x1 minimal update.
/// @param[out] jacobian The output Jacobian.
/// \return True on success.
template<typename Derived_delta, typename Derived_jacobian>
bool oplus(const Eigen::MatrixBase<Derived_delta> & delta, const Eigen::MatrixBase<Derived_jacobian> & jacobian);
/// \brief Gets the jacobian dx/dChi,
///        i.e. lift the minimal Jacobian to a full one (as needed by ceres).
// @param[out] jacobian The output lift Jacobian (6 by 7 matrix).
/// \return True on success.
template<typename Derived_jacobian>
bool liftJacobian(const Eigen::MatrixBase<Derived_jacobian> & jacobian) const;
```

2. PoseLocalParameter.hpp

​		郑帆大佬的[博客](https://fzheng.me/2018/01/23/okvis-transformation/)中有详细的解释。在关于okvis的代码解读中写的非常详细。

+ `liftJacobian`公式推导$$\ref{1}$$:

$$
\begin{align}
\label{liftJacobian}
J_{lift(6,7)} &= I^{6x7} \\
q^{-1} &= q\{-x[3],-x[4],-x[5],x[6]\} \\
J_{q\_p^{-1}} &= \left[ \begin{array}{ccc} 
2.0 & 0.0 & 0.0 & 0.0 \\ 
0.0 & 2.0 & 0.0 & 0.0 \\ 
0.0 & 0.0 & 2.0 & 0.0 \\ 
\end{array} \right] \\
J_{lift(3:5,3:6)} &= J_{q\_p^{-1}} * quatRightComp \{ q^{-1} \}
\end{align}\tag{1}
$$

+ `quatRightComp`是将四元数转换为Eigen style的矩阵。位于*Quternion.hpp*。

3. Quternion.hpp & QuaternionLocalParameter.hpp

​		与四元数相关的函数，后续用到了就仔细研究，四元数的顺序是**xyzw**。其中很多都是从maplab中的*quaternion_param_jpl.h* & *quaternion_param_jpl.hpp* & *quaternion_param_jpl.cpp*借鉴过来的。

4. PoseSpline.hpp & PoseSpline.cpp

​		PoseSpline.hpp是继承于*BSplineBase.hpp*的。

5. BSplineBase.hpp

​		包含详细的B-spline的定义类。为模板类实现形式，需提供元素类型和B-spline阶数作为模板参数。形参为时间间隔。

+ B-spline：一个**n阶**的B-spline函数，是一个变量为x的**n-1次**分段多项式函数f(x)。分段函数之间，段与段之间的断点被称为**knots**。其实是给定阶数**order**根据一系列的**control points**来确定**knots**的参数。应用：借助样条曲线的平滑性，可以根据实现时间来内插中间任意时刻点的参数，也就是该时刻的状态。

​		B-spline的参数有，计算方式为：

6. PoseSplineUtility.hpp & PoseSplineUtility.cpp

   里面很多都包含了QuaternionSpline和VectorSpaceSpline相关的函数，实现方式是一样的。

7. QuaternionSpline.hpp & QuaternionSpline.cpp

​		*QuaternionSpline*是BSplineBase的一个衍生类，元素类型是四元数Quternion，仍是4阶-3次样条曲线。即Quternion作为想条线的节点待估计参数。

6. QuaternionSplineUtility.hpp

   四元数相关的函数，有一部分是从*Continuous-Time Estimation of attitude using B-splines on Lie groups*中的函数原版。有一些经过改动。

7. VectorSpaceSpline.hpp

​		*VectorSpaceSpline*是BSplineBase的一个**衍生模板类**（模板参数是向量的维度，默认为3），元素类型是N-D向量，仍是4阶-3次样条曲线，即N-D向量作为样条线的节点待估计参数。

6. 

4. Time.hpp & Time_imp.hpp

   时间管理的类。大同小异。

5. TypeTraits.hpp

​		用于给不同的自定义元素提供统一的函数接口，主要是为了和模板函数一起使用的。现有的ElementType 包括Pose, Quqternion(Eigen style) and Eigen::Matrix<double, D,1>。配合BSplineBase的输入参数模板一起使用。

### maplab

1. quaternion_param_jpl.h

### internal

​		从HKUST的VINS中借鉴了一部分代码。

1. pose_local_parameterization.h

### extern

​		原版的okvis的代码。

1. PoseLocalParameterization.hpp

​		[郑帆](https://fzheng.me/2018/01/23/okvis-transformation/)的博客对**位姿变换及其局部参数类**有介绍。

​		OKVIS 的变换类虽然也采用了类似预积分论文中用最小表示的局部小量来做更新的 local parameterization，但不同于预积分论文的,更不同于常规的李群/李代数中的更新方式，OKVIS 的更新策略是将旋转量和平移量完全解耦，对旋转量使用左扰动，对平移量则直接加法：其中四元数小量与李代数小量的关系可以进一步近似。**旋转和平移完全解耦的好处是计算雅克比矩阵非常方便**。
$$
预计分论文(lift-solve-retract)：R Exp (\delta \alpha), p+R \delta p\\
常规的李群李代数: Exp(se3)\\
okvis: x= \bar{x} \boxplus \Delta x: q \leftarrow \delta q \otimes \bar{q},p \leftarrow \delta p+\bar{p} \\
where: \delta q \approx [\frac{1}{2}\delta \alpha;1]
$$
​		同时，针对不同场景，OKVIS 还实现了 PoseLocalParameterization  的几个衍生类：

+ PoseLocalParameterization3d：仅对平移量扰动
+ PoseLocalParameterization4d：仅对平移量和 yaw 角进行扰动
+ PoseLocalParameterization2d：仅扰动 roll 角和 pitch 角



