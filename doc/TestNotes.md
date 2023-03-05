[TOC]

## Code Parser

0. data&code
   IMU数据是从euroc上找的。

​		代码中的一部分是从[okvis](https://github.com/ethz-asl/okvis)中拿到的。至于为什么是okvis，可能是借鉴了很多ethz-asl组里面的工作。

## Test从测试看代码如何使用

1. [csv.h](https://github.com/ben-strasser/fast-cpp-csv-parser)
A small, easy-to-use and fast header-only library for reading comma separated value (CSV) files. 
2. PoseTest.cpp主要是pose相关的函数。
3. SplineTest.cpp测试spline相关的函数。已经包含了spline拟合和求导估计的部分。
4. UtilityTest.cpp测试四元数相关的函数。包含jacobian计算和不同类型四元数的基础运算。
4. testPoseSplineSampleError.cpp测试样条线拟合-插值的函数。
4. QuaternionCeresTest.cpp
4. 

### PoseSpline核心

#### Pose.hpp & Pose_imp.hpp

​		Pose类。从okvis中的*kinematics/Transformation.hpp*代码继承过来的，使用了四元数来表示旋转是从*Quaternion* 类借鉴过来的。其中需要注意的就是位姿的表示方式，是**先平移再旋转**，下面的位姿更新为了兼容ceres，添加了不少的重载`liftJacobian`。

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

#### PoseLocalParameter.hpp

​		位姿更新和jacobian矩阵求解。郑帆大佬的[博客](https://fzheng.me/2018/01/23/okvis-transformation/)中有详细的解释。在关于okvis的代码解读中写的非常详细。公式推导见$\ref{okvis_oplusJacobian}$,$\ref{okvis_liftJacobian}$。

+ `liftJacobian`公式推导$\ref{okvis_liftJacobian_0}$,这里将旋转（四元数）和平移写在一起了。分开的形式见Quternion.hpp。

$$
\begin{align}
\label{okvis_liftJacobian_0}
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

+ `quatRightComp`是将四元数转换为4x4的矩阵，是将四元数表示为矩阵形式做乘法（左乘）的方式。

#### Quternion.hpp & QuaternionLocalParameter.hpp

​		与四元数相关的函数，四元数的顺序是**xyzw**。其中很多都是从maplab中的*quaternion_param_jpl.h* & *quaternion_param_jpl.hpp* & *quaternion_param_jpl.cpp*借鉴过来的。这里的四元数实现了两种方式：liftJacobian和plusJacobian。

> plus雅克比（**增量 Jacobian**）描述了四元数对于右乘以小微小旋转的变化的导数（四维误差除以三维增量）。它用于估计四元数对于角速度向量的导数$\frac{\partial q}{\partial \delta\alpha}$。
>
> lift雅可比（lift Jacobian）描述了四元数对于角速度向量的导数和对应的小微小旋转导数之间的关系(三维误差除以4维状态)。它用于将四元数对于角速度向量的导数转换为对应的小微小旋转的导数$\frac{\partial \delta \alpha}{\partial q}$。

+ hamilton和JPL的区别是四元数不同的定义方式，和jacobian的计算方式无关。

- [x] 求导的部分对着公式看，非常好懂。
- [x] 这里四元数的更新是采用了$\ref{okvis_quater_plus}$的方式，旋转和平移解耦开，方便计算jacobian。公式写的很清楚了。

#### BSplineBase.hpp

​		包含详细的B-spline的定义类。为模板类实现形式，需提供元素类型和B-spline阶数作为模板参数。形参为时间间隔。时间间隔和knot的number是一一对应的，原始Spline中knot的分布可以是均匀的，可以不是，如果是按照时间的话，时间采样如果保持稳定，那么基本上可以说是均匀的，否则的话就是不均匀分布的样条线。

+ B-spline：一个**k阶**的B-spline函数，是一个变量为x的**k-1次**分段多项式函数f(x)。分段函数之间，段与段之间的断点被称为**knots**。其实是给定阶数**order**根据一系列的**control points**来确定**knots**的参数。应用：借助样条曲线的平滑性，可以根据实现时间来内插中间任意时刻点的参数，也就是该时刻的状态。[详见](http://whu-lyh.github.io/blogs/2023/02/09/B-Spline/)，及Efficient Derivative Computation for Cumulative B-Splines on Lie Groups。
+ B-spline的控制点的数量和Spline的分段数量，阶数有关=分段数+阶数-1。

##### 从De Boor–Cox 形式改成cumulative representation的形式。

$\mathbf{p}$为控制点上的参数向量。$k$为阶数。$n$为控制点个数（注意和其他文章中的相区别）。
$$
\begin{align}
\label{Cox-de Boor Recursion Formula w gcp} 
N(t)&= \Sigma_{i=0}^{n} \mathbf{p}_{i}*B_{i,p}(t) \\
\end{align}
$$
cumulative 形式如下：
$$
\begin{align}
N(t)&=\widetilde{B}_{0,k}(t)\mathbf{p}_0+\Sigma_{i=1}^{n}\widetilde{B}_{i,k}(t)(\mathbf{p}_i-\mathbf{p}_{i-1})
\end{align}
$$
where $\widetilde{B}_{i,k}(t)=\Sigma_{s=i}^{n}{B}_{s,k}(t)$, $s$ the index of control point $\mathbf{p}$.

##### 再转成Matrix Representation形式(Toeplitz Matrix)。

$$
\underbrace{\mathbf{p}(u)}_{d \times 1}=\underbrace{\left(\mathbf{p}_i, \quad \mathbf{p}_{i+1}, \quad \cdots, \quad \mathbf{p}_{i+k-1}\right)}_{d \times k} \underbrace{M^{(k)}}_{k \times k} \underbrace{\mathbf{u}}_{k \times 1}
$$

where $\mathbf{u}$，$M^{(k)} \in \mathbb{R}^{k \times k}$是由如下组成的混合矩阵：
$$
\begin{gathered}
m_{s, n}^{(k)}=\frac{C_{k-1}^n}{(k-1) !} \sum_{l=s}^{k-1}(-1)^{l-s} C_k^{l-s}(k-1-l)^{k-1-n} \\ s, n \in\{0, \ldots, k-1\},
\end{gathered}
$$
where $C_k^s=\frac{k !}{s !(k-s) !}$. 在上述例子中，$(k-1) !$当k=4，也就是cubic spline的时候值为6，对应于代码中的6.

cumulative splines的形式如下：
$$
\mathbf{p}(u)=\left(\mathbf{p}_i, \quad \mathbf{d}_1^i, \quad \cdots, \quad \mathbf{d}_{k-1}^i\right) \widetilde{M}^{(k)} \mathbf{u}
$$
where $\widetilde{m}_{j, n}^{(k)}=\sum_{s=j}^{k-1} m_{s, n}^{(k)}$，$d_j^i=\mathbf{p}_{i+j}-\mathbf{p}_{i+j-1}$.

​		$\widetilde{M}^{(k)}$的第一行总是等于单位向量$\mathbf{e}_0$. 特别地，定义如下：
$$
\boldsymbol{\lambda}(u)=\widetilde{M}^{(k)} \mathbf{u}, \lambda_0(u)\equiv 1
$$
​		k阶B-spline表达下，在u（就是t经过归一化之后的值，归一化使得计算更稳定）位置处的值可以写为：
$$
\begin{align}
\mathbf{p}(u)=\mathbf{p}_i+\Sigma_{j=1}^{k-1}\lambda_j(u) \cdot d_j^i,d_j^i=\mathbf{p}_{i+j}-\mathbf{p}_{i+j-1}
\end{align}
$$

#### PoseSpline.hpp & PoseSpline.cpp

​		PoseSpline.hpp是继承于*BSplineBase.hpp*的。（模板参数是pose）

#### PoseSplineUtility.hpp & PoseSplineUtility.cpp

​		PSUtility类。里面很多都包含了QuaternionSpline和VectorSpaceSpline相关的函数，实现方式是一样的。**evalue函数就是内插函数**。

​		PoseSplineEvaluation类。

#### QuaternionSpline.hpp & QuaternionSpline.cpp

​		*QuaternionSpline*是BSplineBase的一个衍生类，元素类型是四元数Quternion，仍是4阶-3次样条曲线。即Quternion作为样条线的节点待估计参数。在初始化时根据传入的index（时间戳）+四元数值构造spline（需要进行最小二乘的拟合得到最佳控制点的值），然后通过eval*相关的函数内插到任意t时刻的四元数。最小二乘中使用的误差函数来源于QuaternionSplineSampleError类。还有一些函数不知道是做什么的，`evalOmega`。

#### QuaternionSplineUtility.hpp & QuaternionSplineUtility.cpp

​		QSUtility类（TODO）。四元数+Spline相关的函数，就是提供了如何进行内插的接口函数。注意beta就是不同阶数的基函数值。

+ 这里为了好求导，使用的并不是de boor的方式，而是使用的[Toeplitz Matrix](https://www.cnblogs.com/JingeTU/p/13513011.html)表达的。见[B-Spline探秘](https://whu-lyh.github.io/blogs/2023/02/09/B-Spline/)。

```c++
Quaternion QSUtility::EvaluateQS(double u,
                                 const Quaternion &Q0,
                                 const Quaternion &Q1,
                                 const Quaternion &Q2,
                                 const Quaternion &Q3)
{
    Eigen::Vector3d Phi1 = Phi(Q0, Q1);
    Eigen::Vector3d Phi2 = Phi(Q1, Q2);
    Eigen::Vector3d Phi3 = Phi(Q2, Q3);

    Quaternion r1 = r(beta1(u), Phi1);
    Quaternion r2 = r(beta2(u), Phi2);
    Quaternion r3 = r(beta3(u), Phi3);

    return quatLeftComp(Q0) * quatLeftComp(r1) * quatLeftComp(r2) * r3;
}
template <typename T>
static T beta0(T u)
{
    return T(1.0);
}
template <typename T>
static T beta1(T u)
{// cubicBasisFun1(u) + cubicBasisFun2(u) + cubicBasisFun3(u)
    return T((u * u * u - T(3) * u * u + T(3) * u + T(5)) / T(6.0));
}
template <typename T>
static T beta2(T u)
{// cubicBasisFun2(u) + cubicBasisFun3(u)
    return T((-T(2) * u * u * u + T(3) * u * u + T(3) * u + T(1)) / T(6.0));
}
template <typename T>
static T beta3(T u)
{
    return cubicBasisFun3(u);
}
```

#### VectorSpaceSpline.hpp

​		*VectorSpaceSpline*是BSplineBase的一个**衍生模板类**（模板参数是向量的维度，默认为3），元素类型是N-D向量，仍是4阶-3次样条曲线，即N-D向量作为样条线的节点待估计参数。就和PoseSpline这个类是一样的，只不过换了元素类型。

#### PoseSplineSampleError.hpp & PoseSplineSampleError.cpp

​		主要是cost function函数，误差是6维的，参数包括4个位姿节点的参数。包含了如何计算误差以及poseSpline的jacobian解析解。**样条线对控制点求导**。有一个minimal的jacobian就是jacobian（似乎是一样的？）。

​		Evaluate中都是调用的EvaluateWithMinimalJacobians。

​		首先根据对应的低阶基函数得到所处时刻的样条线的拟合值。然后误差对相关的控制点处的位姿求**liftJacobian**。

#### PoseSplineSampleTemporalError.hpp & PoseSplineSampleTemporalError.cpp

​		同PoseSplineSampleError.hpp & PoseSplineSampleError.cpp。误差项多了一个关于时间的？

#### QuaternionError.hpp

#### QuaternionSplineSampleError.cpp & QuaternionSplineSampleError.hpp

​		A derived Ceres cost functor to compute the Quaternon errer between quaternion measurement and the evaluation value from spline. For cubic bspline, the evaluation at time t is related to **four** contor points, so this functor requires **four** paramters. 

​		误差项是三维的，有4个待估计的参数对应4个控制点。

+ To simplified the jacobian, we define $error = /hat \ominus /meas$.

#### QuaternionOmegaSampleFunctor.hpp & QuaternionOmegaSampleFunctor.cpp

#### LinearAccelerateSampleError.hpp & LinearAccelerateSampleError.cpp

#### VectorSplineSampleAutoError.hpp & VectorSplineSampleAutoError.cpp

#### OmegaExtrinsicTemperalError.hpp

#### NumbDifferentiator.hpp

​		*A simple class to do quickly finite differencing of an error functor to get a numberic-diff jacobian for minimal parameterization.*

#### ErrorInterface.hpp

​		*A simple interface class that other error classes should inherit from*  okvis代价函数的基类。提供一些接口函数。

#### Time.hpp & Time_imp.hpp

​		时间管理的类。大同小异。

#### TypeTraits.hpp

​		适配器，额外类的萃取器，不侵入原始类别的做法。独立于用于给不同的自定义元素提供统一的函数接口，主要是为了和模板函数一起使用的。现有的ElementType 包括Pose, Quqternion(Eigen style) and Eigen::Matrix<double, D,1>。配合BSplineBase的输入参数模板一起使用。

## maplab

1. quaternion_param_jpl.h

## internal

​		从HKUST的VINS中借鉴了一部分代码。

#### pose_local_parameterization.h & pose_local_parameterization.cpp

​		hamilton形式的PoseLocalParameterization。和其他模块的四元数的更新方式不一样，可能是JPL的区别？

#### utility.h & utility.cpp

​		旋转矩阵相关的内容。不同类型的变换。

## extern

​		原版的okvis的代码。

#### PoseLocalParameterization.hpp

​		[郑帆](https://fzheng.me/2018/01/23/okvis-transformation/)的博客对**位姿变换及其局部参数类**有介绍。

​		OKVIS 的变换类虽然也采用了类似预积分论文中用最小表示的局部小量来做更新的 local parameterization，但不同于预积分论文的,更不同于常规的李群/李代数中的更新方式，OKVIS 的更新策略是将旋转量和平移量完全解耦，对旋转量使用**左扰动**，对平移量则直接加法：其中四元数小量与李代数小量的关系可以进一步近似。**旋转和平移完全解耦的好处是计算雅克比矩阵非常方便**。
$$
\begin{align}
\label{okvis_quater_plus}
预计分论文(lift-solve-retract)：&R Exp (\delta \alpha), p+R \delta p\\
常规的李群李代数: &Exp(se3)\\
okvis: &x= \bar{x} \boxplus \Delta x: q \leftarrow \delta q \otimes \bar{q},p \leftarrow \delta p+\bar{p} \\
where: &\delta q \approx [\frac{1}{2}\delta \alpha;1]
\end{align}
$$
​		同时，针对不同场景，OKVIS 还实现了 PoseLocalParameterization  的几个衍生类：

+ PoseLocalParameterization3d：仅对平移量扰动
+ PoseLocalParameterization4d：仅对平移量和 yaw 角进行扰动
+ PoseLocalParameterization2d：仅扰动 roll 角和 pitch 角

#### Transformation.hpp & Transformation_imp.hpp

​		**Homogeneous transformations.** *This relates a frame A and B: T_AB; it consists of translation r_AB (represented in frame A) and Quaternion q_AB (as an Eigen Quaternion). see also the RSS'13 / IJRR'14 paper or the Thesis.*

​		okvis中的公式和源码非常对应。**oplusJacobian和liftJacobian得到的jacobian矩阵维度是不同的**。其中根据四元数的乘法计算采用矩阵$Q$的形式表示为：$q\otimes p=Q_{p}q$,$Q_{p}$是将四元数表示为4x4矩阵。

​		oplusJacobian（**增量jacobian**）如下所示：
$$
\begin{align}
\label{okvis_oplusJacobian}
\frac{\partial x}{\partial \Delta x} &:\\
\frac{\partial q}{\partial \delta \alpha} &= \frac{\partial Q_{\bar{q}}\,\delta q}{\partial \delta \alpha} = \frac{1}{2}Q_{\bar{q}}I_{4\times 3}\\
\frac{\partial p}{\partial \delta p} &= I_{3}\\
\frac{\partial q}{\partial \delta p} &= \frac{\partial p}{\partial \delta \alpha} = 0
\end{align}
$$
​		liftJacobian（**lift jacobian**）如下所示：
$$
\begin{align}
\label{okvis_liftJacobian}
\frac{\partial \Delta x}{\partial x} &:\\
\frac{\partial \delta \alpha}{\partial q} &=\frac{\partial\delta\alpha}{\partial\delta q} \frac{\partial q\otimes\bar{q}^{-1}}{\partial q}  = 2I_{3\times 4}\,Q_{\bar{q}^{-1}}\\
\frac{\partial \delta p}{\partial p} &= I_3\\
\frac{\partial \delta \alpha}{\partial p} &= \frac{\partial \delta p}{\partial q} = 0
\end{align}
$$

```c++
template <typename Derived_jacobian>
inline bool Transformation::oplusJacobian(
const Eigen::MatrixBase<Derived_jacobian> &jacobian) const
{
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 7, 6);
    Eigen::Matrix<double, 4, 3> S = Eigen::Matrix<double, 4, 3>::Zero();
    const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian).setZero();
    const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian).template topLeftCorner<3, 3>().setIdentity();
    S(0, 0) = 0.5;
    S(1, 1) = 0.5;
    S(2, 2) = 0.5;
    const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian).template bottomRightCorner<4, 3>() = okvis::kinematics::oplus(q_) * S;
    return true;
}
template <typename Derived_jacobian>
inline bool Transformation::liftJacobian(const Eigen::MatrixBase<Derived_jacobian> &jacobian) const
{
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 6, 7);
    const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian).setZero();
    const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian).template topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity();
    const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian).template bottomRightCorner<3, 4>() = 2 * okvis::kinematics::oplus(q_.inverse()).template topLeftCorner<3, 4>();
    return true;
}
```

#### TransformVectorError.hpp & TransformVectorError.cpp

齐次变换矩阵的代价函数，涉及到求导。和PoseSplineSampleError中的类似。

####  RoatateVectorError.hpp & RoatateVectorError.cpp

旋转向量的代价函数



