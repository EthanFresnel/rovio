# 视觉　惯性里程计 rovio是一个紧耦合，基于图像块的滤波实现的VIO。
[博士论文](https://www.research-collection.ethz.ch/mapping/eserv/eth:50763/eth-50763-02.pdf)

    他的优点是：计算量小(EKF，稀疏的图像块)，但是对应不同的设备需要调参数，参数对精度很重要。
    没有闭环.
    没有mapping thread。
    经常存在误差会残留到下一时刻。
    
# 代码主要分为EKF实现的部分，和算法相关的部分，EKF是作者自己写的一个框架。

## 先分析EKF代码
    lightweight_filtering
    滤波部分 FilterBase.hpp  预测 Prediction.hpp   更新 update.hpp   系统状态 State.hpp
```C
template<typename Meas>
class MeasurementTimeline{
  typedef Meas mtMeas;
  //imu测量的数据存在map中，相当于一个buffer,key是时间，value 是加速度或者角速度或者图像金字塔
  std::map<double,mtMeas> measMap_;
  void addMeas(const mtMeas& meas,const double &t);
}
```
    
    加入imu测量值                FilterBase::addPredictionMeas();
    图像的MeasurementTimeline   FilterBase::addUpdateMeas();
    根据传入时间进行EKF的更新      FilterBase::updateSafe();
      FilterBase::
      FilterBase::
      FilterBase::
## 算法的部分
[算法公司推导 论文参考](https://arxiv.org/pdf/1606.05285.pdf)

    RovioNode.hpp
    ImuPrediction.hpp
    ImgUpdate.hpp
    图像部分主要的代码是 MultilevelPatchAlignement.hpp 
       是一个高斯牛顿法优化，目标点的位置。

    EKF的优化是特征点位置，要是换成IEKF，优化图像块的像素差，可能效果会更好。
    毕竟这东西是个高度非线性函数。

    那个bearing vector的公式我还不会推导，
    对新增的feature的initial depth的比较精确的估计对算法精度有帮助，可以维护个地图，

    当然在地图中做个local mapping thread，　也是可以的，
    但是感觉不能很好的和原来的算法耦合起来就没做。

    这里最需要改进的应该是特征点的选取，原来算法的效率太低了。
    而且会发现选取的很多特征点不是那么明显的角点，有更好的选择，
    不过为了保持距离的限制，妥协了。还有就是速度太慢了。

    出现发散的情况，一般就是outlier太多了，没有追踪足够的特征点。
    因为速度发散，会导致图像更新为了矫正在特征点深度位置上存在巨大的错误速度，
    把深度设到无穷远去，这样图像更新就没有作用，进一步导致速度发散。一发散就不可能回来了。


# README #

This repository contains the ROVIO (Robust Visual Inertial Odometry) framework. The code is open-source (BSD License). Please remember that it is strongly coupled to on-going research and thus some parts are not fully mature yet. Furthermore, the code will also be subject to changes in the future which could include greater re-factoring of some parts.

Video: https://youtu.be/ZMAISVy-6ao

Paper:  http://dx.doi.org/10.3929/ethz-a-010566547

Please also have a look at the wiki: https://github.com/ethz-asl/rovio/wiki

### Install without opengl scene ###
Dependencies:
* ros
* kindr (https://github.com/ethz-asl/kindr)
* lightweight_filtering (as submodule, use "git submodule update --init --recursive")

```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Install with opengl scene ###
Additional dependencies: opengl, glut, glew (sudo apt-get install freeglut3-dev, sudo apt-get install libglew-dev)
```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```

### Euroc Datasets ###
The rovio_node.launch file loads parameters such that ROVIO runs properly on the Euroc datasets. The datasets are available under:
http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

### Further notes ###
* Camera matrix and distortion parameters should be provided by a yaml file or loaded through rosparam
* The cfg/rovio.info provides most parameters for rovio. The camera extrinsics qCM (quaternion from IMU to camera frame, Hamilton-convention) and MrMC (Translation between IMU and Camera expressed in the IMU frame) should also be set there. They are being estimated during runtime so only a rough guess should be sufficient.
* Especially for application with little motion fixing the IMU-camera extrinsics can be beneficial. This can be done by setting the parameter doVECalibration to false. Please be carefull that the overall robustness and accuracy can be very sensitive to bad extrinsic calibrations.
