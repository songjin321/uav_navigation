## 说明

这个主要是使用双目建立稠密地图，视差法使用的是[ELAS](http://www.cvlibs.net/software/libelas/)

## 使用
dense_reconstruction`是求视差，并发布为点云形式
输入是两张双目的彩色图片和标定文件
输出是视差图和点云

```bash
roslaunch dense_reconstruction dense_reconstruction.launch
```

##　订阅和发布topic
订阅
双目左眼　/camera/left/image_raw
双目右眼  /camera/right/image_raw

发布
点云    /point_cloud
