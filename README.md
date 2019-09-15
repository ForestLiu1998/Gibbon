# Gibbon

## 更新记录

### 2019.9.15
    1. gazebo_simu/launch/lds_scan_rail.launch 
    2.  get_lds_scan_data/
    3.  lds_controler/src/lds_rail_control.cpp
    4.  gazebo_simu/urdf/lds_datasheet/

## Gazebo  Model
   
### lds——lms5xx
    1.  Gazebo中lds模型的坐标在其/joint_states话题发布中是以lds本身为原点建立的坐标系，所以尽管在gazebo世界中定义了lds位置为（0.8,0,0）,/joint_states话题依然发
    　布的位置为（0,0,0）
     
     2.  lms5xx的激光照射角度范围是270°，角分辨率是0.25°，加上初始点，lms5xx扫描范围共能得到1+270/0.25=1081个点，并按顺序存储，get_lds_data_rail.cpp中取中
     心线两侧共save_point_num个点计算x,y,z并保存。
 
 [[lms111工作区域图表]](https://cdn.sick.com/media/pdf/2/42/842/dataSheet_LMS111-10100_1041114_zh.pdf)  
 ![lms111工作区域图表](https://github.com/ForestLiu1998/Gibbon/raw/master/src/picture/lds工作区域图表.png)
