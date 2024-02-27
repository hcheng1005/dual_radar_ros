# PointPillars 部署流程

## 部署一：pfe和backbone分离

原版代码：https://github.com/hova88/PointPillars_MultiHead_40FPS

ROS版本（输出box）： https://github.com/cuiDarchan/PointPillars_MultiHead_40FPS_ROS

> 上述模型采用的数据集为`nuscenes`，pp模型为`MultiHead`

### ONNX文件导出

参考代码： https://github.com/hova88/OpenPCDet

为了方便导出onnx模型，作者重新定义了pfe和backbone_MultHead的python代码。 

代码： https://github.com/hcheng1005/OpenPCDet/tree/master/tools/onnx_utils

### trt文件导出

此处需要用到`onnx2trt`工具。

```python
onnx2trt cbgs_pp_multihead_pfe.onnx -o cbgs_pp_multihead_pfe.trt -b 1 -d 16 
onnx2trt cbgs_pp_multihead_backbone.onnx -o cbgs_pp_multihead_backbone.trt -b 1 -d 16 
```

### ROS集成
主要是CMAKELIST修改。

- 定义CUDA
```cmake
find_package(CUDA REQUIRED QUIET)
include_directories(${CUDA_INCLUDE_DIRS})
```

- 加载相关cu文件
```cmake
# 添加*.cu文件
cuda_add_library(gpu_point_pillars_lib
                src/pointpillars/postprocess.cu
                src/pointpillars/scatter.cu
                src/pointpillars/preprocess.cu
                src/pointpillars/nms.cu
)
```

- 编译gpu相关文件库
```cmake
target_link_libraries(gpu_point_pillars_lib
                    ${CUDA_LIBRARIES} 
                    ${CUDA_CUBLAS_LIBRARIES}
                    ${CUDA_curand_LIBRARY}
                    ${CUDNN_LIBRARY}
                    nvonnxparser
                    nvinfer)
```

- 集成编译库
```cmake
target_link_libraries(lidar_node ${catkin_LIBRARIES}
                                  yaml-cpp
                                  gpu_point_pillars_lib)
```

## 部署二： 一个模型文件

参考工程： https://github.com/NVIDIA-AI-IOT/CUDA-PointPillars/tree/092affc36c72d7b8f7530685d4c0f538d987a94b

> 此工程是基于`KITTI`数据集训练并部署标准PointPillars模型(单头)，并且`只生成一个trt引擎文件`。

由于此工程基于特定的openpcDet版本才能成功导出onnx文件，因此这里重新编写了一个转onnx文件的代码。

地址如下： https://github.com/hcheng1005/OpenPCDet/blob/master/tools/onnx_utils_dual_radar/trans_pointpillar.py 

### ROS部署

TBD

## 基于Dual_Radar数据集的修改
TBD




## 一些报错和解决方法：

- [`GLIBCXX_3.4.29' not found](https://zhuanlan.zhihu.com/p/615111375)
- [undefined reference to `ffi_type_uint32@LIBFFI_BASE_7.0'](https://blog.csdn.net/qq_19278525/article/details/134039141)