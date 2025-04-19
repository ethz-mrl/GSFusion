<h1 align="center"> GSFusion: Online RGB-D Mapping Where Gaussian Splatting Meets TSDF Fusion </h1>

<h3 align="center"> Jiaxin Wei and Stefan Leutenegger </h3>

<h3 align="center">
  <a href="https://arxiv.org/abs/2408.12677">Paper</a> | <a href="https://youtu.be/rW8o_cRPZBg">Video</a> | <a href="https://gs-fusion.github.io/">Project Page</a>
</h3>

<p align="center">
  <a href="">
    <img src="./media/teaser.gif" alt="teaser" width="100%">
  </a>
</p>

<p align="center"> All the reported results are obtained from a single Nvidia RTX 3060 GPU. </p>

Abstract: *Traditional volumetric fusion algorithms preserve the spatial structure of 3D scenes, which is beneficial for many tasks in computer vision and robotics. However, they often lack realism in terms of visualization. Emerging 3D Gaussian splatting bridges this gap, but existing Gaussian-based reconstruction methods often suffer from artifacts and inconsistencies with the underlying 3D structure, and struggle with real-time optimization, unable to provide users with immediate feedback in high quality. One of the bottlenecks arises from the massive amount of Gaussian parameters that need to be updated during optimization. Instead of using 3D Gaussian as a standalone map representation, we incorporate it into a volumetric mapping system to take advantage of geometric information and propose to use a quadtree data structure on images to drastically reduce the number of splats initialized. In this way, we simultaneously generate a compact 3D Gaussian map with fewer artifacts and a volumetric map on the fly. Our method, GSFusion, significantly enhances computational efficiency without sacrificing rendering quality, as demonstrated on both synthetic and real datasets.*

## News
- **[2025-04-19]**: Add a new reader for TUM-RGBD dataset.
- **[2024-12-06]**: Released the code of GSFusion.
- **[2024-11-09]**: Our paper has been accepted by Robotics and Automation Letters (RAL)!
- **[2024-08-22]**: Released an automatic evaluation system for GSFusion and provide several pre-trained models for assessment.


## Build

Install the dependencies

* GCC 7+ or clang 6+ (for C++ 17 features)
* CMake 3.24+
* Eigen 3
* OpenCV 3+
* CUDA 11.7+
* LibTorch (see setup instructions below)
* Open3D (see setup instructions below)
* GLut (optional, for the GUI)
* Threading Building Blocks (TBB) (optional, for some C++ 17 features)
* OpenNI2 (optional, for Microsoft Kinect/Asus Xtion input)
* Make (optional, for convenience)

On Debian/Ubuntu you can install some of the above dependencies by running:

``` sh
sudo apt --yes install git g++ cmake libeigen3-dev libopencv-dev libtbb-dev freeglut3-dev libopenni2-dev liboctomap-dev make
```

Clone the repository and its submodules:

``` sh
git clone --recursive https://github.com/goldoak/GSFusion
# If you cloned the repository without the --recursive option, run the following command:
git submodule update --init --recursive
```

Set up LibTorch:

```sh
cd GSFusion
wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu118.zip  
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cu118.zip -d third_party/
rm libtorch-cxx11-abi-shared-with-deps-2.0.1+cu118.zip
```

Set up Open3D:
```sh
cd GSFusion
wget https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.18.0.tar.xz
tar -xvf open3d-devel-linux-x86_64-cxx11-abi-cuda-0.18.0.tar.xz -C third_party
mv third_party/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.18.0 third_party/open3d
rm open3d-devel-linux-x86_64-cxx11-abi-cuda-0.18.0.tar.xz
```

Build in release mode:

``` sh
cd GSFusion
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -- -j
```


## Download Datasets

### Replica

```
wget https://cvg-data.inf.ethz.ch/nice-slam/data/Replica.zip
unzip Replica.zip
```
The expected file structure is as follows:
```sh
<replica_scene_path>
├── results
│   ├── depthxxxxxx.png
│   ├── ...
│   ├── framexxxxxx.jpg
│   └── ...
└── traj.txt
```

### ScanNet++

Please follow the instructions on [ScanNet++](https://kaldir.vc.in.tum.de/scannetpp/) website to download dataset and use the provided [toolbox](https://github.com/scannetpp/scannetpp/tree/main) to undistort and downscale the DSLR images. We downscale the images by a factor of 2 to prevent memory overflow. The expected file structure is as follows:
```sh
<scannetpp_scene_path>
├── nerfstudio
│   └── transforms_undistorted_2.json
├── undistorted_depths_2
│   ├── DSCxxxxx.png
│   └── ...
├── undistorted_images_2
│   ├── DSCxxxxx.JPG
│   └── ...
└── train_test_lists.json
```

**Note**: If you change the naming or structure of the dataset, ensure to also update the corresponding code in `app/include/reader_<dataset_name>.hpp`, `app/src/reader_<dataset_name>.cpp`, and `app/src/main.cpp` (line 98-101).

### TUM-RGBD

Please follow the instructions on [TUM-RBGD](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download) website to download dataset. The expected file structure is as follows:
```sh
<tum_rgbd_scene_path>
├── groundtruth.txt
├── rgb
│   ├── <timestamp>.png
│   └── ...
├── depth
│   ├── <timestamp>.png
│   └── ...
├── rgb.txt
└── depth.txt
```

**Note**: We set a threshold for timestamp difference to associate depth and RGB images, and we interpolate the ground truth poses at the depth image timestamps. Therefore, some unmatched images will be discarded.

## Usage Example

Adjust the following fields in the YAML file under `config` folder to adapt to a specific scene in the Replica/ScanNet++ dataset:
```sh
# change the map dimension and resolution according to your needs
map:
  dim:                        [15, 15, 15]
  res:                        0.01

# replace the intrinsics if you use other scenes
sensor:
  width:                      1200
  height:                     680
  fx:                         600.0
  fy:                         600.0
  cx:                         599.5
  cy:                         339.5

reader:
  reader_type:                "replica"  # or "scannetpp", "tum"
  sequence_path:              "<replica_scene_path>"  # absolute path
  ground_truth_file:          "<replica_scene_path>/traj.txt"

app:
  optim_params_path:          "<project_root_path>/parameter/optimization_params_replica.json"  # absolute path
  ply_path:                   "<checkpoint_path>/point_cloud"  # absolute path
  mesh_path:                  "<checkpoint_path>/mesh"
```

You can also adjust the hyper-parameters for optimization in the JSON file under `parameter` folder. The provided JSON files are the ones we used for the results reported in the paper. Please refer to our paper for the meaning of those hyper-parameters.

**Note**: We highly recommend adjusting the above parameters when mapping new scenes to achieve better performance.

Now run the executable using the following commands:
```sh
cd GSFusion
# ScanNet++ dataset
./build/app/gsfusion config/scannetpp_8b5caf3398.yaml
# Replica dataset
./build/app/gsfusion config/replica_room0.yaml
# TUM-RGBD dataset
./build/app/gsfusion config/tum_rgbd_freiburg1_desk2.yaml
```


## Evaluation

We develop an automatic evaluation system for GSFusion and provide several pre-trained models for assessment. You can download the necessary data [here](https://cloud.cvai.cit.tum.de/s/dQZgJPE2ydzq32E), and follow the instructions in [GSFusion_eval](https://github.com/goldoak/GSFusion_eval) to get started.


## Citation

If you find our paper and code useful, please cite us:
```bibtex
@article{wei2024gsfusion,
  title={Gsfusion: Online rgb-d mapping where gaussian splatting meets tsdf fusion},
  author={Wei, Jiaxin and Leutenegger, Stefan},
  journal={IEEE Robotics and Automation Letters},
  year={2024},
  publisher={IEEE}
}
```


## License

Copyright (c) 2024, Jiaxin Wei

### Important Notice
- This project, including its main codebase Supereight2, is distributed under the [BSD 3-clause license](./LICENSES/BSD-3-Clause.txt).
- This project includes components licensed under the [Gaussian-Splatting-License](./LICENSES/Gaussian-Splatting-License.md), which restricts the entire project to **non-commercial use only**. If you wish to use this project for commercial purposes, please contact the respective copyright holders for permission.
- Users must comply with all license requirements included in this repository.


## Acknowledgement
This work was supported by the EU project AUTOASSESS. The authors would like to thank Simon Boche and Sebastián Barbas Laina for their assistance in collecting and processing drone data. We also extend our gratitude to Sotiris Papatheodorou for his valuable discussions and support with the Supereight2 software.

We gratefully acknowledge the contributions of the following open-source projects, which have been beneficial in the development of this work:
- [Inria-3DGS](https://github.com/graphdeco-inria/gaussian-splatting): The original Python implementation of Gaussian Splatting, developed by Inria and MPII.
- [MrNeRF-gaussian-splatting-cuda](https://github.com/MrNeRF/gaussian-splatting-cuda): A highly efficient C++ implementation of Gaussian Splatting, adapted for CUDA.
- [SRL-Supereight2](https://bitbucket.org/smartroboticslab/supereight2/src/master/): A high-performance template octree library and a dense volumetric SLAM pipeline implementation.