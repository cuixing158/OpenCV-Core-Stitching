# OpenCV Core Stitching Pipeline

MATLAB example based on OpenCV's [stitching module](https://github.com/opencv/opencv/tree/4.x/modules/stitching), inspired by the source code of [stitching_detailed.cpp](https://github.com/opencv/opencv/blob/4.x/samples/cpp/stitching_detailed.cpp).

## Overview

Welcome to this MATLAB-based project that replicates the core functionalities of the OpenCV Stitching module, specifically tailored for rotation-only homography scenarios. This repository is designed to help you understand and visualize the intricate details behind image stitching, from feature detection to the final panorama projection.

## Features

- **Core Stitching Functionality**:  
  - **Feature Point Detection and Matching**: Identify and match key points across images to prepare for stitching.  
  - **Homography Estimation and Decomposition**: Estimate the homography matrix and decompose it to obtain camera parameters (K, R).  
  - **Bundle Adjustment (BA) Optimization**: Optimize camera parameters for better alignment and stitching accuracy.  

- **Image Correction**:  
  - **Wave Correction**: Correct any waveform distortions in the images, supporting both horizontal and vertical corrections to ensure accurate stitching.  

- **Panorama Projection**:  
  Supports multiple projection types to cater to different stitching needs:  
  - Plane  
  - Spherical  
  - Cylindrical  
  - Fisheye  
  - Stereo  

- **Multi-Band-Blender Fusion**:  
  - **Multi-band Blending**: Seamlessly blend images using multi-band blending techniques to reduce visible seams.  

- **Visualization Tools**:  
  - **Intermediate Step Visualization**: Visualize each step of the stitching process to gain deeper insights into the underlying techniques.  

## Usage

Clone the repository:

```bash
git clone https://github.com/cuixing158/OpenCV-Core-Stitching.git
cd OpenCV-Core-Stitching
```

Open MATLAB and navigate to the project directory.Run the provided example script `main_opencv_stitching_demo.mlx` to see the stitching process in action.

Explore the code and modify parameters to see how different settings affect the stitching outcome.

## Requirements

MathWorks Products(<www.mathworks.com>)

- MATLAB R2024b or later  
- Computer Vision Toolbox™
- Image Processing Toolbox™
- Optimization Toolbox™

## Contributing

If you have ideas for improvements or new features, please feel free to open an issue or submit a pull request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Inspired by the OpenCV Stitching module.
- Special thanks to the MATLAB community for their support and resources.

## Contact & Feedback

If you have any suggestions about this project, feel free to contact me.

[e-mail: cuixingxing150[at]gmail.com]

Happy stitching! ::smile::
