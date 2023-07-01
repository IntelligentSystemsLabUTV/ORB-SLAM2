# ORB-SLAM2

ROS 2-friendly implementation of ORB-SLAM2, based on the Distributed Unified Architecture.

## Abstract

This project is a child project of the original [`raulmur/ORB_SLAM2`](https://github.com/raulmur/ORB_SLAM2) by [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)), inheriting many years of fixes, improvements and upgrades by Fabrizio Romanelli and Roberto Masocco in [`fabrizioromanelli/ORBSLAM2`](https://github.com/fabrizioromanelli/ORBSLAM2).

The scope of this version is of building a stable, reliable implementation that can easily be deployed on modern robots, and that can be easily integrated with other software components. To this end, some core components of the implementation have been changed to make use of two major building blocks:

- [Distributed Unified Architecture](dua-template.md)
- ROS 2

**This project is still in development. Consider this document as a declaration of intent on what the project will be. See the commit history for a detailed description of the project status.**

## Contents

- [`ORB-SLAM2`](src/ORB-SLAM2/README.md): an improved version of the original ORB-SLAM2 implementation, configured to build and run entirely inside a ROS 2 workspace, in a containerized environment.
- [`orbslam2_driver`](src/orbslam2_driver/README.md): a ROS 2 driver for the ORB-SLAM2 system, that exposes the SLAM system as a ROS 2 node, supporting all of its configurations.

### Thirdparty dependencies

The following thirdparty dependencies are required to build the project, and included as ROS 2 packages in the `src/thirdparty` directory:

- [`fbow`](src/thirdparty/fbow/README.md): a C++ implementation of the Fast Bag-of-Words algorithm, based on the original DBoW2 implementation.
- `g2o`: a C++ framework for graph optimization.
- `dlib`: DUtils library.

## Supported targets

Based on the DUA specification, this project supports the following targets:

- `x86-base`
- `x86-dev`
- `x86-cudev`

The support for ARM targets is currently unavailable because of multiple x86-only optimizations in the original implementation.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata

Copyright (c) 2017, Raul Mur-Artal, Juan D. Tardos, J. M. M. Montiel and Dorian Galvez-Lopez
