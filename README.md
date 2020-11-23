# Self-Driving Car Engineer Nanodegree Program
## Unscented Kalman Filter Project

In this project an Unscented Kalman Filter is used to estimate the state of a moving object of interest with noisy lidar and radar measurements.

[//]: # (Image References)
[image1]: ./images/lidarNIS.png
[image2]: ./images/radarNIS.png

### Project Setup

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes a file that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for Linux.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Other Important Dependencies are:
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux)
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

### Project Analisys

#### UKF

  Error (RMSE) using both lidar and radar in the last time step of each dataset:

  |      | RMSE (dataset 1) | RMSE (dataset 2) |
  |:----:|:---------------:|:---------------:|
  |   X  | 0.0751          |     0.0901      |
  |   Y  | 0.0850          |     0.0789      |
  |  VX  |   0.3592        |     0.6695      |
  |  VY  |   0.2459        |     0.3337      |


  Error (RMSE) using only radar in the last time step of each dataset:

  |      | RMSE (dataset 1) | RMSE (dataset 2) |
  |:----:|:---------------:|:---------------:|
  |   X  | 0.1730          |     0.2950      |
  |   Y  | 0.2690          |     0.3050      |
  |  VX  |   0.4417        |     0.8177      |
  |  VY  |   0.4096        |     0.6943      |

  Error (RMSE) using only lidar in the last time step of each dataset:

  |      | RMSE (dataset 1) | RMSE (dataset 2) |
  |:----:|:---------------:|:---------------:|
  |   X  | 0.1182          |     0.1057      |
  |   Y  | 0.0996          |     0.0908      |
  |  VX  |   0.6215        |     0.5178      |
  |  VY  |   0.2668        |     0.3138      |

Looking to the results is clear that we have better results when using both sensors as expected.

Another observation that can be made is that lidar measurements lead to better results than radar measurements.
One way of explaining this fact is the following:
LIDAR only provides positions and radar measure velocity directly and because is easier to derive velocity from position than the other way around, LIDAR measurements when integrated with an unscented kalman filter provides more accurate results.
Another way to explain the last observation made is that LIDAR is less prone to noise than radar.

#### EKF
Error (RMSE) using both lidar and radar in the last time step of each dataset:

|      | RMSE (dataset 1) | RMSE (dataset 2) |
|:----:|:---------------:|:---------------:|
|   X  | 0.0974          |     0.0726      |
|   Y  | 0.0855          |     0.0967      |
|  VX  |   0.4517        |     0.4582      |
|  VY  |   0.4404        |     0.4971      |


Error (RMSE) using only radar in the last time step of each dataset:

|      | RMSE (dataset 1) | RMSE (dataset 2) |
|:----:|:---------------:|:---------------:|
|   X  | 0.2271          |     0.2693      |
|   Y  | 0.3466          |     0.3848      |
|  VX  |   0.6587        |     0.6534      |
|  VY  |   0.7631        |     0.8638      |

Error (RMSE) using only lidar in the last time step of each dataset:

|      | RMSE (dataset 1) | RMSE (dataset 2) |
|:----:|:---------------:|:---------------:|
|   X  | 0.1474          |     0.1186      |
|   Y  | 0.1154          |     0.1285      |
|  VX  |   0.6390        |     0.6481      |
|  VY  |   0.5351        |     0.5483      |

### NIS Analisys

To check is the noise parameters are correct it was used the Normalized innovation squared (NIS). For that purpose the following graphs are used:


| ![image1] |
|:--:|
| *lidar NIS* |

| ![image2] |
|:--:|
| *radar NIS* |

As shown above both the NIS of radar and lidar measurements are below 7.81 in 95% of all update steps. This indicates that the noise parameters are well set.
