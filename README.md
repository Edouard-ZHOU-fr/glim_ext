**!!! This repository constains half-baked code that may not be well-maintained and not suitable for practical purposes. !!!**
**!!! We don't have resource to maintain this extension library. If you find an issue, please fix it by yourself and open a PR to share the solution. !!!**

# glim_ext

glim_ext is a set of extension modules for [GLIM](https://github.com/koide3/glim), 3D LiDAR mapping framework. With this package, we aim to provide reference implementations that demonstrate how GLIM can be extended through the global callback slot mechanism.

[![EXT](https://github.com/koide3/glim_ext/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/glim_ext/actions/workflows/build.yml)

## Disclaimer

Each module in glim_ext uses several external libraries that employ different licensing conditions. You must carefully check and follow their licenses.

## Usage

To enable an extension module, add the so filename to `extension_modules` in `glim/config/config_ros.json`. E.g.,

```
"extension_modules": [
    "libmemory_monitor.so",
    "libstandard_viewer.so",
    "librviz_viewer.so",
    "libimu_validator.so"   // Added
  ],
```


## Config path addressing

Extension modules first try to find the corresponding config path from `config.json` in the main GLIM package. It then fallback to `config_ext.json` in the `glim_ext` package if it failed to find a config path in `config.json`.

Example (`libflat_earther.so`):

1. Tries to find `config_flat_earther` in `glim/config/config.json`. The config path may be changed by `config_path` ROS param (see [here](https://koide3.github.io/glim/quickstart.html#configuration-files)).
2. If not found, find `config_flat_earther` in `glim_ext/config/config_ext.json`.


## Example Modules

### Callback demo (libglim_callback_demo.so)
- This modules subscribes to all available callbacks to demonstrate how mapping states can be retrieved

## Odometry estimation Modules

### Point cloud deskewing (libdeskewer.so)
- Publishing and saving deskewed point clouds (without downsampling).

### IMU validator (libimu_validator.so)
- Validator for LiDAR-IMU transformation configurations.

### IMU prediction (libimu_prediction.so)
- Publishing IMU-based state estimation at a high frequency for low-latency applications.

### Gravity estimation (libgravity_estimator.so)
- Estimating the gravity vector in the sensor frame, and forcing the upward direction of the state estimation to be aligned with the gravity direction.

### Velocity supressor (libvelocity_supressor.so)
- Regulating the velocity range.

### ORB_SLAM odometry (Not Maintained)
- Loosely coupled visual odometry constraints based on ORB_SLAM3
- Dependency: [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) (GPL-3.0)

## Global Optimization Modules

### Flat earther (libflat_earther.so)
- Forcing the height of proximiate submaps be the same.
- This is useful in situations only a single floor exists and the height of the sensor from the floor is mostly unchanged.

### GNSS constraints (libgnss_global.so, ROS2 only)
- GNSS-based constraints for global optimization

#### Milla version

Input: `geometry_msgs/PoseWithCovarianceStamped` in the MGRS frame (`map`), i.e., the Autoware EKF output `/localization/kinematic_state` republished by the `convert_gnss` node. The pose is the `base_link` pose, and the submap origin (IMU frame) is assumed to coincide with `base_link`.

How it works:
1. GNSS messages are interpolated at the timestamp of the origin (middle) frame of each submap. The heading is interpolated along the shortest arc.
2. Once the trajectory is longer than `min_baseline`, `T_mgrs_world` (GLIM world frame -> MGRS) is estimated and written to `Mgrs.txt` in the working directory. It is a rotation around the z axis plus a translation:
   - yaw: circular mean of (GNSS heading - estimated heading) over the associated submaps if `enable_yaw_prior` is true, otherwise a 2D SVD alignment of the positions (sensitive to the GNSS position noise over the short baseline)
   - translation: alignment of the centroids of the estimated and GNSS positions
3. For each new submap, a translation prior (`PoseTranslationPrior`) and, if enabled, a yaw prior are added to the global factor graph.

The yaw prior is a `PriorFactor<Pose3>` whose measured rotation is the estimated rotation turned around the world z axis to the GNSS heading. Only the yaw component of its error is weighted (roll/pitch/translation sigmas = 1e3): it constrains the heading around the world z axis without affecting the roll/pitch given by the IMU. `PriorFactor<Pose3>` is serializable, so the yaw priors are kept in dumped maps.

Parameters (`config/config_gnss_global.json`):

| Parameter | Default (code) | Milla config | Description |
|---|---|---|---|
| `gnss_topic` | `/pose_with_cov` | `/localization/pose_with_covariance_stamped` | Input GNSS pose topic |
| `min_baseline` | `5.0` | `3.0` | Trajectory length [m] required before estimating `T_mgrs_world` |
| `dynamic_scale` | `false` | `true` | Weight the priors with the covariance of the GNSS messages (see below) instead of fixed values |
| `prior_inf_scale` | `[1e3, 1e3, 3e3]` | `[2e4, 2e4, 4e5]` | Information (1/sigma^2) of the translation prior in x, y, z when `dynamic_scale` is false |
| `enable_yaw_prior` | `false` | `true` | Add yaw priors, and estimate the yaw of `T_mgrs_world` from the GNSS headings |
| `yaw_prior_sigma_deg` | `1.0` | `1.0` | Sigma [deg] of the yaw prior when `dynamic_scale` is false, or when the message has no valid yaw covariance |
| `yaw_min_sigma_deg` | `0.5` | `0.5` | Lower bound [deg] of the sigma derived from the message yaw covariance (`dynamic_scale` = true) |
| `yaw_max_sigma_deg` | `5.0` | `5.0` | GNSS headings with a larger sigma [deg] are ignored (no yaw prior, not used for the `T_mgrs_world` yaw) |

`dynamic_scale` = true:
- Translation prior: information = `1.3 / cov_xx`, `1.3 / cov_yy`, `3.2 / cov_zz` (covariances floored to 1e-6), from the **latest** GNSS message. If the smallest information is <= 1e3 or the z information is <= 2e3, the GNSS is considered weak and the information is set to `[1e3, 1e3, 3e3]` ("Gnss faible"), or the prior is skipped if the smallest information is <= 1e-2 ("Gnss abodonne").
- Yaw prior: sigma = max(sqrt(yaw variance `covariance[35]`), `yaw_min_sigma_deg`), interpolated at the submap timestamp. Priors with sigma > `yaw_max_sigma_deg` are not created.

Tuning the yaw prior:
- `yaw_min_sigma_deg` is the main parameter. The Autoware EKF covariance is overconfident (yaw sigma ~0.4 deg, position sigma ~7 mm) and its heading is not independent of the IMU used by GLIM. Increase it to 1-2 deg if the map gets distorted or inconsistent with the point cloud matching. Decrease it only with a reliable heading (e.g., dual-antenna RTK).
- `yaw_max_sigma_deg`: decrease it to 2-3 deg to only accept good headings (fewer priors in degraded GNSS areas: bridges, tunnels, urban canyons). Increasing it is not recommended.
- `yaw_prior_sigma_deg` is only used without `dynamic_scale` (or without a valid yaw covariance).
- Do not use `/gnss_front/odometry` as the heading source: its heading is derived from the motion and is unusable at low speed.

Logs:
- At initialization: `T_mgrs_world yaw from GNSS heading=... deg (spread=... deg), from positions (SVD)=... deg`. The spread (circular standard deviation of the heading differences) should be a few tenths of a degree. A spread of several degrees indicates unstable GNSS headings or an odometry heading drift at the beginning.
- With the debug log level, each yaw prior: `yaw prior submap=... yaw_gnss=... yaw_est=... sigma=... [deg]`.

Known limitations:
- Priors are only added to the last associated submap of each processing loop: when several submaps are associated at once, the others get no prior. Translation priors may also be added several times to the same submap (yaw priors are added once per submap).
- The translation weights of `dynamic_scale` use the covariance of the latest GNSS message, not the one at the submap timestamp.
- When merging dumped maps (`GlobalMapping::load()` with an existing map), all `PoseTranslationPrior` and `PriorFactor<Pose3>` factors, including the yaw priors, are removed.

### ScanContext Loop Detector (libscancontext_loop_detector.so)
- Explicit loop detection based on ScanContext
- Dependency: [ScanContext](https://github.com/irapkaist/scancontext) (CC BY-NC-SA 4.0)

### DBoW Loop Detector (Not Maintained)
- Explicit loop detection based on DBoW3
- Dependency: [DBoW3](https://github.com/rmsalinas/DBow3) ([LICENSE](https://github.com/rmsalinas/DBow3/blob/master/LICENSE.txt))

## Installation

### ROS1

**!!! We do not and will not support ROS1. While some modules would still work on ROS1, no tests are conducted. !!!**

```bash
cd ~/catkin_ws/src
git clone https://github.com/koide3/glim_ext

cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### ROS2

```bash
cd ~/ros2_ws/src
git clone https://github.com/koide3/glim_ext

cd ..
colcon build
```
