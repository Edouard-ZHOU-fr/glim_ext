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
   - no roll/pitch: both world frames are gravity-aligned. If `enable_roll_pitch_prior` is true, the mean and standard deviation of (GNSS roll/pitch - estimated roll/pitch) are only logged as a consistency check
3. In each processing loop (triggered by new submaps), a translation prior (`PoseTranslationPrior`) and, if enabled, a rotation prior (yaw and/or roll/pitch) are added to the **last associated** submap (see the known limitations).

The rotation prior is a `PriorFactor<Pose3>` whose measured rotation is `Rz(yaw) * Ry(pitch) * Rx(roll)`, where each angle is the GNSS one if it is constrained and the estimated (odometry) one otherwise. Each angle is enabled (`enable_yaw_prior`, `enable_roll_pitch_prior`) and gated (`*_max_sigma_deg`) independently, e.g., the roll/pitch can still be constrained when the heading is rejected. The error of `PriorFactor<Pose3>` is a rotation in the body frame, which mixes the Euler angles on slopes (a yaw difference gives a roll error of `-sin(pitch) * d_yaw`). Its information matrix is therefore `J^-T * diag(1/sigma_roll^2, 1/sigma_pitch^2, 1/sigma_yaw^2) * J^-1`, where `J` maps the Euler angle differences to the body frame rotation error: only the constrained angles are weighted (unconstrained angles and translation: sigma = 1e3), and the difference between the odometry yaw and the optimized yaw does not bias a roll/pitch-only prior. With only the yaw enabled, it constrains the heading around the world z axis without affecting the roll/pitch given by the IMU. `PriorFactor<Pose3>` and its Gaussian noise model are serializable, so the rotation priors are kept in dumped maps.

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
| `enable_roll_pitch_prior` | `false` | `true` | Add roll/pitch priors |
| `roll_pitch_prior_sigma_deg` | `1.0` | `1.0` | Sigma [deg] of the roll/pitch prior when `dynamic_scale` is false, or when the message has no valid roll/pitch covariance |
| `roll_pitch_min_sigma_deg` | `0.5` | `0.5` | Lower bound [deg] of the sigma derived from the message roll/pitch covariance (`dynamic_scale` = true) |
| `roll_pitch_max_sigma_deg` | `5.0` | `5.0` | GNSS roll (pitch) with a larger sigma [deg] is ignored (no roll (pitch) prior) |
| `isShow` | `true` | `true` | Draw the GNSS constraints in the viewer (position: cyan/orange/red, yaw: magenta, roll/pitch: yellow up line) |

`dynamic_scale` = true:
- Translation prior: information = `1.3 / cov_xx`, `1.3 / cov_yy`, `3.2 / cov_zz` (covariances floored to 1e-6), from the **latest** GNSS message. If the smallest information is <= 1e3 or the z information is <= 2e3, the GNSS is considered weak and the information is set to `[1e3, 1e3, 3e3]` ("Gnss faible"), or the prior is skipped if the smallest information is <= 1e-2 ("Gnss abodonne"). In terms of the message sigma:

  | GNSS position sigma | Status | Prior sigma |
  |---|---|---|
  | xy < 3.6 cm and z < 4 cm | normal | `sigma / sqrt(1.3)` (xy), `sigma / sqrt(3.2)` (z) |
  | xy >= 3.6 cm or z >= 4 cm (below the abandoned limits) | weak ("Gnss faible") | **3.2 cm (xy), 1.8 cm (z), whatever the message covariance** |
  | xy >= 11.4 m or z >= 17.9 m | abandoned ("Gnss abodonne") | no prior |

  The weak fallback replaces the information instead of bounding it: a degraded GNSS gets a prior much tighter than its covariance. For example, on the Velizy 2026-10-07 bag, a section with a position sigma of about 0.79 m gets priors with a 3.2 cm sigma (25 times tighter), which pulls the map to the degraded GNSS positions.
- In the recorded Autoware EKF output (Velizy 2026-10-07), `cov_xx`, `cov_yy` and `cov_zz` are equal in every message: the altitude variance is not estimated separately, and the z information is only larger because of its `3.2` scale.
- Yaw prior: sigma = max(sqrt(yaw variance `covariance[35]`), `yaw_min_sigma_deg`), interpolated at the submap timestamp. Priors with sigma > `yaw_max_sigma_deg` are not created.
- Roll/pitch prior: same with the roll/pitch variances `covariance[21]` / `covariance[28]`, `roll_pitch_min_sigma_deg` and `roll_pitch_max_sigma_deg`.

Tuning the yaw prior:
- `yaw_min_sigma_deg` is the main parameter. The Autoware EKF covariance is overconfident (Velizy 2026-10-07: yaw sigma 0.14-0.35 deg, position sigma ~7 mm with a good GNSS, so the 0.5 deg floor is always used) and its heading is not independent of the IMU used by GLIM. Increase it to 1-2 deg if the map gets distorted or inconsistent with the point cloud matching. Decrease it only with a reliable heading (e.g., dual-antenna RTK).
- `yaw_max_sigma_deg`: decrease it to 2-3 deg to only accept good headings (fewer priors in degraded GNSS areas: bridges, tunnels, urban canyons). Increasing it is not recommended.
- `yaw_prior_sigma_deg` is only used without `dynamic_scale` (or without a valid yaw covariance).
- Do not use `/gnss_front/odometry` as the heading source: its heading is derived from the motion and is unusable at low speed.

Tuning the roll/pitch prior:
- GLIM already observes the roll/pitch with the gravity (IMU factors). The roll/pitch prior mainly helps against a slow tilt drift of the map (long trajectories, slopes, few loop closures). Keep `roll_pitch_min_sigma_deg` >= 0.5 deg unless the GNSS/INS roll/pitch is known to be accurate.
- Check the initialization log first (see below). A mean difference of more than a few tenths of a degree means a rotation between the IMU frame and `base_link` (mounting) or a gravity misalignment of the GLIM world frame (the first submap is locked): the priors then pull the map against the IMU. Increase `roll_pitch_min_sigma_deg` or disable the roll/pitch prior in this case.
- Some sources do not really observe the roll (e.g., a dual-antenna GNSS gives the heading and the tilt along the baseline). If the message reports a large roll variance, `roll_pitch_max_sigma_deg` rejects it and only the pitch is constrained.
- Checked on the Velizy 2026-10-07 bag (`/localization/kinematic_state`): the roll/pitch vary within about ±4 deg (no invalid orientation), the altitude change predicted from the pitch matches the measured one within an equivalent pitch offset of -0.17 deg over 570 m (the pitch is the actual slope, not a mounting offset), and `/tf_static` gives an identity `base_link -> imu`. The roll/pitch sigma of the EKF is 0.03-0.1 deg, so the 0.5 deg floor (`roll_pitch_min_sigma_deg`) is always used.

Logs:
- At initialization: `T_mgrs_world yaw from GNSS heading=... deg (spread=... deg), from positions (SVD)=... deg`. The spread (circular standard deviation of the heading differences) should be a few tenths of a degree. A spread of several degrees indicates unstable GNSS headings or an odometry heading drift at the beginning.
- At initialization (`enable_roll_pitch_prior`): `GNSS - estimated roll=... deg (std=... deg), pitch=... deg (std=... deg) over N submaps`. The means should be close to zero (see the roll/pitch tuning above).
- With the debug log level, each rotation prior: `rotation prior submap=... gnss/est/sigma roll=... pitch=... yaw=... [deg]` (sigma = inf: angle not constrained).
- With `dynamic_scale`, each processing loop logs `curent_scale=...` (translation information of the latest message), followed by `Gnss faible` or `Gnss abodonne` when applicable.
- `invalid condition in GNSS global module!!`: see the known limitations.

Known limitations:
- Priors are only added to the last associated submap of each processing loop: when several submaps are associated at once, the others get no prior.
- The translation prior is not deduplicated (rotation priors are added once per submap): each processing loop adds a translation prior to the last associated submap, even when no new submap could be associated in this loop (e.g., the GNSS topic stopped or lags behind the submaps). N identical priors act as one prior with N times the information (sigma / sqrt(N)), so the last submap before a GNSS outage is pulled more and more strongly to its GNSS position, e.g., 10 priors after 9 new submaps without GNSS (sigma / 3.2).
- With `dynamic_scale`, a weak GNSS gets a 3.2 cm (xy) / 1.8 cm (z) prior whatever its covariance, see `dynamic_scale` above.
- `invalid condition in GNSS global module!!` is also logged in a valid case: when the GNSS sample right after the submap timestamp is the newest one (e.g., after a GNSS gap). The association is then postponed to the next processing loop, where the submap may get no prior (see the first limitation).
- A NaN position covariance gives a NaN translation information (`std::max` does not filter NaN), which would corrupt the optimization. Not observed in the recorded data.
- `current_inf_scale` (written by the subscription callback) and `submap->T_world_origin` (written by the global mapping thread) are read by the GNSS thread without synchronization.
- The roll/pitch/yaw are interpolated separately (Euler angles), which is accurate for the short GNSS message interval.
- The translation weights of `dynamic_scale` use the covariance of the latest GNSS message, not the one at the submap timestamp.
- When merging dumped maps (`GlobalMapping::load()` with an existing map), all `PoseTranslationPrior` and `PriorFactor<Pose3>` factors, including the rotation priors, are removed.

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
