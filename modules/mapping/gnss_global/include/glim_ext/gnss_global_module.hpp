#include <map>
#include <deque>
#include <algorithm>
#include <atomic>
#include <memory>
#include <thread>
#include <numeric>
#include <cmath>
#include <limits>
#include <Eigen/Core>

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

#define GLIM_ROS2

#include <boost/format.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/concurrent_vector.hpp>

#ifdef GLIM_ROS2
#include <glim/util/extension_module_ros2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>


using ExtensionModuleBase = glim::ExtensionModuleROS2;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using PoseWithCovarianceStampedConstPtr = geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr;

template <typename Stamp>
double to_sec(const Stamp& stamp) {
  return stamp.sec + stamp.nanosec / 1e9;
}
#else
#include <glim/util/extension_module_ros.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.hpp>

using ExtensionModuleBase = glim::ExtensionModuleROS;
#endif

#include <spdlog/spdlog.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <glim/util/logging.hpp>
#include <glim/util/convert_to_string.hpp>
#include <glim_ext/util/config_ext.hpp>

#include <glk/thin_lines.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace glim {

using gtsam::symbol_shorthand::X;

/**
 * @brief Naive implementation of GNSS constraints for the global optimization.
 * @note  This implementation is very naive and ignores the IMU-GNSS transformation and GNSS observation covariance.
 *        If you use a precise GNSS (e.g., RTK), consider asking for a closed-source extension module with better GNSS handling.
 */
class GNSSGlobal : public ExtensionModuleBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // [stamp, x, y, z, yaw, yaw_var, roll, pitch, roll_var, pitch_var] in the MGRS frame
  // (roll, pitch, yaw: orientation of base_link [rad] (ZYX convention), *_var: their variances [rad^2])
  using GnssData = Eigen::Matrix<double, 10, 1>;

  GNSSGlobal() : logger(create_module_logger("gnss_global")) {
    logger->info("initializing GNSS global constraints");
    const std::string config_path = glim::GlobalConfigExt::get_config_path("config_gnss_global");
    logger->info("gnss_global_config_path={}", config_path);

    glim::Config config(config_path);
    gnss_topic = config.param<std::string>("gnss", "gnss_topic", "/pose_with_cov");
    prior_inf_scale = config.param<Eigen::Vector3d>("gnss", "prior_inf_scale", Eigen::Vector3d(1e3, 1e3, 3e3));
    min_baseline = config.param<double>("gnss", "min_baseline", 5.0);
    dynamic_scale =  config.param<bool>("gnss", "dynamic_scale", false);
    enable_yaw_prior = config.param<bool>("gnss", "enable_yaw_prior", false);
    yaw_prior_sigma = config.param<double>("gnss", "yaw_prior_sigma_deg", 1.0) * M_PI / 180.0;
    yaw_min_sigma = config.param<double>("gnss", "yaw_min_sigma_deg", 0.5) * M_PI / 180.0;
    yaw_max_sigma = config.param<double>("gnss", "yaw_max_sigma_deg", 5.0) * M_PI / 180.0;
    enable_roll_pitch_prior = config.param<bool>("gnss", "enable_roll_pitch_prior", false);
    roll_pitch_prior_sigma = config.param<double>("gnss", "roll_pitch_prior_sigma_deg", 1.0) * M_PI / 180.0;
    roll_pitch_min_sigma = config.param<double>("gnss", "roll_pitch_min_sigma_deg", 0.5) * M_PI / 180.0;
    roll_pitch_max_sigma = config.param<double>("gnss", "roll_pitch_max_sigma_deg", 5.0) * M_PI / 180.0;
    last_position_prior_submap_id = -1;
    last_rotation_prior_submap_id = -1;
    transformation_initialized = false;
    T_world_utm.setIdentity();

    visualize_constraints = config.param<bool>("gnss", "isShow", true);
    vis_state = std::make_shared<GnssVisState>();

    kill_switch = false;
    thread = std::thread([this] { backend_task(); });

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&GNSSGlobal::on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&GNSSGlobal::on_smoother_update, this, _1, _2, _3));
    if (visualize_constraints) {
      GlobalMappingCallbacks::on_update_submaps.add(std::bind(&GNSSGlobal::on_update_submaps, this, _1));
    }
  }
  ~GNSSGlobal() {
    kill_switch = true;
    thread.join();
  }

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions() override {
    const auto sub = std::make_shared<TopicSubscription<PoseWithCovarianceStamped>>(gnss_topic, [this](const PoseWithCovarianceStampedConstPtr msg) { gnss_callback(msg); });
    return {sub};
  }

  void gnss_callback(const PoseWithCovarianceStampedConstPtr& gnss_msg) {
    /////////////////////////////////////////////////////////////////////////
    GnssData gnss_data;
    const double stamp = to_sec(gnss_msg->header.stamp);
    // const auto& pos = gnss_msg->pose.pose.position;
    double absolute_mgrs_x = gnss_msg->pose.pose.position.x;
    double absolute_mgrs_y = gnss_msg->pose.pose.position.y;
    double absolute_alt = gnss_msg->pose.pose.position.z; // 
//  recuperer la covariance de Kinematic_state
    if (dynamic_scale) {
      const double gnss_inscale_xy = 1.3;
      const double gnss_inscale_z = 3.2;
      double cov_x = gnss_msg->pose.covariance[0];
      double cov_y = gnss_msg->pose.covariance[7];
      double cov_z = gnss_msg->pose.covariance[14];
      double info_x = (1.0 / std::max(cov_x, 1e-6))*gnss_inscale_xy; // eviter par 0
      double info_y = (1.0 / std::max(cov_y, 1e-6))*gnss_inscale_xy;
      double info_z = (1.0 / std::max(cov_z, 1e-6))*gnss_inscale_z;
      // std::cout << "info_x:" << info_x << "; info_z:" << info_z << std::endl;
      this->current_inf_scale << info_x, info_y, info_z;

    }
    // Orientation of base_link in the MGRS frame and its variances (covariance order: x, y, z, roll, pitch, yaw)
    const auto& o = gnss_msg->pose.pose.orientation;
    Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    double roll_var = gnss_msg->pose.covariance[21];
    double pitch_var = gnss_msg->pose.covariance[28];
    double yaw_var = gnss_msg->pose.covariance[35];
    if (!std::isfinite(q.norm()) || q.norm() < 1e-6) {
      // Invalid orientation (e.g., all zero before the EKF initialization): the orientation is unavailable, the position is still used
      roll_var = pitch_var = yaw_var = std::numeric_limits<double>::infinity();
    } else {
      q.normalize();
      rpy = rpy_from_rotation(q.toRotationMatrix());
    }

    gnss_data << to_sec(gnss_msg->header.stamp), absolute_mgrs_x, absolute_mgrs_y, absolute_alt, rpy[2], yaw_var, rpy[0], rpy[1], roll_var, pitch_var;
    input_gnss_queue.push_back(gnss_data);
    /////////////////////////////////////////////////////////////////////////
  }
  void mgrsLogToFile(const std::string& filename, const std::string& mgrs) {
    // std::ios::app (Append)
    std::ofstream outFile(filename, std::ios::out);

    if (outFile.is_open()) {
        outFile << std::fixed << std::setprecision(6);
        
        outFile << mgrs<< std::endl;
        
        outFile.close(); 
    } else {
        std::cerr << "Error: Could not open file " << filename << std::endl;
    }
  }
    /////////////////////////////////////////////////////////////////////////

  void on_insert_submap(const SubMap::ConstPtr& submap) { input_submap_queue.push_back(submap); }

  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    const auto factors = output_factors.get_all_and_clear();
    if (!factors.empty()) {
      logger->debug("insert {} GNSS prior factors", factors.size());
      new_factors.add(factors);
    }
  }

  void on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
    if (!guik::running()) {
      return;
    }

    // Latest (optimized) submap positions indexed by submap id
    std::vector<Eigen::Vector3f> positions;
    for (const auto& submap : submaps) {
      if (submap->id >= positions.size()) {
        positions.resize(submap->id + 1, Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN()));
      }
      positions[submap->id] = submap->T_world_origin.translation().cast<float>();
    }

    const auto state = vis_state;
    guik::viewer()->invoke([state, positions] {
      state->submap_positions = positions;
      update_viewer(*state);
    });
  }

  void backend_task() {
    logger->info("starting GNSS global thread");
    std::deque<GnssData> utm_queue;
    std::deque<SubMap::ConstPtr> submap_queue;

/////////////////synchoniser le time stamps -->///////////////////:

    while (!kill_switch) {
      // Convert GeoPoint(lat/lon) to UTM
      const auto gnss_data = input_gnss_queue.get_all_and_clear();
      utm_queue.insert(utm_queue.end(), gnss_data.begin(), gnss_data.end());

      // Add new submaps
      const auto new_submaps = input_submap_queue.get_all_and_clear();
      if (new_submaps.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        continue;
      }
      submap_queue.insert(submap_queue.end(), new_submaps.begin(), new_submaps.end());

      // Remove submaps that are created earlier than the oldest GNSS data
      while (!utm_queue.empty() && !submap_queue.empty() && submap_queue.front()->frames.front()->stamp < utm_queue.front()[0]) {
        submap_queue.pop_front();
      }

      // Interpolate UTM coords and associate with submaps
      while (!utm_queue.empty() && !submap_queue.empty() && submap_queue.front()->frames.front()->stamp > utm_queue.front()[0] &&
             submap_queue.front()->frames.back()->stamp < utm_queue.back()[0]) {
        const auto& submap = submap_queue.front();
        const double stamp = submap->frames[submap->frames.size() / 2]->stamp;

        const auto right = std::lower_bound(utm_queue.begin(), utm_queue.end(), stamp, [](const GnssData& utm, const double t) { return utm[0] < t; });
        if (right == utm_queue.end() || (right + 1) == utm_queue.end()) {
          logger->warn("invalid condition in GNSS global module!!");
          break;
        }
        const auto left = right - 1;
        logger->debug("submap={:.6f} utm_left={:.6f} utm_right={:.6f}", stamp, (*left)[0], (*right)[0]);

        const double tl = (*left)[0];
        const double tr = (*right)[0];
        const double p = (stamp - tl) / (tr - tl);
        GnssData interpolated = (1.0 - p) * (*left) + p * (*right);
        // Interpolate the angles (yaw, roll, pitch) along the shortest arc (e.g., -179 deg -> 179 deg)
        for (const int k : {4, 6, 7}) {
          const double diff = std::remainder((*right)[k] - (*left)[k], 2.0 * M_PI);
          interpolated[k] = std::remainder((*left)[k] + p * diff, 2.0 * M_PI);
        }
        // An angle is unavailable if it is unavailable at either end (and 0 * inf would give NaN, i.e., "no covariance")
        for (const int k : {5, 8, 9}) {
          if (std::isinf((*left)[k]) || std::isinf((*right)[k])) {
            interpolated[k] = std::numeric_limits<double>::infinity();
          }
        }

        submaps.push_back(submap);
        submap_coords.push_back(interpolated);

        submap_queue.pop_front();
        utm_queue.erase(utm_queue.begin(), left);
      }
//////////////////////synchoniser le time stamps <-- ///////////////////:

      // Initialize T_world_utm
      if (!transformation_initialized && !submaps.empty() && (submaps.front()->T_world_origin.inverse() * submaps.back()->T_world_origin).translation().norm() > min_baseline) {
        Eigen::Vector3d mean_est = Eigen::Vector3d::Zero();
        Eigen::Vector3d mean_gnss = Eigen::Vector3d::Zero();
        for (int i = 0; i < submaps.size(); i++) {
          mean_est += submaps[i]->T_world_origin.translation();
          mean_gnss += submap_coords[i].segment<3>(1);
        }
        mean_est /= submaps.size();
        mean_gnss /= submaps.size();

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (int i = 0; i < submaps.size(); i++) {
          const Eigen::Vector3d centered_est = submaps[i]->T_world_origin.translation() - mean_est;
          const Eigen::Vector3d centered_gnss = submap_coords[i].segment<3>(1) - mean_gnss;  // 累加所有插值后的 GNSS 点
          cov += centered_gnss * centered_est.transpose();
        }
        cov /= submaps.size();

        const Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov.block<2, 2>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Matrix2d U = svd.matrixU();
        const Eigen::Matrix2d V = svd.matrixV();
        const Eigen::Matrix2d D = svd.singularValues().asDiagonal();
        Eigen::Matrix2d S = Eigen::Matrix2d::Identity();

        const double det = U.determinant() * V.determinant();
        if (det < 0.0) {
          S(1, 1) = -1;
        }

        Eigen::Isometry3d T_utm_world = Eigen::Isometry3d::Identity();
        T_utm_world.linear().block<2, 2>(0, 0) = U * S * V.transpose();

        // The SVD yaw relies on the positions over a short baseline (min_baseline) and is sensitive to the GNSS position noise.
        // When GNSS headings are available, use the circular mean of (GNSS heading - estimated heading) instead, which does not
        // depend on the baseline. This also makes the yaw priors consistent with the (locked) first submap from the beginning
        const double svd_yaw = std::atan2(T_utm_world.linear()(1, 0), T_utm_world.linear()(0, 0));
        double heading_yaw = 0.0;
        double heading_spread = 0.0;
        if (enable_yaw_prior && estimate_yaw_offset_from_heading(heading_yaw, heading_spread)) {
          T_utm_world.linear() = Eigen::AngleAxisd(heading_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
          logger->info(
            "T_mgrs_world yaw from GNSS heading={:.3f} deg (spread={:.3f} deg), from positions (SVD)={:.3f} deg",
            heading_yaw * 180.0 / M_PI,
            heading_spread * 180.0 / M_PI,
            svd_yaw * 180.0 / M_PI);
        } else {
          logger->info("T_mgrs_world yaw from positions (SVD)={:.3f} deg", svd_yaw * 180.0 / M_PI);
        }

        // Both world frames are gravity-aligned: T_mgrs_world has no roll/pitch, only check the consistency
        if (enable_roll_pitch_prior) {
          log_roll_pitch_offset();
        }

        // Given the rotation, the least-squares translation aligns the centroids
        T_utm_world.translation() = mean_gnss - T_utm_world.linear() * mean_est;

        T_world_utm = T_utm_world.inverse();

        for (int i = 0; i < submaps.size(); i++) {
          const Eigen::Vector3d gnss = T_world_utm * submap_coords[i].segment<3>(1);
          logger->debug("submap={} gnss={}", convert_to_string(submaps[i]->T_world_origin.translation().eval()), convert_to_string(gnss));
        }

        // logger->info("T_world_mgrs={}", convert_to_string(T_world_utm));
        logger->info("T_mgrs_world={}", convert_to_string(T_utm_world));
        mgrsLogToFile("Mgrs.txt",convert_to_string(T_utm_world));

        transformation_initialized = true;
      }

      // Add translation prior factor (once per submap)
      // submaps.back() is unchanged when the new submaps could not be associated with GNSS data yet (e.g., GNSS outage),
      // which would add the same prior to the same submap again
      if (transformation_initialized && submaps.back()->id != last_position_prior_submap_id) {
        const Eigen::Vector3d xyz = T_world_utm * submap_coords.back().segment<3>(1);
        // const Eigen::Vector3d xyz = submap_coords.back().segment<3>(1);

        logger->debug("submap={} gnss={}", convert_to_string(submaps.back()->T_world_origin.translation().eval()), convert_to_string(xyz));

        const auto& submap = submaps.back();
        // Marked before the rejection check so that a rejected submap is not evaluated again
        last_position_prior_submap_id = submap->id;
        GnssVisStatus vis_status = GnssVisStatus::NORMAL;
        // note: should use a more accurate information matrix
        if (dynamic_scale) {
          logger->info("curent_scale={}", convert_to_string(current_inf_scale));
          if (current_inf_scale.minCoeff() <= 1e3|| current_inf_scale.z() <= 2e3) {
            if (current_inf_scale.minCoeff() <= 1e-2) {
              logger->error("Gnss abodonne");
              visualize_position_prior(submap->id, xyz, GnssVisStatus::ABANDONED);
              continue;
            }
            else {
              logger->warn("Gnss faible");
              current_inf_scale = Eigen::Vector3d(1e3, 1e3, 3e3);
              vis_status = GnssVisStatus::WEAK;
            }
          }
          const auto model = gtsam::noiseModel::Isotropic::Information(current_inf_scale.asDiagonal());
          gtsam::NonlinearFactor::shared_ptr factor(new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(submap->id), xyz, model));
          output_factors.push_back(factor);
        }
        else {
          const auto model = gtsam::noiseModel::Isotropic::Information(prior_inf_scale.asDiagonal());
          gtsam::NonlinearFactor::shared_ptr factor(new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(submap->id), xyz, model));
          output_factors.push_back(factor);
        }
        // Show the GNSS position prior in the viewer (the viewer does not draw unary factors)
        visualize_position_prior(submap->id, xyz, vis_status);

        // Add rotation (yaw and/or roll/pitch) prior factor (once per submap)
        if ((enable_yaw_prior || enable_roll_pitch_prior) && submap->id != last_rotation_prior_submap_id) {
          add_rotation_prior(submap, submap_coords.back());
          last_rotation_prior_submap_id = submap->id;
        }
      }
    }
  }

private:
  /**
   * @brief Estimate the yaw of T_utm_world as the circular mean of (GNSS heading - estimated heading) over the associated submaps.
   * @param yaw     [out] yaw of T_utm_world [rad]
   * @param spread  [out] circular standard deviation of the differences [rad] (consistency check)
   * @return        false if no submap has a usable GNSS heading
   */
  bool estimate_yaw_offset_from_heading(double& yaw, double& spread) const {
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    int num_used = 0;
    for (int i = 0; i < submaps.size(); i++) {
      const GnssData& gnss = submap_coords[i];
      // Same validity criterion as the yaw priors
      if (std::isinf(yaw_sigma(gnss[5]))) {
        continue;
      }

      // The odometry pose of the origin frame is used (submap->T_world_origin is concurrently updated by the global mapping)
      const Eigen::Matrix3d R_world_est = submaps[i]->origin_frame()->T_world_imu.linear();
      const double yaw_est = std::atan2(R_world_est(1, 0), R_world_est(0, 0));
      const double diff = gnss[4] - yaw_est;
      sum_sin += std::sin(diff);
      sum_cos += std::cos(diff);
      num_used++;
    }

    if (num_used == 0) {
      return false;
    }

    yaw = std::atan2(sum_sin, sum_cos);
    // Circular standard deviation: sqrt(-2 ln R), R = mean resultant length
    const double R = std::min(1.0, std::hypot(sum_sin, sum_cos) / num_used);
    spread = std::sqrt(-2.0 * std::log(std::max(R, 1e-12)));
    return true;
  }

  /**
   * @brief Log the differences between the GNSS and estimated roll/pitch over the associated submaps (consistency check).
   * @note  No roll/pitch alignment is estimated because both world frames are gravity-aligned. A non-zero mean indicates a
   *        rotation between the IMU frame and base_link, or a gravity misalignment of the GLIM world frame.
   */
  void log_roll_pitch_offset() const {
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    Eigen::Vector2d sum_sq = Eigen::Vector2d::Zero();
    int num_used = 0;
    for (int i = 0; i < submaps.size(); i++) {
      const GnssData& gnss = submap_coords[i];
      // Same validity criterion as the roll/pitch priors (both angles required)
      if (std::isinf(roll_pitch_sigma(gnss[8])) || std::isinf(roll_pitch_sigma(gnss[9]))) {
        continue;
      }

      // The odometry pose of the origin frame is used (submap->T_world_origin is concurrently updated by the global mapping)
      const Eigen::Vector3d rpy_est = rpy_from_rotation(submaps[i]->origin_frame()->T_world_imu.linear());
      const Eigen::Vector2d diff(std::remainder(gnss[6] - rpy_est[0], 2.0 * M_PI), gnss[7] - rpy_est[1]);
      sum += diff;
      sum_sq += diff.cwiseAbs2();
      num_used++;
    }

    if (num_used == 0) {
      logger->warn("no usable GNSS roll/pitch to check the consistency with the estimated roll/pitch");
      return;
    }

    const Eigen::Vector2d mean = sum / num_used;
    const Eigen::Vector2d stddev = (sum_sq / num_used - mean.cwiseAbs2()).cwiseMax(0.0).cwiseSqrt();
    logger->info(
      "GNSS - estimated roll={:.3f} deg (std={:.3f} deg), pitch={:.3f} deg (std={:.3f} deg) over {} submaps",
      mean[0] * 180.0 / M_PI,
      stddev[0] * 180.0 / M_PI,
      mean[1] * 180.0 / M_PI,
      stddev[1] * 180.0 / M_PI,
      num_used);
  }

  /**
   * @brief Constrain the orientation (yaw and/or roll/pitch) of a submap with the GNSS orientation.
   * @note  A PriorFactor<Pose3> (serializable) is used as a rotation prior. Its measured rotation is Rz(yaw) * Ry(pitch) * Rx(roll),
   *        where each angle is the GNSS one if it is constrained, and the estimated one otherwise. Keeping the estimated roll/pitch
   *        in a yaw-only prior avoids a heading bias of about roll*pitch/2 on slopes that Rz(yaw) alone would introduce.
   *        The error of the prior is a rotation in the body frame, which mixes the Euler angles on slopes (e.g., a yaw difference
   *        gives a roll error of -sin(pitch) * d_yaw). The information matrix maps it back to (d_roll, d_pitch, d_yaw) (to first
   *        order, exactly for a pure yaw difference) so that only the constrained angles are weighted: the difference between the
   *        estimated yaw and the optimized one does not bias a roll/pitch-only prior. The translation gets a negligible weight.
   *        The submap origin is assumed to be the base_link frame (IMU frame = base_link).
   */
  void add_rotation_prior(const SubMap::ConstPtr& submap, const GnssData& gnss) {
    // Sigma of each angle [roll, pitch, yaw] (+inf: not constrained)
    const double inf = std::numeric_limits<double>::infinity();
    const Eigen::Vector3d sigmas(
      enable_roll_pitch_prior ? roll_pitch_sigma(gnss[8]) : inf,
      enable_roll_pitch_prior ? roll_pitch_sigma(gnss[9]) : inf,
      enable_yaw_prior ? yaw_sigma(gnss[5]) : inf);
    if (sigmas.array().isInf().all()) {
      logger->debug("skip rotation prior for submap {} (no usable GNSS angle)", submap->id);
      return;
    }

    // GNSS orientation in the world frame. T_world_utm is a rotation around the z axis only: yaw_world = yaw_mgrs - yaw(T_utm_world),
    // roll/pitch are unchanged
    const Eigen::Matrix3d R_world_utm = T_world_utm.linear();
    const double yaw_world_utm = std::atan2(R_world_utm(1, 0), R_world_utm(0, 0));
    const Eigen::Vector3d rpy_gnss(gnss[6], gnss[7], std::remainder(gnss[4] + yaw_world_utm, 2.0 * M_PI));

    // The odometry pose of the origin frame is used instead of submap->T_world_origin, which is concurrently updated by the global
    // mapping thread. Both world frames are gravity-aligned. Only the rotation part matters. The translation of the estimate keeps
    // the (negligible) translation error small
    const Eigen::Isometry3d& T_world_est = submap->origin_frame()->T_world_imu;
    const Eigen::Vector3d rpy_est = rpy_from_rotation(T_world_est.linear());

    constexpr double loose = 1e3;
    Eigen::Vector3d rpy_prior = rpy_est;
    Eigen::Vector3d rpy_sigmas = Eigen::Vector3d::Constant(loose);
    for (int k = 0; k < 3; k++) {
      if (std::isfinite(sigmas[k])) {
        rpy_prior[k] = rpy_gnss[k];
        rpy_sigmas[k] = sigmas[k];
      }
    }

    const Eigen::Matrix3d R_prior = (Eigen::AngleAxisd(rpy_prior[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy_prior[1], Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(rpy_prior[0], Eigen::Vector3d::UnitX()))
                                      .toRotationMatrix();
    const gtsam::Pose3 prior(gtsam::Rot3(R_prior), gtsam::Point3(T_world_est.translation()));

    // Body frame rotation error per Euler angle difference at the measured rotation (columns: d_roll, d_pitch, d_yaw)
    const double sr = std::sin(rpy_prior[0]);
    const double cr = std::cos(rpy_prior[0]);
    const double sp = std::sin(rpy_prior[1]);
    const double cp = std::cos(rpy_prior[1]);
    Eigen::Matrix3d J;
    J << 1.0, 0.0, -sp,  //
      0.0, cr, sr * cp,  //
      0.0, -sr, cr * cp;

    const Eigen::Matrix3d sqrt_info = rpy_sigmas.cwiseInverse().asDiagonal() * J.inverse();
    gtsam::Matrix6 information = gtsam::Matrix6::Zero();
    information.topLeftCorner<3, 3>() = sqrt_info.transpose() * sqrt_info;
    information.bottomRightCorner<3, 3>() = gtsam::Matrix3::Identity() / (loose * loose);
    const auto model = gtsam::noiseModel::Gaussian::Information(information);
    output_factors.push_back(gtsam::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(submap->id), prior, model));

    if (std::isfinite(sigmas[2])) {
      visualize_yaw_prior(submap->id, rpy_prior[2]);
    }
    if (std::isfinite(sigmas[0]) || std::isfinite(sigmas[1])) {
      visualize_roll_pitch_prior(submap->id, R_prior.col(2));
    }

    const Eigen::Vector3d gnss_deg = rpy_gnss * 180.0 / M_PI;
    const Eigen::Vector3d est_deg = rpy_est * 180.0 / M_PI;
    const Eigen::Vector3d sigmas_deg = sigmas * 180.0 / M_PI;
    logger->debug(
      "rotation prior submap={} gnss/est/sigma roll={:.2f}/{:.2f}/{:.2f} pitch={:.2f}/{:.2f}/{:.2f} yaw={:.2f}/{:.2f}/{:.2f} [deg] (sigma=inf: not constrained)",
      submap->id,
      gnss_deg[0],
      est_deg[0],
      sigmas_deg[0],
      gnss_deg[1],
      est_deg[1],
      sigmas_deg[1],
      gnss_deg[2],
      est_deg[2],
      sigmas_deg[2]);
  }

  /**
   * @brief Sigma of a rotation prior angle (roll, pitch or yaw) from its GNSS variance.
   * @param var  GNSS variance [rad^2]: +inf if unavailable (invalid orientation), NaN or <= 0 if no covariance is given
   * @return     sigma [rad], or +inf if the angle must not be constrained (unavailable, or sigma > max_sigma)
   */
  double rotation_prior_sigma(const double var, const double default_sigma, const double min_sigma, const double max_sigma) const {
    if (std::isinf(var)) {
      return std::numeric_limits<double>::infinity();
    }

    double sigma = default_sigma;
    if (dynamic_scale && std::isfinite(var) && var > 0.0) {
      // Floor the sigma: the covariance of the EKF output is overconfident
      sigma = std::max(std::sqrt(var), min_sigma);
    }
    return sigma > max_sigma ? std::numeric_limits<double>::infinity() : sigma;
  }

  double yaw_sigma(const double var) const { return rotation_prior_sigma(var, yaw_prior_sigma, yaw_min_sigma, yaw_max_sigma); }
  double roll_pitch_sigma(const double var) const { return rotation_prior_sigma(var, roll_pitch_prior_sigma, roll_pitch_min_sigma, roll_pitch_max_sigma); }

  /// @brief Roll, pitch and yaw [rad] such that R = Rz(yaw) * Ry(pitch) * Rx(roll) (ZYX convention, as ROS)
  static Eigen::Vector3d rpy_from_rotation(const Eigen::Matrix3d& R) {
    const double roll = std::atan2(R(2, 1), R(2, 2));
    const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
  }

  /// @brief Status of a GNSS position prior, shown with the line color in the viewer
  enum class GnssVisStatus {
    NORMAL,     // prior created with the nominal information (cyan)
    WEAK,       // weak GNSS ("Gnss faible"): prior created with the fallback information (orange)
    ABANDONED,  // GNSS rejected ("Gnss abodonne"): no prior created (red)
  };

  struct GnssVisEntry {
    bool has_position = false;
    Eigen::Vector3f xyz = Eigen::Vector3f::Zero();  // GNSS position in the world frame
    GnssVisStatus status = GnssVisStatus::NORMAL;

    bool has_yaw = false;
    float yaw = 0.0f;  // GNSS heading in the world frame [rad]

    bool has_up = false;
    Eigen::Vector3f up = Eigen::Vector3f::UnitZ();  // up direction (z axis) of the roll/pitch prior in the world frame
  };

  /// @brief Viewer side data. It is only accessed in the viewer thread (through invoke), and is owned by the queued tasks
  ///        as well so that they do not depend on the lifetime of this module
  struct GnssVisState {
    std::map<int, GnssVisEntry> entries;            // submap id -> GNSS constraints
    std::vector<Eigen::Vector3f> submap_positions;  // latest submap positions indexed by submap id (NaN if unknown)
  };

  void visualize_position_prior(const int submap_id, const Eigen::Vector3d& xyz, const GnssVisStatus status) {
    if (!visualize_constraints || !guik::running()) {
      return;
    }

    const auto state = vis_state;
    const Eigen::Vector3f xyz_f = xyz.cast<float>();
    guik::viewer()->invoke([state, submap_id, xyz_f, status] {
      auto& entry = state->entries[submap_id];
      // Keep showing the created prior if the same submap is rejected afterwards
      if (status == GnssVisStatus::ABANDONED && entry.has_position && entry.status != GnssVisStatus::ABANDONED) {
        return;
      }
      entry.has_position = true;
      entry.xyz = xyz_f;
      entry.status = status;
      update_viewer(*state);
    });
  }

  void visualize_yaw_prior(const int submap_id, const double yaw_world) {
    if (!visualize_constraints || !guik::running()) {
      return;
    }

    const auto state = vis_state;
    guik::viewer()->invoke([state, submap_id, yaw_world] {
      auto& entry = state->entries[submap_id];
      entry.has_yaw = true;
      entry.yaw = yaw_world;
      update_viewer(*state);
    });
  }

  void visualize_roll_pitch_prior(const int submap_id, const Eigen::Vector3d& up_world) {
    if (!visualize_constraints || !guik::running()) {
      return;
    }

    const auto state = vis_state;
    const Eigen::Vector3f up_f = up_world.cast<float>();
    guik::viewer()->invoke([state, submap_id, up_f] {
      auto& entry = state->entries[submap_id];
      entry.has_up = true;
      entry.up = up_f;
      update_viewer(*state);
    });
  }

  /**
   * @brief Draw the GNSS constraints (must be called in the viewer thread).
   *        Position prior: line from the submap to the GNSS position + cross at the GNSS position (color = GnssVisStatus).
   *        Yaw prior     : magenta line from the submap along the GNSS heading.
   *        Roll/pitch prior: yellow line from the submap along the up direction (z axis) of the prior.
   */
  static void update_viewer(const GnssVisState& state) {
    constexpr float marker_size = 0.3f;      // [m] half size of the cross at the GNSS position
    constexpr float yaw_line_length = 3.0f;  // [m] length of the GNSS heading line
    constexpr float up_line_length = 3.0f;   // [m] length of the roll/pitch prior up line
    const Eigen::Vector4f yaw_color(1.0f, 0.0f, 1.0f, 1.0f);
    const Eigen::Vector4f up_color(1.0f, 1.0f, 0.0f, 1.0f);

    std::vector<Eigen::Vector3f> vertices;
    std::vector<Eigen::Vector4f> colors;
    const auto add_line = [&](const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector4f& color) {
      vertices.push_back(p0);
      vertices.push_back(p1);
      colors.push_back(color);
      colors.push_back(color);
    };

    for (const auto& [submap_id, entry] : state.entries) {
      Eigen::Vector4f color;
      switch (entry.status) {
        case GnssVisStatus::NORMAL:
          color = Eigen::Vector4f(0.0f, 1.0f, 1.0f, 1.0f);
          break;
        case GnssVisStatus::WEAK:
          color = Eigen::Vector4f(1.0f, 0.5f, 0.0f, 1.0f);
          break;
        case GnssVisStatus::ABANDONED:
          color = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
          break;
      }

      // The cross stays visible even when the submap is on the GNSS position
      if (entry.has_position) {
        for (int k = 0; k < 3; k++) {
          const Eigen::Vector3f d = Eigen::Vector3f::Unit(k) * marker_size;
          add_line(entry.xyz - d, entry.xyz + d, color);
        }
      }

      if (submap_id >= state.submap_positions.size() || !state.submap_positions[submap_id].allFinite()) {
        continue;
      }
      const Eigen::Vector3f& pos = state.submap_positions[submap_id];

      if (entry.has_position) {
        add_line(pos, entry.xyz, color);
      }
      if (entry.has_yaw) {
        add_line(pos, pos + yaw_line_length * Eigen::Vector3f(std::cos(entry.yaw), std::sin(entry.yaw), 0.0f), yaw_color);
      }
      if (entry.has_up) {
        add_line(pos, pos + up_line_length * entry.up, up_color);
      }
    }

    if (vertices.empty()) {
      return;
    }

    auto lines = std::make_shared<glk::ThinLines>(vertices, colors);
    lines->set_line_width(2.0f);
    guik::viewer()->update_drawable("gnss_constraints", lines, guik::VertexColor());
  }

  std::atomic_bool kill_switch;
  std::thread thread;

  ConcurrentVector<GnssData> input_gnss_queue;
  ConcurrentVector<SubMap::ConstPtr> input_submap_queue;
  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> output_factors;

  std::vector<SubMap::ConstPtr> submaps;
  std::vector<GnssData> submap_coords;

  std::string gnss_topic;
  Eigen::Vector3d prior_inf_scale;
  Eigen::Vector3d current_inf_scale;
  bool dynamic_scale;
  double min_baseline;

  bool enable_yaw_prior;
  double yaw_prior_sigma;  // [rad] sigma of the yaw prior (used when dynamic_scale is false)
  double yaw_min_sigma;    // [rad] lower bound of the sigma derived from the GNSS covariance
  double yaw_max_sigma;    // [rad] yaw priors with a larger sigma are not created

  bool enable_roll_pitch_prior;
  double roll_pitch_prior_sigma;  // [rad] sigma of the roll/pitch prior (used when dynamic_scale is false)
  double roll_pitch_min_sigma;    // [rad] lower bound of the sigma derived from the GNSS covariance
  double roll_pitch_max_sigma;    // [rad] roll/pitch priors with a larger sigma are not created

  int last_position_prior_submap_id;
  int last_rotation_prior_submap_id;

  bool visualize_constraints;               // draw the GNSS constraints in the viewer
  std::shared_ptr<GnssVisState> vis_state;  // viewer side data (accessed only in the viewer thread)

  bool transformation_initialized;
  Eigen::Isometry3d T_world_utm;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GNSSGlobal();
}
