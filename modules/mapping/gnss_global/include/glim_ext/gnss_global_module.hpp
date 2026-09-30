#include <deque>
#include <atomic>
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

  // [stamp, x, y, z, yaw, yaw_var] in the MGRS frame (yaw: heading of base_link [rad], yaw_var: its variance [rad^2])
  using GnssData = Eigen::Matrix<double, 6, 1>;

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
    last_yaw_prior_submap_id = -1;
    transformation_initialized = false;
    T_world_utm.setIdentity();

    kill_switch = false;
    thread = std::thread([this] { backend_task(); });

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&GNSSGlobal::on_insert_submap, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&GNSSGlobal::on_smoother_update, this, _1, _2, _3));
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
    // Heading of base_link in the MGRS frame and its variance (covariance order: x, y, z, roll, pitch, yaw)
    const auto& o = gnss_msg->pose.pose.orientation;
    Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    double yaw = 0.0;
    double yaw_var = gnss_msg->pose.covariance[35];
    if (!std::isfinite(q.norm()) || q.norm() < 1e-6) {
      // Invalid orientation (e.g., all zero before the EKF initialization): the heading is unavailable, the position is still used
      yaw_var = std::numeric_limits<double>::infinity();
    } else {
      q.normalize();
      yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    gnss_data << to_sec(gnss_msg->header.stamp), absolute_mgrs_x, absolute_mgrs_y, absolute_alt, yaw, yaw_var;
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
        // Interpolate the heading along the shortest arc (e.g., -179 deg -> 179 deg)
        const double dyaw = std::remainder((*right)[4] - (*left)[4], 2.0 * M_PI);
        interpolated[4] = std::remainder((*left)[4] + p * dyaw, 2.0 * M_PI);
        // The heading is unavailable if it is unavailable at either end (and 0 * inf would give NaN, i.e., "no covariance")
        if (std::isinf((*left)[5]) || std::isinf((*right)[5])) {
          interpolated[5] = std::numeric_limits<double>::infinity();
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

      // Add translation prior factor
      if (transformation_initialized) {
        const Eigen::Vector3d xyz = T_world_utm * submap_coords.back().segment<3>(1);
        // const Eigen::Vector3d xyz = submap_coords.back().segment<3>(1);

        logger->debug("submap={} gnss={}", convert_to_string(submaps.back()->T_world_origin.translation().eval()), convert_to_string(xyz));

        const auto& submap = submaps.back();
        // note: should use a more accurate information matrix
        if (dynamic_scale) {
          logger->info("curent_scale={}", convert_to_string(current_inf_scale));
          if (current_inf_scale.minCoeff() <= 1e3|| current_inf_scale.z() <= 2e3) {
            if (current_inf_scale.minCoeff() <= 1e-2) {
              logger->error("Gnss abodonne");
              continue;
            }
            else {
              logger->warn("Gnss faible");
              current_inf_scale = Eigen::Vector3d(1e3, 1e3, 3e3);
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
        // //ajouter une contrainte GNSS (rouge dans la viewer)
        // gtsam::NonlinearFactor::shared_ptr factor(new gtsam::PoseTranslationPrior<gtsam::Pose3>(X(submap->id), xyz, model));
        // output_factors.push_back(factor);

        // Add yaw prior factor (once per submap)
        if (enable_yaw_prior && submap->id != last_yaw_prior_submap_id) {
          add_yaw_prior(submap, submap_coords.back());
          last_yaw_prior_submap_id = submap->id;
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
      if (std::isinf(gnss[5])) {
        continue;  // heading unavailable
      }
      if (dynamic_scale && std::isfinite(gnss[5]) && gnss[5] > 0.0 && std::sqrt(gnss[5]) > yaw_max_sigma) {
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
   * @brief Constrain the heading of a submap with the GNSS heading.
   * @note  A PriorFactor<Pose3> (serializable) is used as a yaw-only prior: its measured rotation is the current estimate rotated
   *        to the GNSS heading, and only the z component of its error is weighted. Roll/pitch and translation get a negligible
   *        weight. This component is the heading difference around the world z axis (up to the small vehicle roll/pitch).
   *        The submap origin is assumed to be the base_link frame (IMU frame = base_link).
   */
  void add_yaw_prior(const SubMap::ConstPtr& submap, const GnssData& gnss) {
    // yaw_var = +inf: heading unavailable (invalid orientation), NaN or <= 0: no covariance given (fallback to yaw_prior_sigma)
    if (std::isinf(gnss[5])) {
      logger->debug("skip yaw prior for submap {} (heading unavailable)", submap->id);
      return;
    }

    double sigma = yaw_prior_sigma;
    if (dynamic_scale) {
      const double yaw_var = gnss[5];
      if (!std::isfinite(yaw_var) || yaw_var <= 0.0) {
        sigma = yaw_prior_sigma;
      } else {
        // Floor the sigma: the covariance of the EKF output is overconfident
        sigma = std::max(std::sqrt(yaw_var), yaw_min_sigma);
      }
    }

    if (sigma > yaw_max_sigma) {
      logger->debug("skip yaw prior for submap {} (sigma={:.2f} deg)", submap->id, sigma * 180.0 / M_PI);
      return;
    }

    // T_world_utm is a rotation around the z axis only: yaw_world = yaw_mgrs - yaw(T_utm_world)
    const Eigen::Matrix3d R_world_utm = T_world_utm.linear();
    const double yaw_world_utm = std::atan2(R_world_utm(1, 0), R_world_utm(0, 0));
    const double yaw_world = std::remainder(gnss[4] + yaw_world_utm, 2.0 * M_PI);

    // Measured rotation = estimated rotation turned around the world z axis to the GNSS heading. Keeping the estimated roll/pitch
    // in the prior avoids a heading bias of about roll*pitch/2 on slopes that Rz(yaw) alone would introduce.
    // The estimate only provides roll/pitch (its yaw cancels out). The odometry pose of the origin frame is used instead of
    // submap->T_world_origin, which is concurrently updated by the global mapping thread. Both world frames are gravity-aligned.
    // Only the rotation part matters. The translation of the estimate keeps the (negligible) translation error small
    const Eigen::Isometry3d& T_world_est = submap->origin_frame()->T_world_imu;
    const Eigen::Matrix3d R_world_est = T_world_est.linear();
    const double yaw_est = std::atan2(R_world_est(1, 0), R_world_est(0, 0));
    const gtsam::Rot3 R_prior = gtsam::Rot3::Rz(std::remainder(yaw_world - yaw_est, 2.0 * M_PI)) * gtsam::Rot3(R_world_est);
    const gtsam::Pose3 prior(R_prior, gtsam::Point3(T_world_est.translation()));

    constexpr double loose = 1e3;
    const gtsam::Vector6 sigmas = (gtsam::Vector6() << loose, loose, sigma, loose, loose, loose).finished();
    const auto model = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    output_factors.push_back(gtsam::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(submap->id), prior, model));

    logger->debug("yaw prior submap={} yaw_gnss={:.2f} yaw_est={:.2f} sigma={:.2f} [deg]", submap->id, yaw_world * 180.0 / M_PI, yaw_est * 180.0 / M_PI, sigma * 180.0 / M_PI);
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
  int last_yaw_prior_submap_id;



  bool transformation_initialized;
  Eigen::Isometry3d T_world_utm;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::GNSSGlobal();
}
