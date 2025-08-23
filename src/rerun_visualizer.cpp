#include "rerun_visualizer.hpp"

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <Eigen/SVD>

namespace pinrerun {

RerunVisualizer::RerunVisualizer(const pinocchio::Model &model,
                                 const pinocchio::GeometryModel &geomModel,
                                 const std::string &appID,
                                 const std::string &recID)
    : BaseVisualizer(model, geomModel, nullptr), stream(appID, recID),
      m_prefix("pinocchio/" + model.name), m_initialized(false),
      m_recordingID(recID) {
  // Try to spawn a viewer, but don't fail hard if unavailable.
  auto err = stream.spawn();
  (void)err; // ignore error; tests/examples can still proceed without a viewer.
  stream.set_time_seconds("stable_time", 0.0);
}

void RerunVisualizer::loadViewerModel() {
  loadPinocchioModel(*m_visualModel, stream, visualPrefix());
  m_initialized = true;
}

void RerunVisualizer::displayImpl() {

  const auto &geomObjs = m_visualModel->geometryObjects;
  long ngeoms = (long)m_visualModel->ngeoms;

  for (uint i = 0; i < ngeoms; i++) {
    const auto &gobj = geomObjs[i];
    const pinocchio::SE3 &M =
        m_visualData->oMg[m_visualModel->getGeometryId(gobj.name)];

    auto path = getEntityPath(gobj, visualPrefix()).string();
    stream.log(path, pinSE3toRerun(M));
  }
}

void RerunVisualizer::drawFrameVelocities(const vector<FrameIndex> &frame_ids) {
  size_t nframes = frame_ids.size();
  vector<Vector3f> frame_vels(nframes);
  vector<Vector3f> frame_pos(nframes);
  vector<std::string> labels(nframes);
  for (size_t i = 0; i < frame_ids.size(); ++i) {
    auto frame_id = frame_ids[i];
    pinocchio::updateFramePlacement(m_model.get(), *m_data, frame_id);

    auto vel = pinocchio::getFrameVelocity(m_model.get(), *m_data, frame_id,
                                           pinocchio::LOCAL_WORLD_ALIGNED)
                   .cast<float>();
    frame_vels[i] = vel.linear();
    frame_pos[i] = m_data->oMf[frame_id].cast<float>().translation();
    labels[i] = m_model.get().frames[frame_id].name;
  }
  std::string frame_vel_prefix = m_prefix + "/frame_vels";
  stream.log(frame_vel_prefix,
             rerun::Arrows3D::from_vectors(std::move(frame_vels))
                 .with_origins(std::move(frame_pos))
                 .with_labels(std::move(labels)));
}

void RerunVisualizer::drawManipulabilityEllipsoid(FrameIndex frame_id,
                                                   const Eigen::VectorXd &q,
                                                   double scale,
                                                   bool static_log) {
  // Update model with current configuration
  pinocchio::forwardKinematics(m_model.get(), *m_data, q);
  pinocchio::updateFramePlacement(m_model.get(), *m_data, frame_id);
  
  // Compute Jacobian matrix
  Eigen::MatrixXd J(6, m_model.get().nv);
  pinocchio::computeFrameJacobian(m_model.get(), *m_data, q, frame_id,
                                  pinocchio::LOCAL_WORLD_ALIGNED, J);
  
  // Extract linear velocity part (first 3 rows)
  Eigen::MatrixXd J_lin = J.topRows(3);
  
  // Compute manipulability matrix A = J * J^T
  Eigen::Matrix3d A = J_lin * J_lin.transpose();
  
  // Perform SVD to get ellipsoid parameters
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d singular_values = svd.singularValues();
  Eigen::Matrix3d U = svd.matrixU();
  
  // Ellipsoid radii are square roots of singular values
  Eigen::Vector3d radii = singular_values.cwiseSqrt() * scale;
  
  // Get frame position
  pinocchio::SE3 frame_pose = m_data->oMf[frame_id];
  Eigen::Vector3f center = frame_pose.translation().cast<float>();
  
  // Convert rotation matrix to quaternion for rerun
  Eigen::Quaternionf quat(U.cast<float>());
  
  // Create a proper Rerun ellipsoid  
  rerun::datatypes::Quaternion quat_data;
  quat_data.xyzw = {{quat.x(), quat.y(), quat.z(), quat.w()}};
  rerun::components::PoseRotationQuat rotation_quat(quat_data);
  
  auto ellipsoid = rerun::Ellipsoids3D::from_centers_and_half_sizes(
      std::vector<rerun::Vec3D>{{static_cast<float>(center.x()), static_cast<float>(center.y()), static_cast<float>(center.z())}},
      std::vector<rerun::Vec3D>{{static_cast<float>(radii(0)), static_cast<float>(radii(1)), static_cast<float>(radii(2))}})
      .with_quaternions({rotation_quat})
      .with_colors(rerun::Rgba32(255, 100, 100, 120)); // Semi-transparent red
  
  // Log the ellipsoid
  std::string ellipsoid_path;
  if (static_log) {
    ellipsoid_path = m_prefix + "/manipulability_ellipsoid/" + 
                     m_model.get().frames[frame_id].name + "_" + 
                     std::to_string(m_ellipsoid_counter++);
    stream.log_static(ellipsoid_path, ellipsoid);
  } else {
    ellipsoid_path = m_prefix + "/manipulability_ellipsoid/" + 
                     m_model.get().frames[frame_id].name;
    stream.log(ellipsoid_path, ellipsoid);
  }
}

void RerunVisualizer::play(const vector<ConstVectorRef> &qs, double dt,
                           const std::string &timeline) {
  auto n = qs.size();
  for (size_t i = 0; i < n; ++i) {
    double t = int(i) * dt;
    stream.set_time_seconds(timeline, t);
    display(qs[i]);
  }
}

} // namespace pinrerun
