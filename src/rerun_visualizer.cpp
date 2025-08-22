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
  
  // Create ellipsoid mesh points
  const int num_points_theta = 20;
  const int num_points_phi = 10;
  std::vector<rerun::Position3D> vertices;
  std::vector<std::array<uint32_t, 3>> indices;
  
  // Generate ellipsoid vertices
  for (int i = 0; i <= num_points_phi; ++i) {
    float phi = M_PI * float(i) / float(num_points_phi);
    for (int j = 0; j < num_points_theta; ++j) {
      float theta = 2.0f * M_PI * float(j) / float(num_points_theta);
      
      // Parametric ellipsoid equations
      float x = radii(0) * sin(phi) * cos(theta);
      float y = radii(1) * sin(phi) * sin(theta);
      float z = radii(2) * cos(phi);
      
      // Rotate by ellipsoid orientation and translate to frame position
      Eigen::Vector3f local_point(x, y, z);
      Eigen::Vector3f world_point = U.cast<float>() * local_point + center;
      
      vertices.push_back(rerun::Position3D(world_point.x(), world_point.y(), world_point.z()));
    }
  }
  
  // Generate triangle indices for the ellipsoid surface
  for (int i = 0; i < num_points_phi; ++i) {
    for (int j = 0; j < num_points_theta; ++j) {
      int curr = i * num_points_theta + j;
      int next = i * num_points_theta + ((j + 1) % num_points_theta);
      int curr_next_row = (i + 1) * num_points_theta + j;
      int next_next_row = (i + 1) * num_points_theta + ((j + 1) % num_points_theta);
      
      if (i < num_points_phi) {
        // First triangle
        indices.push_back({static_cast<uint32_t>(curr), 
                          static_cast<uint32_t>(next), 
                          static_cast<uint32_t>(curr_next_row)});
        // Second triangle
        indices.push_back({static_cast<uint32_t>(next), 
                          static_cast<uint32_t>(next_next_row), 
                          static_cast<uint32_t>(curr_next_row)});
      }
    }
  }
  
  // Create wireframe ellipsoid using LineStrips instead of filled mesh
  std::vector<rerun::LineStrip3D> wireframe_strips;
  
  // Create horizontal circular wireframe strips
  for (int i = 0; i <= num_points_phi; i += 2) {  // Every other phi level for cleaner wireframe
    float phi = M_PI * float(i) / float(num_points_phi);
    std::vector<rerun::Vec3D> strip_points;
    
    for (int j = 0; j <= num_points_theta; ++j) {
      float theta = 2.0f * M_PI * float(j % num_points_theta) / float(num_points_theta);
      
      // Parametric ellipsoid equations
      float x = radii(0) * sin(phi) * cos(theta);
      float y = radii(1) * sin(phi) * sin(theta);
      float z = radii(2) * cos(phi);
      
      // Rotate by ellipsoid orientation and translate to frame position
      Eigen::Vector3f local_point(x, y, z);
      Eigen::Vector3f world_point = U.cast<float>() * local_point + center;
      
      strip_points.push_back(rerun::Vec3D(world_point.x(), world_point.y(), world_point.z()));
    }
    wireframe_strips.push_back(rerun::LineStrip3D(strip_points));
  }
  
  // Create vertical meridian wireframe strips
  for (int j = 0; j < num_points_theta; j += 3) {  // Every 3rd theta for cleaner wireframe
    float theta = 2.0f * M_PI * float(j) / float(num_points_theta);
    std::vector<rerun::Vec3D> strip_points;
    
    for (int i = 0; i <= num_points_phi; ++i) {
      float phi = M_PI * float(i) / float(num_points_phi);
      
      // Parametric ellipsoid equations
      float x = radii(0) * sin(phi) * cos(theta);
      float y = radii(1) * sin(phi) * sin(theta);
      float z = radii(2) * cos(phi);
      
      // Rotate by ellipsoid orientation and translate to frame position
      Eigen::Vector3f local_point(x, y, z);
      Eigen::Vector3f world_point = U.cast<float>() * local_point + center;
      
      strip_points.push_back(rerun::Vec3D(world_point.x(), world_point.y(), world_point.z()));
    }
    wireframe_strips.push_back(rerun::LineStrip3D(strip_points));
  }
  
  // Create wireframe with red color
  auto wireframe = rerun::LineStrips3D(wireframe_strips)
                     .with_colors(rerun::Rgba32(255, 100, 100, 180)); // Semi-transparent red wireframe
  
  // Log the wireframe ellipsoid
  std::string ellipsoid_path;
  if (static_log) {
    ellipsoid_path = m_prefix + "/manipulability_ellipsoid/" + 
                     m_model.get().frames[frame_id].name + "_" + 
                     std::to_string(m_ellipsoid_counter++);
    stream.log_static(ellipsoid_path, wireframe);
  } else {
    ellipsoid_path = m_prefix + "/manipulability_ellipsoid/" + 
                     m_model.get().frames[frame_id].name;
    stream.log(ellipsoid_path, wireframe);
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
