/*
  multi_sensor_calibration
  Copyright (C) 2019 Intelligent Vehicles, Delft University of Technology

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "robust_least_squares.hpp"

// Given two sets of 3D points, find the rotation + translation + scale
// which best maps the first set to the second.
// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm

// The input 3D points are stored as columns.
Eigen::Isometry3d estimateIsometryTransform(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out) {
  // Adapted version from: https://github.com/oleg-alexandrov/

  // Default output
  Eigen::Isometry3d A;
  A.linear() = Eigen::Matrix3d::Identity(3, 3);
  A.translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < in.cols()-1; col++) {
    dist_in  += (in.col(col+1) - in.col(col)).norm();
    dist_out += (out.col(col+1) - out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return A;
  double scale = dist_out/dist_in;
  out /= scale;

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr  += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= in.cols();
  out_ctr /= out.cols();
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= in_ctr;
    out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  A.linear() = scale * R;
  A.translation() = scale*(out_ctr - R*in_ctr);

  return A;
}


Eigen::Matrix3Xd index_eigen_matrix(Eigen::Matrix3Xd in, Eigen::VectorXi indices) {
  Eigen::Matrix3Xd indexed(3,indices.size());
  int n = 0;
  for (int c = 0; c < indices.size(); c++) {
    for (int r = 0; r < indexed.rows(); r++) {
      indexed(r, n) = in(r, indices(c));
    }
    n = n + 1;
  }

  return indexed;
}

Eigen::MatrixXi findAllPermutations(std::vector<int> v)
{
    std::vector<std::vector<int> > out;
    std::sort(v.begin(), v.end());
    do {
        // Push back input std::vector
        out.push_back(v);
    } while (std::next_permutation(v.begin(), v.end()));

    // Convert to Eigen
    Eigen::MatrixXi out_matrix(out.size(), v.size());
    for (int i = 0; i < out.size(); i++) {
        for (int j = 0; j < v.size(); j++) {
          out_matrix(i,j) = out[i][j];
        }
    }

    return out_matrix;
}

double compute_distance(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Affine3d Tm) {
  Eigen::Vector3d delta;
  delta  = Tm.rotation() * v1 + Tm.translation() - v2;

  return (delta.transpose()*delta).sum();
}

Eigen::ArrayXi findCorrespondences(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out, Eigen::MatrixXi all_permutations) {
  // Find best match
  double best_error = std::numeric_limits<double>::max();
  int best_i;
  Eigen::VectorXi indices(3);
  for (int i = 0; i < all_permutations.rows(); i++) {
    // Index it using this  combinations
    indices << all_permutations.row(i)[0], all_permutations.row(i)[1], all_permutations.row(i)[2];

    // Find Transformation
    Eigen::Isometry3d A = estimateIsometryTransform(index_eigen_matrix(in, indices), out.leftCols(3));

    // Compute errors for this transformation matrix:
    Eigen::Vector3d error = Eigen::Vector3d::Zero();
    for (int  j = 0; j < 3; j++) {
      error(j) = compute_distance(in.col(indices(j)), out.col(j), A);
    }

    // Check if better than best up till now:
    if (error.sum() < best_error) {
      best_error = error.sum();
      best_i = i;
    }
  }

  // Find best correspondeces and return those:
  Eigen::VectorXi indices_best(4);
  indices_best << all_permutations.row(best_i)[0], all_permutations.row(best_i)[1], all_permutations.row(best_i)[2], all_permutations.row(best_i)[3];

  return indices_best;
}

Eigen::Matrix3Xd refine3D(Eigen::Matrix3Xd calibration_board, Eigen::Matrix3Xd detections, double threshold_inlier) {
    // Find all combinations:
  std::vector<int> all_possible_indices;
  all_possible_indices.push_back(0);
  all_possible_indices.push_back(1);
  all_possible_indices.push_back(2);
  all_possible_indices.push_back(3);
  Eigen::MatrixXi all_permutations = findAllPermutations(all_possible_indices);

  // Find best correspondences
  Eigen::ArrayXi indices_correspondences = findCorrespondences(detections, calibration_board, all_permutations);

  // Find initial estimate of Tm
  Eigen::Isometry3d initial_tm = estimateIsometryTransform(index_eigen_matrix(detections, indices_correspondences.head(3)), calibration_board.leftCols(3));

  // Refine using all?
  Eigen::Isometry3d final_tm;
  double worst_detection_error = compute_distance(index_eigen_matrix(detections, indices_correspondences.tail(1)), calibration_board.rightCols(1), final_tm);

  if (worst_detection_error < threshold_inlier) {
    final_tm = estimateIsometryTransform(index_eigen_matrix(detections,indices_correspondences), calibration_board);
  }
  else {
    final_tm = initial_tm;
  }

  // Return LS estimates:
  Eigen::Isometry3d inverse_tm = final_tm.inverse();
  Eigen::Matrix3Xd ls_estimates = Eigen::Matrix3Xd::Zero(3,4);
  for (int  j = 0; j < calibration_board.cols(); j++) {
    ls_estimates.col(j) = inverse_tm.rotation() * calibration_board.col(j) + inverse_tm.translation();
  }

  return ls_estimates;
}

// The input 3D points are stored as columns.
Eigen::Isometry2d estimateIsometryTransform2D(Eigen::Matrix2Xd in, Eigen::Matrix2Xd out) {
  // Adapted version from: https://github.com/oleg-alexandrov/

  // Default output
  Eigen::Isometry2d A;
  A.linear() = Eigen::Matrix2d::Identity(2, 2);
  A.translation() = Eigen::Vector2d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < in.cols()-1; col++) {
    dist_in  += (in.col(col+1) - in.col(col)).norm();
    dist_out += (out.col(col+1) - out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return A;
  double scale = dist_out/dist_in;
  out /= scale;

  // Find the centroids then shift to the origin
  Eigen::Vector2d in_ctr = Eigen::Vector2d::Zero();
  Eigen::Vector2d out_ctr = Eigen::Vector2d::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr  += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= in.cols();
  out_ctr /= out.cols();
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= in_ctr;
    out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix2d I = Eigen::Matrix2d::Identity(2, 2);
  I(1, 1) = d;

  Eigen::Matrix2d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  A.linear() = scale * R;
  A.translation() = scale*(out_ctr - R*in_ctr);

  return A;
}

Eigen::Matrix2Xd index_eigen_matrix2D(Eigen::Matrix2Xd in, Eigen::VectorXi indices) {
  Eigen::Matrix2Xd indexed(2,indices.size());
  int n = 0;
  for (int c = 0; c < indices.size(); c++) {
    for (int r = 0; r < indexed.rows(); r++) {
      indexed(r, n) = in(r, indices(c));
    }
    n = n + 1;
  }

  return indexed;
}

Eigen::MatrixXi findAllPermutations2D(std::vector<int> v)
{
    std::vector<std::vector<int> > out;
    std::sort(v.begin(), v.end());
    do {
        // Push back input std::vector
        out.push_back(v);
    } while (std::next_permutation(v.begin(), v.end()));

    // Convert to Eigen
    Eigen::MatrixXi out_matrix(out.size(), v.size());
    for (int i = 0; i < out.size(); i++) {
        for (int j = 0; j < v.size(); j++) {
          out_matrix(i,j) = out[i][j];
        }
    }

    return out_matrix;
}

double compute_distance2D(Eigen::Vector2d v1, Eigen::Vector2d v2, Eigen::Isometry2d Tm) {
  Eigen::Vector2d delta;
  delta  = Tm.rotation() * v1 + Tm.translation() - v2;
  return (delta.transpose()*delta).sum();
}

Eigen::ArrayXi findCorrespondences2D(Eigen::Matrix2Xd in, Eigen::Matrix2Xd out, Eigen::MatrixXi all_permutations) {
  // Find best match
  double best_error = std::numeric_limits<double>::max();
  int best_i;
  Eigen::VectorXi indices(3);
  for (int i = 0; i < all_permutations.rows(); i++) {
    // Index it using this  combinations
    indices << all_permutations.row(i)[0], all_permutations.row(i)[1], all_permutations.row(i)[2];

    // Find Transformation
    Eigen::Isometry2d A = estimateIsometryTransform2D(index_eigen_matrix2D(in, indices), out.leftCols(3));

    // Compute errors for this transformation matrix:
    Eigen::Vector3d error = Eigen::Vector3d::Zero();
    for (int  j = 0; j < 3; j++) {
      error(j) = compute_distance2D(in.col(indices(j)), out.col(j), A);
    }

    // Check if better than best up till now:
    if (error.sum() < best_error) {
      best_error = error.sum();
      best_i = i;
    }
  }

  // Find best correspondeces and return those:
  Eigen::VectorXi indices_best(4);
  indices_best << all_permutations.row(best_i)[0], all_permutations.row(best_i)[1], all_permutations.row(best_i)[2], all_permutations.row(best_i)[3];

  return indices_best;
}

Eigen::Isometry3d compute_plane_tm(Eigen::Matrix3Xd calibration_board, Eigen::Matrix3Xd detections) {
  Eigen::Isometry3d Tm;

  // Compute normal vector
  double best_angle =  std::numeric_limits<double>::max();
  Eigen::Vector3d normal_vector;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  for (size_t k = 0; k<detections.cols(); k++ ) {
    for (size_t i = k+1 ; i < 3; i++) {
      for (size_t j = i+1 ; j < 3; j++) {
          // Select two points and determine vectors
          Eigen::Vector3d c1 = detections.col(i)- detections.col(k);
          Eigen::Vector3d c2 = detections.col(j)- detections.col(k);

          // Compute angle between vectors
          double angle = c1.dot(c2);

          // Find set of 3 points without outlier
          // If angle between c1 and c2 is 90 degrees this is a good triangle
          if (angle < best_angle) {
            best_angle = angle;
            normal_vector = c1.cross(c2);
            normal_vector = normal_vector/normal_vector.norm();

            // Find R, T
            R.col(0) = c1/ c1.norm();
            R.col(2) = normal_vector;
            R.col(1) = R.col(0).cross(R.col(2));
            t =detections.col(k) - (R*calibration_board.col(0));

            // Save them in Tm
            Tm.linear() = R;
            Tm.translation()  = t;
          }
      }
    }
  }

  return Tm;
}
Eigen::Matrix2Xd projectToPlanarCoordinateFrame(Eigen::Matrix3Xd X, Eigen::Isometry3d Tm) {
  // Map to plane
  return (Tm*X).topRows(2);
}
Eigen::Matrix3Xd projectBackSensor(Eigen::Matrix2Xd X, Eigen::Isometry3d Tm) {
  Eigen::Matrix3Xd Xout = Eigen::Matrix3Xd::Zero(3, X.cols());
  Xout.topRows(2) = X;
  Xout = Tm * Xout;

  return Xout;
}

Eigen::Matrix3Xd refine2D(Eigen::Matrix3Xd calibration_board, Eigen::Matrix3Xd detections_3d, double threshold_inlier) {
  // Projec to plane
  Eigen::Isometry3d tm = compute_plane_tm(calibration_board, detections_3d);
  Eigen::Matrix2Xd detections = projectToPlanarCoordinateFrame(detections_3d, tm.inverse());

  // Find all combinations:
  std::vector<int> all_possible_indices;
  all_possible_indices.push_back(0);
  all_possible_indices.push_back(1);
  all_possible_indices.push_back(2);
  all_possible_indices.push_back(3);
  Eigen::MatrixXi all_permutations = findAllPermutations(all_possible_indices);

  // Find best correspondences
  Eigen::ArrayXi indices_correspondences = findCorrespondences2D(detections, calibration_board.topRows(2), all_permutations);

  // Find initial estimate of Tm
  Eigen::Isometry2d initial_tm = estimateIsometryTransform2D(index_eigen_matrix2D(detections, indices_correspondences.head(3)), calibration_board.topRows(2).leftCols(3));

  // Refine using all?
  Eigen::Isometry2d final_tm;
  double worst_detection_error = compute_distance2D(index_eigen_matrix2D(detections, indices_correspondences.tail(1)), calibration_board.topRows(2).rightCols(1), final_tm);

  if (worst_detection_error < threshold_inlier) {
    final_tm = estimateIsometryTransform2D(index_eigen_matrix2D(detections,indices_correspondences), calibration_board.topRows(2));
  }
  else {
    final_tm = initial_tm;
  }

  // Use the transformation matrix and the calibration_board to get new estimates:
  Eigen::Isometry2d inverse_tm = final_tm.inverse();
  Eigen::Matrix2Xd ls_estimates = Eigen::Matrix2Xd::Zero(2,4);
  for (int  j = 0; j < calibration_board.cols(); j++) {
    ls_estimates.col(j) = inverse_tm.rotation() * calibration_board.topRows(2).col(j) + inverse_tm.translation();
  }

  // Transform back to 3d:
  Eigen::Matrix3Xd detections3d = projectBackSensor(ls_estimates, tm);

  return detections3d;
}
