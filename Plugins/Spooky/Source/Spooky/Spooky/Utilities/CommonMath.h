/*  This file is part of Spooky, a sensor fusion plugin for VR in the Unreal Engine
    
   Copyright 2017 Jake Fountain

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

//#include "Logging.h"
#pragma once

#include<iostream>
#include<string>
#include<numeric>
#include<complex>
#include<math.h>
#include<algorithm>
#include<Eigen/Core>
#include<Eigen/SVD>
#include<Eigen/Geometry>
#include<Eigen/Eigenvalues>
#include<Eigen/unsupported/KroneckerProduct>
#include "ComplexMath.h"
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"


namespace spooky{
	namespace utility{


		//Source: https://fuyunfei1.gitbooks.io/c-tips/content/pinv_with_eigen.html
		//Pseudo inverse
		template<typename _Matrix_Type_>
		static inline _Matrix_Type_ pInv(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
		{
			Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
			double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
			return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
		}

		//Source: https://fuyunfei1.gitbooks.io/c-tips/content/pinv_with_eigen.html
		//Pseudo inverse
		template<typename _Matrix_Type_>
		static inline void getSVDComponents(const _Matrix_Type_ &a, Eigen::MatrixXf& U, Eigen::VectorXf& s, Eigen::MatrixXf& V, double epsilon = std::numeric_limits<double>::epsilon())
		{
			Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
			double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
			V = svd.matrixV();
			U =  svd.matrixU();
			s = svd.singularValues();
		}

		static inline Eigen::Matrix3f normalize3D(const Eigen::Matrix3f& M, Eigen::Vector3f* scale_out = NULL) {
			auto result = M;
			for (int i = 0; i < M.cols(); i++) {
				float norm = M.col(i).norm();
				if(scale_out) (*scale_out)[i] = norm;
				result.col(i) = M.col(i) / norm;
			}
			return result;
		}

		static inline Eigen::Matrix3f orthogonaliseBasic(const Eigen::Matrix3f& M) {
			Eigen::Vector3f x = M.col(0);
			x.normalize();
			Eigen::Vector3f y = M.col(1);
			y.normalize();
			Eigen::Vector3f z = M.col(2);
			z.normalize();
			
			float error = x.dot(y);
			float theta = std::acos(error);
			float x_angle = theta / 2 - M_PI / 4;

			Eigen::Vector3f n = x.cross(y);
			n.normalize();

			Eigen::Matrix3f Rx_angle = Eigen::AngleAxisf(x_angle, n).matrix();
			Eigen::Matrix3f Ry_angle = Rx_angle.transpose();

			Eigen::Vector3f x_temp = Rx_angle * x;
			Eigen::Vector3f y_temp = Ry_angle * y;

			Eigen::Vector3f xy = (x + y);
			xy.normalize();
			float phi = std::atan2(z.dot(xy), z.dot(n));

			Eigen::Vector3f r = z.cross(xy);
			
			Eigen::Matrix3f Rxy = Eigen::AngleAxisf(phi / 2, r).matrix();
			Eigen::Matrix3f Rz = Rxy.transpose();

			Eigen::Vector3f x_new = Rxy * x_temp;
			Eigen::Vector3f y_new = Rxy * y_temp;
			Eigen::Vector3f z_new = Rz * z;

			Eigen::Matrix3f result;
			result.col(0) = x_new;
			result.col(1) = y_new;
			result.col(2) = z_new;

			return result;

		}

		static inline Eigen::Transform<float, 3, Eigen::Affine> matrixToTransform3D(const Eigen::Matrix4f& X) {
			//Make sure normalised
			//Eigen::Quaternionf q(X.block<3, 3>(0, 0));
			//q.normalize();
			Eigen::Translation3f t(X.block<3, 1>(0, 3));

			Eigen::Transform<float, 3, Eigen::Affine> TX(t);
			//TX.rotate(q);
			TX.rotate(orthogonaliseBasic(X.topLeftCorner(3, 3)));
			return TX;
		}

		static inline Eigen::Transform<float, 3, Eigen::Affine> slerpTransform3D(
			const Eigen::Transform<float, 3, Eigen::Affine>& T1,
			const Eigen::Transform<float, 3, Eigen::Affine>& T2, 
			float t
		) {
			Eigen::Quaternionf q1(T1.rotation());
			Eigen::Quaternionf q2(T2.rotation());

			Eigen::Quaternionf q = q1.slerp(t, q2);

			Eigen::Vector3f v1(T1.translation());
			Eigen::Vector3f v2(T2.translation());
			Eigen::Translation3f v(v1 * (1 - t) + v2 * t);

			Eigen::Transform<float, 3, Eigen::Affine> T(v);
			T.rotate(q);
			return T;
		}


		static inline float qualityFromError(float error, float scale) {
			float e = error / scale;
			return 1 / (1 + e*e);
		}

		//Takes on the order of 3N, where N = X.size()
		static inline float getHistogramPeak(std::vector<float> X){
			float min = std::numeric_limits<float>::max();
			float max = std::numeric_limits<float>::min();
			//Compute range of data
			for(auto& x : X){
				if(x < min) {
					min = x;
				}
				if(x > max) {
					max = x;
				}
			}
			float range = max - min;
			float delta = range / X.size();

			//Create histogram
			std::vector<int> histogram(X.size(),0);
			for(auto& x : X){
				//Numeric limits excludes the case where x == max, which would give histogram[histogram.size()] which is invalid
				histogram[std::floor((x - min) * (1-std::numeric_limits<float>::min()) / delta)]++; 
			}

			//Find peak of histogram
			int max_index = 0;
			int max_count = 0;
			for(int i = 0; i < histogram.size(); i++){
				if(histogram[i] > max_count){
					max_count = histogram[i];
					max_index = i;
				}
				//SPOOKY_LOG("Histogram: " + std::to_string(i * delta + min) + " count = " + std::to_string(histogram[i]));
			}

			//Return centred max bin
			return (max_index + 0.5) * delta + min;

		}


		//TODO: put this somewhere nicer
		class Sphere {
		public:
			float r = 0;
			Eigen::VectorXf center;

			float getDistanceToPoint(const Eigen::Vector3f& p) const {
				return std::fabs((p - center).norm() - r);
			}

			float getMedianError(const Eigen::MatrixXf& points, const std::vector<int>& inliers) const{
				int num_points = inliers.size();
				std::vector<float> errors;
				for(int i = 0; i < num_points; i++){
					float error = getDistanceToPoint(points.col(inliers[i]));
					//Insert in order
					std::vector<float>::iterator below = errors.begin();
					std::vector<float>::iterator above = errors.end();
					//Binary search for location to insert
					while(std::distance(above,below) > 1){
						std::vector<float>::iterator middle = below;
						std::advance(middle, std::distance(above, below) / 2);
						if (*middle > error) {
							above = middle;
						}
						else {
							below = middle;
						}
					}
					errors.insert(above, error);
				}
				//Return median
				return errors[errors.size() / 2];
			}

			Eigen::Vector3f getAvgErrorVec(const Eigen::MatrixXf& points, const std::vector<int>& inliers) const {
				int num_points = inliers.size();
				Eigen::Vector3f sum_errors;
				for (int i = 0; i < num_points; i++) {
					//Vector from origin to projection of point to sphere
					Eigen::Vector3f proj = (points.col(inliers[i]) - center).normalized() * r + center;
					//Vector from projected point to point
					sum_errors += points.col(inliers[i]) - proj;
				}
				return sum_errors / num_points;
			}

			std::vector<int> getInliers(const Eigen::MatrixXf& points, float inlier_threshold) const {
				int num_points = points.cols();
				std::vector<int> inliers;
				for (int i = 0; i < num_points; i++) {
					float error = getDistanceToPoint(points.col(i));;
					if (error < inlier_threshold) {
						inliers.push_back(i);
					}
				}
				//Return medium
				return inliers;
			}

			Sphere refine(const Eigen::MatrixXf& points, const std::vector<int>& inliers, int iterations) const {
				Sphere s = *this;
				float error = s.getMedianError(points, inliers);
				for (int i = 0; i < iterations; i++) {
					Sphere s_new = s;
					s_new.r = error + s_new.r;
					Eigen::Vector3f error_vec = s_new.getAvgErrorVec(points, inliers);
					s_new.center += error_vec;
					float error_new = s_new.getMedianError(points, inliers);
					if (error_new < error) {
						s = s_new;
						error = error_new;
					}
				}
				return s;
			}

		};

		class Line {
		public:

			Eigen::VectorXf origin;
			Eigen::VectorXf direction;

			Eigen::VectorXf getPoint(const float& t) const {
				return origin + t * direction;
			}

			Eigen::VectorXf intersect(const Line& other, bool* success) {
				//X * [t1,t2]^T = y
				Eigen::MatrixXf X(origin.rows(), 2);
				Eigen::MatrixXf y = other.origin - origin;
				
				X.col(0) = direction;
				X.col(1) = -other.direction;

				Eigen::MatrixXf Xinv = pInv(X);
				Eigen::Vector2f tVals = Xinv * y;

				Eigen::VectorXf thisPoint = getPoint(tVals(0));
				Eigen::VectorXf thatPoint = other.getPoint(tVals(1));

				if((thisPoint - thatPoint).norm() < 0.01){
					*success = true;
				} else {
					*success = false;
				}
				return (thisPoint + thatPoint) / 2;
			}
		};


		static inline Line getCircleNormal(const Eigen::Vector3f& A,const Eigen::Vector3f& B, const Eigen::Vector3f& C){
			Eigen::Vector3f AB = A-B;
			Eigen::Vector3f BC = B-C;
			Eigen::Vector3f CA = C-A;
			
			Line result;
			Eigen::Vector3f normal = AB.cross(BC);
			normal.normalize();
			result.direction = normal; 

			Line lineAB;
			lineAB.origin = B + AB / 2;
			lineAB.direction = AB.cross(normal);

			Line lineBC;
			lineBC.origin = C + BC / 2;
			lineBC.direction = BC.cross(normal);

			bool success = false;
			result.origin = lineBC.intersect(lineAB, &success);
			
			return result;
		}

		static inline Sphere getSphereFrom4Points(const Eigen::Vector3f& A, const Eigen::Vector3f& B, const Eigen::Vector3f& C, const Eigen::Vector3f& D) {
			Sphere result;
			Line ray1 = getCircleNormal(A, B, C);
			Line ray2 = getCircleNormal(B, C, D);
			bool success = false;
			result.center = ray1.intersect(ray2, &success);
			result.r = (result.center - A).norm();
			return result;
		}

		static inline Sphere sphereRANSAC(const Eigen::MatrixXf& points) {
			//Number of points to fit sphere to
			int num_points = points.cols();

			if (num_points < 4) {
				return Sphere();
			}
			//State of optimisation
			std::vector<Sphere> models;
			std::vector<int> best_inliers;
			float best_error = 100000000;
			int best_model_index = 0;
			int max_models = 100;
			int max_search = 1000;
			float inlier_threshold = 0.01;
			int inliers_needed = num_points / 5;

			//Random number selection
			std::vector<int> indices(num_points);
			//Fill blank vector from zero to num_points-1
			std::iota(std::begin(indices), std::end(indices), 0);

			//Point stats
			Eigen::Vector3f mean = points.rowwise().mean();
			float variance = (points.colwise() + mean).colwise().norm().rowwise().mean()[0];

			//Compute models and record the best one
			int i = 0;
			while(models.size() < max_models && i++ < max_search) {
				//Shuffle points to sample
				std::random_shuffle(std::begin(indices), std::end(indices));
				//Get a model
				Sphere model = getSphereFrom4Points(points.col(indices[0]), points.col(indices[1]), points.col(indices[2]), points.col(indices[3]));
				if (model.center.hasNaN() || std::isnan(model.r)) continue;
				//Count inliers
				std::vector<int> inliers = model.getInliers(points, inlier_threshold);

				//If there are enough inliers, check the actual error
				if (inliers.size() > inliers_needed) {
					float error = model.getMedianError(points, inliers);
					models.push_back(model);
					if (error < best_error) {
						best_error = error;
						best_model_index = models.size() - 1;
						best_inliers = inliers;
					}
				}
			}
			//return best + refinement
			if (models.size() == 0) {
				return Sphere();
			}
			else
			{
				return models[best_model_index].refine(points, best_inliers, 10);
			}
		}

		static inline Sphere fitSphere(const Eigen::MatrixXf& points){
			Sphere result;
			Eigen::VectorXf mean = points.rowwise().mean();
			result.center = mean;
			result.r = 0;

			//Check dimension
			if (mean.rows() == 3) {
				Sphere ransacResult = sphereRANSAC(points);
				if (!ransacResult.center.hasNaN() && !std::isnan(ransacResult.r)) {
					result = ransacResult;
				}
			}

			return result;
		}

		//source: http://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
		//sources source: http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf
		//wQ (4xn mat) = unit quaternions in columns multiplied by their weights
		static inline Eigen::Quaternionf averageQuaternions(const Eigen::MatrixXf& wQ) {
			Eigen::MatrixXf Q2 = wQ * wQ.transpose();
			Eigen::EigenSolver<Eigen::MatrixXf> es(Q2);
			//Eigenvalues will be real and positive because matrix is pos semidefinite
			Eigen::VectorXf eval = es.eigenvalues().real();
			Eigen::VectorXf::Index bestEVIndex;
			eval.maxCoeff(&bestEVIndex);
			Eigen::MatrixXf evec = es.eigenvectors().real();
			Eigen::Vector4f best_evec = evec.col(bestEVIndex);

			//DEBUG: Eigen::Vector4f best_evec = wQ.col(0);
			best_evec.normalize();
			return Eigen::Quaternionf(best_evec);
			
		}

		static inline Eigen::Transform<float, 3, Eigen::Affine> 
			getMeanTransform(const std::vector<Eigen::Transform<float, 3, Eigen::Affine>>& T, const std::vector<float>& weights)
		{
			assert(T.size() > 0 && weights.size() > 0);
			if (T.size() == 1) return T[0];
			Eigen::MatrixXf wQ = Eigen::MatrixXf::Zero(4, T.size());
			Eigen::Vector3f t_sum(0, 0, 0);
			float sum_weights = 0;
			float normalising_weight = 0;
			int k = 0;
			while (normalising_weight == 0) {
				normalising_weight = weights[k++];
			}
			for (int i = 0; i < T.size(); i++) {
				wQ.col(i) = Eigen::Quaternionf(T[i].rotation()).coeffs() * weights[i] / normalising_weight;
				t_sum += T[i].translation() * weights[i] / normalising_weight;
				sum_weights += weights[i] / normalising_weight;
			}
			Eigen::Quaternionf q = utility::averageQuaternions(wQ);
			Eigen::Translation3f t(t_sum / sum_weights);
			Eigen::Transform<float, 3, Eigen::Affine> transform(t);
			transform.rotate(q);
			return transform;
		}

		struct TransformNorm{
			float distance = 0;
			float angle = 0;
		};

		static inline TransformNorm transformNorm(const Eigen::Transform<float, 3, Eigen::Affine>& T) {
			Eigen::Quaternionf q(T.rotation());
			TransformNorm tnorm;
			tnorm.angle = q.angularDistance(Eigen::Quaternionf::Identity());
			tnorm.distance = T.translation().norm();
			return tnorm;
		}

		static inline Eigen::Matrix3f jacobianExp(const Eigen::Vector3f& w, const Eigen::Vector3f& p){
			double h = 1e-20;
			Eigen::Matrix3d W_im = Eigen::Matrix3d::Identity() * h;
			Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
			for(int i = 0; i < 3; i++){
				Eigen::Vector3cd W;
				W.real() = w.cast<double>();
				W.imag() = W_im.col(i);
				Eigen::Vector3cd exp = Sophus::SO3<std::complex<double>>::exp(W).matrix() * p.cast<std::complex<double>>() / h;
				result.col(i) = exp.imag();
			}
			return result.cast<float>();
		}

		template <typename Scalar>
		static inline Eigen::Matrix<Scalar, 3, 3> skewSymmetric(const Eigen::Matrix<Scalar, 3, 1>& t) {
			Eigen::Matrix<Scalar, 3, 3> t_hat;
			t_hat << 0, -t(2), t(1),
				t(2), 0, -t(0),
				-t(1), t(0), 0;
			return t_hat;
		}
		
		template <typename Scalar>
		static inline Eigen::Matrix<Scalar,3,3> rodriguezFormula(const Eigen::Matrix<Scalar,3,1>& w) {
			Eigen::Matrix<Scalar, 3, 3> w_hat = skewSymmetric(w.normalized());
			//TODO: is this correct for complex numbers?
			//Scalar theta_sq = w.transpose() * w;
			//Scalar theta = std::pow(theta_sq,0.5);
			Scalar theta = w.norm();
			return Eigen::Matrix<Scalar, 3, 3>::Identity() + w_hat * std::sin(theta) + w_hat * w_hat * (Scalar(1) - std::cos(theta));
		}
		//TODO: clean up unused functions
		static inline Eigen::Matrix3f getPositionVarianceFromRotation(const Eigen::Vector3f& w, const Eigen::Vector3f& p, const Eigen::Matrix3f& sigmaW) {
			Eigen::Matrix3f J = jacobianExp(w, p);
			return J * sigmaW * J.transpose();
		}

		template <typename Scalar>
		static inline Eigen::Matrix<Scalar, 3, 1> toAxisAngle(const Eigen::Matrix<Scalar, 3, 3>& R, Eigen::Vector3f* scale = NULL) {
			return Sophus::SO3<Scalar>::log(Sophus::SO3<Scalar>(normalize3D(R,scale)));
		}

		template <typename Scalar>
		static inline Eigen::Matrix<Scalar, 6, 1> toAxisAnglePos(const Eigen::Transform<Scalar, 3, Eigen::Affine>& T, Eigen::Vector3f* scale = NULL) {
			Eigen::Matrix<Scalar, 6, 1> result;
			//Warning - some functions squash imaginary component - e.g. transform.rotation()
			result.head(3) = toAxisAngle<Scalar>(T.matrix().topLeftCorner(3, 3),scale);
			result.tail(3) = T.translation();
			return result;
		}

		template <typename Scalar>
		static inline Eigen::Matrix<Scalar, 9, 1> toAxisAnglePosScale(const Eigen::Transform<Scalar, 3, Eigen::Affine>& T) {
			Eigen::Matrix<Scalar, 9, 1> result;
			//Warning - some functions squash imaginary component - e.g. transform.rotation()
			Eigen::Vector3f scale;
			result.head(3) = toAxisAngle<Scalar>(T.matrix().topLeftCorner(3, 3),&scale);
			result.segment<3>(3) = T.translation();
			result.tail(3) = scale;
			return result;
		}

		static inline Eigen::Matrix<float, 3, 4> getQuatToAxisJacobian(Eigen::Quaternionf q) {
			Eigen::Matrix<float, 3, 4> result = Eigen::Matrix<float, 3, 4>::Zero();
			float d_sq = 1 - q.w()  * q.w();
			if (d_sq == 0) {
				result.topLeftCorner(3,3) = Eigen::Matrix3f::Identity();
			} else {
				float d = std::sqrt(d_sq);
				float angle = 2 * std::acos(q.w());
				float qwFactor = (2+q.w() * angle / d) / d_sq;
				result(0,3) = q.x() * qwFactor;
				result(1,3) = q.y() * qwFactor;
				result(2,3) = q.z() * qwFactor;
				result(0,0) = angle / d;
				result(1,1) = angle / d;
				result(2,2) = angle / d;
			}
			return result;
			
		}

		//template <typename Scalar>
		//static inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> numericalDerivative
		//(const std::function<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>(const Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic>&)> f,
		// const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& x, 
		// double h = 1e-10) {
		//	//Current state
		//	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> f_x = f(x);
		//	int n = x.rows();
		//	int m = x.cols();
		//	int p = f_x.rows();
		//	int q = f_x.cols();
		//	//Atlas of derivatives
		//	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> df_dx = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n*p,m*q);
		//	//Vary each independent variable and use change in f to estimate derivative
		//	for (int i = 0; i < x.rows(); i++) {
		//		for (int j = 0; j < x.cols(); j++) {
		//			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> xplush = x;
		//			x(i, j) += Scalar(h);
		//			Eigen::Matrix<Scalar, Eigen::Dynmic, Eigen::Dynamic> delf_delx = (f(xplush) - f_x) / h;
		//			df_dx.block(i*p, j*q, p, q) = delf_delx;
		//		}
		//	}
		//	return df_dx;
		//}

		template <typename Scalar>
		static inline Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> numericalVectorDerivative
		(const std::function<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>&)> f,
			const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& x,
			double h = 0.01) {
			//Current state
			Eigen::Matrix<Scalar, Eigen::Dynamic, 1> f_x = f(x);
			int n = x.size();
			int m = f_x.size();
			//Atlas of derivatives
			Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> df_dx = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(m,n);
			//Vary each independent variable and use change in f to estimate derivative
			for (int i = 0; i < n; i++) {
				Eigen::Matrix<Scalar, Eigen::Dynamic, 1> xplush = x;
				xplush(i) += Scalar(h);
				Eigen::Matrix<Scalar, Eigen::Dynamic, 1> f2 = f(xplush);
				Eigen::Matrix<Scalar, Eigen::Dynamic, 1> delf_delx = (f2 - f_x) / h;
				df_dx.col(i) = delf_delx;
			}
			return df_dx;
		}

		//Returns the twist equivalent to w that is closest to target
		// w = (2*pi*n + norm(w))*unit(w) for all n integer
		static inline Eigen::Vector3f twistClosestRepresentation(const Eigen::Vector3f& w, const Eigen::Vector3f& target) {
			float w_angle = w.norm();

			float target_angle = target.norm();

			//handle zero case
			if (w_angle == 0) {
				if (target_angle == 0) {
					return Eigen::Vector3f::Zero();
				}
				//Round target to nearest multiple of 2pi
				int target_ring = std::round(target_angle / (2 * M_PI));
				return (2 * M_PI * target_ring) * target / target_angle;
			}
			else {
				//Project target onto line of w, then round to nearest whole 2pi offset magnitude from w
				int k = std::round(((target).dot(w/w_angle)-w_angle)/(2 * M_PI));
				//Two equivalent twists
				return  w * (w_angle + 2*M_PI*k)/w_angle;
			}
		}

		static inline Eigen::Vector3f composeTwists(const Eigen::Vector3f& w1, const Eigen::Vector3f& w2) {
			return toAxisAngle<float>(rodriguezFormula(w1) * rodriguezFormula(w2));
		}

		static inline Eigen::Vector4f quatClosestRepresentation(const Eigen::Vector4f& q, const Eigen::Vector4f& target) {
			Eigen::Vector4f neg_q = -q;
			if ((q - target).norm() < (neg_q - target).norm()) {
				return q;
			}
			else {
				return neg_q;
			}
		}
	}
}
