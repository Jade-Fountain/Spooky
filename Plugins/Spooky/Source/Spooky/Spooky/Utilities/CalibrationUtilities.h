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
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/unsupported/KroneckerProduct>
#include "CommonMath.h"

//#include "Logging.h"
#pragma once
namespace spooky{
	namespace utility{
		namespace calibration {

			namespace Time {
				//Returns the time that that x leads y
				static inline float estimateLatency(
					const Eigen::VectorXf& x,
					const Eigen::VectorXf& tx,
					const Eigen::VectorXf& y,
					const Eigen::VectorXf& ty
				) {
					if (x.cols() != tx.cols() || y.cols() != ty.cols()) {
						throw std::runtime_error(__LINE__ + "data and timestamps differ in count");
					}
					
					std::vector<float> latencies;
					for (int i = 0; i < y.size(); i++) {
						for (int j = 0; j < x.size() - 1; j++) {
							float det = (x(j) - y(i))*(x(j + 1) - y(i));
							//If the value of y intersects x graph in this interval
							if (det <= 0) {
								//Interpolate fraction alpha between x(j+1) and x(j)
								float alpha = (y(i) - x(j)) / (x(j + 1) - x(j));
								//Get t based on interpolation between two successive measurements
								float t = tx(j + 1) * alpha + tx(j) * (1 - alpha);
								//Latency corresponding to this timestamp
								float latency = t - ty(j);
								//TODO: throwout some latencies bigger or smaller than maximum possible
								latencies.push_back(latency);
							}
						}
					}
					//Debug:
					//std::stringstream ss;
					//ss << "Data: x = " << std::endl << x << std::endl << "tx" << std::endl << tx << std::endl;
					//ss << "Data: y = " << std::endl << y << std::endl << "ty" << std::endl << ty;
					//SPOOKY_LOG(ss.str());

					return getHistogramPeak(latencies);
				}
			}

			//For calibrating data with position only
			namespace Position {

				//Common functions for position data
				static inline float errorFunc(const Eigen::MatrixXf E) {
					//return E.rowwise().mean().norm();
					return E.norm() / E.cols();
				}
				static inline float getError(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					const Eigen::Transform<float, 3, Eigen::Affine>& X
				) {
					Eigen::MatrixXf A(4, samplesA.size());
					Eigen::MatrixXf B(4, samplesB.size());

					for (int i = 0; i < samplesA.size(); i++) {
						A.col(i) << samplesA[i], 1;
						B.col(i) << samplesB[i], 1;
					}

					return errorFunc(X.matrix() * A - B);
				}
				//For calibrating a pair of systems with two sensors measuring the same point from two different reference frames
				// Xa = b
				// or XA = B
				static inline Eigen::Transform<float, 3, Eigen::Affine> calibrateIdenticalPairTransform(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					float* error = NULL
				) {
					if (samplesA.size() != samplesB.size()) {
						throw std::runtime_error(std::string(__FILE__) + std::to_string(__LINE__) + std::string(" : samplesA and samplesB of different size"));
					}

					Eigen::MatrixXf A(4, samplesA.size());
					Eigen::MatrixXf B(4, samplesB.size());

					for (int i = 0; i < samplesA.size(); i++) {
						A.col(i) << samplesA[i], 1;
						B.col(i) << samplesB[i], 1;
					}

					Eigen::Matrix4f X = B * pInv(A);
					//Make sure normalised
					Eigen::Transform<float, 3, Eigen::Affine> TX = matrixToTransform3D(X);

					if (error != NULL) {
						auto E = TX.matrix() * A - B;
						*error = errorFunc(E);
					}

					return TX;
				}

				//For calibrating a pair of systems with two sensors measuring the same point from two different reference frames
				// Xa = b
				// or XA = B
				//K.S.Arun, T.S.Huang and S.D.Blostein, "Least-Squares Fitting of Two 3-D Point Sets," in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol.PAMI - 9, no. 5, pp. 698 - 700, Sept. 1987.
				//doi: 10.1109 / TPAMI.1987.4767965
				//keywords : {Application software; Computer vision; Economic indicators; Iterative algorithms; Matrix decomposition; Motion estimation; Parameter estimation; Position measurement; Quaternions; Singular value decomposition; Computer vision; least - squares; motion estimation; quaternion; singular value decomposition},
				//URL : http ://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4767965&isnumber=4767950
				static inline Eigen::Transform<float, 3, Eigen::Affine> calibrateIdenticalPairTransform_Arun(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					float* error = NULL
				) {
					if (samplesA.size() != samplesB.size()) {
						throw std::runtime_error(std::string(__FILE__) + std::to_string(__LINE__) + std::string(" : samplesA and samplesB of different size"));
					}
					//Compute centroids
					Eigen::Vector3f meanA(0, 0, 0);
					Eigen::Vector3f meanB(0, 0, 0);

					Eigen::MatrixXf A(4, samplesA.size());
					Eigen::MatrixXf B(4, samplesB.size());

					for (int i = 0; i < samplesA.size(); i++) {
						meanA += samplesA[i];
						meanB += samplesB[i];

						A.col(i) << samplesA[i], 1;
						B.col(i) << samplesB[i], 1;
					}
					meanA = meanA / samplesA.size();
					meanB = meanB / samplesB.size();

					//init key matrix
					Eigen::Matrix3f H = Eigen::Matrix3f::Zero();

					//Compute centered point matrix H
					for (int i = 0; i < samplesA.size(); i++) {
						Eigen::Vector3f q = samplesA[i] - meanA;
						Eigen::Vector3f q_primed = samplesB[i] - meanB;
						H += q * q_primed.transpose();
					}

					//Compute svd of H
					Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
					Eigen::Matrix3f RX = svd.matrixV() * svd.matrixU().transpose();
					//If det(RX) != 1 then fail!
					if (std::fabs(RX.determinant() - 1) > 0.01) {
						//Failed!
						if (error != NULL) {
							*error = -1;
						}
						return Eigen::Transform<float, 3, Eigen::Affine>::Identity();
					}

					//Calculate translation
					Eigen::Translation3f t = Eigen::Translation3f(meanB - RX * meanA);
					
					//Put into transform
					Eigen::Transform<float, 3, Eigen::Affine> TX(t);
					TX.rotate(RX);

					//Get error
					if (error != NULL) {
						auto E = TX.matrix() * A - B;
						*error = errorFunc(E);
					}

					return TX;
				}

				static inline Eigen::Transform<float, 3, Eigen::Affine> calibrateWeightedIdenticalPair(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					const std::vector<Eigen::Matrix3f>& invVariances,
					float* error = NULL
				) {
					if (samplesA.size() != samplesB.size()) {
						throw std::runtime_error(std::string(__FILE__) + std::to_string(__LINE__) + std::string(" : samplesA and samplesB of different size"));
					}

					Eigen::MatrixXf A(4, samplesA.size());
					Eigen::MatrixXf B(4, samplesB.size());
					Eigen::MatrixXf sigInv = Eigen::MatrixXf::Identity(4*samplesA.size(), 4 * samplesA.size());

					for (int i = 0; i < samplesA.size(); i++) {
						A.col(i) << samplesA[i], 1;
						B.col(i) << samplesB[i], 1;
						//Variance for each vector
						sigInv.block<3, 3>(4 * i, 4 * i) = invVariances[i];
						//Low variance for fourth component
						sigInv(4 * i + 3, 4 * i + 3) = 1.0/0.001;
					}

					//Get vecB, stacked columns
					Eigen::Map<Eigen::MatrixXf> vecB(B.data(), B.rows()*B.cols(), 1);

					//kronA = kron(A.t,I_m) and m=p=4
					Eigen::MatrixXf kronA = Eigen::kroneckerProduct(A.transpose(),Eigen::MatrixXf::Identity(4,4));

					//kronA.T * sig
					Eigen::MatrixXf AsigAInv = (kronA.transpose() * sigInv * kronA).inverse();					

					Eigen::MatrixXf vecX = AsigAInv * kronA.transpose() * sigInv * vecB;

					Eigen::Map<Eigen::MatrixXf> X(vecX.data(), 4, 4);
					//Make sure normalised
					Eigen::Transform<float, 3, Eigen::Affine> TX = matrixToTransform3D(X);
					
					//DEBUG
					//std::stringstream ss;
					//ss << "B = \n" << B << std::endl;
					//ss << "vecB = \n" << vecB << std::endl;
					//ss << "sigInv =\n " << sigInv << std::endl;
					//ss << "vecX = \n" << vecX << std::endl;
					//ss << "X = \n" << X << std::endl;
					//SPOOKY_LOG(ss.str());
					
					if (error != NULL) {
						auto E = TX.matrix() * A - B;
						*error = errorFunc(E);
					}

					return TX;
				}

				//For calibrating rotation between a pair of systems with two sensors measuring the same point from two different reference frames
				// Xa = b
				// or XA = B
				static inline Eigen::Transform<float, 3, Eigen::Affine> refineIdenticalPairRotation(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					const Eigen::Transform<float, 3, Eigen::Affine>& X,
					float* error = NULL
				) {
					if (samplesA.size() != samplesB.size()) {
						throw std::runtime_error(std::string(__FILE__) + std::to_string(__LINE__) + std::string(" : samplesA and samplesB of different size"));
					}

					Eigen::MatrixXf A(3, samplesA.size());
					Eigen::MatrixXf B(3, samplesB.size());
					Eigen::Vector3f p = X.translation();

					for (int i = 0; i < samplesA.size(); i++) {
						A.col(i) << samplesA[i];
						B.col(i) << samplesB[i] - p;
					}

					Eigen::Matrix3f R = B * pInv(A);
					Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
					T.topLeftCorner(3, 3) = R;
					T.topRightCorner(3, 1) = X.translation();

					//Make sure normalised
					Eigen::Transform<float, 3, Eigen::Affine> TX = matrixToTransform3D(T);

					if (error != NULL) {
						auto E = TX.matrix().topLeftCorner(3,3) * A - B;
						*error = errorFunc(E);
					}


					//DEBUG
					//std::stringstream ss;
					//ss << "Rotation matrix raw = \n" << T << std::endl;
					//ss << "Rotation matrix reorth = \n" << TX.matrix() << std::endl;
					//SPOOKY_LOG(ss.str());

					//If answer invalid, return same as before
					if (TX.matrix().hasNaN()) return X;
					//Return improved
					return TX;
				}

				//Simple refinement
				static inline Eigen::Transform<float, 3, Eigen::Affine> refineIdenticalPairTransform(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					const Eigen::Transform<float, 3, Eigen::Affine>& X,
					float* error = NULL
				) {
					float learning_rate = 0.1;
					Eigen::Transform<float, 3, Eigen::Affine> X_new = calibrateIdenticalPairTransform(samplesA, samplesB);

					Eigen::Transform<float, 3, Eigen::Affine> TX = slerpTransform3D(X, X_new, learning_rate);

					if (error != NULL) {
						*error = getError(samplesA,samplesB,X_new) * (learning_rate) + *error;
					}

					return TX;
				}


				//Simple refinement
				static inline Eigen::Transform<float, 3, Eigen::Affine> refineIdenticalPairPosition(
					const std::vector<Eigen::Vector3f>& samplesA,
					const std::vector<Eigen::Vector3f>& samplesB,
					const Eigen::Transform<float, 3, Eigen::Affine>& X,
					float* error = NULL
				) {
					assert(samplesA.size() >= 4);
					assert(samplesA.size() == samplesB.size());

					float learning_rate = 1;
					
					Eigen::MatrixXf A(4, samplesA.size());
					Eigen::MatrixXf B(4, samplesB.size());

					for (int i = 0; i < samplesA.size(); i++) {
						A.col(i) << samplesA[i], 1;
						B.col(i) << samplesB[i], 1;
					}

					Eigen::MatrixXf E = B - X.matrix() * A;

					Eigen::Vector3f meanError = E.rowwise().mean();

					if (meanError.hasNaN()) {
						meanError = Eigen::Vector3f::Zero();
					}
					//Be careful of sphere error
					utility::Sphere sphere = fitSphere(E.topLeftCorner(3,E.cols()));

					Eigen::Vector3f centerError = sphere.center;


					if (meanError.hasNaN()) {
						centerError = Eigen::Vector3f::Zero();
					}
					//std::stringstream ss;
					//ss << "errorMat = " << E.transpose() << std::endl;
					//ss << "average error = " << meanError.transpose() << std::endl;
					//ss << "sphere center = " << sphere.center.transpose();
					//ss << "sphere radius = " << sphere.r << std::endl;
					//SPOOKY_LOG(ss.str());

					Eigen::Transform<float, 3, Eigen::Affine> X_new = X;

					X_new.pretranslate(centerError);
					//X_new.pretranslate(meanError);

					if (error != NULL) {
						*error = getError(samplesA, samplesB, X_new) * (learning_rate) + *error * (1-learning_rate);
					}

					return X_new;
				}

			}

			//For calibrating rotation only
			namespace Rotation {

			}

			//For calibrating position and rotation data simultaneously
			namespace Transform {

				static inline float getTwoSystemsError(
					Eigen::Transform<float, 3, Eigen::Affine> X, Eigen::Transform<float, 3, Eigen::Affine> Y,
					const std::vector<Eigen::Matrix4f>& samplesA, const std::vector<Eigen::Matrix4f>& samplesB) 
				{
					float error = 0;
					for (int i = 0; i < samplesA.size(); i++) {
						const Eigen::MatrixXf& A = samplesA[i];
						const Eigen::MatrixXf& B = samplesB[i];
						error += (A * X.matrix()  - Y.matrix() * B).norm();
					}
					return error / samplesA.size();
				}
				
				static inline std::pair<Eigen::Vector3f,Eigen::Vector3f> getTranslationComponent
				(const std::vector<Eigen::Matrix4f>& samplesA, const std::vector<Eigen::Matrix4f>& samplesB,const Eigen::Matrix3f& Ry, bool& success){
					Eigen::MatrixXf combinedF; 
					Eigen::VectorXf combinedD; 

					for (int i = 0; i < samplesA.size(); i++){
						Eigen::Matrix3f RA = samplesA[i].topLeftCorner(3,3); 
						Eigen::Vector3f pA = samplesA[i].topRightCorner(3,1); 
						Eigen::Vector3f pB = samplesB[i].topRightCorner(3,1); 

						Eigen::MatrixXf F(3,6);
						F << RA, -Eigen::Matrix3f::Identity(); 

						Eigen::Vector3f D = Ry * pB - pA; 

						if (i == 0){	
							combinedF = F; 
							combinedD = D; 
						}else{
							auto tempF = combinedF;
							combinedF = Eigen::MatrixXf(combinedF.rows() + F.rows(), combinedF.cols());
							combinedF << tempF,F;
							auto tempD = combinedD;
							combinedD = Eigen::VectorXf(combinedD.rows() + D.rows());
							combinedD << tempD, D;
						}
					}

					//Eigen::JacobiSVD<Eigen::MatrixXf> svd(combinedF);
					//TODO: evaluate success
					bool pxpySuccess = true;//solveWithSVD(combinedF,combinedD,pxpy); 
					//Eigen::VectorXf pxpy = svd.solve(combinedD);
					Eigen::VectorXf pxpy = pInv(combinedF) * combinedD;

					if(!pxpySuccess){
						//If SVD fails, return identity
						std::cout << __FILE__ << " : " << __LINE__ << " - WARNING: SVD FAILED" << std::endl;
						success = false;
						return std::pair<Eigen::Vector3f, Eigen::Vector3f>();
					}
					
					std::pair<Eigen::Vector3f, Eigen::Vector3f> txty = std::make_pair(pxpy.topLeftCorner(3,1), pxpy.bottomLeftCorner(3,1));
					return txty;
				}

				/*
				solves AX=YB for X,Y and A in sampleA, B in sampleB

				Typically X is the sensor link transform and Y is the sensor space transform Y:SystemB->System1

				Source:
				@ARTICLE{Zhuang1994, 
				author={Hanqi Zhuang and Roth, Zvi S. and Sudhakar, R.}, 
				journal={Robotics and Automation, IEEE Transactions on}, 
				title={Simultaneous robot/world and tool/flange calibration by solving homogeneous transformation equations of the form AX=YB}, 
				year={1994}, 
				month={Aug}, 
				volume={10}, 
				number={4}, 
				pages={549-554}
				}	
				*/
				static inline std::pair<Eigen::Transform<float, 3, Eigen::Affine>, Eigen::Transform<float, 3, Eigen::Affine>>
				twoSystems_Kronecker_Shah2013(
					const std::vector<Eigen::Matrix4f>& samplesA, 
					const std::vector<Eigen::Matrix4f>& samplesB,
					float* error = NULL)
				{
					if(samplesA.size() < 3 || samplesB.size() < 3){
						std::cout << "CalibrationTools - NEED MORE THAN 2 SAMPLES" << std::endl;
						throw std::domain_error("CalibrationTools - NEED MORE THAN 2 SAMPLES");
					}
					std::pair<Eigen::Transform<float, 3, Eigen::Affine>, Eigen::Transform<float, 3, Eigen::Affine>> 
						result(Eigen::Transform<float, 3, Eigen::Affine>::Identity(),Eigen::Transform<float, 3, Eigen::Affine>::Identity());

					//--------------------------
					//Rotation part
					//--------------------------

					//Create kronecker matrix K
					int n = samplesA.size();
					Eigen::MatrixXf K = Eigen::MatrixXf::Zero(9,9);
					for(int i = 0; i < n; i++){
						Eigen::MatrixXf RA = samplesA[i].topLeftCorner(3, 3);
						Eigen::MatrixXf RB = samplesB[i].topLeftCorner(3, 3);

						K += Eigen::KroneckerProduct<Eigen::MatrixXf, Eigen::MatrixXf>(RB,RA);
					}

					//Take singular value decomposition of K
					Eigen::MatrixXf U,V;
					Eigen::VectorXf s;
					//Get U,V,s
					getSVDComponents<Eigen::MatrixXf>(K,U,s,V);

					// std::cout << "U = \n" << U << std::endl;
					// std::cout << "s = \n" << s << std::endl;
					// std::cout << "V = \n" << V << std::endl;

					//Get index of singular values closest to n
					// Eigen::VectorXf sMinusN = arma::abs(s-double(n));
					// sMinusN.min(index);

					//Use largest singular value
					int index = 0;

					Eigen::VectorXf u = U.col(index);
					Eigen::VectorXf v = V.col(index);

					// std::cout << "u = \n" << u << std::endl;
					// std::cout << "s(index)  = \n" << s(index) << std::endl;
					// std::cout << "v = \n" << v << std::endl;

					Eigen::Matrix3f V_x = Eigen::Map<Eigen::MatrixXf>(u.data(), 3,3);
					Eigen::Matrix3f V_y = Eigen::Map<Eigen::MatrixXf>(v.data(), 3,3);

					float detV_x = V_x.determinant();
					float detV_y = V_y.determinant();

					// std::cout << "det(V_x) = \n" << detV_x << std::endl;
					// std::cout << "det(V_y) = \n" << detV_y << std::endl;

					float alpha_x =  1 / std::cbrt(detV_x);
					float alpha_y =  1 / std::cbrt(detV_y);

					// std::cout << "alpha_x = \n" << alpha_x << std::endl;
					// std::cout << "alpha_y = \n" << alpha_y << std::endl;

					Eigen::Matrix3f R_y = orthogonaliseBasic(alpha_x * V_x);
					Eigen::Matrix3f R_x = orthogonaliseBasic(alpha_y * V_y);

					// std::cout << "det(R_x) = \n" << arma::det(R_x) << std::endl;
					// std::cout << "det(R_y) = \n" << arma::det(R_y) << std::endl;

					//Translation part
					bool success = false;
					auto translation = getTranslationComponent(samplesA, samplesB, R_y, success);

					result.first.translate(translation.first);
					result.second.translate(translation.second);

					result.first.rotate(R_x);
					result.second.rotate(R_y);

					if (error != NULL) {
						*error = getTwoSystemsError(result.first, result.second, samplesA, samplesB);
					}
					return result;
				}
			}

		}
	}
}
