#include "stdafx.h"
#include "CppUnitTest.h"
#include <vector>
#include "../Source/Spooky/Spooky/Utilities/CalibrationUtilities.h"
#include "../Source/Spooky/Spooky/Utilities/DataStructures.h"
#include "../Source/Spooky/Spooky/Utilities/CommonMath.h"
#include "../Spooky/Eigen/Core"
#include "../Spooky/Spooky/Core.h"
#include "../Spooky/sophus/so3.hpp"
#include <Windows.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace FusionTesting
{		
	TEST_CLASS(UnitTest1)
	{
	public:
		// Convert a wide Unicode string to an UTF8 string
		std::string utf8_encode(const std::wstring &wstr)
		{
			if (wstr.empty()) return std::string();
			int size_needed = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL);
			std::string strTo(size_needed, 0);
			WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), &strTo[0], size_needed, NULL, NULL);
			return strTo;
		}

		// Convert an UTF8 string to a wide Unicode String
		std::wstring utf8_decode(const std::string &str)
		{
			if (str.empty()) return std::wstring();
			int size_needed = MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), NULL, 0);
			std::wstring wstrTo(size_needed, 0);
			MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), &wstrTo[0], size_needed);
			return wstrTo;
		}

		TEST_METHOD(CalibratePositionalData)
		{
			int N = 1000;
			std::vector<Eigen::Vector3f> samplesA(N);
			std::vector<Eigen::Vector3f> samplesB(N);
			float noise = 0.01f;

			Eigen::Transform<float, 3, Eigen::Affine> X = Eigen::Quaternionf::UnitRandom() * Eigen::Translation3f(Eigen::Vector3f::Random());

			for (int i = 0; i < N; i++) {
				samplesA[i] = Eigen::Vector3f::Random();
				samplesB[i] = X * samplesA[i] + Eigen::Vector3f::Random() * 0.01;
			}

			float error = 0;
			Eigen::Transform<float, 3, Eigen::Affine> X_ = spooky::utility::calibration::Position::calibrateIdenticalPairTransform_Arun(samplesA, samplesB, &error);

			std::stringstream ss;
			ss << "Correct X = \n" << X.matrix() << std::endl;
			ss << "Calibration Result X = \n" << X_.matrix() << std::endl;
			ss << "Error = " << error << std::endl;
			std::wstring widestr = utf8_decode(ss.str());

			bool close_enough = X.isApprox(X_,noise);
			Assert::AreEqual(close_enough, true, widestr.c_str());
		}

		TEST_METHOD(GetCircleNormal) 
		{
			Eigen::Vector3f A(1,0,0);
			Eigen::Vector3f B(0,1,0);
			Eigen::Vector3f C(0,0,1);

			spooky::utility::Line n = spooky::utility::getCircleNormal(A,B,C);

			Eigen::Vector3f dir = n.direction;
			bool success = dir.cross(Eigen::Vector3f(1,1,1)).isApprox(Eigen::Vector3f(0,0,0),0.0001);
			success = success && n.origin.isApprox(Eigen::Vector3f(1,1,1) * 1/3,0.0001);

			Assert::AreEqual(success, true);
		}

		TEST_METHOD(GetSphere) {
			//Basic sphere
			Eigen::Vector3f A(1, 1, 0);
			Eigen::Vector3f B(0, 1, 1);
			Eigen::Vector3f C(1, 0, 1);
			Eigen::Vector3f D(2, 1, 1);

			spooky::utility::Sphere s = spooky::utility::getSphereFrom4Points(A,B,C,D);

			bool success = s.center.isApprox(Eigen::Vector3f(1,1,1)) && std::fabs(s.r-1) < 0.0001;
			Assert::AreEqual(success, true);

			//Random spheres
			for(int j = 0; j < 100 ; j++){
				std::vector<Eigen::Vector3f> points(4);
				for (int i = 0; i < 4; i++) {
					points[i] = Eigen::Vector3f::Random();
				}
				spooky::utility::Sphere s = spooky::utility::getSphereFrom4Points(points[0], points[1], points[2], points[3]);
				Eigen::Vector4f errors(4);
				for (int i = 0; i < 4; i++) {
					errors(i) = (points[i] - s.center).norm() - s.r;
				}
				success = errors.norm() < 0.0001;


				std::stringstream ss;
				ss << "Points = \n" << points[0] << "," << points[1] << "," << points[2] << "," << points[3] << std::endl;
				ss << "Sphere = \n" << s.center << "," << s.r << std::endl;
				ss << "Errors = " << errors << std::endl;
				std::wstring widestr = utf8_decode(ss.str());

				Assert::AreEqual(success,true, widestr.c_str());
			}

		}

		TEST_METHOD(RANSACSphere) {
			//Number of points
			int N = 50;
			//Noise level
			float noise_level = 0.1;
			//Sphere parameters to test
			Eigen::Vector3f center(1,2,-4);
			float radius = 0.6;
			
			//Construct data
			Eigen::MatrixXf points(3,N);
			for(int i = 0; i < N; i++){
				Eigen::Vector2f random = Eigen::Vector2f::Random();
				Eigen::Vector3f r;
				r[2] = random[1];
				r[0] = std::sqrt(1 - r[2] * r[2]) * std::cos(random[0] * M_PI);
				r[1] = std::sqrt(1 - r[2] * r[2]) * std::sin(random[0] * M_PI);
				points.col(i) = r * radius + center + noise_level * Eigen::Vector3f::Random();
			}
			//Compute ransac
			spooky::utility::Sphere ransac = spooky::utility::sphereRANSAC(points);
			
			//Check correct
			bool success = ransac.center.isApprox(center, noise_level) && std::fabs(ransac.r - radius) < noise_level;

			std::stringstream ss;
			ss << "Sphere = \n" << center << "," << radius << std::endl;
			ss << "Ransac sphere = \n" << ransac.center << "," << ransac.r << std::endl;
			std::wstring widestr = utf8_decode(ss.str());

			Assert::AreEqual(success, true, widestr.c_str());

		}

		TEST_METHOD(QuaternionAverage) {
			Eigen::MatrixXf single_sample(4,1);
			single_sample.col(0) = Eigen::Vector4f(1, 1, 2, 4).normalized();
			Eigen::Quaternionf q_single = spooky::utility::averageQuaternions(single_sample);
			bool single_success = q_single.coeffs().isApprox(single_sample.col(0));
			Assert::AreEqual(single_success, true, L"Failed single sample test");

			//Single example
			Eigen::MatrixXf wQ(4,3);
			wQ.col(0) = Eigen::Vector4f(0, 1, 0.1, 0) * 1;
			wQ.col(1) = Eigen::Vector4f(0, 1, 0, 0.1) * 1;
			wQ.col(2) = Eigen::Vector4f(0.1, 1, 0, 0) * 1;

			Eigen::Quaternionf q = spooky::utility::averageQuaternions(wQ);

			Eigen::Vector4f expected(0, 1, 0, 0);
			bool success = q.coeffs().isApprox(expected, 0.1) ||
				q.coeffs().isApprox(-expected, 0.1);

			std::stringstream ss;
			ss << "Input wQ = \n" << wQ << std::endl;
			ss << "Output q = \n" << q.coeffs().transpose() << std::endl;
			ss << "Output q error = \n" << (q.coeffs() - expected) << std::endl;
			ss << "Output -q error = \n" << (q.coeffs() + expected)<< std::endl;
			std::wstring widestr = utf8_decode(ss.str());

			Assert::AreEqual(success, true, widestr.c_str());

			//Many example
			Eigen::Vector4f q2(1, 2, 3, 4);
			q2.normalize();
			int N = 100;
			float max_noise = 0.1;
			Eigen::MatrixXf wQ2(4, N);
			for (int i = 0; i < N; i++) {
				float noise_amount = (Eigen::Vector2f::Random()[0] + 1) * max_noise / 2;
				wQ2.col(i) = q2 + Eigen::Vector4f::Random() * noise_amount;
				//Weighted quaternions
				wQ2.col(i) = wQ2.col(i).normalized() * (max_noise - noise_amount) / max_noise;
			}
			Eigen::Quaternionf q2_fit = spooky::utility::averageQuaternions(wQ2);
			
			success =
			q2_fit.coeffs().isApprox(q2, max_noise / std::sqrt(N)) ||
			q2_fit.coeffs().isApprox(-q2, max_noise / std::sqrt(N));

			std::stringstream ss2;
			ss2 << "Input wQ2 = \n" << wQ2.transpose() << std::endl;
			ss2 << "Input q2 = \n" << q2.transpose() << std::endl;
			ss2 << "Output q2 = \n" << q2_fit.coeffs().transpose() << std::endl;
			ss2 << "Output q2 error = \n" << (q2_fit.coeffs() - q2) << std::endl;
			ss2 << "Output -q2 error = \n" << (q2_fit.coeffs() + q2) << std::endl;
			std::wstring widestr2 = utf8_decode(ss2.str());

			Assert::AreEqual(success, true, widestr2.c_str());


		}

		Eigen::Matrix4f randomTransform() {
			Eigen::Matrix4f X = Eigen::Matrix4f::Identity();
			X.topLeftCorner(3, 3) = Eigen::Quaternionf::UnitRandom().toRotationMatrix();
			X.topRightCorner(3, 1) = Eigen::Vector3f::Random();
			return X;
		}

		TEST_METHOD(HandshakeCalibration) {
			Eigen::Matrix4f X = randomTransform();
			Eigen::Matrix4f Y = randomTransform();

			float pos_noise = 0.1;

			int N = 20;
			std::vector<Eigen::Matrix4f> samplesA;
			std::vector<Eigen::Matrix4f> samplesB;
			for (int i = 0; i < N; i++) {
				Eigen::Matrix4f A = randomTransform();
				//Get some noise
				Eigen::Matrix4f noise = randomTransform();
				noise.topRightCorner(3, 1) *= pos_noise;
				noise.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();
				//AX=YB => B = Y'AX
				Eigen::Matrix4f B = Y.inverse() * noise * A * X;

				samplesA.push_back(A);
				samplesB.push_back(B);
			}

			float error = 1000;
			auto result = spooky::utility::calibration::Transform::twoSystems_Kronecker_Shah2013(samplesA, samplesB, &error);

			auto Xest = result.first;
			auto Yest = result.second;

			bool success = Xest.matrix().isApprox(X, pos_noise) && Yest.matrix().isApprox(Y, pos_noise);

			std::stringstream ss2;
			ss2 << "X = \n" << X << std::endl;
			ss2 << "Xest = \n" << Xest.matrix() << std::endl;
			ss2 << "Y = \n" << Y << std::endl;
			ss2 << "Yest = \n" << Yest.matrix() << std::endl;
			ss2 << "error = \n" << error << std::endl;
			std::wstring widestr2 = utf8_decode(ss2.str());

			Assert::AreEqual(success, true, widestr2.c_str());
		}

		TEST_METHOD(MultiUseStream) {
			spooky::utility::MultiUseStream<int, std::string> stream;
			std::vector<std::string> names;
			names.push_back("A");
			names.push_back("B");
			names.push_back("C");
			for (auto& name : names) {
				stream.addInitialCounter(name);
			}
			for (int i = 0; i < 100; i++) {
				stream.push_back(i);
			}
			stream.clear(names[0]);
			for (int i = 0; i < 100; i++) {
				stream.push_back(i);
			}
			stream.clear(names[1]);
			for (int i = 0; i < 100; i++) {
				stream.push_back(i);
			}
			stream.clear(names[2]);
			bool success = stream.raw_size() == 200
				&& stream.size("A") == 200
				&& stream.size("B") == 100
				&& stream.size("C") == 0
				&& stream.get("A").size() == 200
				&& stream.get("B").size() == 100
				&& stream.get("C").size() == 0;
				

			std::stringstream ss2;
			ss2 << "size = \n" << stream.raw_size() << std::endl;
			ss2 << "sizes = \n" << stream.size("A") << ", " << stream.size("B") << ", " << stream.size("C") << std::endl;
			ss2 << "get results = \n" << stream.get("A").size() << ", " << stream.get("B").size() << ", " << stream.get("C").size() << std::endl;
			std::wstring widestr2 = utf8_decode(ss2.str());

			Assert::AreEqual(success, true, widestr2.c_str());

		}

		TEST_METHOD(RotationJacobian) {
			Eigen::Vector3f w(M_PI / 2, 0, 0);
			Eigen::Matrix3f sigmaw;
			sigmaw << 0.1, 0, 0,
				0, 0, 0,
				0, 0, 0;
			Eigen::Vector3f p(0, 1, 0);


			std::stringstream ss2;
			ss2 << "w " << w.transpose() << std::endl;
			ss2 << "p = " << p.transpose() << std::endl;
			ss2 << "rodriguesW * p = " << Sophus::SO3f::exp(w).matrix() * p << std::endl;
			ss2 << "sigmaw = \n" << sigmaw << std::endl;
			ss2 << "sigmap = \n" << spooky::utility::getPositionVarianceFromRotation(w, p, sigmaw) << std::endl;
			std::wstring widestr2 = utf8_decode(ss2.str());

			Assert::AreEqual(false, true, widestr2.c_str());
		}

		TEST_METHOD(ArticulatedModelFusionTest) {
			//Set up spooky
			spooky::Core core;
			spooky::Transform3D bonePose = spooky::Transform3D::Identity();
			bonePose.translate(Eigen::Vector3f(1, 0, 0));
			bool modelVelocity = true;
			int dim = modelVelocity ? 6 : 3;
			Eigen::MatrixXf c_var = Eigen::MatrixXf::Identity(dim, dim);
			Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity(dim, dim) * 0.1;
			Eigen::VectorXf c_centre = Eigen::VectorXf::Zero(dim);
			core.addBoneNode(spooky::NodeDescriptor("bone1"), spooky::NodeDescriptor(""), bonePose, c_centre, c_var, process_noise,modelVelocity);
			core.addBoneNode(spooky::NodeDescriptor("bone2"), spooky::NodeDescriptor("bone1"), bonePose, c_centre, c_var, process_noise, modelVelocity);
			core.addBoneNode(spooky::NodeDescriptor("bone3"), spooky::NodeDescriptor("bone2"), bonePose, c_centre, c_var, process_noise, modelVelocity);
			core.setReferenceSystem(spooky::NodeDescriptor("sys1"));
			core.finaliseSetup();

			std::stringstream ss2;
			//Run simulation
			float time_to_run = 1;
			float fps = 10;
			int iterations = fps * time_to_run;
			float deltaT = 1 / fps;
			Eigen::Vector3f pos;
			for (int i = 0; i < iterations; i++) {
				float t = deltaT * i;

				//Simulate measurements
				pos = Eigen::Vector3f(1,std::sqrt(2),0);//std::sin(t) + 2, std::cos(t), 0);
				Eigen::Quaternionf quat = Eigen::Quaternionf::Identity();
				//Create measurement
				spooky::Measurement::Ptr measurement = spooky::Measurement::createPoseMeasurement(pos, quat, Eigen::Matrix<float, 7, 7>::Identity() * 0.1);
				//Get sensor and system info from spooky
				core.setMeasurementSensorInfo(measurement, spooky::SystemDescriptor("sys1"), spooky::SensorID(0));
				//Set metadata and check returns true
				bool valid = measurement->setMetaData(t, 1);
				//TODO: For some reason measurement set metadata doesnt return true
				if (false && !valid) {
					std::cout << "WARNING - Measurement not created correctly - " << __FILE__ << " : " << __LINE__ << std::endl;
				}
				else {
					core.addMeasurement(measurement,spooky::NodeDescriptor("bone3"));
				}
				core.fuse(t);

			}

			spooky::Transform3D pose = core.getNodeGlobalPose(spooky::NodeDescriptor("bone3"));
			float error = (pose.translation() - pos).norm();
			float errorAngle = Eigen::AngleAxisf(pose.rotation()).angle();
			ss2 << "Frame last " << " error = " << error << " m, " <<  errorAngle << " radians" << std::endl;
			ss2 << pose.matrix() << std::endl;
			std::wstring widestr2 = utf8_decode(ss2.str());
			Assert::AreEqual(error < 0.01 && errorAngle < 0.01, true, widestr2.c_str());
			//Check result
		}

		TEST_METHOD(QuaternionToAxisJacobian) {
			Eigen::Quaternionf q0 = Eigen::Quaternionf::Identity();
			Eigen::Quaternionf q1(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f(1, 0, 0)));
			Eigen::Quaternionf q2(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f(0, 0, 1)));


			Eigen::Matrix<float, 3, 4> result0 = spooky::utility::getQuatToAxisJacobian(q0);
			Eigen::Matrix<float, 3, 4> result1 = spooky::utility::getQuatToAxisJacobian(q1);
			Eigen::Matrix<float, 3, 4> result2 = spooky::utility::getQuatToAxisJacobian(q2);

			Eigen::Matrix<float, 3, 4> expected_result0;
			Eigen::Matrix<float, 3, 4> expected_result1;
			Eigen::Matrix<float, 3, 4> expected_result2;
			expected_result0 << 1,0,0,0,
								0,1,0,0,
								0,0,1,0;
			expected_result1 <<
				2.22144,       0,       0, 5.04987,
				0, 2.22144,       0,       0,
				0,       0, 2.22144,       0;
			expected_result2 <<
				2.22144,       0,       0,       0,
				0, 2.22144,       0,       0,
				0,       0, 2.22144, 5.04987;


			//expected_result1 << 

			float error = (result0-expected_result0).norm() + (result1-expected_result1).norm() + (result2-expected_result2).norm();
			std::stringstream ss2;
			ss2 << "error = " << error << std::endl;
			ss2 << "error 0  = \n" << result0 - expected_result0 << std::endl;
			ss2 << "error 1  = \n" << result1 - expected_result1 << std::endl;
			ss2 << "error 2  = \n" << result2 - expected_result2 << std::endl;
			ss2 << "q0 = " << q0.coeffs().transpose() << std::endl;
			ss2 << "result0 = \n" << result0 << std::endl;
			ss2 << "q1 = " << q1.coeffs().transpose() << std::endl;
			ss2 << "result1 = \n" << result1 << std::endl;
			ss2 << "q2 = " << q2.coeffs().transpose() << std::endl;
			ss2 << "result2 = \n" << result2 << std::endl;
			std::wstring widestr2 = utf8_decode(ss2.str());
			Assert::AreEqual(error < 0.001, true, widestr2.c_str());
		}

		TEST_METHOD(NumericalVectorDerivative) {
			auto f = [](const Eigen::Vector3f& x) {
				return Eigen::Vector3f(x[0] * cos(x[1]) * sin(x[2]), x[0] * sin(x[1])*sin(x[2]), x[0] * cos(x[2]));
			};
			Eigen::Matrix3f df_dx1 = spooky::utility::numericalVectorDerivative<float>(f, Eigen::Vector3f(1, 0, M_PI / 2),0.01);
			Eigen::Matrix3f expected_result1;
			expected_result1 << 1, 0, 0,
							   0, 1, 0,
							   0, 0, -1;
			float error1 = (df_dx1 - expected_result1).norm();

			Eigen::Matrix3f df_dx2 = spooky::utility::numericalVectorDerivative<float>(f, Eigen::Vector3f(1, M_PI / 4, M_PI / 4), 0.01);
			Eigen::Matrix3f expected_result2;
			expected_result2 << 0.5, -0.5, 0.5,
								0.5,  0.5, 0.5,
								1 / std::sqrt(2), 0, -1/std::sqrt(2);
			float error2 = (df_dx2 - expected_result2).norm();

			std::stringstream ss2;
			ss2 << "df_dx1 = " << std::endl << df_dx1 << std::endl;
			ss2 << "expected_result1 = " << std::endl << expected_result1 << std::endl;

			ss2 << "df_dx2 = " << std::endl << df_dx2 << std::endl;
			ss2 << "expected_result2 = " << std::endl << expected_result2 << std::endl;
			std::wstring widestr2 = utf8_decode(ss2.str());
			Assert::AreEqual(error1 <  0.01 && error2 < 0.01, true, widestr2.c_str());

		}

		TEST_METHOD(ClosestTwist) {
			{
				Eigen::Vector3f w_m(-0.1, 0, 0);
				Eigen::Vector3f w_t(3, 0, 0);
				auto closest = spooky::utility::twistClosestRepresentation(w_m, w_t);

				std::stringstream ss2;
				ss2 << "w_m = " << std::endl << w_m << std::endl;
				ss2 << "w_t = " << std::endl << w_t << std::endl;
				ss2 << "closest = " << std::endl << closest << std::endl;
				std::wstring widestr2 = utf8_decode(ss2.str());
				Assert::AreEqual((w_m - closest).norm() < 0.001, true, widestr2.c_str());
			}

			{
				Eigen::Vector3f w_m(-3.1, 0, 0);
				Eigen::Vector3f w_t(3, 0, 0);
				auto closest = spooky::utility::twistClosestRepresentation(w_m, w_t);

				std::stringstream ss2;
				ss2 << "w_m = " << std::endl << w_m << std::endl;
				ss2 << "w_t = " << std::endl << w_t << std::endl;
				ss2 << "closest = " << std::endl << closest << std::endl;
				std::wstring widestr2 = utf8_decode(ss2.str());
				Assert::AreEqual((closest - Eigen::Vector3f(M_PI + (M_PI - 3.1),0,0)).norm() < 0.001, true, widestr2.c_str());
			}

			{
				float T = 10;
				float fps = 30;
				int iterations = T * fps;
				Eigen::Vector3f w_t1(2 * M_PI - 0.00001, 0, 0);
				int failcount = 0;
				std::stringstream ss2;

				for (int i = 0; i < iterations; i++) {
					float t = i / 30;
					Eigen::Vector3f w_t2 = w_t1 + Eigen::Vector3f(0, 0.1*i,0);
					Eigen::Vector3f w_t2_renormalized = spooky::utility::toAxisAngle(spooky::utility::rodriguezFormula(w_t2));
					Eigen::Vector3f w_new = spooky::utility::twistClosestRepresentation(w_t2_renormalized, w_t1);
					if ((w_new - w_t2).norm() > 0.1) {
						failcount++;
						ss2 << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
						ss2 << "w_new = " << std::endl << w_new.transpose() << std::endl;
						ss2 << "w_new.norm = " << std::endl << w_new.norm() << std::endl;
						ss2 << "w_t2 = " << std::endl << w_t2.transpose() << std::endl;
						ss2 << "w_t2.norm = " << std::endl << w_t2.norm() << std::endl;
						ss2 << "(w_new - w_t2).norm() = " << std::endl << (w_new - w_t2).norm() << std::endl;
						ss2 << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
					}
					w_t1 = w_new;
				}
				std::wstring widestr2 = utf8_decode(ss2.str());
				Assert::AreEqual(failcount==0,true, widestr2.c_str());

			}
			

		}

		TEST_METHOD(MeasurementTransform) {
			
			spooky::Transform3D T = spooky::Transform3D::Identity();
			T.translate(Eigen::Vector3f::Random());
			T.rotate(Eigen::Quaternionf::UnitRandom());

			Eigen::Vector3f mp = Eigen::Vector3f::Random();
			Eigen::Quaternionf mq = Eigen::Quaternionf::UnitRandom();
			Eigen::Matrix<float, 7, 7> var = Eigen::Matrix<float, 7, 7>::Identity() * 0.1;
			spooky::Measurement::Ptr m = spooky::Measurement::createPoseMeasurement(mp,mq,var);
			
			spooky::Measurement::Ptr m_2 = std::make_unique<spooky::Measurement>(m->transform(T));
			spooky::Measurement::Ptr m_3 = std::make_unique<spooky::Measurement>(m_2->transform(T.inverse()));

			std::stringstream ss2;
			ss2 << m->getData() << std::endl;
			ss2 << m_2->getData() << std::endl;
			ss2 << m_3->getData() << std::endl;
			std::wstring widestr2 = utf8_decode(ss2.str());
			Assert::AreEqual(m_3->compare(m) < 0.01, true, widestr2.c_str());

		}
	};
}