#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <vector>

int main(int /*argc*/, char* /*argv*/[])
{
	cv::RNG m_Rng;

	// Rotation axis.
	cv::Vec3f const v3fRotationAxis(
		static_cast<float>(m_Rng.uniform(-1.0, 1.0)),
		static_cast<float>(m_Rng.uniform(-1.0, 1.0)),
		static_cast<float>(m_Rng.uniform(-1.0, 1.0)));
	std::cout << "v3fRotationAxis: " << v3fRotationAxis << std::endl;

	// Rotation angle.
	float const v3fRotationAngle(
		static_cast<float>(m_Rng.uniform(-3.0, 3.0)));

	std::cout << "v3fRotationAngle: " << v3fRotationAngle << std::endl;

	cv::Vec3f const v3fRotationAxisAngle(
		v3fRotationAxis*v3fRotationAngle);
	std::cout << "v3fRotationAxisAngle: " << v3fRotationAxisAngle << std::endl;

	// Rotation matrix.
	cv::Matx33f mat33fRotation;
	cv::Rodrigues(v3fRotationAxisAngle, mat33fRotation);
	std::cout << "mat33fRotation: " << mat33fRotation << std::endl;
	mat33fRotation = mat33fRotation.inv();
	std::cout << "inv mat33fRotation: " << mat33fRotation << std::endl;

	// Translation.
	cv::Vec3f const v3fTranslation(
		static_cast<float>(m_Rng.uniform(-10.0, 10.0)),
		static_cast<float>(m_Rng.uniform(-10.0, 10.0)),
		static_cast<float>(m_Rng.uniform(-10.0, 10.0)));
	std::cout << "v3fTranslation: " << v3fTranslation << std::endl;

	std::vector<std::pair<cv::Vec3f, cv::Vec3f>> vv2v3fCorrespondences;

	for(std::size_t i(0); i<10; ++i)
	{
		cv::Vec3f const v3fSrc(
			static_cast<float>(m_Rng.uniform(50.0, 300.0)),
			static_cast<float>(m_Rng.uniform(50.0, 300.0)),
			static_cast<float>(m_Rng.uniform(50.0, 300.0)));
		std::cout << "v3fSrc: " << v3fSrc << std::endl;

		cv::Vec3f const v3fDst(mat33fRotation * v3fSrc - v3fTranslation);
		std::cout << "v3fDst: " << v3fDst << std::endl;

		vv2v3fCorrespondences.push_back(std::make_pair(v3fDst, v3fSrc));
	}

	cv::Matx44f const rbt(kinjo::calibration::Calibrator::estimateRigidBodyTransformation(
		vv2v3fCorrespondences));
	std::cout << "rbt: " << rbt << std::endl;

	for(std::size_t i(0); i<10; ++i)
	{
		std::cout << "Res: " << (rbt * vv2v3fCorrespondences[i].second) << std::endl;
	}

	std::cout << std::endl;
}