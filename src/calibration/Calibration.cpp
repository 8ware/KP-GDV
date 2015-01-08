#include <kinjo/calibration/Calibration.hpp>

#include <kinjo/recognition/Recognition.hpp>

#include <vector>
#include <iostream>

namespace kinjo
{
    namespace calibration
    {
        /**
         *
         **/
        Calibrator::Calibrator(
            arm::Arm * const pArm, 
            vision::Vision * const pVision) :
			m_fPi(std::atan2(0, -1)),
            m_matCurrentRigidBodyTransformation(),
            m_bCalibrationAvailable(false),
            m_pArm(pArm),
            m_pVision(pVision)
		{
		}
		/**
		 *
		 **/
		bool Calibrator::getIsValidTransformationAvailable() const
		{
			return m_bCalibrationAvailable;
		}
        /**
         *
         **/
		cv::Matx44f Calibrator::getRigidBodyTransformation() const
        {
			if(getIsValidTransformationAvailable())
            {
                return m_matCurrentRigidBodyTransformation;
            }
            else
            {
                throw std::logic_error("getRigidBodyTransformation() is not allowed to be called before calibrate()!");
            }
		}
		/**
		*
		**/
		void Calibrator::calibrateAsync(
			std::size_t const uiCalibrationPointCount,
			std::size_t const uiCalibrationRotationCount,
			std::size_t const uiRecognitionAttemptCount)
		{
			m_Thread = std::thread(
				&Calibrator::calibrationThreadMain,
				this, 
				uiCalibrationPointCount,
				uiCalibrationRotationCount,
				uiRecognitionAttemptCount);
			m_Thread.detach();
		}
        /**
         *
         **/
        void Calibrator::calibrationThreadMain(
			std::size_t const uiCalibrationPointCount,
			std::size_t const uiCalibrationRotationCount,
			std::size_t const uiRecognitionAttemptCount)
		{
			std::cout << "Begin calibration..." << std::endl;

			std::vector<std::pair<cv::Vec3f, cv::Vec3f>> m_vCorrespondences;
			m_vCorrespondences.reserve(uiCalibrationPointCount);

            // Get the point correspondences.
			for(std::size_t uiCalibrationPoint(0); uiCalibrationPoint<uiCalibrationPointCount; ++uiCalibrationPoint)
			{
				std::cout << "uiCalibrationPoint: " << uiCalibrationPoint << std::endl;

				cv::Vec3f v3fAveragedPosition(0.0f, 0.0f, 0.0f);
				do
				{
					auto const v3fArmDesiredPosition(getArmCalibrationPosition(uiCalibrationPointCount));
					m_pArm->moveTo(v3fArmDesiredPosition);
					std::cout << "v3fArmDesiredPosition: " << v3fArmDesiredPosition << std::endl;
					// Get the final position the arm haltet at and store it with the estimated calibration object position.

					v3fAveragedPosition = getAveragedCalibrationObjectVisionPosition(
						uiCalibrationRotationCount,
						uiRecognitionAttemptCount);
					std::cout << "v3fAveragedPosition: " << v3fAveragedPosition << std::endl;
				}
				// If the object was not recognized, retry.
				while(v3fAveragedPosition == cv::Vec3f(0.0f, 0.0f, 0.0f));
                
				m_vCorrespondences.push_back(std::make_pair(
					m_pArm->getPosition(),
					v3fAveragedPosition));
            }

            // Estimate the rigid body transformation.
			m_matCurrentRigidBodyTransformation = estimateRigidBodyTransformation(m_vCorrespondences);
			m_bCalibrationAvailable = true;

			std::cout << "Finished calibration..." << std::endl;
        }
        /**
         *
         **/
        cv::Vec3f Calibrator::getAveragedCalibrationObjectVisionPosition(
			std::size_t const uiCalibrationRotationCount,
			std::size_t const uiRecognitionAttemptCount) const
		{
			std::cout << "[+] getAveragedCalibrationObjectVisionPosition" << std::endl;

			cv::Vec3f v3fSummedVisionPosition(0.0f, 0.0f, 0.0f);
			std::size_t uiValidPositions(0);

            // Multiple hand rotations at same position to prevent occultation.
			for(std::size_t uiCalibrationRotation(0); uiCalibrationRotation<uiCalibrationRotationCount; ++uiCalibrationRotation)
            {
                // Rotate the arm around the point.
				auto const fRotationAngle(((2.0*m_fPi)/static_cast<float>(uiCalibrationRotationCount))*static_cast<float>(uiCalibrationRotation));
				auto const v3fRotation(m_pArm->getRotation());
				m_pArm->rotateTo(cv::Vec3f(v3fRotation[0], v3fRotation[1], fRotationAngle));

				// Multiple recognition attempts at the same position/rotation.
				for(std::size_t uiRecognitionAttempt(0); uiRecognitionAttempt<uiRecognitionAttemptCount; ++uiRecognitionAttempt)
				{
					std::cout << "uiRecognitionAttempt: " << uiRecognitionAttempt << std::endl;
					// We need new images.
					m_pVision->updateImages(true);
					// Get current calibration object vision position.
					std::pair<cv::Vec2f, float> const calib(
						recognition::getCalibrationObjectVisionPositionPx(
							m_pVision->getRgb()));

					// If this is a valid point.
					if(calib.second != 0.0f)
					{
						std::cout << "Found at: (" << calib.first[0u] << "," << calib.first[1u] << ")" << std::endl;
						// Accumulate the positions for averaging.
						cv::Vec3f const v3fEstimatedVisionPosition(m_pVision->estimatePositionFromImagePointPx(
							cv::Point(
								static_cast<int>(calib.first[0u]),
								static_cast<int>(calib.first[1u]))));
						v3fSummedVisionPosition += v3fEstimatedVisionPosition;
						++uiValidPositions;
						std::cout << "v3fEstimatedVisionPosition: " << v3fEstimatedVisionPosition << std::endl;
					}
				}
            }

			cv::Vec3f v3fAveragedVisionPosition(v3fSummedVisionPosition);

			if(uiValidPositions>0)
			{
				// Average the positions.
				v3fAveragedVisionPosition *= (1.0f / static_cast<float>(uiValidPositions));
			}
			else
			{
				// If the object could not be recognized (maybe too far away or out of image, return a zero vector)
				std::cout << "No calibration object found..." << std::endl;
			}

			std::cout << "[-] getAveragedCalibrationObjectVisionPosition" << std::endl;

			return v3fAveragedVisionPosition;
        }
        /**
         * Implements "Least-Squares Rigid Motion Using SVD"
		 * See: http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
         **/
		cv::Matx44f Calibrator::estimateRigidBodyTransformation(
			std::vector<std::pair<cv::Vec3f, cv::Vec3f>> const & vv2v3fCorrespondences)
		{
			std::cout << "[+] estimateRigidBodyTransformation" << std::endl;

			std::size_t const uiCorrespondenceCount(vv2v3fCorrespondences.size());
			if(uiCorrespondenceCount < 3)
			{
				throw std::runtime_error("The number of correspondences given to estimateRigidBodyTransformation is required to be >= 3!");
			}

			// Compute centers.
			cv::Vec3f v3CenterLeft;
			cv::Vec3f v3CenterRight;
			for(auto && corr : vv2v3fCorrespondences)
			{
				v3CenterLeft += corr.first;
				v3CenterRight += corr.second;
			}
			v3CenterLeft *= (1.0f / static_cast<float>(uiCorrespondenceCount));
			v3CenterRight *= (1.0f / static_cast<float>(uiCorrespondenceCount));

			// Create a vector with the centered correspondences.
			std::vector<std::pair<cv::Vec3f, cv::Vec3f>> vv2v3fCenteredCorrespondences;
			for(auto && corr : vv2v3fCorrespondences)
			{
				vv2v3fCenteredCorrespondences.push_back(std::make_pair(
					corr.first-v3CenterLeft,
					corr.second-v3CenterRight));
			}

			// Fill the covariance matrix.
			cv::Matx33f H(cv::Matx33f::zeros());
			for(std::size_t i(0); i<3; ++i)
			{
				for(std::size_t j(0); j<3; ++j)
				{
					for(auto && corr : vv2v3fCenteredCorrespondences)
					{
						H(i, j) += corr.second(i) * corr.first(j);
					}
				}
			}

			// Compute the SVD.
			cv::SVD svd(H, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
			cv::Matx33f U(reinterpret_cast<float*>(svd.u.data));
			cv::Matx33f S(reinterpret_cast<float*>(svd.w.data));
			cv::Matx33f V(reinterpret_cast<float*>(svd.vt.data));

			// Transpose U.
			cv::Matx33f const Ut(U.t());

			// Correct it so that it is a rotation.
			cv::Matx33f DCorrect(cv::Matx33f::eye());
			DCorrect(2, 2) = static_cast<float>(cv::determinant(V*Ut));

			// Compute the roation matrix.
			cv::Matx33f const R(V * DCorrect * Ut);
			// Compute the translation vector.
			cv::Vec3f const t(v3CenterLeft - R * v3CenterRight);

			std::cout << "[-] estimateRigidBodyTransformation" << std::endl;

			return cv::Matx44f{
				R(0, 0), R(0, 1), R(0, 2), t(0),
				R(1, 0), R(1, 1), R(1, 2), t(1),
				R(2, 0), R(2, 1), R(2, 2), t(2),
				0.0f, 0.0f, 0.0f, 1.0f};
        }
        /**
         *
         **/
        cv::Vec3f Calibrator::getArmCalibrationPosition(std::size_t const i) const
		{
			std::cout << "[+] getArmCalibrationPosition" << std::endl;

			// Clock-wise rotation angle when looking from the top. 
			auto const fTheta(m_Rng.uniform(0.0, 2.0*m_fPi));
			// Because we can not come too close to the arm base we have to keep a minimum distance.
			auto const fDist(m_Rng.uniform(300.0, 450.0));

			std::cout << "[-] getArmCalibrationPosition" << std::endl;

			return cv::Vec3f(
				static_cast<float>(fDist * std::cos(fTheta)),
				static_cast<float>(fDist * std::sin(fTheta)),
				static_cast<float>(m_Rng.uniform(50.0, 300.0)));
        }
    }
}