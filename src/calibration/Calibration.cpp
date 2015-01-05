#include <kinjo/calibration/Calibration.hpp>

#include <kinjo/recognition/Recognition.hpp>

#include <vector>

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
			std::vector<std::pair<cv::Vec3f, cv::Vec3f>> m_vCorrespondences;
			m_vCorrespondences.reserve(uiCalibrationPointCount);

            // Get the point correspondences.
			for(std::size_t i(0); i<uiCalibrationPointCount; ++i)
            {
                m_pArm->moveTo(getRandomArmPosition());
                // Get the final position the arm haltet at and store it with the estimated calibration object position.
				m_vCorrespondences.push_back(std::make_pair(
					m_pArm->getPosition(), 
					getAveragedCalibrationObjectVisionPosition(
						uiCalibrationRotationCount,
						uiRecognitionAttemptCount)));
            }

            // Estimate the rigid body transformation.
			m_matCurrentRigidBodyTransformation = estimateRigidBodyTransformation(m_vCorrespondences);
            m_bCalibrationAvailable = true;
        }
        /**
         *
         **/
        cv::Vec3f Calibrator::getAveragedCalibrationObjectVisionPosition(
			std::size_t const uiCalibrationRotationCount,
			std::size_t const uiRecognitionAttemptCount) const
        {
            cv::Vec3f v3fVisionPosition(0.0f, 0.0f, 0.0f);
			std::size_t uiValidPositions(0);

            // Multiple hand rotations at same position to prevent occultation.
			for(std::size_t j(0); j<uiCalibrationRotationCount; ++j)
            {
                // Rotate the arm around the point.
				cv::Vec3f const v3fArmRotation(getRandomArmRotation());
                m_pArm->rotateTo(v3fArmRotation);

				// Multiple recognition attempts at the same position/rotation.
				for(std::size_t uiTry(0); uiTry<uiRecognitionAttemptCount; ++uiTry)
				{
					// We need new images.
					m_pVision->updateImages(true);
					// Get current calibration object vision position.
					std::pair<cv::Vec2f, float> const calib(
						recognition::getCalibrationObjectVisionPositionPx(
							m_pVision->getRgb()));

					// If this is a valid point.
					if(calib.second != 0.0f)
					{
						// Accumulate the positions for averaging.
						v3fVisionPosition += m_pVision->estimatePositionFromImagePointPx(
							cv::Point(
								static_cast<int>(calib.first[0u]),
								static_cast<int>(calib.first[1u])));
						++uiValidPositions;
					}
				}
            }
			// Average the positions.
			v3fVisionPosition *= (1.0 / static_cast<float>(uiValidPositions));

            return v3fVisionPosition;
        }
        /**
         * Implements "Least-Squares Rigid Motion Using SVD"
		 * See: http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
         **/
		cv::Matx44f Calibrator::estimateRigidBodyTransformation(
			std::vector<std::pair<cv::Vec3f, cv::Vec3f>> const & vv2v3fCorrespondences)
        {
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
			v3CenterLeft *= (1.0 / static_cast<float>(vv2v3fCorrespondences.size()));
			v3CenterRight *= (1.0 / static_cast<float>(vv2v3fCorrespondences.size()));

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
			cv::Matx33f U, S, V;
			cv::SVD::compute(H, U, S, V);

			// Transpose U.
			cv::Matx33f const Ut(U.t());

			// Correct it so that it is a rotation.
			cv::Matx33f DCorrect(cv::Matx33f::eye());
			DCorrect(2, 2) = static_cast<float>(cv::determinant(V*Ut));

			// Compute the roation matrix.
			cv::Matx33f const R(V * DCorrect * Ut);
			// Compute the translation vector.
			cv::Vec3f const t(v3CenterLeft - R * v3CenterRight);

			return cv::Matx44f{
				R(0, 0), R(0, 1), R(0, 2), t(0),
				R(1, 0), R(1, 1), R(1, 2), t(1),
				R(2, 0), R(2, 1), R(2, 2), t(2),
				0.0f, 0.0f, 0.0f, 1.0f};
        }
        /**
         *
         **/
        cv::Vec3f Calibrator::getRandomArmPosition() const
        {
            // TODO: Implement better algorithm!
			cv::RNG rng;
			return cv::Vec3f(
				static_cast<float>(rng.uniform(0.2, 0.7)),
				static_cast<float>(rng.uniform(-0.2, -0.7)),
				static_cast<float>(rng.uniform(0.2, 0.7)));
        }
        /**
         *
         **/
        cv::Vec3f Calibrator::getRandomArmRotation() const
        {
            // TODO: Implement!
            return cv::Vec3f(0.5, 0.5, 0.5);
        }
    }
}