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
        {}
        /**
        *
        **/
		cv::Matx44f Calibrator::getRigidBodyTransformation() const
        {
            if(m_bCalibrationAvailable)
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
        void Calibrator::calibrate(
			std::size_t const uiCalibrationPointCount,
			std::size_t const uiCalibrationRotationCount)
        {
			std::vector<std::pair<cv::Vec3f, cv::Vec3f>> m_vCorrespondences;
			m_vCorrespondences.reserve(uiCalibrationPointCount);

            // Get the point correspondences.
			for(std::size_t i(0); i<uiCalibrationPointCount; ++i)
            {
                m_pArm->moveTo(getRandomArmPosition());
                // Get the final position the arm haltet at and store it.
				m_vCorrespondences.push_back(std::make_pair(m_pArm->getPosition(), getAveragedCalibrationObjectVisionPosition(uiCalibrationRotationCount)));
            }

            // Estimate the rigid body transformation.
			m_matCurrentRigidBodyTransformation = estimateRigidBodyTransformation(m_vCorrespondences);
            m_bCalibrationAvailable = true;
        }
        /**
        *
        **/
        cv::Vec3f Calibrator::getAveragedCalibrationObjectVisionPosition(
			std::size_t const uiCalibrationRotationCount) const
        {
            cv::Vec3f v3fVisionPosition;

            // Multiple hand rotations at same position to prevent occultation.
			for(std::size_t j(0); j<uiCalibrationRotationCount; ++j)
            {
                // Rotate the arm around the point.
                auto const v3fArmRotation(getRandomArmRotation());
                m_pArm->rotateTo(v3fArmRotation);

                // Get current calibration object vision position.
                v3fVisionPosition += recognition::getCalibrationObjectVisionPosition(m_pVision->getRgb());
            }
			// Average the positions.
			v3fVisionPosition *= (1.0 / static_cast<float>(uiCalibrationRotationCount));

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
			auto const itCorrEnd(vv2v3fCorrespondences.cend());
			for(auto itCorr(vv2v3fCorrespondences.begin()); itCorr != itCorrEnd; ++itCorr)
			{
				v3CenterLeft += itCorr->first;
				v3CenterRight += itCorr->second;
			}
			v3CenterLeft *= (1.0 / static_cast<float>(vv2v3fCorrespondences.size()));
			v3CenterRight *= (1.0 / static_cast<float>(vv2v3fCorrespondences.size()));

			// Create a vector with the centered correspondences.
			std::vector<std::pair<cv::Vec3f, cv::Vec3f>> vv2v3fCenteredCorrespondences;
			for(auto itCorr(vv2v3fCorrespondences.begin()); itCorr != itCorrEnd; ++itCorr)
			{
				vv2v3fCenteredCorrespondences.push_back(std::make_pair(itCorr->first-v3CenterLeft, itCorr->second-v3CenterRight));
			}

			// Fill the covariance matrix.
			cv::Matx33f H(cv::Matx33f::zeros());
			for(size_t i(0); i<3; ++i)
			{
				for(size_t j(0); j<3; ++j)
				{
					auto const itCentCorrEnd(vv2v3fCenteredCorrespondences.cend());
					for(auto itCorr(vv2v3fCenteredCorrespondences.begin()); itCorr != itCentCorrEnd; ++itCorr)
					{
						H(i, j) += (*itCorr).second(i) * (*itCorr).first(j);
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
				static_cast<float>(rng.uniform(0.2, 0.7)),
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