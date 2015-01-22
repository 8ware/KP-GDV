#include <kinjo/calibration/Calibration.hpp>

#include <kinjo/recognition/Recognizer.hpp>

#include <vector>
#include <iostream>
#include <map>

namespace kinjo
{
    namespace calibration
    {
        /**
         *
         **/
        Calibrator::Calibrator(
            arm::Arm * const pArm, 
			vision::Vision * const pVision,
			recognition::Recognizer const * const pRecognizer,
			CalibrationPointGenerator * const pCalibrationPointGenerator) :
            m_matCurrentRigidBodyTransformation(cv::Matx44f::zeros()),
            m_bCalibrationAvailable(false),
            m_pArm(pArm),
            m_pVision(pVision),
			m_pRecognizer(pRecognizer),
			m_pCalibrationPointGenerator(pCalibrationPointGenerator)
		{}
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
			std::cout << "Calib:: Begin calibration..." << std::endl;

			std::vector<std::pair<cv::Vec3f, cv::Vec3f>> m_vCorrespondences;
			m_vCorrespondences.reserve(uiCalibrationPointCount);

            // Get the point correspondences.
			for(std::size_t uiCalibrationPoint(0); uiCalibrationPoint<uiCalibrationPointCount; ++uiCalibrationPoint)
			{
				std::cout << "Calib:: uiCalibrationPoint: " << uiCalibrationPoint << std::endl;

				cv::Vec3f v3fAveragedVisionPosition(0.0f, 0.0f, 0.0f);
				do
				{
					auto const v3fArmDesiredPosition(m_pCalibrationPointGenerator->getNextCalibrationPoint());
					std::cout << "Calib:: v3fArmDesiredPosition: " << v3fArmDesiredPosition << std::endl;
					m_pArm->moveTo(v3fArmDesiredPosition);
					std::cout << "Calib:: reached psotition: " << m_pArm->getPosition() << std::endl;
					// \TODO: Maybe we should check if the position was approximately reached and retry else.

					v3fAveragedVisionPosition = getAveragedCalibrationObjectVisionPosition(
						uiCalibrationRotationCount,
						uiRecognitionAttemptCount);
					std::cout << "Calib:: v3fAveragedVisionPosition: " << v3fAveragedVisionPosition << std::endl;
				}
				// If the object was not recognized, retry.
				while(v3fAveragedVisionPosition[2] == 0.0f);

				// Get the final position the arm haltet at and store it with the estimated calibration object vision position.
				m_vCorrespondences.push_back(std::make_pair(
					m_pArm->getPosition(),
					v3fAveragedVisionPosition));
            }

            // Estimate the rigid body transformation.
			m_matCurrentRigidBodyTransformation = estimateRigidBodyTransformation(m_vCorrespondences);
			m_bCalibrationAvailable = true;

			std::cout << "Calib:: Finished calibration..." << std::endl;
        }
        /**
         *
         **/
        cv::Vec3f Calibrator::getAveragedCalibrationObjectVisionPosition(
			std::size_t const uiCalibrationRotationCount,
			std::size_t const uiRecognitionAttemptCount) const
		{
			std::cout << "Calib:: [+] getAveragedCalibrationObjectVisionPosition" << std::endl;

			std::vector<cv::Vec3f> vv3fVisionPositions;

            // Multiple hand rotations at same position to prevent occultation.
			for(std::size_t uiCalibrationRotation(0); uiCalibrationRotation<uiCalibrationRotationCount; ++uiCalibrationRotation)
            {
                // Rotate the hand.
				auto const fPi(std::atan2(0, -1));
				// +1 to not reach the initial position again.
				auto const fRotationAngle((static_cast<float>(2.0*fPi)/static_cast<float>(uiCalibrationRotationCount+1)));
				m_pArm->rotateHandBy(fRotationAngle);

				// Multiple recognition attempts at the same position/rotation.
				for(std::size_t uiRecognitionAttempt(0); uiRecognitionAttempt<uiRecognitionAttemptCount; ++uiRecognitionAttempt)
				{
					std::cout << "Calib:: uiRecognitionAttempt: " << uiRecognitionAttempt << std::endl;
					// We need new images.
					m_pVision->updateImages(true);
					// Get current calibration object vision position.
					auto const v2iPointPx(
						m_pRecognizer->estimateCalibrationObjectImagePointPx(
							m_pVision->getRgb()));

					// If this is a valid point.
					if(v2iPointPx != cv::Point(0,0))
					{
						std::cout << "Calib:: Found at image point: (" << v2iPointPx.x << "," << v2iPointPx.y << ")" << std::endl;
						// Accumulate the positions for averaging.
						cv::Vec3f const v3fEstimatedVisionPosition(m_pVision->estimateVisionPositionFromImagePointPx(
							v2iPointPx));
						vv3fVisionPositions.push_back(v3fEstimatedVisionPosition);
						std::cout << "Calib:: v3fEstimatedVisionPosition: " << v3fEstimatedVisionPosition << std::endl;
					}
				}
            }

			if(vv3fVisionPositions.size()==0)
			{
				// If the object could not be recognized (maybe too far away or out of image, return a zero vector)
				std::cout << "Calib:: No calibration object found..." << std::endl;
			}
			else
			{
				// Filter out outliers.
				filterPointList(
					vv3fVisionPositions,
					100.0f);
			}

			std::cout << "Calib:: [-] getAveragedCalibrationObjectVisionPosition" << std::endl;

			return average(vv3fVisionPositions);
		}
		/**
		 * Filter by iteratively removing the point that is outside the limit and is farthest away.
		 **/
		std::vector<cv::Vec3f> Calibrator::filterPointList(
			std::vector<cv::Vec3f> vv3fVisionPositions,
			float fInlierDistanceMm)
		{
			std::cout << "Calib:: #points before filtering: " << vv3fVisionPositions.size() << std::endl;

			while(vv3fVisionPositions.size()>1)
			{
				// Calculate the average.
				cv::Vec3f const v3fAverage(average(vv3fVisionPositions));

				// std::map automatically sorts by the key.
				std::map<float, cv::Vec3f> mfv3fPositionDistances;
				// Fill a map with the points sorted by distance.
				for(auto const & v3fVisionPosition : vv3fVisionPositions)
				{
					mfv3fPositionDistances[static_cast<float>(norm(v3fVisionPosition-v3fAverage))] = v3fVisionPosition;
				}

				// Remove the last point if it is too far away ...
				auto const itLastPoint(std::prev(mfv3fPositionDistances.end()));
				if(itLastPoint->first > fInlierDistanceMm)
				{
					mfv3fPositionDistances.erase(itLastPoint);
				}
				// ... or finish if no more points are outside.
				else
				{
					break;
				}

				// Refill the list of points.
				vv3fVisionPositions.clear();
				for(auto const & pairf3fVisionPosition : mfv3fPositionDistances)
				{
					vv3fVisionPositions.push_back(pairf3fVisionPosition.second);
				}
			}

			std::cout << "Calib:: #points after filtering: " << vv3fVisionPositions.size() << std::endl;

			return vv3fVisionPositions;
		}
		/**
		 *
		 **/
		cv::Vec3f Calibrator::average(
			std::vector<cv::Vec3f> const & vv3fVisionPositions)
		{
			cv::Vec3f v3fSumVisionPositions(0.0f, 0.0f, 0.0f);
			for(auto const & v3fVisionPosition : vv3fVisionPositions)
			{
				v3fSumVisionPositions += v3fVisionPosition;
			}

			cv::Vec3f v3fAveragedVisionPosition(v3fSumVisionPositions);
			if(vv3fVisionPositions.size()>0)
			{
				// Average the positions.
				v3fAveragedVisionPosition *= (1.0f / static_cast<float>(vv3fVisionPositions.size()));
			}

			return v3fAveragedVisionPosition;
		}
        /**
         * Implements "Least-Squares Rigid Motion Using SVD"
		 * See: http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
         **/
		cv::Matx44f Calibrator::estimateRigidBodyTransformation(
			std::vector<std::pair<cv::Vec3f, cv::Vec3f>> const & vv2v3fCorrespondences)
		{
			std::cout << "Calib:: [+] estimateRigidBodyTransformation" << std::endl;

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
			cv::Matx33f Vt(reinterpret_cast<float*>(svd.vt.data));
			
			// Transpose Vt.
			cv::Matx33f V(Vt.t());
			// Transpose U.
			cv::Matx33f const Ut(U.t());

			// Correct it so that it is a rotation.
			cv::Matx33f DCorrect(cv::Matx33f::eye());
			DCorrect(2, 2) = static_cast<float>(cv::determinant(V*Ut));

			// Compute the roation matrix.
			cv::Matx33f const R(V * DCorrect * Ut);
			// Compute the translation vector.
			cv::Vec3f const t(v3CenterLeft - R * v3CenterRight);

			std::cout << "Calib:: [-] estimateRigidBodyTransformation" << std::endl;

			return cv::Matx44f{
				R(0, 0), R(0, 1), R(0, 2), t(0),
				R(1, 0), R(1, 1), R(1, 2), t(1),
				R(2, 0), R(2, 1), R(2, 2), t(2),
				0.0f, 0.0f, 0.0f, 1.0f};
        }
    }
}
