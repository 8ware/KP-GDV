#include <kinjo/calibration/AutomaticCalibrator.hpp>

#include <kinjo/recognition/Recognizer.hpp>

#include <easylogging++.h>

#include <vector>
#include <iostream>
#include <map>


static el::Logger* LOG = el::Loggers::getLogger("Calib");

namespace kinjo {
namespace calibration {

	/**
	 *
	 **/
	AutomaticCalibrator::AutomaticCalibrator(
		arm::Arm * const pArm, 
		vision::Vision * const pVision,
		recognition::Recognizer const * const pRecognizer,
		CalibrationPointGenerator * const pCalibrationPointGenerator,
		std::size_t const & uiCalibrationPointCount,
		std::size_t const & uiCalibrationRotationCount,
		std::size_t const & uiMinimumValidPositionsAfterFilteringPercent,
		float const & fMaximumFilterEuclideanDistancePointToAverage) :
			m_matCurrentRigidBodyTransformation(cv::Matx44f::zeros()),
			m_bCalibrationAvailable(false),
			m_pArm(pArm),
			m_pVision(pVision),
			m_pRecognizer(pRecognizer),
			m_pCalibrationPointGenerator(pCalibrationPointGenerator),
			m_uiCalibrationPointCount(uiCalibrationPointCount),
			m_uiCalibrationRotationCount(uiCalibrationRotationCount),
			m_uiMinimumValidPositionsAfterFilteringPercent(uiMinimumValidPositionsAfterFilteringPercent),
			m_fMaximumFilterEuclideanDistancePointToAverage(fMaximumFilterEuclideanDistancePointToAverage)
	{}
	/**
	 *
	 **/
	bool AutomaticCalibrator::getIsValidTransformationAvailable() const
	{
		return m_bCalibrationAvailable;
	}
	/**
	 *
	 **/
	cv::Matx44f AutomaticCalibrator::getRigidBodyTransformation() const
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
	void AutomaticCalibrator::calibrateAsync()
	{
		m_Thread = std::thread(
			&AutomaticCalibrator::calibrationThreadMain,
			this);
		m_Thread.detach();
	}
	/**
	 *
	 **/
	void AutomaticCalibrator::calibrationThreadMain()
	{
		LOG->info("Begin calibration...");

		std::vector<std::pair<cv::Vec3f, cv::Vec3f>> m_vCorrespondences;
		m_vCorrespondences.reserve(m_uiCalibrationPointCount);

		// Get the point correspondences.
		for(std::size_t uiCalibrationPoint(0); uiCalibrationPoint<m_uiCalibrationPointCount; ++uiCalibrationPoint)
		{
			LOG->debug("uiCalibrationPoint: %v", uiCalibrationPoint);

			cv::Vec3f v3fAveragedVisionPosition(0.0f, 0.0f, 0.0f);
			do
			{
				auto const v3fArmDesiredPosition(m_pCalibrationPointGenerator->getNextCalibrationPoint());
				LOG->debug("v3fArmDesiredPosition: %v", v3fArmDesiredPosition);
				m_pArm->moveTo(v3fArmDesiredPosition);
				LOG->debug("reached position: %v", m_pArm->getPosition());
				// \TODO: Maybe we should check if the position was approximately reached and retry else.

				v3fAveragedVisionPosition = getAveragedCalibrationObjectVisionPosition();
				LOG->debug("v3fAveragedVisionPosition: %v", v3fAveragedVisionPosition);
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

		LOG->info("Finished calibration...");
	}
	/**
	 *
	 **/
	cv::Vec3f AutomaticCalibrator::getAveragedCalibrationObjectVisionPosition() const
	{
		LOG->debug("[+] getAveragedCalibrationObjectVisionPosition");

		std::vector<cv::Vec3f> vv3fVisionPositions;

		std::size_t const uiRecognitionAttempts(m_pRecognizer->getRecommendedRecognitionAttempCount());

		// Multiple hand rotations at same position to prevent occultation.
		for(std::size_t uiCalibrationRotation(0); uiCalibrationRotation<m_uiCalibrationRotationCount; ++uiCalibrationRotation)
		{
			// Multiple recognition attempts at the same position/rotation.
			for(std::size_t uiRecognitionAttempt(0); uiRecognitionAttempt<uiRecognitionAttempts; ++uiRecognitionAttempt)
			{
				LOG->debug("uiRecognitionAttempt: %v", uiRecognitionAttempt);
				// We need new images.
				m_pVision->updateImages(true);
				// Get current calibration object vision position.
				auto const v2iImagePointPx(
					m_pRecognizer->estimateCalibrationObjectImagePointPx(
						m_pVision->getRgb()));

				// If this is a valid point.
				if(v2iImagePointPx != cv::Point(0,0))
				{
					LOG->debug("v2iImagePointPx: (%v,%v)", v2iImagePointPx.x, v2iImagePointPx.y);
					cv::Vec3f const v3fEstimatedVisionPosition(m_pVision->estimateVisionPositionFromImagePointPx(v2iImagePointPx));
					if(v3fEstimatedVisionPosition[2] != 0.0f)
					{
						vv3fVisionPositions.push_back(v3fEstimatedVisionPosition);
						LOG->debug("v3fEstimatedVisionPosition: %v", v3fEstimatedVisionPosition);
					}
				}
			}

			if(uiCalibrationRotation<(m_uiCalibrationRotationCount-1))
			{
				// Rotate the hand.
				auto const fPi(std::atan2(0, -1));
				// +1 to not reach the initial position again.
				auto const fRotationAngle((static_cast<float>(2.0*fPi)/static_cast<float>(m_uiCalibrationRotationCount+1)));
				m_pArm->rotateHandBy(fRotationAngle);
			}
		}

		// Filter out outliers.
		filterPointList(
			vv3fVisionPositions,
			m_fMaximumFilterEuclideanDistancePointToAverage);
			
		std::size_t const uiMaxPositions(m_uiCalibrationRotationCount * uiRecognitionAttempts);
		std::size_t const uiMinimumValidPositionsAfterFilteringCount(
			static_cast<std::size_t>(static_cast<float>(uiMaxPositions) * static_cast<float>(m_uiMinimumValidPositionsAfterFilteringPercent)*0.01f));
		// If the object could not be recognized (maybe too far away or out of image).
		if(vv3fVisionPositions.size() < uiMinimumValidPositionsAfterFilteringCount)
		{
			LOG->warn("No valid calibration object found...");
			// Delete all points from the list to return a zero vector.
			vv3fVisionPositions.clear();
		}

		LOG->debug("[-] getAveragedCalibrationObjectVisionPosition");

		return average(vv3fVisionPositions);
	}
	/**
	 * Filter by iteratively removing the point that is outside the limit and is farthest away.
	 **/
	void AutomaticCalibrator::filterPointList(
		std::vector<cv::Vec3f> & vv3fVisionPositions,
		float fInlierDistanceMm)
	{
		LOG->debug("#points before filtering: %v", vv3fVisionPositions.size());

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

			// Remove the last point int the list if it is too far away ...
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

		LOG->debug("#points after filtering: %v", vv3fVisionPositions.size());
	}
	/**
	 *
	 **/
	cv::Vec3f AutomaticCalibrator::average(
		std::vector<cv::Vec3f> const & vv3fPositions)
	{
		// Sum them up.
		cv::Vec3f v3fSumPositions(0.0f, 0.0f, 0.0f);
		for(auto const & v3fPosition : vv3fPositions)
		{
			v3fSumPositions += v3fPosition;
		}

		// Divide by the number of entries.
		cv::Vec3f v3fAveragedPosition(v3fSumPositions);
		if(vv3fPositions.size()>0)
		{
			v3fAveragedPosition *= (1.0f / static_cast<float>(vv3fPositions.size()));
		}

		return v3fAveragedPosition;
	}
	/**
	 * Implements "Least-Squares Rigid Motion Using SVD"
	 * See: http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
	 * NOTE: We do not directly follow the paper because we allow mirroring!
	 **/
	cv::Matx44f AutomaticCalibrator::estimateRigidBodyTransformation(
		std::vector<std::pair<cv::Vec3f, cv::Vec3f>> const & vv2v3fCorrespondences)
	{
		LOG->debug("[+] estimateRigidBodyTransformation");

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

		// The original paper only allows rotations and no reflections.
		// We explicitly want to allow this because the arm coordinate system is not necessarily the same as the vision coordinate system.
		//DCorrect(2, 2) = static_cast<float>(cv::determinant(V*Ut));

		// Compute the roation matrix.
		cv::Matx33f const R(V * DCorrect * Ut);
		// Compute the translation vector.
		cv::Vec3f const t(v3CenterLeft - R * v3CenterRight);

		LOG->debug("[-] estimateRigidBodyTransformation");

		return cv::Matx44f{
			R(0, 0), R(0, 1), R(0, 2), t(0),
			R(1, 0), R(1, 1), R(1, 2), t(1),
			R(2, 0), R(2, 1), R(2, 2), t(2),
			0.0f, 0.0f, 0.0f, 1.0f};
	}

	void AutomaticCalibrator::reset() {
		m_bCalibrationAvailable = false;
	}

}
}
