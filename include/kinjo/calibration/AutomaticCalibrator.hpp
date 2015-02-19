#pragma once

#include <kinjo/calibration/Calibrator.hpp>

#include <kinjo/arm/Arm.hpp>
#include <kinjo/vision/Vision.hpp>
#include <kinjo/recognition/Recognizer.hpp>
#include <kinjo/calibration/CalibrationPointGenerator.hpp>

#include <thread>

namespace kinjo {
namespace calibration {

	/**
	 * Allows calibration of arm to vision.
	 **/
	class AutomaticCalibrator : public Calibrator
	{
	public:
		/**
		 * Constructor.
		 **/
		AutomaticCalibrator(
			arm::Arm * const pArm, 
			vision::Vision * const pVision,
			recognition::Recognizer const * const pRecognizer,
			CalibrationPointGenerator * const pCalibrationPointGenerator,
			std::size_t const & uiCalibrationPointCount, 
			std::size_t const & uiCalibrationRotationCount,
			std::size_t const & uiMinimumValidPositionsAfterFilteringPercent,
			std::size_t const & fMaximumFilterEuclideanDistancePointToAverage;);

		/**
		 * Copy assignment operator.
		 **/
		AutomaticCalibrator & operator=(AutomaticCalibrator const &) = delete;

		/**
		 * \return If there is a valid transformation available.
		 **/
		virtual bool getIsValidTransformationAvailable() const override;

		/**
		 * \return The current rigid body transformation betwween vision and arm.
		 **/
		virtual cv::Matx44f getRigidBodyTransformation() const override;
			
		/**
		 * Calibrates the vision and the arm.
		 * The calibration object has to be grabbed before!
		 * This function returns immediately and performs the calibration asynchronously.
		 * The calibration has finished when getIsValidTransformationAvailable returns true.
		 **/
		virtual void calibrateAsync() override;

	private:
		/**
		 * Filters a point list for outliers.
		 **/
		static void filterPointList(
			std::vector<cv::Vec3f> & vv3fVisionPositions,
			float fInlierDistanceMm);

		/**
		 * \return The average point.
		 **/
		static cv::Vec3f average(
			std::vector<cv::Vec3f> const & vv3fVisionPositions);

		/**
		 * \return Estimates the rigid body transformation from the given point correspondences.
		 **/
		static cv::Matx44f estimateRigidBodyTransformation(
			std::vector<std::pair<cv::Vec3f, cv::Vec3f>> const & vv2v3fCorrespondences);

		/**
		 * Calibrates the vision and the arm.
		 **/
		void calibrationThreadMain();

		/**
		 * \return The averaged position of the calibration object in the vision over multiple frames/rotations.
		 *		   The vector is zero if it was not recognized.
		 **/
		cv::Vec3f getAveragedCalibrationObjectVisionPosition() const;

	private:
		std::thread m_Thread;

		cv::Matx44f m_matCurrentRigidBodyTransformation;

		bool m_bCalibrationAvailable;

		arm::Arm * const m_pArm;
		vision::Vision * const m_pVision; 
		recognition::Recognizer const * const m_pRecognizer;
		CalibrationPointGenerator * const m_pCalibrationPointGenerator;

		std::size_t const m_uiCalibrationPointCount;
		std::size_t const m_uiCalibrationRotationCount;
		std::size_t const m_uiMinimumValidPositionsAfterFilteringPercent;
		std::size_t const m_fMaximumFilterEuclideanDistancePointToAverage;
	};

}
}
