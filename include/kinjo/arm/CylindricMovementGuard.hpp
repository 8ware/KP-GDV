#pragma once

#include <kinjo/arm/MovementGuard.hpp>


namespace kinjo {
namespace arm {

	class CylindricMovementGuard final : public MovementGuard {

	public:
		CylindricMovementGuard();
		~CylindricMovementGuard() = default;


		/**
		 * Handling Results will yield a result as an Integer which must be treated after calling this method
		 * 0 everything went fine
		 * 1 simple detour (no problem)
		 * 2 endpoint in a deadzone
		 * 3 startpoint in a deadzone!
		 */
		void Handle_Deathzones(cv::Vec3f startPos, cv::Vec3f endPos, int *HandlingResult, cv::Vec3f *PosToTravelFirst) override;
		
		/**
		 * The Following functions are helper functions
		 */
	private:
		/**
		 * initializes the size of the deadzones
		 */
		void Init_Deadzones();

		/**
		 * private member funktion to calculate if line between startpos and
		 * endpos intersects with the inner deadzone
		 * \param startpos startposition of the arm in millimeter
		 * \param endpos endposition which the arm tries to reach in millimeter
		 */
		bool LineCircleIntersection(cv::Vec3f startPos, cv::Vec3f endPos);

		/**
		 * private member function
		 * calculates a Position to reach first
		 * \param startpos startposition of the arm in millimeter
		 * \param endpos endposition which the arm tries to reach in millimeter
		 * \return Position to reach next
		 */
		cv::Vec3f CalculateDetour(cv::Vec3f startPos, cv::Vec3f endPos);

		/**
		 * checks if the start position is legal
		 * \param startpos the start position
		 */
		bool StartpointLegal(cv::Vec3f startPos);
		/**
		 * checks if the end position is legal
		 * \param endPos the end position
		 */
		bool EndpointLegal(cv::Vec3f endPos);
		
		/**
		 * checks if the endposition is too close to the table
		 * \param endPos the end position
		 */
		bool EndpointNotInTable(cv::Vec3f endPos);

	private:
		float InnerCircleRadius; ///< radius of the inner cylinder
		float OuterCircleRadius; ///< radius of the outer cylinder
		float maxHeight;         ///< the height the arm can't reach anymore
		float tableHeight;       ///< distance between cartesian coordinates
								 ///< and table


	};

}//arm
}//kinjo