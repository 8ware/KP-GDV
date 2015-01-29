#pragma once

#include <kinjo/arm/MovementGuard.hpp>


namespace kinjo {
	namespace arm {

		class MovementGuardOne final : public MovementGuard {

		public:
			MovementGuardOne();
			~MovementGuardOne() = default;
			void Init_Deadzones() override;

			/**
			* Handling Results will yield a result as an Integer which must be treated after calling this method
			* 0 everything went fine
			* 1 simple detour (no problem)
			* 2 endpoint in a deadzone
			* 3 startpoint in a deadzone!
			**/
			void Handle_Deathzones(cv::Vec3f startPos, cv::Vec3f endPos, int *HandlingResult ,cv::Vec3f *PosToTravelFirst) override;
			
			bool LineCircleIntersection(cv::Vec3f startPos, cv::Vec3f endPos) override;

			cv::Vec3f CalculateDetour(cv::Vec3f startPos, cv::Vec3f endPos) override;
			
			bool StartpointLegal(cv::Vec3f startPos, bool *InInnerCircle) override;
			
			bool EndpointLegal(cv::Vec3f endPos, bool *InInnerCircle) override;

			bool EndpointNotInTable(cv::Vec3f endPos) override;
		
		private:
			float InnerCircleRadius = 0.2f;
			float OuterCircleRadius = 0.7f;
			float maxHeight = 0.8f;
			float tableHeight = 0.0f;


		};

	}//arm
}//kinjo