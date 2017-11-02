#ifndef NEWPOSITIONHELPER_H_
#define NEWPOSITIONHELPER_H_

#include "veins/modules/application/platooning/utilities/BasePositionHelper.h"
typedef std::vector<int> platoon;

class NewPositionHelper : public BasePositionHelper
{

	public:

		virtual void initialize(int stage);
		virtual void finish();

		/**
		 * Sets all the ids of the vehicles of the platoon
		 */
		virtual void setPlatoon(platoon p);

		/**
		 * Returns whether this vehicle is the leader of the platoon
		 */
		virtual bool isLeader (int id);
		bool isInSamePlatoon(int vehicleId);

		virtual void setLeader(int id, std::string sumoId);
		virtual void setFrontVehicle(int id, std::string sumoId);
		/**
		 * Inserts a follower in the last position of the platoon
		 */
		virtual void insertFollower (int id, std::string sumoId);

		/**
		 * Removes a follower from the platoon
		 */
		virtual void removeFollower (int id);
		virtual void removeLastFollower();

		/**
		 * Returns the id of the last vehicle of the platoon
		 */
		virtual int getLastVehicle ();

		/**
		 * Returns all the ids of the vehicles of the platoon
		 */
		virtual platoon& getPlatoon();

		/**
		 * Returns the platoon size
		 */
		virtual int getPlatoonSize();

		/**
		 * Returns the id of the vehicle in front of me
		 */
		virtual int getFrontVehicle();

		virtual std::string getSumoId();
		virtual std::string getLeaderSumoId();
		virtual std::string getFrontVehicleSumoId();
		virtual std::string getLastVehicleSumoId();
		virtual std::vector<std::string>& getSumoIds();
		virtual void resetVectors();


		static int getIdFromExternalId(std::string externalId);

	protected :
		/*
		 * Vector of ids
		 */
		platoon p;
		std::vector<std::string> sumoIds;
		std::string mySumoId;
		std::string myLeaderSumoId;
		std::string myFrontVehicleSumoId;


	public:
		NewPositionHelper() : BasePositionHelper() {
			mySumoId = "";
			myLeaderSumoId = "";
			myFrontVehicleSumoId = "";
		}

};

#endif
