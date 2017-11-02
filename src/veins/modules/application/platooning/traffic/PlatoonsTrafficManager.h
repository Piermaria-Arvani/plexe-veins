//
// Copyright (C) 2014-2016 Michele Segata <segata@ccs-labs.org>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#ifndef PLATOONSTRAFFICMANAGER_H_
#define PLATOONSTRAFFICMANAGER_H_

#include <veins/modules/mobility/traci/TraCIBaseTrafficManager.h>
typedef std::vector<int> platoon;
typedef std::pair<int,int> vehicleToBeInserted;
typedef std::vector<vehicleToBeInserted> vehiclesPerRamp;

class PlatoonsTrafficManager : public TraCIBaseTrafficManager
{

	public:

		virtual void initialize(int stage);
		virtual bool isLeader(unsigned int myId);
		virtual platoon& getLeaderInfo(int myId);
		virtual void getFollowerInfo(int myId, int& leaderId, int& frontVehicle, int& backVehicle);
		virtual void increaseExitedCars();
		virtual int getRouteNumber(std::string routeId);
		virtual void finish();

		PlatoonsTrafficManager() {
			insertPlatoonMessage = 0;
			platoonInsertDistance = 0;
			platoonInsertHeadway = 0;
			platoonInsertSpeed = 0;
			platoonInsertTime = SimTime(0);
			platoonLeaderHeadway = 0;
			platoonSize = 0;
			nCars = 0;
			nLanes = 0;
			counterVehiclesFromBegin = 0;
			counter = 0;
			carsCounter = 0;
			exitedCars = 0;
			routeNumber = 0;
		}

	protected:

		//this is used to start traffic generation
		cMessage *insertPlatoonMessage;
		cMessage *insertVehicleFromBegin;
		cMessage *insertVehicleInRamps;
		cMessage *insertVehicle;
		cMessage *recordNcars;

		//inserisce un nuovo veicolo
		virtual void insertFollower(int followerId, int leaderId);

		void insertPlatoons();

		void insertVehiclesFromBegin();
		void insertVehiclesInRamps();

		void insertVehicles();

		void recordCarsNumber();

		virtual void handleSelfMsg(cMessage *msg);

		SimTime platoonInsertTime;
		double platoonInsertSpeed;
		//vehicles to be inserted
		struct Vehicle automated;

		//total number of vehicles to be injected
		int nCars;
		//vehicles per platoon
		int platoonSize;
		//number of lanes
		int nLanes;
		unsigned int counterVehiclesFromBegin;
		//
		unsigned int counter;
		//
		int routeNumber;

		//it counts the number of vehicles that have entered the highway
		int carsCounter;
		//it count the number of vehicles that have left the highway
		unsigned int exitedCars;
		//insert distance
		double platoonInsertDistance;
		//insert headway
		double platoonInsertHeadway;
		//headway for leader vehicles
		double platoonLeaderHeadway;
		//sumo vehicle type of platooning cars
		std::string platooningVType;

		cPar *distributionInsertionTime;

		//output vectors for mobility stats
		cOutVector carNumber;
		cOutVector exitedCarsNumber;

		virtual void scenarioLoaded();

	private:
		virtual void printMatrix();
		std::vector<platoon> platoons;
		std::vector<vehiclesPerRamp> vehiclesToBeInsertedInRamps;
		std::vector<vehicleToBeInserted> vehiclesToBeInsertedFromBegin;
		//vector of all the routes ids
		std::vector<std::string> routeIds;
};

#endif
