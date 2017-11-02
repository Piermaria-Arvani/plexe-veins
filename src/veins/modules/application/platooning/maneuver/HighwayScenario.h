#ifndef HIGHWAYSCENARIO_H_
#define HIGHWAYSCENARIO_H_

#include "veins/modules/application/platooning/scenarios/BaseScenario.h"
#include "veins/modules/application/platooning/protocols/BaseProtocol.h"

#include "veins/modules/application/platooning/utilities/NewPositionHelper.h"
#include "veins/modules/application/platooning/messages/ManeuverMessage_m.h"
#include "veins/modules/application/platooning/traffic/PlatoonsTrafficManager.h"

class HighwayScenario : public BaseScenario
	{

	protected:
		//define the roles
		enum JOIN_ROLE {LEADER, JOINER, LEADER_OR_JOINER};
		//data that each car needs to keep
		struct JOINER_DATA{
			int					id;
			double				distance;
		};
		struct VEHICLE_DATA {
			double              speed;      //speed of the platoon
			double 				maxSpeed;	//maximum speed a machine wants to go
			double				minSpeed;	//minimum speed a machine wants to go
			double				platoonSpeed;
			int                 joinLane;   //the lane chosen for joining the platoon
			int                 joinerId;   //the id of the vehicle joining the platoon
			int 				lastDirection;
			int					consecutivelyBlockedTimes;
			int					lastFailedMessageDestination;
			std::vector<JOINER_DATA> joiners;
			bool 				inHighway;
			std::string			destionationId; //the id of the destination
		};

		//define the states for each role
		typedef enum _VEHICLE_STATES {
			V_INIT = 0,
			V_IDLE = 1,
			LS_LEADING = 2,
			LS_WAIT_JOINER_IN_POSITION = 3,
			LS_WAIT_JOINER_TO_JOIN = 4,
			JS_WILL_TO_BE_FOLLOWER = 5,
			JS_WAIT_REPLY = 6,
			JS_MOVE_IN_POSITION = 7,
			JS_WAIT_JOIN = 8,
			JS_FOLLOW = 9,
			LJS_WILL_TO_BE_LEADER_OR_FOLLOWER = 10,
			LJS_WAIT_REPLY = 11,
			LJS_MOVE_IN_POSITION = 12,
			LJS_WAIT_JOIN = 13,
			V_EXIT = 14
		} VEHICLE_STATES;
		//define the messages that can be sent by each role
		enum VEHICLE_MSGS {
			LM_PROPOSE_AS_LEADER = 0,
			LM_REFUSE_JOIN = 1,
			LM_MOVE_IN_POSITION = 2,
			LM_JOIN_PLATOON = 3,
			JM_REQUEST_JOIN = 4,
			JM_IN_POSITION = 5,
			JM_IN_PLATOON = 6,
			LM_ABORT_MANEUVER = 7,
			JM_ABORT_MANEUER = 8,
			LM_CHANGE_LEADER = 9,
			LM_IN_PLATOON = 10,
			LM_TAKE_CAR_CONTROL = 11,
			LM_ABORT_MANEUVER_SEND_FAIL = 12
		};

		typedef enum _JOIN_FAILURE{
			VEHICLE_INTRUSION = 0,
			WAIT_REPLY_TIME_OUT = 1,
			MOVE_IN_POSITION_TIME_OUT = 2,
			MESSAGE_FAIL = 3
		}JOIN_FAILURE;
		VEHICLE_STATES vehicleState;
		//the role of this vehicle
		JOIN_ROLE role;

		int timer;
		int timeoutTimer;
		int counterSendFail;
		int MAX_LANE_INDEX;

		const int MAX_TIME_WAITING_REPLY = 240;
		const int MAX_TIME_LEADER_WAITING_IN_POSITION = 180;
		const int MAX_DISTANCE_FROM_PLATOON_TO_JOIN =  1000;
		const int DISTANCE_FROM_EXIT = 2000; //at this distance dismount platoons and switch to driver control
		const int MAX_BLOCKED_TIMES = 3;
		const double detectingVehiclesInterval = 1.5;
		const double leaderProposalInterval = 30 / detectingVehiclesInterval;
		const int controlInHighway = 2;
		const int dismountingPlatoonTime = 2;
		const int maxSendFail = 3;

		int speedRangeAccepted;
		//data known by the vehicle
		struct VEHICLE_DATA vehicleData;
		//message used to start the maneuver
		cMessage *startManeuver;
		cMessage *movingInPosition;
		cMessage *detectInFrontVehicle;
		cMessage *gettingInHighway;
		cMessage *platoonDismount;
		//pointer to protocol
		BaseProtocol *protocol;
		NewPositionHelper *newPositionHelper;
		PlatoonsTrafficManager *manager;


		//output vectors for mobility stats
		//id of the vehicle
		cOutVector nodeIdOut;
		//speed and position
		cOutVector speedOut, posxOut, posyOut;
		//size of the platoon
		cOutVector platoonSize;
		//
		cOutVector routeOut;
		//
		cOutVector failure;
	public:

		static const int MANEUVER_TYPE = 12347;

		virtual void initialize(int stage);
		virtual void finish();

	protected:
		void sendUnicast(cPacket *msg, int destination);

	private:

	public:
		HighwayScenario()  {
			startManeuver = 0;
			positionHelper = 0;
			counterSendFail = 0;
			speedRangeAccepted = 0;
		}

	protected:

		virtual void handleSelfMsg(cMessage *msg);
		//override this method of BaseApp. we want to handle it ourself
		virtual void handleLowerMsg(cMessage *msg);
		virtual void handleLowerControl(cMessage *msg);
		//sends leader proposal and information about its platoon
		virtual void sendLeaderProposal();
		virtual void sendNegativeAck(int destination, int type);
		//determines the role of a vehicle depending on the vehicle id
		virtual int determineRole ();
		//analyzes a joiner request. If the request can be accepted it will insert the joiner into the queue
		virtual void insertNewJoiner(int joinerId, double hisPosX);
		virtual void leaderDetectsInFrontVehicles();
		virtual void joinerDetectsInFrontVehicles();
		virtual void changeState(VEHICLE_STATES state);
		virtual void resetJoiner();
		virtual void resetLeader();
		virtual void removeVehicleFromJoiners(int id);
		virtual void setDestinationId(std::string routeId);
		virtual void dismountPlatoon();
		virtual void changeOneLane();
		virtual void recordFailure(JOIN_FAILURE fail);

		ManeuverMessage *generateMessage();

		void handleVehicleMsg(cMessage *msg);

		void prepareManeuverCars();
		virtual void leaderSendMoveOrLeading();

	};

#endif
