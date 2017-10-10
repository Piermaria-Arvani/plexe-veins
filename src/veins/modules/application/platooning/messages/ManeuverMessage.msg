//
// Copyright (c) 2012-2016 Michele Segata <segata@ccs-labs.org>
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

cplusplus {{ typedef std::vector<int> platoon;
             typedef std::vector<char*> SumoIds; }} // C++ typedef
class noncobject platoon; // announcement for the message compiler
class noncobject SumoIds;

packet ManeuverMessage {
    //id of the originator
    int vehicleId;
    //sumo id of the originator
    string sumoId;
    //platoon id
    int platoonId;
    //message type
    int messageType;
    //platoon speed
    double platoonSpeed;
    //my speed
    double mySpeed;
    //platoon lane
    int platoonLane;
    //last vehicle position
    double posX;
    //id of vehicle in front of the joiner (G)
    int frontVehicleId;
    //sumo id of the front vehicle
    string frontVehicleSumoId;
    //id of the follower vehicle (F)
    int followerVehicleId;
    //platoon formation
    int platoonFormation[];
    //id of the leader
    int leaderId;
    //sumo id of the leader
    string leaderSumoId;
    //vector of the ids of the platoon's vehicles
    platoon ids ;
    SumoIds sumoIds;
}
