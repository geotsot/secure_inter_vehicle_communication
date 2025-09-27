#include "veins/modules/application/traci/NormalAppLayer_Secure.h"
#include "veins/modules/messages/CustomBasicSafetyMessage_m.h"
#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "veins/modules/phy/DeciderResult80211.h"
#include "veins/modules/application/traci/SignalDistanceMap.h"

using namespace veins;

Define_Module(veins::NormalAppLayer_Secure);

void NormalAppLayer_Secure::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;

        messageLength = par("messageLength");

        delayBSM_P1_transmission = 0;
        delayBSM_P2_transmission = 0;
        delayBSM_P1_end_to_end = 0;
        delayBSM_P2_end_to_end = 0;

        signDelay = par("signDelay").doubleValue();
        verifyDelay = par("verifyDelay").doubleValue();
    }
}

double NormalAppLayer_Secure::getEuclideanDistance(Coord x1, Coord x2) {
    return sqrt(pow(x1.x - x2.x, 2) + pow(x1.y - x2.y, 2));
}

int getDistanceFromRSSI(double rssi) {
    int startIdx = 0;
    int endIdx = LOOKUP_TABLE_SIZE - 1;

    if (rssi < lookup_table[endIdx][0] || rssi > lookup_table[startIdx][0])
    {
        // signal out of range
        return -1;
    }
    else
    {
        while (startIdx <= endIdx)
        {
            int currentIdx = startIdx + ((endIdx - startIdx) / 2);
            if (rssi > lookup_table[currentIdx][0])
            {
                endIdx = currentIdx - 1;
            }
            else if (rssi < lookup_table[currentIdx][0])
            {
                startIdx = currentIdx + 1;
            }
            else
            {
                return currentIdx;
            }
        }
        return startIdx;
    }
}

bool isSuspect(double rcvPower, double claimedDist) {
    int lookup_idx = getDistanceFromRSSI(rcvPower);
    if (lookup_idx >= 0 && lookup_idx <= LOOKUP_TABLE_SIZE - 1)
    {
        double estimatedDist = lookup_table[lookup_idx][1];
        EV_DEBUG << "Estimated distance: " << estimatedDist << " m" << std::endl;
        EV_DEBUG << "Claimed distance: " << claimedDist << " m" << std::endl;
        if (std::abs(estimatedDist - claimedDist) > 2)
        {
            EV_DEBUG << "Potential Sybil node detected." << std::endl;
            return true;
        }
    }
    else
    {
        EV_DEBUG << "Could not estimate distance from given RSSI." << std::endl;
    }

    return false;
}

void NormalAppLayer_Secure::onBSM(DemoSafetyMessage* bsm) {
    EV_DEBUG << "Received a BSM" << std::endl;

    double rcvPower = ((DeciderResult80211*)((PhyToMacControlInfo*)bsm->getControlInfo())->getDeciderResult())->getRecvPower_dBm();
    double claimedDist = getEuclideanDistance(Coord(bsm->getSenderPos().x, bsm->getSenderPos().y), Coord(curPosition.x, curPosition.y));
    bool senderIsSuspect = isSuspect(rcvPower, claimedDist);

    //

    simtime_t currentTime = simTime();
    simtime_t messageSent = bsm->getTimestamp();

    Coord senderPosition = bsm->getSenderPos();
    double x = senderPosition.x;
    double y = senderPosition.y;
    Coord senderSpeed = bsm->getSenderSpeed();
    double v = senderSpeed.x;

    int neighborIdx = -1;
    double obsoleteRecord = 0.3; // time in seconds, records with lastUpdate older than obsoleteRecord will be discarded
    double threshold = 5; // distance in meters, if neighbor switch the lane, we have to take the lane width (3.3m in this sim) into consideration
    double closestToSender = threshold;

    // tracking neighbors
    // check if the sender of this BSM is in my list of neighbors
    for (int i = 0; i < neighborsList.size(); i++)
    {
        simtime_t timeDifference = messageSent - neighborsList[i].lastUpdate;
        Coord neighborSpeed = neighborsList[i].speed;
        Coord neighborLastPosition = neighborsList[i].position;
        double neighborDistanceTravelledSinceLastUpdate = neighborSpeed.x * timeDifference.dbl();
        Coord neighborExpectedPosition = Coord(neighborLastPosition.x + neighborDistanceTravelledSinceLastUpdate, neighborLastPosition.y);
        double distanceFromRealToExpectedPosition = getEuclideanDistance(senderPosition, neighborExpectedPosition);
        if (distanceFromRealToExpectedPosition < closestToSender)
        {
            closestToSender = distanceFromRealToExpectedPosition;
            neighborIdx = i;
        }
    }

    // update neighbors records
    if (neighborIdx > -1)
    {
        // known neighbor, update record
        neighborsList[neighborIdx].lastUpdate = messageSent;
        neighborsList[neighborIdx].position = senderPosition;
        neighborsList[neighborIdx].speed = senderSpeed;

        if (senderIsSuspect) neighborsList[neighborIdx].repNegative++;
        else neighborsList[neighborIdx].repPositive++;
    }
    else
    {
        // new neighbor detected, create new record
        neighborsList.push_back(neighbor());
        neighborIdx = neighborsList.size() - 1;

        neighborsList[neighborIdx].lastUpdate = messageSent;
        neighborsList[neighborIdx].position = senderPosition;
        neighborsList[neighborIdx].speed = senderSpeed;

        if (senderIsSuspect)
        {
            neighborsList[neighborIdx].repPositive = 0;
            neighborsList[neighborIdx].repNegative = 1;
        }
        else
        {
            neighborsList[neighborIdx].repPositive = 1;
            neighborsList[neighborIdx].repNegative = 0;
        }
    }

    //

    simtime_t delay = currentTime - messageSent;
    delayBSM_P1_transmission += delay.dbl();
    delayBSM_P1_end_to_end += delay.dbl() + signDelay + verifyDelay;
}

void NormalAppLayer_Secure::onWSM(BaseFrame1609_4* wsm) {
    EV_DEBUG << "received a WSM" << std::endl;

    if (CustomBasicSafetyMessage* cbsm = dynamic_cast<CustomBasicSafetyMessage*>(wsm)) {
        simtime_t currentTime = simTime();
        simtime_t messageSent = cbsm->getTimestamp();
        int messageValidity = cbsm->getValidity();

        EV_DEBUG << "currentTime: " + std::to_string(currentTime.dbl()) << std::endl;
        simtime_t validTo = messageSent + SimTime(messageValidity, SIMTIME_US);
        EV_DEBUG << "validTo: " + std::to_string(validTo.dbl()) << std::endl;

        // check if message is valid
        if (currentTime <= messageSent + SimTime(messageValidity, SIMTIME_US))
        {
            simtime_t delay = currentTime - messageSent;
            delayBSM_P2_transmission += delay.dbl();
            delayBSM_P2_end_to_end += delay.dbl() + signDelay + verifyDelay;


            Coord senderPosition = cbsm->getSenderPos();
            Coord senderVelocity = cbsm->getSenderSpeed();

            EV_DEBUG << "Sender position: " + std::to_string(senderPosition.x) << std::endl;
            EV_DEBUG << "My position: " + std::to_string(curPosition.x) << std::endl;

            EV_DEBUG << "Sender speed: " + std::to_string(senderVelocity.x) << std::endl;
            EV_DEBUG << "My speed: " + std::to_string(curSpeed.x) << std::endl;

            // https://en.wikipedia.org/wiki/Braking_distance
            // d_total = d_pr + -v^2/2a
            double perceptionReactionTime = 1.0;
            double perceptionReactionDistance = curSpeed.x * perceptionReactionTime;
            double stoppingDistance = perceptionReactionDistance + (curSpeed.x * curSpeed.x)/(2*traciVehicle->getDeccel());

            if ((std::fabs(senderPosition.x - curPosition.x) < 100) && (cbsm->getEventIndicator() == 0) && (std::fabs(senderVelocity.x - curSpeed.x) > 1))
            {
                EV_DEBUG << "Slowing down..." << std::endl;
                EV_DEBUG << "Stopping distance: " + std::to_string(stoppingDistance) << std::endl;

                // stop at the rightmost lane for 3 seconds after traveling the stopping distance
                traciVehicle->stopAt(traciVehicle->getRoadId(), curPosition.x + stoppingDistance, 0, 0, 3);
            }
        }
        else
        {
            EV_DEBUG << "Message is not valid" << std::endl;
        }
    }
}

void NormalAppLayer_Secure::handleSelfMsg(cMessage* msg) {
    // WSM message will be sent here
    if (BaseFrame1609_4* wsm = dynamic_cast<BaseFrame1609_4*>(msg)) {
        //send this message on the service channel until the counter is 3 or higher.
        //this code only runs when channel switching is enabled
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() +1);
        if (wsm->getSerial() >= 3) {
            //stop service advertisements
            stopService();
            delete(wsm);
        }
        else {
            scheduleAt(simTime()+1, wsm);
        }
    }
    else {
        // discard obsolete neighbors records
        neighborsList.erase(std::remove_if(neighborsList.begin(),
                neighborsList.end(),
                [](const neighbor& n)
                {
                    return (simTime().dbl() - n.lastUpdate.dbl() > 0.3);
                }), neighborsList.end());

        // debug, show my neighbors
        EV_DEBUG << "MY NEIGHBORS" << std::endl;
        for (int i = 0; i < neighborsList.size(); i++)
        {
            EV_DEBUG << "#" << i << ". " << neighborsList[i].lastUpdate << "\t[" << neighborsList[i].position.x << "," << neighborsList[i].position.y << "]\t+ " << neighborsList[i].repPositive << " / - " << neighborsList[i].repNegative << std::endl;
        }

        // adjusting speed to slower vehicles ahead
        double my_x = curPosition.x;
        double my_y = curPosition.y;

        // my stopping distance
        double my_perceptionReactionTime = 1.0;
        double my_perceptionReactionDistance = curSpeed.x * my_perceptionReactionTime;
        double my_stoppingDistance = my_perceptionReactionDistance + (curSpeed.x * curSpeed.x)/(2*traciVehicle->getDeccel());
        double my_stoppingPosition = my_x + my_stoppingDistance;
        EV_DEBUG << "I am able to safely stop at [" << my_stoppingPosition << "," << my_y << "]" << std::endl;

        int neighborIdx = -1;
        double closestObstacle = my_stoppingPosition;

        for (int i = 0; i < neighborsList.size(); i++)
        {
            double v = neighborsList[i].speed.x;

            // stopping distance of neighbor #i
            double ah_perceptionReactionTime = 1.0;
            double ah_perceptionReactionDistance = v * ah_perceptionReactionTime;
            double ah_stoppingDistance = ah_perceptionReactionDistance + (v * v)/(2*traciVehicle->getDeccel());
            double ah_stoppingPosition = neighborsList[i].position.x + ah_stoppingDistance;
            //EV_DEBUG << "Vehicle ahead may potentially stop at [" << ah_stoppingPosition << "," << neighborsList[i].position.y << "]" << std::endl;

            // neighbor #i is in front, in my lane and its stopping position is closer than currently closest obstacle
            if (neighborsList[i].position.x > my_x
             && neighborsList[i].position.y == my_y
             && ah_stoppingPosition <= closestObstacle
             && (neighborsList[i].repPositive - neighborsList[i].repNegative) >= 0)
            {
                closestObstacle = ah_stoppingPosition;
                neighborIdx = i;
            }
        }

        if (neighborIdx > -1)
        {
            // one of my neighbors is potential obstacle, reaction required
            double v = neighborsList[neighborIdx].speed.x;

            EV_DEBUG << "Obstacle detected (" << neighborIdx << "); I should adjust my speed to the speed of vehicle ahead." << std::endl;
            traciVehicle->setSpeed(v);
            EV_DEBUG << "Changing speed from " << curSpeed.x << " m/s to " << v << "m/s." << std::endl;
        }
        else
        {
            // no obstacles detected
            traciVehicle->setSpeed(-1);
            EV_DEBUG << "No further action required. Continue with normal behavior." << std::endl;
        }

        // BSM beacon will be sent using base class method
        DemoBaseApplLayer::handleSelfMsg(msg);
    }
}

void NormalAppLayer_Secure::finish() {
    DemoBaseApplLayer::finish();
    if (receivedBSMs > 0) recordScalar("TransmissionDelay_BSM_part1", delayBSM_P1_transmission/receivedBSMs);
    else recordScalar("TransmissionDelay_BSM_part1", 0);

    if (receivedWSMs > 0) recordScalar("TransmissionDelay_BSM_part2", delayBSM_P2_transmission/receivedWSMs);
    else recordScalar("TransmissionDelay_BSM_part2", 0);

    if (receivedBSMs > 0) recordScalar("End2EndDelay_BSM_part1", delayBSM_P1_end_to_end/receivedBSMs);
    else recordScalar("End2EndDelay_BSM_part1", 0);

    if (receivedWSMs > 0) recordScalar("End2EndDelay_BSM_part2", delayBSM_P2_end_to_end/receivedWSMs);
    else recordScalar("End2EndDelay_BSM_part2", 0);
}
