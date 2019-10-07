#include "NormalAppLayer.h"
#include "veins/modules/messages/CustomBasicSafetyMessage_m.h"

Define_Module(NormalAppLayer);

void NormalAppLayer::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;

        messageLength = par("messageLength").longValue();

        delayBSM_P1 = 0;
        delayBSM_P2 = 0;

        carAheadLastUpdate = NULL;
    }
}

double NormalAppLayer::getEuclideanDistance(Coord x1, Coord x2) {
    return sqrt(pow(x1.x - x2.x, 2) + pow(x1.y - x2.y, 2));
}

void NormalAppLayer::onBSM(BasicSafetyMessage* bsm) {
    DBG_APP << "Received a BSM" << std::endl;

    simtime_t currentTime = simTime();
    simtime_t messageSent = bsm->getTimestamp();

    Coord& senderPosition = bsm->getSenderPos();
    double x = senderPosition.x;
    double y = senderPosition.y;
    Coord& senderSpeed = bsm->getSenderSpeed();
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
    }
    else
    {
        // new neighbor detected, create new record
        neighborsList.push_back(neighbor());
        neighborIdx = neighborsList.size() - 1;

        neighborsList[neighborIdx].lastUpdate = messageSent;
        neighborsList[neighborIdx].position = senderPosition;
        neighborsList[neighborIdx].speed = senderSpeed;
        neighborsList[neighborIdx].repPositive = 0;
        neighborsList[neighborIdx].repNegative = 0;
    }

    //

    simtime_t delay = currentTime - messageSent;
    delayBSM_P1 += delay.dbl();
}

void NormalAppLayer::onWSM(WaveShortMessage* wsm) {
    DBG_APP << "Received a WSM" << std::endl;
    if (CustomBasicSafetyMessage* cbsm = dynamic_cast<CustomBasicSafetyMessage*>(wsm)) {
        simtime_t currentTime = simTime();
        simtime_t messageSent = cbsm->getTimestamp();

        simtime_t delay = currentTime - messageSent;
        DBG_APP << "delay: " + std::to_string(delay.dbl()) << std::endl;
        delayBSM_P2 += delay.dbl();

        Coord senderPosition = cbsm->getSenderPos();
        Coord senderVelocity = cbsm->getSenderSpeed();

        DBG_APP << "Sender position: " + std::to_string(senderPosition.x) << std::endl;
        DBG_APP << "My position: " + std::to_string(curPosition.x) << std::endl;

        DBG_APP << "Sender speed: " + std::to_string(senderVelocity.x) << std::endl;
        DBG_APP << "My speed: " + std::to_string(curSpeed.x) << std::endl;

        // https://en.wikipedia.org/wiki/Braking_distance
        // d_total = d_pr + -v^2/2a
        double perceptionReactionTime = 1.0;
        double perceptionReactionDistance = curSpeed.x * perceptionReactionTime;
        double stoppingDistance = perceptionReactionDistance + (curSpeed.x * curSpeed.x)/(2*traciVehicle->getDecel());

        if ((std::fabs(senderPosition.x - curPosition.x) < 100) && (cbsm->getEventIndicator() == 0) && (std::fabs(senderVelocity.x - curSpeed.x) > 1))
        {
            DBG_APP << "Slowing down..." << std::endl;
            DBG_APP << "Stopping distance: " + std::to_string(stoppingDistance) << std::endl;

            // stop at the rightmost lane for 3 seconds after traveling the stopping distance
            traciVehicle->stopAt(traciVehicle->getRoadId(), curPosition.x + stoppingDistance, 0, 0, 3);
        }
    }
}

void NormalAppLayer::handleSelfMsg(cMessage* msg) {
    if (carAheadLastUpdate != NULL)
    {
        // more than 3 seconds elapsed since the last warning about the danger in front and my speed is lower than maximum allowed speed
        if ((simTime() - carAheadLastUpdate > 3) && curSpeed.x < traci->lane(traciVehicle->getLaneId()).getMaxSpeed())
        {
            DBG_APP << "No obstacle in front of me. Reverting to original behavior." << std::endl;
            traciVehicle->setSpeed(-1);
        }
    }

    DBG_APP << "NormalAppLayer::handleSelfMsg triggered" << std::endl;
    // WSM message will be sent here
    if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) {
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
        DBG_APP << "MY NEIGHBORS" << std::endl;
        for (int i = 0; i < neighborsList.size(); i++)
        {
            DBG_APP << "#" << i << ". " << neighborsList[i].lastUpdate << "\t[" << neighborsList[i].position.x << "," << neighborsList[i].position.y << "]" << std::endl;
        }

        // adjusting speed to slower vehicles ahead
        double my_x = curPosition.x;
        double my_y = curPosition.y;

        // my stopping distance
        double my_perceptionReactionTime = 1.0;
        double my_perceptionReactionDistance = curSpeed.x * my_perceptionReactionTime;
        double my_stoppingDistance = my_perceptionReactionDistance + (curSpeed.x * curSpeed.x)/(2*traciVehicle->getDecel());
        double my_stoppingPosition = my_x + my_stoppingDistance;
        DBG_APP << "I am able to safely stop at [" << my_stoppingPosition << "," << my_y << "]" << std::endl;

        int neighborIdx = -1;
        double closestObstacle = my_stoppingPosition;

        for (int i = 0; i < neighborsList.size(); i++)
        {
            double v = neighborsList[i].speed.x;

            // stopping distance of neighbor #i
            double ah_perceptionReactionTime = 1.0;
            double ah_perceptionReactionDistance = v * ah_perceptionReactionTime;
            double ah_stoppingDistance = ah_perceptionReactionDistance + (v * v)/(2*traciVehicle->getDecel());
            double ah_stoppingPosition = neighborsList[i].position.x + ah_stoppingDistance;
            DBG_APP << "Vehicle ahead may potentially stop at [" << ah_stoppingPosition << "," << neighborsList[i].position.y << "]" << std::endl;

            // neighbor #i is in front, in my lane and its stopping position is closer than currently closest obstacle
            if (neighborsList[i].position.x > my_x
             && neighborsList[i].position.y == my_y
             && ah_stoppingPosition <= closestObstacle)
            {
                closestObstacle = ah_stoppingPosition;
                neighborIdx = i;
            }
        }

        if (neighborIdx > -1)
        {
            // one of my neighbors is potential obstacle, reaction required
            double v = neighborsList[neighborIdx].speed.x;

            DBG_APP << "Obstacle detected; I should adjust my speed to the speed of vehicle ahead." << std::endl;
            traciVehicle->setSpeed(v);
            DBG_APP << "Changing speed from " << curSpeed.x << " m/s to " << v << "m/s." << std::endl;
        }
        else
        {
            // no obstacles detected
            traciVehicle->setSpeed(-1);
            DBG_APP << "No further action required. Continue with normal behavior." << std::endl;
        }

        if (msg->getKind() == SEND_BEACON_EVT) {
            BasicSafetyMessage* bsm = new BasicSafetyMessage();
            populateWSM(bsm);
            bsm->setByteLength(messageLength);
            sendDown(bsm);
            scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        }
    }
}

void NormalAppLayer::finish() {
    BaseWaveApplLayer::finish();
    if (receivedBSMs > 0) recordScalar("delayBSM_P1", delayBSM_P1/receivedBSMs);
    else recordScalar("delayBSM_P1", 0);

    if (receivedWSMs > 0) recordScalar("delayBSM_P2", delayBSM_P2/receivedWSMs);
    else recordScalar("delayBSM_P2", 0);
}
