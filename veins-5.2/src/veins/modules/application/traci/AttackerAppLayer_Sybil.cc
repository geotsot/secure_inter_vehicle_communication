#include "AttackerAppLayer_Sybil.h"

using namespace veins;

Define_Module(AttackerAppLayer_Sybil);

void AttackerAppLayer_Sybil::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
    }
}

void AttackerAppLayer_Sybil::onBSM(BasicSafetyMessage* bsm) {
    double my_x = curPosition.x;
    double my_y = curPosition.y;

    simtime_t currentTime = simTime();
    Coord currentPosition = bsm->getSenderPos();
    double currentPositionX = currentPosition.x;
    double currentPositionY = currentPosition.y;

    DBG_APP(this) << "Sybil attacker received a BSM at " << currentTime.dbl() << std::endl;

    double threshold = 10;

    if (currentPositionX > my_x - threshold)
    {
        double minEuclideanDistance = threshold;
        int index = -1;

        for (int i = 0; i < lastReportedPosition.size(); i++)
        {
            simtime_t messageSent = lastReportedPosition[i]->getTimestamp();
            simtime_t t = currentTime - messageSent;

            Coord velocity = lastReportedPosition[i]->getSenderSpeed();
            double v = velocity.x;

            double s = v * t.dbl();

            Coord pastPosition = lastReportedPosition[i]->getSenderPos();

            double expectedPositionX = s + pastPosition.x;
            double expectedPositionY = pastPosition.y;

            double euclideanDistance = sqrt(pow(currentPositionX - expectedPositionX, 2) + pow(currentPositionY - expectedPositionY, 2));

            DBG_APP(this) << "euclideanDistance: " << euclideanDistance << std::endl;
            if (euclideanDistance < minEuclideanDistance)
            {
                minEuclideanDistance = euclideanDistance;
                index = i;
            }
        }

        DBG_APP(this) << "Victim position: " << currentPositionX << "," << currentPositionY << std::endl;

        if (minEuclideanDistance < threshold
            && msgStack[index]->getTimestamp() > currentTime - SimTime(3, SIMTIME_S)
            && msgStack[index]->getSenderPos().x > currentPositionX)
        {
            DBG_APP(this) << "I have already attacked this vehicle, so I should use existing Sybil identity." << std::endl;

            lastReportedPosition[index] = bsm->dup();
            BasicSafetyMessage* bsmPt1 = new BasicSafetyMessage();
            populateWSM(bsmPt1);
            bsmPt1->setSenderPos(msgStack[index]->getSenderPos());
            bsmPt1->setSenderSpeed(msgStack[index]->getSenderSpeed());
            msgStack[index]->setTimestamp(currentTime);

            DBG_APP(this) << "Sybil node position: " << bsmPt1->getSenderPos().x << "," << bsmPt1->getSenderPos().y << std::endl;

            sendDown(bsmPt1);
        }
        else
        {
            DBG_APP(this) << "New vehicle detected. Fabricating new Sybil identity..." << std::endl;

            Coord currentVelocity = bsm->getSenderSpeed();
            double currentV = currentVelocity.x;

            simtime_t newTimestamp = bsm->getTimestamp();
            simtime_t delay = currentTime - newTimestamp;

            // estimate victim's safe stopping distance
            double perceptionReactionTime = 1.0;
            double perceptionReactionDistance = currentV * perceptionReactionTime;
            double deliveryDistance = currentV * delay.dbl();
            double margin = 15; // m, 3 car lengths margin
            double deceleration = 3; // m/s^2, slightly harsher braking than comfortable
            double stoppingDistance = deliveryDistance + perceptionReactionDistance + (currentV * currentV)/(2*deceleration) + margin;

            BasicSafetyMessage* bsmPt1 = new BasicSafetyMessage();
            populateWSM(bsmPt1);
            bsmPt1->setSenderPos(Coord(currentPositionX + stoppingDistance, currentPositionY));
            bsmPt1->setSenderSpeed(Coord(0, 0));

            DBG_APP(this) << "Sybil node position: " << bsmPt1->getSenderPos().x << "," << bsmPt1->getSenderPos().y << std::endl;

            lastReportedPosition.push_back(bsm->dup());
            msgStack.push_back(bsmPt1->dup());

            sendDown(bsmPt1);
        }
    }
    else
    {
        DBG_APP(this) << "Sender of the BSM is more than 10 meters behind. No attack will be performed." << std::endl;
    }
}
