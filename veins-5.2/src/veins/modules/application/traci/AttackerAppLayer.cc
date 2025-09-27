#include "veins/modules/application/traci/AttackerAppLayer.h"

using namespace veins;

Define_Module(veins::AttackerAppLayer);

void AttackerAppLayer::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
    }
}

void AttackerAppLayer::onWSM(BaseFrame1609_4* wsm) {
    EV_DEBUG << "received a WSM" << std::endl;
    if (CustomBasicSafetyMessage* cbsm = dynamic_cast<CustomBasicSafetyMessage*>(wsm)) {
        if (cbsm->getEventIndicator() == 0)
        {
            EV_DEBUG << "Message saved for repeating..." << std::endl;
            msgStack.push_back(cbsm->dup());
        }
    }
}

void AttackerAppLayer::handleSelfMsg(cMessage* msg) {
    // replay attack will be performed periodically after beacon interval
    if (msg->getKind() == SEND_BEACON_EVT)
    {
        EV_DEBUG << "Replay attack triggered..." << std::endl;

        Coord myPosition = curPosition;
        double minDiff = 0;
        int index = 0; // index of message selected for replaying

        if (msgStack.size() > 0)
        {
            EV_DEBUG << "Messages in stack: " + std::to_string(msgStack.size()) << std::endl;

            minDiff = std::fabs(msgStack[0]->getSenderPos().x - myPosition.x);

            // get index of message with sender position closest to my current position
            for (int i = 0; i < msgStack.size(); i++)
            {
                Coord senderPosition = msgStack[i]->getSenderPos();
                double thisDiff = std::fabs(senderPosition.x - myPosition.x);

                if (thisDiff < minDiff)
                {
                    minDiff = thisDiff;
                    index = i;
                }
            }

            EV_DEBUG << "Repeating message #" + std::to_string(index) << std::endl;

            // repeat message with sender position closest to my current position
            sendDown(msgStack[index]->dup());
            scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        }
        else
        {
            // if there is no message to replay, BSM beacon
            // will be broadcasted to keep beaconing alive
            DemoSafetyMessage* bsm = new DemoSafetyMessage();
            populateWSM(bsm);
            sendDown(bsm);
            scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        }
    }

}
