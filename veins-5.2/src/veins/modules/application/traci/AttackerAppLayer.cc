#include "AttackerAppLayer.h"

using namespace veins;

Define_Module(AttackerAppLayer);

void AttackerAppLayer::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
    }
}

void AttackerAppLayer::onWSM(WaveShortMessage* wsm) {
    DBG_APP(this) << "received a WSM" << std::endl;
    if (CustomBasicSafetyMessage* cbsm = dynamic_cast<CustomBasicSafetyMessage*>(wsm)) {
        if (cbsm->getEventIndicator() == 0)
        {
            DBG_APP(this) << "Message saved for repeating..." << std::endl;
            msgStack.push_back(cbsm->dup());
        }
    }
}

void AttackerAppLayer::handleSelfMsg(cMessage* msg) {
    // replay attack will be performed periodically after beacon interval
    if (msg->getKind() == SEND_BEACON_EVT)
    {
        DBG_APP(this) << "Replay attack triggered..." << std::endl;

        Coord myPosition = curPosition;
        double minDiff = 0;
        int index = 0; // index of message selected for replaying

        if (msgStack.size() > 0)
        {
            DBG_APP(this) << "Messages in stack: " + std::to_string(msgStack.size()) << std::endl;

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

            DBG_APP(this) << "Repeating message #" + std::to_string(index) << std::endl;

            // repeat message with sender position closest to my current position
            sendDown(msgStack[index]->dup());
            scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        }
        else
        {
            // if there is no message to replay, BSM beacon
            // will be broadcasted to keep beaconing alive
            BasicSafetyMessage* bsm = new BasicSafetyMessage();
            populateWSM(bsm);
            sendDown(bsm);
            scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        }
    }

}
