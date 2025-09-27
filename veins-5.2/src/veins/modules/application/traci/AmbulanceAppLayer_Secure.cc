#include "veins/modules/application/traci/AmbulanceAppLayer_Secure.h"
#include "veins/modules/messages/CustomBasicSafetyMessage_m.h"

using namespace veins;

Define_Module(veins::AmbulanceAppLayer_Secure);

void AmbulanceAppLayer_Secure::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;

        messageLength = par("messageLength");
        messageValidity = par("messageValidity");

        signDelay = par("signDelay").doubleValue();
        verifyDelay = par("verifyDelay").doubleValue();
    }
}

void AmbulanceAppLayer_Secure::handleSelfMsg(cMessage* msg) {
    // BSM Part 2 message with emergency event indicator
    // will be disseminated periodically after beacon interval
    if (msg->getKind() == SEND_BEACON_EVT)
    {
        EV_DEBUG << "Broadcasting emergency event..." << std::endl;

        CustomBasicSafetyMessage* bsmPt2 = new CustomBasicSafetyMessage();
        populateWSM(bsmPt2);
        bsmPt2->setSenderPos(curPosition);
        bsmPt2->setSenderSpeed(curSpeed);
        bsmPt2->setEventIndicator(0); // emergency event indicator
        bsmPt2->setByteLength(messageLength);
        bsmPt2->setValidity(messageValidity);

        sendDown(bsmPt2);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
    }
}
