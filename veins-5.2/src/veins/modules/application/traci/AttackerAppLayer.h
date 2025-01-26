#ifndef AttackerAppLayer_H
#define AttackerAppLayer_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/messages/CustomBasicSafetyMessage_m.h"

class AttackerAppLayer : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;
		std::vector<CustomBasicSafetyMessage*> msgStack;
	protected:
        virtual void onWSM(WaveShortMessage* wsm);

        virtual void handleSelfMsg(cMessage* msg);
};

#endif
