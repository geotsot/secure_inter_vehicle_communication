#ifndef AmbulanceAppLayer_Secure_H
#define AmbulanceAppLayer_Secure_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

class AmbulanceAppLayer_Secure : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
	protected:
		simtime_t lastDroveAt;
		bool sentMessage;
		int currentSubscribedServiceId;
	protected:
		int messageLength;
		int messageValidity;

        double signDelay;
        double verifyDelay;

        virtual void handleSelfMsg(cMessage* msg);
};

#endif
