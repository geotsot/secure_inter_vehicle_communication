#include "veins/base/phyLayer/ChannelState.h"

using namespace veins;

bool ChannelState::isIdle() const {

	return idle;
}

double ChannelState::getRSSI() const {

	return rssi;
}

