cplusplus {{
#include "veins/base/utils/Coord.h"
#include "veins/modules/messages/WaveShortMessage_m.h"
}}

class WaveShortMessage;
class noncobject Coord;
class noncobject EventFlag;

packet CustomBasicSafetyMessage extends WaveShortMessage {
	Coord senderPos;
	Coord senderSpeed;
	int16_t eventIndicator;
	uint64_t validity;
	
	uint8_t certificate[126];
	uint8_t signature[56];
}
