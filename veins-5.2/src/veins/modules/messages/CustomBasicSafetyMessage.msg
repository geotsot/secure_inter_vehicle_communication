//class noncobject EventFlag;

import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;
import veins.base.utils.SimpleAddress;

namespace veins;

packet CustomBasicSafetyMessage extends BaseFrame1609_4 {
	Coord senderPos;
	Coord senderSpeed;
	int16_t eventIndicator;
	uint64_t validity;
	
	uint8_t certificate[126];
	uint8_t signature[56];
}
