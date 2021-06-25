/*
    methods and constants for use in relation to robot positioning 
*/

#include <spot/frame.h>

std::string frameNameGravAligned(gravAlignedFrame frame){
	switch (frame){
        case ODOM: return ODOM_FRAME_NAME;  
		case VISION: return VISION_FRAME_NAME;
		case FLAT_BODY: return FLAT_BODY_FRAME_NAME;  
    }
}