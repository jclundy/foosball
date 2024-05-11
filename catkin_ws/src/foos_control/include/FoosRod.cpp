#include "FoosRod.h"
/*
* Todo:
- make numbers all ints, in terms of MM
- if ball in zone, check neighbour for overlap
- fix scale of ball coordinates output by colorTracker
*/

FoosRod::FoosRod(uint8_t numFoosMen, float footWidth, const float relativeOffsets[], float motionRange, float tableWidth) {
  numFoosMen = numFoosMen;
  footWidth = footWidth;
  relativeOffsets = relativeOffsets;
  motionRange = motionRange;
  tableWidth = tableWidth;
  zones = (foosManZone *) malloc(numFoosMen * sizeof(foosManZone));
  
  // first zone will go from 0 to 
  zones[0].start = 0;
  zones[0].end = relativeOffsets[0] + motionRange + footWidth / 2;

  for (int i = 0; i < numFoosMen; ) {
     zones[i].start = relativeOffsets[0] - footWidth / 2;
     zones[i].end = relativeOffsets[0] + motionRange + footWidth / 2;
  }
  zones[0].start = 0;
  if(numFoosMen > 1) {
    zones[numFoosMen - 1].end = tableWidth;
  }
}

int8_t FoosRod::getZoneNumber(float ballPositionY) {
  if(ballPositionY < 0) {
    return 0;
  }
  if(ballPositionY > tableWidth) {
    return numFoosMen - 1;
  }
  for(int i = 0; i < numFoosMen; i++) {
    if(ballPositionY > zones[i].start && ballPositionY < zones[i].end) {
      return i;
    }
  }

  // technically shouldn't reach here
  return -1;
}

float FoosRod::getFoosManOffset(uint8_t foosManNumber) {
    if(foosManNumber >= 0 && foosManNumber < numFoosMen) {
        return relativeOffsets[foosManNumber];
    }
    return 0;
}