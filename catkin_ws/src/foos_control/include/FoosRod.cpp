#include "FoosRod.h"
#include "stdio.h"
/*
* Todo:
- make numbers all ints, in terms of MM
- if ball in zone, check neighbour for overlap
- fix scale of ball coordinates output by colorTracker
*/

FoosRod::FoosRod(uint8_t numFoosMen, float footWidth, const float relativeOffsetArray[], float motionRange, float tableWidth) {
  m_numFoosMen = numFoosMen;
  m_footWidth = footWidth;
  m_motionRange = motionRange;
  m_tableWidth = tableWidth;
  printf("before initializing zone vector\n");
  
  // first zone will go from 0 to 
  for (int i = 0; i < m_numFoosMen; i++) {
    m_relativeOffsets.push_back(relativeOffsetArray[i]);
    foosManZone zone;
    zone.start = m_relativeOffsets[i] - m_footWidth / 2;
    zone.end = m_relativeOffsets[i] + m_motionRange + m_footWidth / 2;
    m_zones.push_back(zone);
  }
  m_zones[0].start = 0;
  if(m_numFoosMen > 1) {
    m_zones[m_numFoosMen - 1].end = m_tableWidth;
  }
  printf("after initializing zone vector\n");
  printf("m_numFoosMen=%i, m_footWidth=%f, m_motionRange=%f, m_tableWidth=%f", m_numFoosMen, m_footWidth, m_motionRange, m_tableWidth);
}

int8_t FoosRod::getZoneNumber(float ballPositionY) {

  printf("ballPositionY=%f, m_tableWidth=%f\n", ballPositionY, m_tableWidth);

  if(ballPositionY < 0) {
    printf("return 0 since ball position is negative\n");
    return 0;
  }
  if(ballPositionY > m_tableWidth) {
    printf("return N-1 since ball position is over table Width\n");
    return (int8_t) m_numFoosMen - 1;
  }
  for(int i = 0; i < m_numFoosMen; i++) {
    printf("i=%i\n", i);

    if(ballPositionY > m_zones[i].start && ballPositionY < m_zones[i].end) {
      return i;
    }
  }

  printf("return -1 \n");
  // technically shouldn't reach here
  return -1;
}

float FoosRod::getFoosManOffset(uint8_t foosManNumber) {
    if(foosManNumber >= 0 && foosManNumber < m_numFoosMen) {
        return m_relativeOffsets[foosManNumber];
    }
    return 0;
}