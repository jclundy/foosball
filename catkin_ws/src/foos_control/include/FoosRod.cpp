#include "FoosRod.h"
#include "stdio.h"
/*
* Todo:
- make numbers all ints, in terms of MM
- if ball in zone, check neighbour for overlap
- fix scale of ball coordinates output by colorTracker
*/

FoosRod::FoosRod(int numFoosMen, float footWidth, const float relativeOffsetArray[], float motionRange, float tableWidth) {
  m_numFoosMen = numFoosMen;
  m_footWidth = footWidth;
  m_motionRange = motionRange;
  m_tableWidth = tableWidth;
  
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
}

int FoosRod::getZoneNumber(float ballPositionY) {

  if(ballPositionY < 0) {
    return 0;
  }
  if(ballPositionY > m_tableWidth) {
    return (int) m_numFoosMen - 1;
  }
  for(int i = 0; i < m_numFoosMen; i++) {
    if(ballPositionY > m_zones[i].start && ballPositionY < m_zones[i].end) {
      return i;
    }
  }

  // technically shouldn't reach here
  return -1;
}

float FoosRod::getFoosManOffset(int foosManNumber) {
    if(foosManNumber >= 0 && foosManNumber < m_numFoosMen) {
        return m_relativeOffsets[foosManNumber];
    }
    return 0;
}