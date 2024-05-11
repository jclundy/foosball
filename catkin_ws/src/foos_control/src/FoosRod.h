#include <math.h>
#include <stdint.h>

typedef struct foosManZone {
  float start;
  float end;
};


class FoosRod {
  public:
    FoosRod(uint8_t numFoosMen, float footWidth, float relativeOffsets[], float motionRange, float tableWidth);
    int8_t getZoneNumber(float ballPositionY);

  private:
    uint8_t numFoosMen;
    float footWidth;
    float motionRange;
    float tableWidth;

    foosManZone *zones;
    float relativeOffsets[];

};
