#include <math.h>
#include <stdint.h>

typedef struct {
  float start;
  float end;
} foosManZone;


class FoosRod {
  public:
    FoosRod(uint8_t numFoosMen, float footWidth, const float relativeOffsets[], float motionRange, float tableWidth);
    int8_t getZoneNumber(float ballPositionY);
    uint8_t getNumFoosMen() {return numFoosMen;};
    float getFoosManOffset(uint8_t foosManNumber);

  private:
    uint8_t numFoosMen;
    float footWidth;
    float motionRange;
    float tableWidth;

    foosManZone *zones;
    float relativeOffsets[];

};
