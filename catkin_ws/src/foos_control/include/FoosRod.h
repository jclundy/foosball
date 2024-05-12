#include <math.h>
#include <stdint.h>
#include <vector>

typedef struct {
  float start;
  float end;
} foosManZone;


class FoosRod {
  public:
    FoosRod(uint8_t numFoosMen, float footWidth, const float relativeOffsets[], float motionRange, float tableWidth);
    int8_t getZoneNumber(float ballPositionY);
    uint8_t getNumFoosMen() {return m_numFoosMen;};
    float getFoosManOffset(uint8_t foosManNumber);

  private:
    uint8_t m_numFoosMen;
    float m_footWidth;
    float m_motionRange;
    float m_tableWidth;

    std::vector<foosManZone> m_zones;
    std::vector<float> m_relativeOffsets;
};
