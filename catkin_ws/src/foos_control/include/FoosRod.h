#include <math.h>
#include <stdint.h>
#include <vector>

typedef struct {
  float start;
  float end;
} foosManZone;


class FoosRod {
  public:
    FoosRod(int numFoosMen, float footWidth, const float relativeOffsets[], float motionRange, float tableWidth);
    int getZoneNumber(float ballPositionY);
    int getNumFoosMen() {return m_numFoosMen;};
    float getFoosManOffset(int foosManNumber);

  private:
    int m_numFoosMen;
    float m_footWidth;
    float m_motionRange;
    float m_tableWidth;

    std::vector<foosManZone> m_zones;
    std::vector<float> m_relativeOffsets;
};
