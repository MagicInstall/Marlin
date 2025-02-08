#pragma once

#include "../../inc/MarlinConfig.h"
// #include "float.h"

#if HAS_FILAMENT_SENSOR
  #include "../runout.h"
#endif

void floatToStr(float value, char *buffer, int precision = 2);

class PMMMU {
private:
    bool ready = false;
public:
    int       ToolIndex;              // 从0开始, 0表示第一个槽位
    float     PerExtrusionDistance;   // 移动选线头后预挤出到断料传感器的距离(mm)
    int       PerExtrusionFeedRate;   // 预挤出的速率(mm/s)
    int       WAxisFeedRate;          // W轴的速率(mm/s)
    float     ForwardDistance[TOOLS_COUNT];
    float     BackwardDistance[TOOLS_COUNT];
    int       FilamentBackup[TOOLS_COUNT];  // 每个槽可指定一个备用槽, 若备用槽号等于当前槽号表示没有指定备用槽.
    uint32_t  FilamentColor[TOOLS_COUNT];  
//   PMMMU();

    void init();

    void parseReport(char *c);
    bool isReady(void); 
    bool waitUntilReady(uint32_t timeout);
    // bool toolChange(GcodeSuite *gcode, uint32_t index, uint16_t temperatrue = -1);
};

extern PMMMU pmmmu;