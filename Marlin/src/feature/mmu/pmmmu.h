#pragma once

#include "../../inc/MarlinConfig.h"
#if HAS_FILAMENT_SENSOR
  #include "../runout.h"
#endif

#define PMMMU_RUNOUT_DEBOUNCE_COUNT 6
#define PMMMU_SMALL_FEED_ATTEMPTS   50
#define PMMMU_INSTALL_ATTEMPTS      2

// void floatToStr(float value, char *buffer, int precision = 2);

class PMMMU {
private:
    bool ready = false;
public:
    int       ToolIndex;              // 从0开始, 0表示第一个槽位
    // float     PerExtrusionDistance;   // 移动选线头后预挤出到断料传感器的距离(mm)
    // int       PerExtrusionFeedRate;   // 预挤出的速率(mm/s)
    // int       WAxisFeedRate;          // W轴的速率(mm/s)
    float     FixedLength;      // 中间无检测的固定挤出长度(mm)
    float     PurgeLength;      // 清洗长度(mm)
    float     ForwardDistance[TOOLS_COUNT];
    float     BackwardDistance[TOOLS_COUNT];
    // float     LoadLength[TOOLS_COUNT];  
    // float     UnloadLength[TOOLS_COUNT];
    int       FilamentBackup[TOOLS_COUNT];  // 每个槽可指定一个备用槽, 若备用槽号等于当前槽号表示没有指定备用槽.
    // uint32_t  FilamentColor[TOOLS_COUNT];  
//   PMMMU();

    void init();
    void reset();

    // void parseReport(char *c);
    bool isReady(void); 
    void waitUntilReady(void);
    // MMU端是否断料, 返回true 表示断料.
    // MMU端传感器也作为打印时的断料检测使用.
    bool isRunout(void);
    // 挤出机端是否断料, 返回true 表示断料
    bool isExtruerRunout(void);
    // 不使用消抖直接取得传感器即时的值
    bool ExtruerRunoutTest(void);
    // bool toolChange(int8_t next_tool, bool skip_park = false);
    void resetTool(void);
    void filamentInstallWizard(void);
    void purgeWizard(void);
};

extern PMMMU pmmmu;