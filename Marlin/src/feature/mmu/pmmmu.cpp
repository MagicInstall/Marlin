#include "../../inc/MarlinConfig.h"

#if ENABLED(PRODMACH)

#include "pmmmu.h"
#include "../../gcode/gcode.h"

static char ftosrt_buf[48];
/**
 * @note 在M403() 方法里printf无法使用%f输出, 使用ftostr() 更是崩溃, 
 *       凡是在M403() 的栈申请数组的语句都会崩溃, 我更崩溃... Stack size 已经加到很大很大了啊...
 *       只能自己写个函数, 顺便扩展一下%f做不到的去掉无意义的0。
 *       
 * @note !!重点!! : 上面的ftosrt_buf[] 不能在M403()里面占用栈空间,
 *       目前没有精力测试线程冲突, 反正现在能跑起来...
 * 
 * @note 原本找AI写floatToStr(), 改了无数个版本都不能输出正确的小数部分,
 *       而且AI代码极其智障, 巨多临时变量, 绕来绕去, 运行效率又低, 多次重复的乘除...
 *       实在忍不了只能自己写(参考了AI, 懒得自己一句一句组织)。
 * 
 * @brief 将浮点数转换为字符串，并按指定精度格式化, 并且去掉末尾无意义的0, 若小数部分等于0, 连小数点也去掉。
 *                                                                          -- wing 25.2.4
 *
 * @param str 输出的字符数组，转换后的浮点数字符串将存储在此数组中, 含'\0'结束符最长大概需要约48字节。
 * @param num 要转换的浮点数。
 * @param precision 保留的小数位数, 剩余的小数部分将被四舍五入。
 */
void floatToStr(char* str, float num, int precision = 2) {
    int intPart = (int)num;   // 整数部分
    float fracPart = num - intPart; // 小数部分

    char* p = str;

    // 处理负号
    if (num < 0) {
        *p++ = '-';
        intPart = -intPart;  // 转换成正数处理
        fracPart = -fracPart;
    }

    // 处理整数部分
    if (intPart == 0) {
        *p++ = '0';
    } else {
        // 根据整数部分的位数计算出后面需要用的除数
        int divisor = 10;
        while (intPart >= divisor) {
            divisor *= 10;
        }

        // 根据位数填充整数部分
        while(divisor > 1) {
            divisor /= 10;
            *p++ = (intPart / divisor) + '0';
            intPart %= divisor;
        }
    }

    // 处理小数部分
    if (precision > 0) {
        *p++ = '.';  // 小数点

        // 处理小数部分：逐步放大以获取每一位小数
        for (int i = 0; i < precision; i++) {
            fracPart *= 10;
        }

        intPart= (int)(fracPart);  // 取整部分,复用intPart变量
        fracPart -= intPart;  // 更新剩余的小数部分

        // 四舍五入：检查剩余小数部分是否大于等于 0.5
        if (fracPart >= 0.5) {
            intPart++;  // 四舍五入
        }

        // 逐位输出小数部分
        p += precision - 1;
        for (int i = precision; i > 0; i--) {
            *p-- = intPart % 10 + '0';  // 写入当前数字
            intPart /= 10;
        }

        // 使用循环去掉末尾无意义的零
        p += precision;
        while (*p == '.' || (*p == '0' && *(p + 1) != '.')){
            p--; 
        };
        p++;
    }

    *p = '\0';  // 结束字符串
}

PMMMU pmmmu;

void PMMMU::init() {
    ready = false;
    ToolIndex = -1;             
    PerExtrusionDistance = PER_EXTRUSION_DISTANCE;
    PerExtrusionFeedRate = PER_EXTRUSION_FEEDRATE;
    WAxisFeedRate = W_AXIS_FEEDRATE;
    for (int i = 0; i < TOOLS_COUNT; i++)
    {       
        ForwardDistance[i] = __FLT_MIN__;
        BackwardDistance[i] = __FLT_MIN__;
        FilamentBackup[i] = -1;
        FilamentColor[i] = 0xFFFFFFFF;
    }

    // MMU_UART.printf("M503\n");
}

void PMMMU::parseReport(char *c) {

}

// bool PMMMU::toolChange(GcodeSuite *gcode, uint32_t index, uint16_t temperatrue) {
//     gcode.M702();
    

//     return true;
// }

bool PMMMU::isReady (void) {
    // 不再重复验证
    if (ready) return true;
    
    // 通过检查本地数据是否全部有效来判断PMMMU 是否在线

    // 验证不通过就直接返回false
    if (pmmmu.ToolIndex == -1) return false;
    // if (pmmmu.PerExtrusionDistance == __FLT_MIN__) return false;
    // if (pmmmu.PerExtrusionFeedRate == -1) return false;
    // if (pmmmu.WAxisFeedRate == -1) return false;
    for (int i = 0; i < TOOLS_COUNT; i++)
    {
        if (pmmmu.ForwardDistance[i] == __FLT_MIN__) return false;
        if (pmmmu.BackwardDistance[i] == __FLT_MIN__) return false;
        if (pmmmu.FilamentBackup[i] == -1) return false;
    }

    // 上面的验证全部通过就设置就绪
    if (ready == false) {
        ready = true;
        MMU_UART.printf("Pmmmu is ready.\n");
    }
    return true;
}

bool PMMMU::waitUntilReady (uint32_t timeout) {
    // TODO: 在执行T代码时先等MMU就绪...
    return true;
}

void GcodeSuite::M403() {
    int param = parser.intval('P', -1),
        index = parser.intval('I', -1);

    bool err = false;

    if (param != -1 || index != -1) {   
        switch (param) {
        case 1: // ToolIndex
            pmmmu.ToolIndex = index;
            MMU_UART.printf("M403 P1 I%d\n", pmmmu.ToolIndex);
            break;
        case 2: // ForwardDistance
            if (parser.seenval('W')) {
                pmmmu.ForwardDistance[index] = parser.linearval('W', __FLT_MIN__);
                floatToStr(ftosrt_buf, pmmmu.ForwardDistance[index]);
                MMU_UART.printf("M403 P2 I%d W%s\n", index, ftosrt_buf);
            }
            else err = true;
            // else {
            //     MMU_UART.println("Not seenval 'W'");
            //     err = true;
            // }
            break;
        case 3: // BackwardDistance
            if (parser.seenval('W')) {
                pmmmu.BackwardDistance[index] = parser.linearval('W', __FLT_MIN__);
                floatToStr(ftosrt_buf, pmmmu.BackwardDistance[index]);
                MMU_UART.printf("M403 P3 I%d W%s\n", index, ftosrt_buf);
            }
            else err = true;
            // if (parser.seenval('F')) {
            //     float dist = parser.floatval('F', __FLT_MIN__);
            //     if (dist != __FLT_MIN__)
            //     {
            //         pmmmu.BackwardDistance[index] = dist;
            //         MMU_UART.printf("M403 P3 I%d F%f\n", index, pmmmu.BackwardDistance[index]);
            //     }
            //     else err = true;
            // }
            break;
        case 4: // FilamentBackup
           if (parser.seenval('B')) {
                pmmmu.FilamentBackup[index] = parser.intval('B', -1);
                MMU_UART.printf("M403 P4 I%d B%d\n", index, pmmmu.FilamentBackup[index]);
            }
            else err = true;
            break;
        case 5: // FilamentColor
            break;
        
        default:
            break;
        }        
    }

    if (err)
    {
        SERIAL_ECHO_MSG("M403 - bad arguments.");
        // TODO: 点样报错?
    }

}

#endif