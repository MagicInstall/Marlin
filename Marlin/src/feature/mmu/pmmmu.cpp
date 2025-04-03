#include "../../inc/MarlinConfig.h"
#include "../../gcode/gcode.h"

#if ENABLED(PRODMACH)

#include "pmmmu.h"
#include "../runout.h"
#include "../../module/servo.h"
#include "../../module/temperature.h"
#include "../../module/endstops.h"
#include "../../libs/buzzer.h"

#define MMU_UART_M403_Pd_Id_Wf(P, I, W) \
  SERIAL_ECHOPGM("M403 P"); \
  SERIAL_ECHO(P); \
  SERIAL_ECHOPGM(" I"); \
  SERIAL_ECHO(I); \
  SERIAL_ECHOPGM(" W"); \
  SERIAL_ECHO_F(W); \
  SERIAL_EOL()

#define MMU_UART_M403_Pd_Id_Bd(P, I, B) \
  SERIAL_ECHOPGM("M403 P"); \
  SERIAL_ECHO(P); \
  SERIAL_ECHOPGM(" I"); \
  SERIAL_ECHO(I); \
  SERIAL_ECHOPGM(" B"); \
  SERIAL_ECHO(B); \
  SERIAL_EOL()

// #define MMU_UART_Td_Pd(T, P) \
//   SERIAL_ECHOPGM("T"); \
//   SERIAL_ECHO(T); \
//   SERIAL_ECHOPGM(" P"); \
//   SERIAL_ECHO(P); \
//   SERIAL_EOL()

#define FLOAT_MIN 0.01


// static char ftosrt_buf[48];
// /**
//  * @note 在M403() 方法里printf无法使用%f输出, 使用ftostr() 更是崩溃, 
//  *       凡是在M403() 的栈申请数组的语句都会崩溃, 我更崩溃... Stack size 已经加到很大很大了啊...
//  *       只能自己写个函数, 顺便扩展一下%f做不到的去掉无意义的0。
//  *       
//  * @note !!重点!! : 上面的ftosrt_buf[] 不能在M403()里面占用栈空间,
//  *       目前没有精力测试线程冲突, 反正现在能跑起来...
//  * 
//  * @note 原本找AI写floatToStr(), 改了无数个版本都不能输出正确的小数部分,
//  *       而且AI代码极其智障, 巨多临时变量, 绕来绕去, 运行效率又低, 多次重复的乘除...
//  *       实在忍不了只能自己写(参考了AI, 懒得自己一句一句组织)。
//  * 
//  * @brief 将浮点数转换为字符串，并按指定精度格式化, 并且去掉末尾无意义的0, 若小数部分等于0, 连小数点也去掉。
//  *                                                                          -- wing 25.2.4
//  *
//  * @param str 输出的字符数组，转换后的浮点数字符串将存储在此数组中, 含'\0'结束符最长大概需要约48字节。
//  * @param num 要转换的浮点数。
//  * @param precision 保留的小数位数, 剩余的小数部分将被四舍五入。
//  */
// void floatToStr(char* str, float num, int precision = 2) {
//     int intPart = (int)num;   // 整数部分
//     float fracPart = num - intPart; // 小数部分

//     char* p = str;

//     // 处理负号
//     if (num < 0) {
//         *p++ = '-';
//         intPart = -intPart;  // 转换成正数处理
//         fracPart = -fracPart;
//     }

//     // 处理整数部分
//     if (intPart == 0) {
//         *p++ = '0';
//     } else {
//         // 根据整数部分的位数计算出后面需要用的除数
//         int divisor = 10;
//         while (intPart >= divisor) {
//             divisor *= 10;
//         }

//         // 根据位数填充整数部分
//         while(divisor > 1) {
//             divisor /= 10;
//             *p++ = (intPart / divisor) + '0';
//             intPart %= divisor;
//         }
//     }

//     // 处理小数部分
//     if (precision > 0) {
//         *p++ = '.';  // 小数点

//         // 处理小数部分：逐步放大以获取每一位小数
//         for (int i = 0; i < precision; i++) {
//             fracPart *= 10;
//         }

//         intPart= (int)(fracPart);  // 取整部分,复用intPart变量
//         fracPart -= intPart;  // 更新剩余的小数部分

//         // 四舍五入：检查剩余小数部分是否大于等于 0.5
//         if (fracPart >= 0.5) {
//             intPart++;  // 四舍五入
//         }

//         // 逐位输出小数部分
//         p += precision - 1;
//         for (int i = precision; i > 0; i--) {
//             *p-- = intPart % 10 + '0';  // 写入当前数字
//             intPart /= 10;
//         }

//         // 使用循环去掉末尾无意义的零
//         p += precision;
//         while (*p == '.' || (*p == '0' && *(p + 1) != '.')){
//             p--; 
//         };
//         p++;
//     }

//     *p = '\0';  // 结束字符串
// }

PMMMU pmmmu;

void PMMMU::init() {
    #if ENABLED(MMU_RUNOUT_PULLUP)
        SET_INPUT_PULLUP(MMU_RUNOUT_PIN);
    #elif ENABLED(MMU_RUNOUT_PULLDOWN)
        SET_INPUT_PULLDOWN(MMU_RUNOUT_PIN);
    #else
        SET_INPUT(MMU_RUNOUT_PIN);
    #endif

    ready = false;
    ToolIndex = -1;             
    // PerExtrusionDistance = TO_SWITCH_HEAD_DISTANCE;
    // PerExtrusionFeedRate = MMU_SLOW_FEEDRATE;
    // WAxisFeedRate = W_AXIS_FEEDRATE;
    for (int i = 0; i < TOOLS_COUNT; i++)
    {       
        // ForwardDistance[i] = FLOAT_MIN;
        // BackwardDistance[i] = FLOAT_MIN;
        FilamentBackup[i] = -1;
        // FilamentColor[i] = 0xFFFFFFFF;
    }

    // 请求MMU配置
    SERIAL_ECHOPGM("M503\n");
}

void PMMMU::reset() {
    FixedLength = FIXED_LENGTH;
    PurgeLength = ADVANCED_PAUSE_PURGE_LENGTH;
    const float forward_dist[TOOLS_COUNT] = MMU_FORWARD_DISTANCE;
    memcpy(ForwardDistance, forward_dist, sizeof(forward_dist));
    const float backward_dist[TOOLS_COUNT] = MMU_BACKWARD_DISTANCE;
    memcpy(BackwardDistance, backward_dist, sizeof(backward_dist));
    // const float load_len[TOOLS_COUNT] = MMU_LOAD_LENGTH;
    // memcpy(LoadLength, load_len, sizeof(load_len));
    // const float unload_len[TOOLS_COUNT] = MMU_UNLOAD_LENGTH;
    // memcpy(UnloadLength, unload_len, sizeof(unload_len));

    // const int filament_backup[TOOLS_COUNT] = MMU_FILAMENT_BACKUP;
    // memcpy(FilamentBackup, filament_backup, sizeof(filament_backup));
    
    // for (int i = 0; i < TOOLS_COUNT; i++) {
    //     FilamentColor[i] = 0xFFFFFFFF;
    // }
}

bool PMMMU::isReady(void) {
    bool rdy = true;
    if (ToolIndex == -1) {
        rdy = false;
    }
    for (int i = 0; i < TOOLS_COUNT; i++)
    {
        if (FilamentBackup[i] == -1) {
            rdy = false;
            break;
        }
    }
    return rdy;
}

void PMMMU::waitUntilReady (void) {
    while (!isReady()) {
        // TODO: 显示警告上次换线未完成(ToolIndex != ChangingToolIndex);

    //   SERIAL_ECHO_START();
    //   SERIAL_ECHOLNPGM("Pmmmu not ready...");
      // #if HAS_SOUND
      //   BUZZ(400, 415); BUZZ(500, 0); // 响两声
      //   BUZZ(400, 415); 
      // #endif
      // if (while_cont ++ > 5) {
      //   // 重试多次后跳过
      //   MMU_UART.printf("Ignore T%d Code!\n", next_tool);
      //  return;
      // }
      // MMU_UART.printf("M503\n"); // TODO: 依家先发M503 系要好长时间嘅
        if (pmmmu.ToolIndex == -1) {
            SERIAL_ECHOLNPGM("M503 P1");
            BUZZ(10, 415);
            safe_delay(1000);
            continue;
        }
        for (size_t i = 0; i < TOOLS_COUNT; i++)
        {
            if (pmmmu.FilamentBackup[i] == -1) {
                SERIAL_ECHOPGM("M503 P4 I"); SERIAL_ECHO(i); SERIAL_EOL();
                BUZZ(10, 415);
                safe_delay(1000);
                break;
            }
        }
    }

    SERIAL_ECHO_START();
    SERIAL_ECHOPGM("Pmmmu is ready.\n");
}

bool PMMMU::isRunout(void) {
    #if defined(MMU_RUNOUT_PIN)
        int check = 0;
        for (int i = 0; i < PMMMU_RUNOUT_DEBOUNCE_COUNT; i++) {
            safe_delay(5);
            // 当传感器的IO值等于 MMU_RUNOUTn_STATE 时，表示断料触发
            if (READ(MMU_RUNOUT_PIN) == MMU_RUNOUT_STATE) {
                check++;
                continue;
            }
        }
        return PMMMU_RUNOUT_DEBOUNCE_COUNT - check <= PMMMU_RUNOUT_DEBOUNCE_COUNT / 2;
    #endif

    return false;
}

bool PMMMU::isExtruerRunout(void) {
    #if defined(FIL_RUNOUT_PIN)
        int check = 0;
        for (int i = 0; i < PMMMU_RUNOUT_DEBOUNCE_COUNT; i++) {
            safe_delay(5);
            // 当传感器的IO值等于 FIL_RUNOUT_STATE 时，表示断料触发
            if (READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_STATE) {
                check++;
                continue;
            }
        }
        return PMMMU_RUNOUT_DEBOUNCE_COUNT - check <= PMMMU_RUNOUT_DEBOUNCE_COUNT / 2;
    #endif

    return false;
}

bool PMMMU::ExtruerRunoutTest(void) {
    #if defined(FIL_RUNOUT_PIN)
        return READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_STATE;
    #endif
    return false;
}

void PMMMU::resetTool(void) {
    ToolIndex = -1;

    if (!isRunout()) {
        // 切料
        planner.synchronize();
        servo[CUTTING_SERVO_NUM].move(SERVO_CUT_OFF_ANGLE);
        safe_delay(SERVO_AFTER_MOVING_DELAY);
        servo[CUTTING_SERVO_NUM].move(0);
        safe_delay(SERVO_AFTER_MOVING_DELAY);    

        // 退线
        thermalManager.allow_cold_extrude = true; // 暂时取消冷挤出限制,不然无法启动E轴
        stepper.enable_e_steppers();
        while (!isRunout())
        {
            unscaled_e_move(-ABS(SMALL_FEED_DISTANCE), feedRate_t(MMU_SLOW_FEEDRATE));
        }
        unscaled_e_move(-ABS(FROM_SWITCH_HEAD_DISTANCE), feedRate_t(MMU_FAST_FEEDRATE));
        #if ENABLED(PREVENT_COLD_EXTRUSION)
            thermalManager.allow_cold_extrude = false;
        #endif    
    }

    SERIAL_ECHOPGM("T0 P2\n");

    endstops.enable(true);
    homeaxis(I_AXIS);
    endstops.not_homing();
    do_blocking_move_to_i(ForwardDistance[0], W_AXIS_FEEDRATE);

    SERIAL_ECHOPGM("T0 P1\n");
    ToolIndex = 0;
    
    BUZZ(30, 415);
}

void PMMMU::filamentInstallWizard(void) {
    stepper.disable_e_steppers();
    // 等待用户点击继续(从pause.cpp复制过来的)
    // ui.pause_show_message(PAUSE_MESSAGE_WAITING, PAUSE_MODE_PAUSE_PRINT, 0/* 只考虑一个挤出机 */);
    ui.pause_show_message(PAUSE_MESSAGE_INSERT);
    wait_for_user_response(0, true); // Wait for LCD click or M108

    // wait_for_confirmation(false, 2);

    thermalManager.allow_cold_extrude = true; // 暂时取消冷挤出限制,不然无法启动E轴
    stepper.enable_e_steppers();    
    // 进线
    int check = PMMMU_INSTALL_ATTEMPTS;
    while (isRunout()) {
        if (check == 0) {
            // #if ENABLED(PREVENT_COLD_EXTRUSION)
            //     thermalManager.allow_cold_extrude = false;
            // #endif  
            // ERR_BUZZ();
            // ui.return_to_status();
            // ui.pause_show_message(PAUSE_MESSAGE_STATUS);
            break;
        }
        unscaled_e_move(TO_SWITCH_HEAD_DISTANCE, feedRate_t(MMU_SLOW_FEEDRATE));
        check--;
    }
    // 退线
    while (!isRunout()) {
        unscaled_e_move(-ABS(SMALL_FEED_DISTANCE), feedRate_t(MMU_SLOW_FEEDRATE));
    }
    unscaled_e_move(-ABS(FROM_SWITCH_HEAD_DISTANCE), feedRate_t(MMU_SLOW_FEEDRATE));
    
    #if ENABLED(PREVENT_COLD_EXTRUSION)
        thermalManager.allow_cold_extrude = false;
    #endif  

    if (check) OKAY_BUZZ();
    else ERR_BUZZ();
    ui.return_to_status();
    // ui.pause_show_message(PAUSE_MESSAGE_STATUS);
}

void PMMMU::purgeWizard(void) {
    set_axis_homed(I_AXIS); //  wing: 让I轴不回原点也能触发do_park
    const bool do_park = !axes_should_home();
    xyz_pos_t park_point = NOZZLE_PARK_POINT;   
    // 记住之前的位置
    xyz_pos_t resume_position = current_position;  

    // 停靠喷头
    if (do_park) {
        if (parser.seenval('Z')) park_point.z = parser.linearval('Z');
        nozzle.park(0, park_point); // Park the nozzle by doing a Minimum Z Raise followed by an XY Move
    }
    else {
        if (!axis_was_homed(X_AXIS)) {
        endstops.enable(true);
        homeaxis(X_AXIS);
        endstops.not_homing();
        do_blocking_move_to_x(park_point.x, feedRate_t(NOZZLE_PARK_XY_FEEDRATE));
        resume_position.x = park_point.x;
        }
    }

    #if ENABLED(PREVENT_COLD_EXTRUSION)
        thermalManager.allow_cold_extrude = false;
    #endif  
    static const float _CLEAN_NOZZLE[] = CLEAN_NOZZLE_X_OFFSET;
    static const int _CLEAN_NOZZLE_COUNT = sizeof(_CLEAN_NOZZLE) / sizeof(_CLEAN_NOZZLE[0]);
    stepper.enable_e_steppers();

    if (_CLEAN_NOZZLE_COUNT > 0) do_blocking_move_to_x(park_point.x + _CLEAN_NOZZLE[0], feedRate_t(CLEAN_NOZZLE_FEEDRATE));
    unscaled_e_move(PER_PURGE_LENGTH_MAX, feedRate_t(ADVANCED_PAUSE_PURGE_FEEDRATE));
    if (_CLEAN_NOZZLE_COUNT > 1) {
        for (int i = 1; i < _CLEAN_NOZZLE_COUNT; i++) {
            do_blocking_move_to_x(park_point.x + _CLEAN_NOZZLE[i], CLEAN_NOZZLE_FEEDRATE);
        }
    }
}

void GcodeSuite::M403() {
    int param = parser.intval('P', -1),
        index = parser.intval('I', -1);

    bool err = false;

    if (param != -1 || index != -1) {   
        switch (param) {
        case 1: // ToolIndex
            pmmmu.ToolIndex = index;
            // MMU_UART.printf("M403 P1 I%d\n", pmmmu.ToolIndex);
            SERIAL_ECHOPGM("M403 P1 I"); SERIAL_ECHO(pmmmu.ToolIndex); SERIAL_EOL();
            break;
        case 2: // ForwardDistance
            if (parser.seenval('W')) {
                pmmmu.ForwardDistance[index] = parser.linearval('W', FLOAT_MIN);
                // floatToStr(ftosrt_buf, pmmmu.ForwardDistance[index]);
                MMU_UART_M403_Pd_Id_Wf(2, index, pmmmu.ForwardDistance[index]);
            }
            else err = true;
            // else {
            //     MMU_UART.println("Not seenval 'W'");
            //     err = true;
            // }
            break;
        case 3: // BackwardDistance
            if (parser.seenval('W')) {
                pmmmu.BackwardDistance[index] = parser.linearval('W', FLOAT_MIN);
                // floatToStr(ftosrt_buf, pmmmu.BackwardDistance[index]);
                // MMU_UART.printf("M403 P3 I%d W%s\n", index, ftosrt_buf);
                MMU_UART_M403_Pd_Id_Wf(3, index, pmmmu.ForwardDistance[index]);
            }
            else err = true;
            // if (parser.seenval('F')) {
            //     float dist = parser.floatval('F', FLOAT_MIN);
            //     if (dist != FLOAT_MIN)
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
                // MMU_UART.printf("M403 P4 I%d B%d\n", index, pmmmu.FilamentBackup[index]);
                MMU_UART_M403_Pd_Id_Bd(4, index, pmmmu.FilamentBackup[index]);
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
        SERIAL_ECHOPGM("M403 - bad arguments.\n");
        // TODO: 点样报错?
    }

}

#endif