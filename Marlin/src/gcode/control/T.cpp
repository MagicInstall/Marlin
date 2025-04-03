/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../gcode.h"
#include "../../module/tool_change.h"

#if EITHER(HAS_MULTI_EXTRUDER, DEBUG_LEVELING_FEATURE)
  #include "../../module/motion.h"
#endif

#if ENABLED(PRODMACH)
  #include "../../feature/mmu/pmmmu.h"
  #include "../../libs/buzzer.h"
  #include "../../module/temperature.h"
  #include "../../module/servo.h"
  #include "../../module/endstops.h"
  #include "../../lcd/extui/ui_api.h"

  #define MMU_UART_Td_Pd(T, P) \
    SERIAL_ECHOPGM("T"); \
    SERIAL_ECHO(T); \
    SERIAL_ECHOPGM(" P"); \
    SERIAL_ECHO(P); \
    SERIAL_EOL()

  // 喷头X轴停靠位置
  static const float _CLEAN_NOZZLE[] = CLEAN_NOZZLE_X_OFFSET;
  
  static const int _CLEAN_NOZZLE_COUNT = sizeof(_CLEAN_NOZZLE) / sizeof(_CLEAN_NOZZLE[0]);

  // 挤出多余料丝并清刷喷头
  static inline void perform_purge(const xyz_pos_t &park_point, float purge_length = PER_PURGE_LENGTH_MAX) {
    stepper.enable_e_steppers();
    while (purge_length > 0) {
      if (_CLEAN_NOZZLE_COUNT > 0) do_blocking_move_to_x(park_point.x + _CLEAN_NOZZLE[0], feedRate_t(CLEAN_NOZZLE_FEEDRATE));
      unscaled_e_move(PER_PURGE_LENGTH_MAX, feedRate_t(ADVANCED_PAUSE_PURGE_FEEDRATE));
      if (_CLEAN_NOZZLE_COUNT > 1) {
          for (int i = 1; i < _CLEAN_NOZZLE_COUNT; i++) {
              do_blocking_move_to_x(park_point.x + _CLEAN_NOZZLE[i], CLEAN_NOZZLE_FEEDRATE);
          }
      }
      purge_length -= PER_PURGE_LENGTH_MAX;
    }  
  }

#endif

#if HAS_PRUSA_MMU2
  #include "../../feature/mmu/mmu2.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

/**
 * T0-T<n>: Switch tool, usually switching extruders
 *
 *   F[units/min] Set the movement feedrate
 *   S1           Don't move the tool in XY after change
 *
 * For PRUSA_MMU2(S) and EXTENDABLE_EMU_MMU2(S)
 *   T[n] Gcode to extrude at least 38.10 mm at feedrate 19.02 mm/s must follow immediately to load to extruder wheels.
 *   T?   Gcode to extrude shouldn't have to follow. Load to extruder wheels is done automatically.
 *   Tx   Same as T?, but nozzle doesn't have to be preheated. Tc requires a preheated nozzle to finish filament load.
 *   Tc   Load to nozzle after filament was prepared by Tc and nozzle is already heated.
 * 
 *  For PMMMU
 *    换料过程包含停靠挤出头, 退线进线, 移动切换头, 喷头加热等步骤;
 *    
 *    Z   覆盖NOZZLE_PARK_POINT 中的z轴提升值;
 *    S   加热喷头温度;
 *    E   先清除余下料丝直至打印头的挤出齿轮无法夹住料丝,才能继续换线;
 *        主要用于打印时触发断料事件后(目前只有event_filament_runout事件才会传递这个参数);
 *        清除过程受PurgeLength 宏影响, E的值大于PER_PURGE_LENGTH_MAX 的话, 可能会重复多次动作;
 * 
 *  退线失败时的用户操作方法:
 *    1. 关闭电机后,用户需要自行旋动挤出齿轮,将线完全退出;
 *    2. 此时选择头应该是在当前线夹的位置,将线插入直至挤出齿轮的深度;
 *    3. 点击继续,手动轻微将线推入,确保线材顺利进入.          
 */
void GcodeSuite::T(const int8_t tool_index) {
  DEBUG_SECTION(log_T, "T", DEBUGGING(LEVELING));
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("...(", tool_index, ")");

  // Count this command as movement / activity
  reset_stepper_timeout();

  #if ENABLED(PRODMACH)
    // T号超出上限则跳过换料继续打印
    if (tool_index >= TOOLS_COUNT) {
      SERIAL_ECHO_START();
      SERIAL_ECHO_MSG("t%d Exceeding upper!\n", tool_index);
      return;
    }       

    // 检查PMMMU是否在线   
    pmmmu.waitUntilReady(); 
    // TODO: MMU在启动时警告上次换线未完成(ToolIndex != ChangingToolIndex);
    
    // 将常量参数转换为变量
    int8_t next_tool = tool_index;

    set_axis_homed(I_AXIS); //  wing: 让I轴不回原点也能触发do_park
    const bool do_park = !axes_should_home();
    xyz_pos_t park_point = NOZZLE_PARK_POINT;    
    // 记住之前的位置
    xyz_pos_t resume_position = current_position;  

    // 停靠喷头
    if (do_park) {
      if (parser.seenval('Z')) park_point.z += ABS(parser.linearval('Z'));
      nozzle.park(0, park_point); // Park the nozzle by doing a Minimum Z Raise followed by an XY Move
    }
    else {
      if (!axis_was_homed(X_AXIS)) {
        endstops.enable(true);
        homeaxis(X_AXIS);
        endstops.not_homing();
        do_blocking_move_to_x(park_point.x, feedRate_t(NOZZLE_PARK_XY_FEEDRATE));
        // resume_position.x = park_point.x;
      }
    }

    // 触发了断料事件
    float purge_length = parser.seenval('E') ? parser.value_axis_units(E_AXIS) : 0; // 目前使用E参数表示是否触发断料事件
    if (purge_length > 0) perform_purge(park_point, purge_length);

    // 先解决runout1 与runout2 不是相同的状态的异常, 需要用户手动干预 
    while (pmmmu.isRunout() != pmmmu.isExtruerRunout())
    {
      pmmmu.filamentInstallWizard();
    }

    // 记录断料传感器初始状态
    // const bool switchHasFil = !pmmmu.isRunout();
    // const bool extruerHasFil = !pmmmu.isExtruerRunout();
    const bool has_filament = !pmmmu.isExtruerRunout(); // || !pmmmu.isRunout() ;
    const bool same_tool = next_tool == pmmmu.ToolIndex;

    // 需要换槽并且传感器有线, 则需要先切料并退线
    const bool do_unload = !same_tool && has_filament;
    // T号相同且已经有线,则不需要切换线槽 || 如果没有线,则利用换线过程装线或者切换备用槽
    const bool do_change = !same_tool || !has_filament;     
    // 需要切换线槽 || 相同T号但传感器没有线,则需要进线
    const bool do_load = do_change;

    // 暂时取消冷挤出限制,不然无法启动E轴
    thermalManager.allow_cold_extrude = true; 
    stepper.enable_e_steppers();

    // 切换头检测到有线在才退线, 不然线可能会被退到挤出齿之前, 下次将无法进线
    if (do_unload) {
      // 切料
      planner.synchronize();
      servo[CUTTING_SERVO_NUM].move(SERVO_CUT_OFF_ANGLE);
      safe_delay(SERVO_AFTER_MOVING_DELAY);
      servo[CUTTING_SERVO_NUM].move(SERVO_SEMI_OCCLUSION_ANGLE);
      safe_delay(SERVO_AFTER_MOVING_DELAY);

      // 退一段无检测的距离+传感器到切刀的距离
      unscaled_e_move(-ABS(FIXED_LENGTH + TO_CUTTER_DISTANCE), feedRate_t(MMU_FAST_FEEDRATE));

      // 小步退线, 直到检测到线已经超过传感器
      while (!pmmmu.isRunout())
      {
        unscaled_e_move(-ABS(SMALL_FEED_DISTANCE), feedRate_t(MMU_SLOW_FEEDRATE));
      }

      // 退到线夹位置
      unscaled_e_move(-ABS(FROM_SWITCH_HEAD_DISTANCE), feedRate_t(MMU_SLOW_FEEDRATE));
    }
    
    if (do_change) {
      // 向MMU发送换线开始事件
      MMU_UART_Td_Pd(next_tool, 2);

      do {
        // 移动切换头
        #if HAS_I_AXIS
          if (pmmmu.ToolIndex != next_tool) {
            float i_dist = 0;
            // int direction = next_tool > pmmmu.ToolIndex ? 1 : -1;
            if (pmmmu.ToolIndex < next_tool) {
              for (int i = pmmmu.ToolIndex; i < next_tool; i ++) {
                i_dist += pmmmu.ForwardDistance[i + 1];
              }          
            } else {
              for (int i = pmmmu.ToolIndex; i > next_tool; i --) {
                i_dist += pmmmu.BackwardDistance[i - 1];
              }
            }
          
            do_blocking_move_to_i(current_position.i + i_dist, W_AXIS_FEEDRATE);
          }
        #endif
        
        // 预挤出到断料传感器
        unscaled_e_move(TO_SWITCH_HEAD_DISTANCE, feedRate_t(MMU_SLOW_FEEDRATE));

        // 传感器检测到线正常进入
        if (!pmmmu.isRunout()) { 
          pmmmu.ToolIndex = next_tool;
        }
        // 若有指定备用槽则切换到备用槽
        else if (pmmmu.FilamentBackup[next_tool] != next_tool) {        
          // unscaled_e_move(-TO_SWITCH_HEAD_DISTANCE, feedRate_t(MMU_SLOW_FEEDRATE));
          next_tool = pmmmu.FilamentBackup[next_tool];
          continue;
        }
        // 需用户干预装线
        else { 
          MMU_UART_Td_Pd(next_tool, 1); // 将切换头已就位也视为换线完成
          pmmmu.ToolIndex = next_tool;

          // TODO: 当检测到不是打印状态时, 提供取消换线的选项
          do {
            // 等待用户手动干预 
            pmmmu.filamentInstallWizard();
            unscaled_e_move(TO_SWITCH_HEAD_DISTANCE, feedRate_t(MMU_SLOW_FEEDRATE));
          } while (pmmmu.isRunout());
        }
      } while (next_tool != pmmmu.ToolIndex);

      // 发送换线完成事件
      MMU_UART_Td_Pd(next_tool, 1);
    }
    
    // 加热
    if (parser.seenval('S')) {
      M104_M109(true); 
    } 

    if (do_load) {
      // 移动喷头到穿孔点
      if (_CLEAN_NOZZLE_COUNT > 0) do_blocking_move_to_x(park_point.x + _CLEAN_NOZZLE[0], feedRate_t(CLEAN_NOZZLE_FEEDRATE));

      // 快速挤出无检测的长度
      unscaled_e_move(FIXED_LENGTH, feedRate_t(MMU_FAST_FEEDRATE));

      // 小距离进线至打印头端传感器
      while (pmmmu.isExtruerRunout()) {
        unscaled_e_move(SMALL_FEED_DISTANCE, feedRate_t(MMU_SLOW_FEEDRATE));
      }

      servo[CUTTING_SERVO_NUM].move(SERVO_SEMI_OCCLUSION_ANGLE);
      safe_delay(SERVO_AFTER_MOVING_DELAY);

      // 进线到切刀位置
      unscaled_e_move(TO_CUTTER_DISTANCE, feedRate_t(MMU_SLOW_FEEDRATE));

      servo[CUTTING_SERVO_NUM].move(0);
      safe_delay(SERVO_AFTER_MOVING_DELAY);    

      #if ENABLED(PREVENT_COLD_EXTRUSION)
        thermalManager.allow_cold_extrude = false;
      #endif   

      // 清除残留料丝
      float purge_length = pmmmu.PurgeLength;
      while (purge_length > 0) {
        perform_purge(park_point);
        purge_length -= PER_PURGE_LENGTH_MAX;
      }
    }

    #if ENABLED(PREVENT_COLD_EXTRUSION)
      thermalManager.allow_cold_extrude = false;
    #endif   

    // 返回到原位
    if (do_park) {
      do_blocking_move_to_xy(resume_position.x, resume_position.y, feedRate_t(NOZZLE_PARK_XY_FEEDRATE));
      do_blocking_move_to_z(_MAX(resume_position.z - park_point.z, 0), feedRate_t(NOZZLE_PARK_Z_FEEDRATE));
    }

    // TODO: E轴清零


  #elif HAS_PRUSA_MMU2
    if (parser.string_arg) {
      mmu2.tool_change(parser.string_arg);   // Special commands T?/Tx/Tc
      return;
    }
  #else
    tool_change(tool_index
      #if HAS_MULTI_EXTRUDER
        ,  TERN(PARKING_EXTRUDER, false, tool_index == active_extruder) // For PARKING_EXTRUDER motion is decided in tool_change()
        || parser.boolval('S')
      #endif
    );
  #endif

  SERIAL_ECHOLNPGM("T() method completed.");
}
