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
  #include "../../module/servo.h"
  #include "../../module/endstops.h"
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
 *    U   覆盖FILAMENT_CHANGE_UNLOAD_LENGTH 或EEPROM 中的退线长度;
 */
void GcodeSuite::T(int8_t tool_index) {
  DEBUG_SECTION(log_T, "T", DEBUGGING(LEVELING));
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("...(", tool_index, ")");

  // Count this command as movement / activity
  reset_stepper_timeout();

  #if HAS_PRUSA_MMU2
    if (parser.string_arg) {
      mmu2.tool_change(parser.string_arg);   // Special commands T?/Tx/Tc
      return;
    }
  #endif

  #if ENABLED(PRODMACH)
    // 检查PMMMU是否在线
    int while_cont = 0;
    while (!pmmmu.isReady())
    {
      MMU_UART.printf("pmmmu not ready\n");
      #if HAS_SOUND
        BUZZ(400, 415); BUZZ(500, 0); // 响两声
        BUZZ(400, 415); 
      #endif
      if (while_cont ++ > 5) {
        // 重试多次后跳过
        MMU_UART.printf("Ignore T%d Code!\n", tool_index);
        return;
      }
      MMU_UART.printf("M503\n"); // TODO: 依家先发M503 系要好长时间嘅
      safe_delay(5000);
    }

    if (tool_index >= TOOLS_COUNT) {
      SERIAL_ECHO_MSG("T%d Exceeding upper!\n", tool_index);
      return;
    }    
    
    bool skip_unload = false;
    if (tool_index == pmmmu.ToolIndex) {
      if (1) { // TODO: 若传感器判断有线在则直接返回
        return;
      }
      // 没有线的话会运行这里
      skip_unload = true;
    }

    set_axis_homed(I_AXIS); //  wing: 让I轴不回原点也能触发do_park
    const bool do_park = !axes_should_home();
    static xyz_pos_t park_point = NOZZLE_PARK_POINT;

    // 记住之前的位置
    xyz_pos_t resume_position = current_position;  
    // 停靠喷头  
    if (do_park) {
      if (parser.seenval('Z')) 
        park_point.z = parser.linearval('Z');
      nozzle.park(0, park_point); // Park the nozzle by doing a Minimum Z Raise followed by an XY Move
    }
    else {
      if (!axis_was_homed(X_AXIS)) {
        endstops.enable(true);
        homeaxis(X_AXIS);
        endstops.not_homing();
        // planner.synchronize(); // TODO: 手动测试home 需唔需要等同步
        do_blocking_move_to_x(park_point.x, feedRate_t(NOZZLE_PARK_XY_FEEDRATE));
        resume_position.x = park_point.x;
      }
    }

    // 向MMU发送换线开始事件
    // TODO: MMU在启动时警告上次换线未完成(ToolIndex != ChangingToolIndex), 
    MMU_UART.printf("T%d P2\n", tool_index);

    // 切换头检测到有线在才退线, 不然线会退到挤出齿之前, 下次将无法进线
    if (!skip_unload) {
      // TODO: 若传感器判断有线在才执行退线
      if (1) {
        // 切料
        planner.synchronize();
        servo[CUTTING_SERVO_NUM].move(SERVO_CUT_OFF_ANGLE);
        safe_delay(SERVO_AFTER_MOVING_DELAY);
        servo[CUTTING_SERVO_NUM].move(0);
        safe_delay(SERVO_AFTER_MOVING_DELAY);    

        // 退线
        // const float unload_length = -ABS(parser.seen('U') ? parser.value_axis_units(E_AXIS)
        //                                                   : fc_settings[0].unload_length);
        // unload_filament(unload_length, false, PAUSE_MODE_UNLOAD_FILAMENT);
      unscaled_e_move(fc_settings[0].unload_length, feedRate_t(FILAMENT_CHANGE_UNLOAD_FEEDRATE));
      }    
    }
    
    do {
      // 移动切换头
      #if HAS_I_AXIS
        float i_dist = 0;
        // int direction = tool_index > pmmmu.ToolIndex ? 1 : -1;
        if (pmmmu.ToolIndex < tool_index) {
          for (int i = pmmmu.ToolIndex; i < tool_index; i ++) {
            i_dist += pmmmu.ForwardDistance[i + 1];
          }          
        } else {
          for (int i = pmmmu.ToolIndex; i > tool_index; i --) {
            i_dist += pmmmu.BackwardDistance[i - 1];
          }
        }
        
        do_blocking_move_to_i(current_position.i + i_dist, W_AXIS_FEEDRATE);
      #endif

      // 预挤出到断料传感器
      unscaled_e_move(PER_EXTRUSION_DISTANCE, feedRate_t(PER_EXTRUSION_FEEDRATE));
      if (1) { // TODO: 传感器检测到线正常进入
        pmmmu.ToolIndex = tool_index;
      }
      else if (pmmmu.FilamentBackup[tool_index] != pmmmu.ToolIndex) {
        tool_index = pmmmu.FilamentBackup[tool_index];
      }
      else {
        // TODO: 改为暂停?
        return;
      }
    } while (tool_index != pmmmu.ToolIndex);

    MMU_UART.printf("T%d P1\n", tool_index);
    
    // 加热
    if (parser.seenval('S')) {
      M104_M109(true);
    } 

    // 移动喷头到穿孔点
    static float clean_nozzle[] = CLEAN_NOZZLE_X_OFFSET;
    static int point_cnt = sizeof(clean_nozzle);
    // MMU_UART.printf("clean_nozzle point_cnt:%d", point_cnt);
    if (point_cnt > 0)
      do_blocking_move_to_x(park_point.x + clean_nozzle[0], feedRate_t(NOZZLE_PARK_XY_FEEDRATE));

    servo[CUTTING_SERVO_NUM].move(SERVO_SEMI_OCCLUSION_ANGLE);
    safe_delay(SERVO_AFTER_MOVING_DELAY);
    // 进线
    // const float fast_load_length = ABS(parser.seenval('L') ? parser.value_axis_units(E_AXIS)
    //                                                         : fc_settings[active_extruder].load_length);
    // load_filament(
    //   FILAMENT_CHANGE_SLOW_LOAD_LENGTH, fast_load_length - PER_EXTRUSION_DISTANCE, ADVANCED_PAUSE_PURGE_LENGTH,
    //   FILAMENT_CHANGE_ALERT_BEEPS,
    //   false,                             // show_lcd
    //   false,                            // pause_for_user
    //   PAUSE_MODE_LOAD_FILAMENT          // pause_mode
    //   OPTARG(DUAL_X_CARRIAGE, 0)        // Dual X target
    // );
    unscaled_e_move(fc_settings[0/*直接指定第一个喷头*/].load_length - PER_EXTRUSION_DISTANCE, feedRate_t(FILAMENT_CHANGE_FAST_LOAD_FEEDRATE));
    servo[CUTTING_SERVO_NUM].move(0);
    safe_delay(SERVO_AFTER_MOVING_DELAY);
    
    unscaled_e_move(ADVANCED_PAUSE_PURGE_LENGTH, feedRate_t(ADVANCED_PAUSE_PURGE_FEEDRATE));

    // 刷喷头
    if (point_cnt > 1) {
      for (int i = 1; i < point_cnt; i++)
      {
        do_blocking_move_to_x(park_point.x + clean_nozzle[i], CLEAN_NOZZLE_POINT_FEEDRATE);
      }      
    }

    // 返回到原位
    if (axis_was_homed(X_AXIS)) {
      do_blocking_move_to_xy(resume_position.x, resume_position.y, feedRate_t(NOZZLE_PARK_XY_FEEDRATE));
      do_blocking_move_to_z(_MAX(current_position.z - park_point.z, 0), feedRate_t(NOZZLE_PARK_Z_FEEDRATE));
    }
    
  #else
    tool_change(tool_index
      #if HAS_MULTI_EXTRUDER
        ,  TERN(PARKING_EXTRUDER, false, tool_index == active_extruder) // For PARKING_EXTRUDER motion is decided in tool_change()
        || parser.boolval('S')
      #endif
    );
  #endif
}
