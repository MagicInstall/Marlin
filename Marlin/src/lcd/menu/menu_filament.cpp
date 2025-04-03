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

//
// Filament Change Menu
//

#include "../../inc/MarlinConfigPre.h"

#if BOTH(HAS_MARLINUI_MENU, ADVANCED_PAUSE_FEATURE)

#include "menu_item.h"
#include "../../module/servo.h"
#include "../../module/temperature.h"
#include "../../feature/pause.h"
#include "../../gcode/queue.h"
#if HAS_FILAMENT_SENSOR
  #include "../../feature/runout.h"
#endif
#if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
  #include "../../MarlinCore.h"
#endif
#if ENABLED(PRODMACH)
  #include "../../feature/mmu/pmmmu.h"
#endif

//
// Change Filament > Change/Unload/Load Filament
//
static PauseMode _change_filament_mode; // = PAUSE_MODE_PAUSE_PRINT
static int8_t _change_filament_extruder; // = 0

inline FSTR_P _change_filament_command() {
  switch (_change_filament_mode) {
    case PAUSE_MODE_LOAD_FILAMENT:    return F("M701 T%d");
    case PAUSE_MODE_UNLOAD_FILAMENT:  return _change_filament_extruder >= 0
                                           ? F("M702 T%d") : F("M702 ;%d");
    case PAUSE_MODE_CHANGE_FILAMENT:
    case PAUSE_MODE_PAUSE_PRINT:
    default: break;
  }
  return F("M600 B0 T%d");
}

// Initiate Filament Load/Unload/Change at the specified temperature
static void _change_filament_with_temp(const uint16_t celsius) {
  char cmd[11];
  sprintf_P(cmd, FTOP(_change_filament_command()), _change_filament_extruder);
  thermalManager.setTargetHotend(celsius, _change_filament_extruder);
  queue.inject(cmd);
}

#if HAS_PREHEAT
  static void _change_filament_with_preset() {
    _change_filament_with_temp(ui.material_preset[MenuItemBase::itemIndex].hotend_temp);
  }
#endif

static void _change_filament_with_custom() {
  _change_filament_with_temp(thermalManager.degTargetHotend(MenuItemBase::itemIndex));
}

//
// Menu to choose the temperature and start Filament Change
//

inline FSTR_P change_filament_header(const PauseMode mode) {
  switch (mode) {
    case PAUSE_MODE_LOAD_FILAMENT:   return GET_TEXT_F(MSG_FILAMENTLOAD);
    case PAUSE_MODE_UNLOAD_FILAMENT: return GET_TEXT_F(MSG_FILAMENTUNLOAD);
    default: break;
  }
  return GET_TEXT_F(MSG_FILAMENTCHANGE);
}

void _menu_temp_filament_op(const PauseMode mode, const int8_t extruder) {
  _change_filament_mode = mode;
  _change_filament_extruder = extruder;
  const int8_t old_index = MenuItemBase::itemIndex;
  START_MENU();
  if (LCD_HEIGHT >= 4) STATIC_ITEM_F(change_filament_header(mode), SS_DEFAULT|SS_INVERT);
  BACK_ITEM(MSG_BACK);
  #if HAS_PREHEAT
    LOOP_L_N(m, PREHEAT_COUNT)
      ACTION_ITEM_N_f(m, ui.get_preheat_label(m), MSG_PREHEAT_M, _change_filament_with_preset);
  #endif
  EDIT_ITEM_FAST_N(int3, extruder, MSG_PREHEAT_CUSTOM, &thermalManager.temp_hotend[extruder].target,
    EXTRUDE_MINTEMP, thermalManager.hotend_max_target(extruder),
    _change_filament_with_custom
  );
  END_MENU();
  MenuItemBase::itemIndex = old_index;
}

enum menu_pmmmu_loop_type {
  MENU_BACKUP_SLOT = 0,
  MENU_FORWARD_DISTANCE,
  MENU_BACKWARD_DISTANCE,
  MENU_LOAD_LENGTH,
  MENU_UNLOAD_LENGTH,
};

/**
  * 数组类型子项设置 submenu
  */
void menu_pmmmu_array_op(const menu_pmmmu_loop_type type) {
  // const int8_t old_index = MenuItemBase::itemIndex;

  START_MENU();
  BACK_ITEM(MSG_BACK);

  for (int i = 0; i < TOOLS_COUNT; i++) {
    switch (type) {
      case MENU_BACKUP_SLOT:
          editable.int16 = pmmmu.FilamentBackup[i] + 1;
          EDIT_ITEM_FAST_N(int3, i + 1, MSG_BACKUP_SLOT_N, &editable.int16, 1, TOOLS_COUNT, []{ 
            pmmmu.FilamentBackup[MenuItemBase::itemIndex - 1] = editable.int16 - 1; 
          });
        break;
      case MENU_FORWARD_DISTANCE:
          editable.decimal = pmmmu.ForwardDistance[i];
          EDIT_ITEM_FAST_N(float61, i + 1, MSG_FORWARD_DISTANCE_N, &editable.decimal, 0.0f, 40.9f, []{ 
            pmmmu.ForwardDistance[MenuItemBase::itemIndex - 1] = editable.decimal; 
          });
          break;
      case MENU_BACKWARD_DISTANCE:
          if (i >= TOOLS_COUNT - 1) break; // 最后一个工具不需要设置
          editable.decimal = pmmmu.BackwardDistance[i];
          EDIT_ITEM_FAST_N(float42_52, i + 1, MSG_BACKWARD_DISTANCE_N, &editable.decimal, -40.9f, 0.0f, []{ 
            pmmmu.BackwardDistance[MenuItemBase::itemIndex - 1] = editable.decimal; 
          });
          break;
      default:
        break;
    }
  }

  END_MENU();
  // MenuItemBase::itemIndex = old_index;
}

/**
 * "Reset Tool" submenu
 */
void reset_tool_menu() {
  START_MENU();
  BACK_ITEM(MSG_RESET_TOOL);

  ACTION_ITEM(MSG_YES, []{
    pmmmu.resetTool();
    ui.return_to_status();  // 执行完指令后返回主状态屏
  });
  END_MENU();
}

/**
 * "Change Filament" submenu
 */
#if E_STEPPERS > 1 || ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
  bool printingIsPaused();
#endif

void menu_change_filament() {
  #if E_STEPPERS > 1 || ENABLED(FILAMENT_LOAD_UNLOAD_GCODES) 
    // Say "filament change" when no print is active
    editable.int8 = printingIsPaused() ? PAUSE_MODE_PAUSE_PRINT : PAUSE_MODE_CHANGE_FILAMENT;

    #if E_STEPPERS > 1 && ENABLED(FILAMENT_UNLOAD_ALL_EXTRUDERS)
      bool too_cold = false;
      for (uint8_t s = 0; !too_cold && s < E_STEPPERS; s++)
        too_cold = thermalManager.targetTooColdToExtrude(s);
    #endif

    #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
      const bool is_busy = printer_busy();
    #endif

    START_MENU();
    BACK_ITEM(MSG_MAIN_MENU);

    // wing:     
    #if ENABLED(PRODMACH)
      if (!is_busy) {
        // 换槽
        editable.int16 = pmmmu.ToolIndex + 1;
        EDIT_ITEM_FAST(int3, MSG_TOOL_CHANGE, &editable.int16, 1, TOOLS_COUNT, []{
          char cmd[8];
          sprintf_P(cmd, PSTR("T%i"), editable.int16 - 1);
          queue.inject(cmd);
          ui.return_to_status();
        });

        // 装线向导
        SUBMENU(MSG_FILAMENTLOAD, []{ pmmmu.filamentInstallWizard(); });
      }

      // 备用槽
      SUBMENU(MSG_BACKUP_SLOT, []{ menu_pmmmu_array_op(MENU_BACKUP_SLOT); });

      if (!is_busy) {
        // 切刀舵机手动控制
        editable.int16 = servo[CUTTING_SERVO_NUM].read();
        EDIT_ITEM_FAST(int3, MSG_SERVO_ANGLE, &editable.int16, 0, 180, []{servo[CUTTING_SERVO_NUM].move(editable.int16); });

        // 切换头向前距离设置
        SUBMENU(MSG_FORWARD_DISTANCE, []{ menu_pmmmu_array_op(MENU_FORWARD_DISTANCE); });

        // 切换头向后距离设置
        SUBMENU(MSG_BACKWARD_DISTANCE, []{ menu_pmmmu_array_op(MENU_BACKWARD_DISTANCE); });

        // 无检测快速挤出长度设置
        EDIT_ITEM_FAST(float5, MSG_FIXED_LENGTH, &pmmmu.FixedLength, 0, 9999.0);
      }

      // 消除旧料设置
      EDIT_ITEM_FAST(float4, MSG_FILAMENT_PURGE_LENGTH, &pmmmu.PurgeLength, 0, 999.0);

      if (!is_busy) {   
        if (!pmmmu.ExtruerRunoutTest())
        {
          // 消除旧料测试向导
          SUBMENU(MSG_PURGE_LENGTH_WIZARD, []{ pmmmu.purgeWizard(); });
        }
        
        // 重置切换头
        SUBMENU(MSG_RESET_TOOL, reset_tool_menu);

        #if ENABLED(EEPROM_SETTINGS)
          ACTION_ITEM(MSG_STORE_EEPROM, ui.store_settings);
        #endif  
      }
    #else
      // Change filament
      #if E_STEPPERS == 1
        FSTR_P const fmsg = GET_TEXT_F(MSG_FILAMENTCHANGE);
        if (thermalManager.targetTooColdToExtrude(active_extruder))
          SUBMENU_F(fmsg, []{ _menu_temp_filament_op(PAUSE_MODE_CHANGE_FILAMENT, 0); });
        else
          GCODES_ITEM_F(fmsg, F("M600 B0"));
      #else
        FSTR_P const fmsg = GET_TEXT_F(MSG_FILAMENTCHANGE_E);
        LOOP_L_N(s, E_STEPPERS) {
          if (thermalManager.targetTooColdToExtrude(s))
            SUBMENU_N_F(s, fmsg, []{ _menu_temp_filament_op(PAUSE_MODE_CHANGE_FILAMENT, MenuItemBase::itemIndex); });
          else {
            ACTION_ITEM_N_F(s, fmsg, []{
              PGM_P const cmdpstr = PSTR("M600 B0 T%i");
              char cmd[strlen_P(cmdpstr) + 3 + 1];
              sprintf_P(cmd, cmdpstr, int(MenuItemBase::itemIndex));
              queue.inject(cmd);
            });
          }
        }
      #endif

      #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
        if (!is_busy) {
          // Load filament
          #if E_STEPPERS == 1
            FSTR_P const msg_load = GET_TEXT_F(MSG_FILAMENTLOAD);
            if (thermalManager.targetTooColdToExtrude(active_extruder))
              SUBMENU_F(msg_load, []{ _menu_temp_filament_op(PAUSE_MODE_LOAD_FILAMENT, 0); });
            else
              GCODES_ITEM_F(msg_load, F("M701"));
          #else
            FSTR_P const msg_load = GET_TEXT_F(MSG_FILAMENTLOAD_E);
            LOOP_L_N(s, E_STEPPERS) {
              if (thermalManager.targetTooColdToExtrude(s))
                SUBMENU_N_F(s, msg_load, []{ _menu_temp_filament_op(PAUSE_MODE_LOAD_FILAMENT, MenuItemBase::itemIndex); });
              else {
                ACTION_ITEM_N_F(s, msg_load, []{
                  char cmd[12];
                  sprintf_P(cmd, PSTR("M701 T%i"), int(MenuItemBase::itemIndex));
                  queue.inject(cmd);
                });
              }
            }
          #endif

          // Unload filament
          #if E_STEPPERS == 1
            FSTR_P const msg_unload = GET_TEXT_F(MSG_FILAMENTUNLOAD);
            #if DISABLED(PRODMACH)
              if (thermalManager.targetTooColdToExtrude(active_extruder))
                SUBMENU_F(msg_unload, []{ _menu_temp_filament_op(PAUSE_MODE_UNLOAD_FILAMENT, 0); });
              else
            #endif
                GCODES_ITEM_F(msg_unload, F("M702"));
          #else
            #if ENABLED(FILAMENT_UNLOAD_ALL_EXTRUDERS)
              if (too_cold)
                SUBMENU(MSG_FILAMENTUNLOAD_ALL, []{ _menu_temp_filament_op(PAUSE_MODE_UNLOAD_FILAMENT, -1); });
              else
                GCODES_ITEM(MSG_FILAMENTUNLOAD_ALL, F("M702"));
            #endif
            FSTR_P const msg_unload = GET_TEXT_F(MSG_FILAMENTUNLOAD_E);
            LOOP_L_N(s, E_STEPPERS) {
              if (thermalManager.targetTooColdToExtrude(s))
                SUBMENU_N_F(s, msg_unload, []{ _menu_temp_filament_op(PAUSE_MODE_UNLOAD_FILAMENT, MenuItemBase::itemIndex); });
              else {
                ACTION_ITEM_N_F(s, msg_unload, []{
                  char cmd[12];
                  sprintf_P(cmd, PSTR("M702 T%i"), int(MenuItemBase::itemIndex));
                  queue.inject(cmd);
                });
              }
            }
          #endif
        } // printer_busy
      #endif
    #endif

    END_MENU();
  #else

    if (thermalManager.targetHotEnoughToExtrude(active_extruder))
      queue.inject(F("M600B0"));
    else
      ui.goto_screen([]{ _menu_temp_filament_op(PAUSE_MODE_CHANGE_FILAMENT, 0); });

  #endif
}

static uint8_t hotend_status_extruder = 0;

static FSTR_P pause_header() {
  switch (pause_mode) {
    case PAUSE_MODE_CHANGE_FILAMENT:  return GET_TEXT_F(MSG_FILAMENT_CHANGE_HEADER);
    case PAUSE_MODE_LOAD_FILAMENT:    return GET_TEXT_F(MSG_FILAMENT_CHANGE_HEADER_LOAD);
    case PAUSE_MODE_UNLOAD_FILAMENT:  return GET_TEXT_F(MSG_FILAMENT_CHANGE_HEADER_UNLOAD);
    default: break;
  }
  return GET_TEXT_F(MSG_FILAMENT_CHANGE_HEADER_PAUSE);
}

// Portions from STATIC_ITEM...
#define HOTEND_STATUS_ITEM() do { \
  if (_menuLineNr == _thisItemNr) { \
    if (ui.should_draw()) { \
      IF_DISABLED(HAS_GRAPHICAL_TFT, MenuItem_static::draw(_lcdLineNr, GET_TEXT_F(MSG_FILAMENT_CHANGE_NOZZLE), SS_INVERT)); \
      ui.draw_hotend_status(_lcdLineNr, hotend_status_extruder); \
    } \
    if (_skipStatic && encoderLine <= _thisItemNr) { \
      ui.encoderPosition += ENCODER_STEPS_PER_MENU_ITEM; \
      ++encoderLine; \
    } \
    ui.refresh(LCDVIEW_CALL_REDRAW_NEXT); \
  } \
  ++_thisItemNr; \
}while(0)

void menu_pause_option() {
  START_MENU();
  #if LCD_HEIGHT > 2
    STATIC_ITEM(MSG_FILAMENT_CHANGE_OPTION_HEADER);
  #endif
  ACTION_ITEM(MSG_FILAMENT_CHANGE_OPTION_PURGE, []{ pause_menu_response = PAUSE_RESPONSE_EXTRUDE_MORE; });

  #if HAS_FILAMENT_SENSOR
    const bool still_out = runout.filament_ran_out;
    if (still_out)
      EDIT_ITEM(bool, MSG_RUNOUT_SENSOR, &runout.enabled, runout.reset);
  #else
    constexpr bool still_out = false;
  #endif

  if (!still_out)
    ACTION_ITEM(MSG_FILAMENT_CHANGE_OPTION_RESUME, []{ pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT; });

  END_MENU();
}

//
// ADVANCED_PAUSE_FEATURE message screens
//
// Warning: fmsg must have three null bytes to delimit lines!
//
void _lcd_pause_message(FSTR_P const fmsg) {
  PGM_P const msg1 = FTOP(fmsg);
  PGM_P const msg2 = msg1 + strlen_P(msg1) + 1;
  PGM_P const msg3 = msg2 + strlen_P(msg2) + 1;
  const bool has2 = msg2[0], has3 = msg3[0],
             skip1 = !has2 && (LCD_HEIGHT) >= 5;

  START_SCREEN();
  STATIC_ITEM_F(pause_header(), SS_DEFAULT|SS_INVERT);          // 1: Header
  if (skip1) SKIP_ITEM();                                       // Move a single-line message down
  STATIC_ITEM_F(FPSTR(msg1));                                   // 2: Message Line 1
  if (has2) STATIC_ITEM_F(FPSTR(msg2));                         // 3: Message Line 2
  if (has3 && (LCD_HEIGHT) >= 5) STATIC_ITEM_F(FPSTR(msg3));    // 4: Message Line 3 (if LCD has 5 lines)
  if (skip1 + 1 + has2 + has3 < (LCD_HEIGHT) - 2) SKIP_ITEM();  // Push Hotend Status down, if needed
  HOTEND_STATUS_ITEM();                                         // 5: Hotend Status
  END_SCREEN();
}

void lcd_pause_parking_message()  { _lcd_pause_message(GET_TEXT_F(MSG_PAUSE_PRINT_PARKING));     }
void lcd_pause_changing_message() { _lcd_pause_message(GET_TEXT_F(MSG_FILAMENT_CHANGE_INIT));    }
void lcd_pause_unload_message()   { _lcd_pause_message(GET_TEXT_F(MSG_FILAMENT_CHANGE_UNLOAD));  }
void lcd_pause_heating_message()  { _lcd_pause_message(GET_TEXT_F(MSG_FILAMENT_CHANGE_HEATING)); }
void lcd_pause_heat_message()     { _lcd_pause_message(GET_TEXT_F(MSG_FILAMENT_CHANGE_HEAT));    }
void lcd_pause_insert_message()   { _lcd_pause_message(GET_TEXT_F(MSG_FILAMENT_CHANGE_INSERT));  }
void lcd_pause_load_message()     { _lcd_pause_message(GET_TEXT_F(MSG_FILAMENT_CHANGE_LOAD));    }
void lcd_pause_waiting_message()  { _lcd_pause_message(GET_TEXT_F(MSG_ADVANCED_PAUSE_WAITING));  }
void lcd_pause_resume_message()   { _lcd_pause_message(GET_TEXT_F(MSG_FILAMENT_CHANGE_RESUME));  }

void lcd_pause_purge_message() {
  #if ENABLED(ADVANCED_PAUSE_CONTINUOUS_PURGE)
    _lcd_pause_message(GET_TEXT_F(MSG_FILAMENT_CHANGE_CONT_PURGE));
  #else
    _lcd_pause_message(GET_TEXT_F(MSG_FILAMENT_CHANGE_PURGE));
  #endif
}

FORCE_INLINE screenFunc_t ap_message_screen(const PauseMessage message) {
  switch (message) {
    case PAUSE_MESSAGE_PARKING:  return lcd_pause_parking_message;
    case PAUSE_MESSAGE_CHANGING: return lcd_pause_changing_message;
    case PAUSE_MESSAGE_UNLOAD:   return lcd_pause_unload_message;
    case PAUSE_MESSAGE_WAITING:  return lcd_pause_waiting_message;
    case PAUSE_MESSAGE_INSERT:   return lcd_pause_insert_message;
    case PAUSE_MESSAGE_LOAD:     return lcd_pause_load_message;
    case PAUSE_MESSAGE_PURGE:    return lcd_pause_purge_message;
    case PAUSE_MESSAGE_RESUME:   return lcd_pause_resume_message;
    case PAUSE_MESSAGE_HEAT:     return lcd_pause_heat_message;
    case PAUSE_MESSAGE_HEATING:  return lcd_pause_heating_message;
    case PAUSE_MESSAGE_OPTION:   pause_menu_response = PAUSE_RESPONSE_WAIT_FOR;
                                 return menu_pause_option;
    case PAUSE_MESSAGE_STATUS:
    default: break;
  }
  return nullptr;
}

void MarlinUI::pause_show_message(
  const PauseMessage message,
  const PauseMode mode/*=PAUSE_MODE_SAME*/,
  const uint8_t extruder/*=active_extruder*/
) {
  if (mode != PAUSE_MODE_SAME) pause_mode = mode;
  hotend_status_extruder = extruder;
  const screenFunc_t next_screen = ap_message_screen(message);
  if (next_screen) {
    ui.defer_status_screen();
    ui.goto_screen(next_screen);
  }
  else
    ui.return_to_status();
}

#endif // HAS_MARLINUI_MENU && ADVANCED_PAUSE_FEATURE
