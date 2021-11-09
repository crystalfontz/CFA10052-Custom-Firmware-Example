/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 *
 * Crystalfontz CFA10052 (hardware v1.1 onwards) example/base firmware.
 *
 * For more information about this CFA10052 example custom firmware package,
 * please see the README.md file.
 *
 ******************************************************************************
 *
 * Crystalfontz supplied source-code is provided using The Unlicense.
 * A license with no conditions whatsoever which dedicates works to the public
 * domain. Unlicensed works, modifications, and larger works may be distributed
 * under different terms and without source code.
 * See the UNLICENCE file, or https://unlicense.org/ for details.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
#pragma once

// Define the pixel per byte, line, and column attributes of the LCD panel connected to the ST7529 controller
#define LCD_EXT_SET_0               (uint8_t)0x0030
#define LCD_EXT_SET_1               (uint8_t)0x0031

// EXT = 0 Commands
#define LCD_DISPLAY_ON              (uint8_t)0x00af
#define LCD_DISPLAY_OFF             (uint8_t)0x00ae
#define LCD_DISPLAY_NORMAL          (uint8_t)0x00a6
#define LCD_DISPLAY_INVERT          (uint8_t)0x00a7
#define LCD_COM_SCAN_DIR            (uint8_t)0x00bb
#define LCD_DISPLAY_CONTROL         (uint8_t)0x00ca
#define LCD_SLEEP_IN                (uint8_t)0x0095
#define LCD_SLEEP_OUT               (uint8_t)0x0094
#define LCD_LINE_ADDR_SET           (uint8_t)0x0075
#define LCD_COL_ADDR_SET            (uint8_t)0x0015
#define LCD_DATA_SCAN_DIR           (uint8_t)0x00bc
#define LCD_MEM_WRITE               (uint8_t)0x005c
#define LCD_MEM_READ                (uint8_t)0x005d
#define LCD_PARTIAL_DISPLAY_IN      (uint8_t)0x00a8
#define LCD_PARTIAL_DISPLAY_OUT     (uint8_t)0x00a9
#define LCD_READ_MODIFY_WRITE       (uint8_t)0x00e0
#define LCD_READ_MODIFY_WRITE_END   (uint8_t)0x00ee
#define LCD_AREA_SCROLL_SET         (uint8_t)0x00aa
#define LCD_SCROLL_START_SET        (uint8_t)0x00ab
#define LCD_INTERNAL_OSC_ON         (uint8_t)0x00d1
#define LCD_INTERNAL_OSC_OFF        (uint8_t)0x00d2
#define LCD_POWER_CONTROL           (uint8_t)0x0020
#define LCD_CONTRAST_CONTROL        (uint8_t)0x0081
#define LCD_CONTRAST_INCREASE       (uint8_t)0x00d6
#define LCD_CONTRAST_DECREASE       (uint8_t)0x00d7
#define LCD_READ_REGISTER_1         (uint8_t)0x007c
#define LCD_READ_REGISTER_2         (uint8_t)0x007d
#define LCD_NOP                     (uint8_t)0x0025
#define LCD_INITIAL_CODE            (uint8_t)0x0007

// EXT = 1 Commands
#define LCD_FRAME_1_GRAY_PWM_SET    (uint8_t)0x0020
#define LCD_FRAME_2_GRAY_PWM_SET    (uint8_t)0x0021
#define LCD_ANALOG_CIRCUIT_SET      (uint8_t)0x0032
#define LCD_SOFTWARE_INITIAL        (uint8_t)0x0034
#define LCD_CONTROL_EEPROM          (uint8_t)0x00cd
#define LCD_CANCEL_EEPROM           (uint8_t)0x00cc
#define LCD_WRITE_EEPROM            (uint8_t)0x00fc
#define LCD_READ_EEPROM             (uint8_t)0x00fd

// LCD_CONTRAST_CONTROL command parameter byte 1, 2, and Minimum value
#define LCD_CONTRAST_LSB        	0x1a
#define LCD_CONTRAST_MSB        	0x04

// LCD_ANALOG_CIRCUIT_SET Command parameter byte 1
#define OSC_FREQ_ADJ_12_7_KHZ   	0b000
#define OSC_FREQ_ADJ_13_2_KHZ   	0b100
#define OSC_FREQ_ADJ_14_3_KHZ   	0b010
#define OSC_FREQ_ADJ_15_7_KHZ   	0b110
#define OSC_FREQ_ADJ_17_3_KHZ   	0b001
#define OSC_FREQ_ADJ_19_3_KHZ   	0b101
#define OSC_FREQ_ADJ_21_9_KHZ   	0b011
#define OSC_FREQ_ADJ_25_4_KHZ   	0b111

// LCD_ANALOG_CIRCUIT_SET Command parameter byte 2
#define BOOSTER_FREQ_SET_3_KHZ  	0b00
#define BOOSTER_FREQ_SET_6_KHZ  	0b01
#define BOOSTER_FREQ_SET_12_KHZ		0b10
#define BOOSTER_FREQ_SET_24_KHZ		0b11

// LCD_ANALOG_CIRCUIT_SET Command parameter byte 3
#define LCD_BIAS_RATIO_1_14		0b000
#define LCD_BIAS_RATIO_1_13     0b001
#define LCD_BIAS_RATIO_1_12     0b010
#define LCD_BIAS_RATIO_1_11     0b011
#define LCD_BIAS_RATIO_1_10     0b100
#define LCD_BIAS_RATIO_1_9      0b101
#define LCD_BIAS_RATIO_1_7      0b110
#define LCD_BIAS_RATIO_1_5      0b111

// LCD_COM_SCAN_DIR Command parameter byte 1
#define COM_0_79__80_159     0       // 0 -> 79, 80 -> 159
#define COM_0_79__159_80     1       // 0 -> 79, 159 -> 80
#define COM_79_0__80_159     2       // 79 -> 0, 80 -> 159
#define COM_79_0__159_80     3       // 79 -> 0, 159 -> 80

#define COM_SCAN_DIR_PB1_CD_LSB                                     0
#define COM_SCAN_DIR_PB1_CD_FIELD_LEN                               3
#define COM_SCAN_DIR_PB1_CD_MASK                                    (((1 << COM_SCAN_DIR_PB1_CD_FIELD_LEN) - 1) << COM_SCAN_DIR_PB1_CD_LSB)
#define COM_SCAN_DIR_PB1_CD_GET(COM_SCAN_DIR_PB1_WORD)              (((COM_SCAN_DIR_PB1_WORD) & COM_SCAN_DIR_PB1_CD_MASK) >> COM_SCAN_DIR_PB1_CD_LSB)
#define COM_SCAN_DIR_PB1_CD_PUT(COM_SCAN_DIR_PB1_WORD)              (((COM_SCAN_DIR_PB1_WORD) << COM_SCAN_DIR_PB1_CD_LSB) & COM_SCAN_DIR_PB1_CD_MASK)

// LCD_DSIPLAY_CONTROL Command Parameter Bytes 1, 2, & 3
#define CLD_NO_DIVIDE   0
#define CLD_DIVIDE_2    1
#define DT              ((LCD_LINES / 4) - 1)
#define LF              15
#define FI              1

#define DISPLAY_CONTROL_PB1_CLD_LSB                                 2
#define DISPLAY_CONTROL_PB1_CLD_FIELD_LEN                           1
#define DISPLAY_CONTROL_PB1_CLD_MASK                                (((1 << DISPLAY_CONTROL_PB1_CLD_FIELD_LEN) - 1) << DISPLAY_CONTROL_PB1_CLD_LSB)
#define DISPLAY_CONTROL_PB1_CLD_GET(DISPLAY_CONTROL_PB1_WORD)       (((DISPLAY_CONTROL_PB1_WORD) & DISPLAY_CONTROL_PB1_CLD_MASK) >> DISPLAY_CONTROL_PB1_CLD_LSB)
#define DISPLAY_CONTROL_PB1_CLD_PUT(DISPLAY_CONTROL_PB1_WORD)       (((DISPLAY_CONTROL_PB1_WORD) << DISPLAY_CONTROL_PB1_CLD_LSB) & DISPLAY_CONTROL_PB1_CLD_MASK)

#define DISPLAY_CONTROL_PB2_DT_LSB                                  0
#define DISPLAY_CONTROL_PB2_DT_FIELD_LEN                            6
#define DISPLAY_CONTROL_PB2_DT_MASK                                 (((1 << DISPLAY_CONTROL_PB2_DT_FIELD_LEN) - 1) << DISPLAY_CONTROL_PB2_DT_LSB)
#define DISPLAY_CONTROL_PB2_DT_GET(DISPLAY_CONTROL_PB2_WORD)        (((DISPLAY_CONTROL_PB2_WORD) & DISPLAY_CONTROL_PB2_DT_MASK) >> DISPLAY_CONTROL_PB2_DT_LSB)
#define DISPLAY_CONTROL_PB2_DT_PUT(DISPLAY_CONTROL_PB2_WORD)        (((DISPLAY_CONTROL_PB2_WORD) << DISPLAY_CONTROL_PB2_DT_LSB) & DISPLAY_CONTROL_PB2_DT_MASK)

#define DISPLAY_CONTROL_PB3_LF_LSB                                  0
#define DISPLAY_CONTROL_PB3_LF_FIELD_LEN                            4
#define DISPLAY_CONTROL_PB3_LF_MASK                                 (((1 << DISPLAY_CONTROL_PB3_LF_FIELD_LEN) - 1) << DISPLAY_CONTROL_PB3_LF_LSB)
#define DISPLAY_CONTROL_PB3_LF_GET(DISPLAY_CONTROL_PB3_WORD)        (((DISPLAY_CONTROL_PB3_WORD) & DISPLAY_CONTROL_PB3_LF_MASK) >> DISPLAY_CONTROL_PB3_LF_LSB)
#define DISPLAY_CONTROL_PB3_LF_PUT(DISPLAY_CONTROL_PB3_WORD)        (((DISPLAY_CONTROL_PB3_WORD) << DISPLAY_CONTROL_PB3_LF_LSB) & DISPLAY_CONTROL_PB3_LF_MASK)

#define DISPLAY_CONTROL_PB3_FI_LSB                                  4
#define DISPLAY_CONTROL_PB3_FI_FIELD_LEN                            1
#define DISPLAY_CONTROL_PB3_FI_MASK                                 (((1 << DISPLAY_CONTROL_PB3_FI_FIELD_LEN) - 1) << DISPLAY_CONTROL_PB3_FI_LSB)
#define DISPLAY_CONTROL_PB3_FI_GET(DISPLAY_CONTROL_PB3_WORD)        (((DISPLAY_CONTROL_PB3_WORD) & DISPLAY_CONTROL_PB3_FI_MASK) >> DISPLAY_CONTROL_PB3_FI_LSB)
#define DISPLAY_CONTROL_PB3_FI_PUT(DISPLAY_CONTROL_PB3_WORD)        (((DISPLAY_CONTROL_PB3_WORD) << DISPLAY_CONTROL_PB3_FI_LSB) & DISPLAY_CONTROL_PB3_FI_MASK)

// LCD_LINE_ADDR_SET Command Parameter Bytes 1 & 2
#define LINE_ADDR_SET_PB1_START_LINE_LSB                            0
#define LINE_ADDR_SET_PB1_START_LINE_FIELD_LEN                      8
#define LINE_ADDR_SET_PB1_START_LINE_MASK                           (((1 << LINE_ADDR_SET_PB1_START_LINE_FIELD_LEN) - 1) << LINE_ADDR_SET_PB1_START_LINE_LSB)
#define LINE_ADDR_SET_PB1_START_LINE_GET(LINE_ADDR_SET_PB1_WORD)    (((LINE_ADDR_SET_PB1_WORD) & LINE_ADDR_SET_PB1_START_LINE_MASK) >> LINE_ADDR_SET_PB1_START_LINE_LSB)
#define LINE_ADDR_SET_PB1_START_LINE_PUT(LINE_ADDR_SET_PB1_WORD)    (((LINE_ADDR_SET_PB1_WORD) << LINE_ADDR_SET_PB1_START_LINE_LSB) & LINE_ADDR_SET_PB1_START_LINE_MASK)

#define LINE_ADDR_SET_PB2_END_LINE_LSB                              0
#define LINE_ADDR_SET_PB2_END_LINE_FIELD_LEN                        8
#define LINE_ADDR_SET_PB2_END_LINE_MASK                             (((1 << LINE_ADDR_SET_PB2_END_LINE_FIELD_LEN) - 1) << LINE_ADDR_SET_PB2_END_LINE_LSB)
#define LINE_ADDR_SET_PB2_END_LINE_GET(LINE_ADDR_SET_PB2_WORD)      (((LINE_ADDR_SET_PB2_WORD) & LINE_ADDR_SET_PB2_END_LINE_MASK) >> LINE_ADDR_SET_PB2_END_LINE_LSB)
#define LINE_ADDR_SET_PB2_END_LINE_PUT(LINE_ADDR_SET_PB2_WORD)      (((LINE_ADDR_SET_PB2_WORD) << LINE_ADDR_SET_PB2_END_LINE_LSB) & LINE_ADDR_SET_PB2_END_LINE_MASK)

// LCD_COL_ADDR_SET Command parameter bytes 1 & 2
#define COL_ADDR_SET_PB1_START_COL_LSB                              0
#define COL_ADDR_SET_PB1_START_COL_FIELD_LEN                        8
#define COL_ADDR_SET_PB1_START_COL_MASK                             (((1 << COL_ADDR_SET_PB1_START_COL_FIELD_LEN) - 1) << COL_ADDR_SET_PB1_START_COL_LSB)
#define COL_ADDR_SET_PB1_START_COL_GET(COL_ADDR_SET_PB1_WORD)       (((COL_ADDR_SET_PB1_WORD) & COL_ADDR_SET_PB1_START_COL_MASK) >> COL_ADDR_SET_PB1_START_COL_LSB)
#define COL_ADDR_SET_PB1_START_COL_PUT(COL_ADDR_SET_PB1_WORD)       (((COL_ADDR_SET_PB1_WORD) << COL_ADDR_SET_PB1_START_COL_LSB) & COL_ADDR_SET_PB1_START_COL_MASK)

#define COL_ADDR_SET_PB2_END_COL_LSB                                0
#define COL_ADDR_SET_PB2_END_COL_FIELD_LEN                          8
#define COL_ADDR_SET_PB2_END_COL_MASK                               (((1 << COL_ADDR_SET_PB2_END_COL_FIELD_LEN) - 1) << COL_ADDR_SET_PB2_END_COL_LSB)
#define COL_ADDR_SET_PB2_END_COL_GET(COL_ADDR_SET_PB2_WORD)         (((COL_ADDR_SET_PB2_WORD) & COL_ADDR_SET_PB2_END_COL_MASK) >> COL_ADDR_SET_PB2_END_COL_LSB)
#define COL_ADDR_SET_PB2_END_COL_PUT(COL_ADDR_SET_PB2_WORD)         (((COL_ADDR_SET_PB2_WORD) << COL_ADDR_SET_PB2_END_COL_LSB) & COL_ADDR_SET_PB2_END_COL_MASK)

// LCD_DATA_SCAN_DIR Command parameter bytes 1, 2, & 3
#define LI_NORMAL   0
#define LI_INVERSE  1
#define CI_NORMAL   0
#define CI_INVERSE  1
#define C_L_COLUMN_DIRECTION    0
#define C_L_LINE_DIRECTION      1
#define CLR_NORMAL  0
#define CLR_REVERSE 1
#define GS_2BYTE_3PIXEL 1
#define GS_3BYTE_3PIXEL 2

#define DATA_SCAN_DIR_PB1_LI_LSB                                    0
#define DATA_SCAN_DIR_PB1_LI_FIELD_LEN                              1
#define DATA_SCAN_DIR_PB1_LI_MASK                                   (((1 << DATA_SCAN_DIR_PB1_LI_FIELD_LEN) - 1) << DATA_SCAN_DIR_PB1_LI_LSB)
#define DATA_SCAN_DIR_PB1_LI_GET(DATA_SCAN_DIR_PB1_WORD)            (((DATA_SCAN_DIR_PB1_WORD) & DATA_SCAN_DIR_PB1_LI_MASK) >> DATA_SCAN_DIR_PB1_LI_LSB)
#define DATA_SCAN_DIR_PB1_LI_PUT(DATA_SCAN_DIR_PB1_WORD)            (((DATA_SCAN_DIR_PB1_WORD) << DATA_SCAN_DIR_PB1_LI_LSB) & DATA_SCAN_DIR_PB1_LI_MASK)

#define DATA_SCAN_DIR_PB1_CI_LSB                                    1
#define DATA_SCAN_DIR_PB1_CI_FIELD_LEN                              1
#define DATA_SCAN_DIR_PB1_CI_MASK                                   (((1 << DATA_SCAN_DIR_PB1_CI_FIELD_LEN) - 1) << DATA_SCAN_DIR_PB1_CI_LSB)
#define DATA_SCAN_DIR_PB1_CI_GET(DATA_SCAN_DIR_PB1_WORD)            (((DATA_SCAN_DIR_PB1_WORD) & DATA_SCAN_DIR_PB1_CI_MASK) >> DATA_SCAN_DIR_PB1_CI_LSB)
#define DATA_SCAN_DIR_PB1_CI_PUT(DATA_SCAN_DIR_PB1_WORD)            (((DATA_SCAN_DIR_PB1_WORD) << DATA_SCAN_DIR_PB1_CI_LSB) & DATA_SCAN_DIR_PB1_CI_MASK)

#define DATA_SCAN_DIR_PB1_C_L_LSB                                   2
#define DATA_SCAN_DIR_PB1_C_L_FIELD_LEN                             1
#define DATA_SCAN_DIR_PB1_C_L_MASK                                  (((1 << DATA_SCAN_DIR_PB1_C_L_FIELD_LEN) - 1) << DATA_SCAN_DIR_PB1_C_L_LSB)
#define DATA_SCAN_DIR_PB1_C_L_GET(DATA_SCAN_DIR_PB1_WORD)           (((DATA_SCAN_DIR_PB1_WORD) & DATA_SCAN_DIR_PB1_C_L_MASK) >> DATA_SCAN_DIR_PB1_C_L_LSB)
#define DATA_SCAN_DIR_PB1_C_L_PUT(DATA_SCAN_DIR_PB1_WORD)           (((DATA_SCAN_DIR_PB1_WORD) << DATA_SCAN_DIR_PB1_C_L_LSB) & DATA_SCAN_DIR_PB1_C_L_MASK)

#define DATA_SCAN_DIR_PB2_CLR_LSB                                   0
#define DATA_SCAN_DIR_PB2_CLR_FIELD_LEN                             1
#define DATA_SCAN_DIR_PB2_CLR_MASK                                  (((1 << DATA_SCAN_DIR_PB2_CLR_FIELD_LEN) - 1) << DATA_SCAN_DIR_PB2_CLR_LSB)
#define DATA_SCAN_DIR_PB2_CLR_GET(DATA_SCAN_DIR_PB2_WORD)           (((DATA_SCAN_DIR_PB2_WORD) & DATA_SCAN_DIR_PB2_CLR_MASK) >> DATA_SCAN_DIR_PB2_CLR_LSB)
#define DATA_SCAN_DIR_PB2_CLR_PUT(DATA_SCAN_DIR_PB2_WORD)           (((DATA_SCAN_DIR_PB2_WORD) << DATA_SCAN_DIR_PB2_CLR_LSB) & DATA_SCAN_DIR_PB2_CLR_MASK)

#define DATA_SCAN_DIR_PB3_GS_LSB                                    0
#define DATA_SCAN_DIR_PB3_GS_FIELD_LEN                              3
#define DATA_SCAN_DIR_PB3_GS_MASK                                   (((1 << DATA_SCAN_DIR_PB3_GS_FIELD_LEN) - 1) << DATA_SCAN_DIR_PB3_GS_LSB)
#define DATA_SCAN_DIR_PB3_GS_GET(DATA_SCAN_DIR_PB3_WORD)            (((DATA_SCAN_DIR_PB3_WORD) & DATA_SCAN_DIR_PB3_GS_MASK) >> DATA_SCAN_DIR_PB3_GS_LSB)
#define DATA_SCAN_DIR_PB3_GS_PUT(DATA_SCAN_DIR_PB3_WORD)            (((DATA_SCAN_DIR_PB3_WORD) << DATA_SCAN_DIR_PB3_GS_LSB) & DATA_SCAN_DIR_PB3_GS_MASK)

// LCD_MEM_WRITE Command parameter byte/halfword 1
#define MEM_WRITE_PB1_DATA_LSB                                      0
#define MEM_WRITE_PB1_DATA_FIELD_LEN                                sizeof(uint8_t)
#define MEM_WRITE_PB1_DATA_MASK                                     (((1 << MEM_WRITE_PB1_DATA_FIELD_LEN) - 1) << MEM_WRITE_PB1_DATA_LSB)
#define MEM_WRITE_PB1_DATA_GET(MEM_WRITE_PB1_WORD)                  (((MEM_WRITE_PB1_WORD) & MEM_WRITE_PB1_DATA_MASK) >> MEM_WRITE_PB1_DATA_LSB)
#define MEM_WRITE_PB1_DATA_PUT(MEM_WRITE_PB1_WORD)                  (((MEM_WRITE_PB1_WORD) << MEM_WRITE_PB1_DATA_LSB) & MEM_WRITE_PB1_DATA_MASK)

// LCD_MEM_READ Command parameter byte/halfword 1
#define MEM_READ_PB1_DATA_LSB                                       0
#define MEM_READ_PB1_DATA_FIELD_LEN                                 sizeof(uint8_t)
#define MEM_READ_PB1_DATA_MASK                                      (((1 << MEM_READ_PB1_DATA_FIELD_LEN) - 1) << MEM_READ_PB1_DATA_LSB)
#define MEM_READ_PB1_DATA_GET(MEM_READ_PB1_WORD)                    (((MEM_READ_PB1_WORD) & MEM_READ_PB1_DATA_MASK) >> MEM_READ_PB1_DATA_LSB)
#define MEM_READ_PB1_DATA_PUT(MEM_READ_PB1_WORD)                    (((MEM_READ_PB1_WORD) << MEM_READ_PB1_DATA_LSB) & MEM_READ_PB1_DATA_MASK)

// LCD_PARTIAL_DISPLAY_IN Command parameter bytes 1 & 2
#define PARTIAL_DISPLAY_IN_PB1_PTS_LSB                              0
#define PARTIAL_DISPLAY_IN_PB1_PTS_FIELD_LEN                        6
#define PARTIAL_DISPLAY_IN_PB1_PTS_MASK                             (((1 << PARTIAL_DISPLAY_IN_PB1_PTS_FIELD_LEN) - 1) << PARTIAL_DISPLAY_IN_PB1_PTS_LSB)
#define PARTIAL_DISPLAY_IN_PB1_PTS_GET(PARTIAL_DISPLAY_IN_PB1_WORD) (((PARTIAL_DISPLAY_IN_PB1_WORD) & PARTIAL_DISPLAY_IN_PB1_PTS_MASK) >> PARTIAL_DISPLAY_IN_PB1_PTS_LSB)
#define PARTIAL_DISPLAY_IN_PB1_PTS_PUT(PARTIAL_DISPLAY_IN_PB1_WORD) (((PARTIAL_DISPLAY_IN_PB1_WORD) << PARTIAL_DISPLAY_IN_PB1_PTS_LSB) & PARTIAL_DISPLAY_IN_PB1_PTS_MASK)

#define PARTIAL_DISPLAY_IN_PB2_PTE_LSB                              0
#define PARTIAL_DISPLAY_IN_PB2_PTE_FIELD_LEN                        6
#define PARTIAL_DISPLAY_IN_PB2_PTE_MASK                             (((1 << PARTIAL_DISPLAY_IN_PB2_PTE_FIELD_LEN) - 1) << PARTIAL_DISPLAY_IN_PB2_PTE_LSB)
#define PARTIAL_DISPLAY_IN_PB2_PTE_GET(PARTIAL_DISPLAY_IN_PB2_WORD) (((PARTIAL_DISPLAY_IN_PB2_WORD) & PARTIAL_DISPLAY_IN_PB2_PTE_MASK) >> PARTIAL_DISPLAY_IN_PB2_PTE_LSB)
#define PARTIAL_DISPLAY_IN_PB2_PTE_PUT(PARTIAL_DISPLAY_IN_PB2_WORD) (((PARTIAL_DISPLAY_IN_PB2_WORD) << PARTIAL_DISPLAY_IN_PB2_PTE_LSB) & PARTIAL_DISPLAY_IN_PB2_PTE_MASK)

// LCD_AREA_SCROLL_SET Command parameter bytes 1, 2, 3, & 4
#define AREA_SCROLL_SET_PB1_TB_LSB                                  0
#define AREA_SCROLL_SET_PB1_TB_FIELD_LEN                            6
#define AREA_SCROLL_SET_PB1_TB_MASK                                 (((1 << AREA_SCROLL_SET_PB1_TB_FIELD_LEN) - 1) << AREA_SCROLL_SET_PB1_TB_LSB)
#define AREA_SCROLL_SET_PB1_TB_GET(AREA_SCROLL_SET_PB1_WORD)        (((AREA_SCROLL_SET_PB1_WORD) & AREA_SCROLL_SET_PB1_TB_MASK) >> AREA_SCROLL_SET_PB1_TB_LSB)
#define AREA_SCROLL_SET_PB1_TB_PUT(AREA_SCROLL_SET_PB1_WORD)        (((AREA_SCROLL_SET_PB1_WORD) << AREA_SCROLL_SET_PB1_TB_LSB) & AREA_SCROLL_SET_PB1_TB_MASK)

#define AREA_SCROLL_SET_PB2_BB_LSB                                  0
#define AREA_SCROLL_SET_PB2_BB_FIELD_LEN                            6
#define AREA_SCROLL_SET_PB2_BB_MASK                                 (((1 << AREA_SCROLL_SET_PB2_BB_FIELD_LEN) - 1) << AREA_SCROLL_SET_PB2_BB_LSB)
#define AREA_SCROLL_SET_PB2_BB_GET(AREA_SCROLL_SET_PB2_WORD)        (((AREA_SCROLL_SET_PB2_WORD) & AREA_SCROLL_SET_PB2_BB_MASK) >> AREA_SCROLL_SET_PB2_BB_LSB)
#define AREA_SCROLL_SET_PB2_BB_PUT(AREA_SCROLL_SET_PB2_WORD)        (((AREA_SCROLL_SET_PB2_WORD) << AREA_SCROLL_SET_PB2_BB_LSB) & AREA_SCROLL_SET_PB2_BB_MASK)

#define AREA_SCROLL_SET_PB3_NSB_LSB                                 0
#define AREA_SCROLL_SET_PB3_NSB_FIELD_LEN                           6
#define AREA_SCROLL_SET_PB3_NSB_MASK                                (((1 << AREA_SCROLL_SET_PB3_NSB_FIELD_LEN) - 1) << AREA_SCROLL_SET_PB3_NSB_LSB)
#define AREA_SCROLL_SET_PB3_NSB_GET(AREA_SCROLL_SET_PB3_WORD)       (((AREA_SCROLL_SET_PB3_WORD) & AREA_SCROLL_SET_PB3_NSB_MASK) >> AREA_SCROLL_SET_PB3_NSB_LSB)
#define AREA_SCROLL_SET_PB3_NSB_PUT(AREA_SCROLL_SET_PB3_WORD)       (((AREA_SCROLL_SET_PB3_WORD) << AREA_SCROLL_SET_PB3_NSB_LSB) & AREA_SCROLL_SET_PB3_NSB_MASK)

#define AREA_SCROLL_SET_PB4_SCM_LSB                                 0
#define AREA_SCROLL_SET_PB4_SCM_FIELD_LEN                           2
#define AREA_SCROLL_SET_PB4_SCM_MASK                                (((1 << AREA_SCROLL_SET_PB4_SCM_FIELD_LEN) - 1) << AREA_SCROLL_SET_PB4_SCM_LSB)
#define AREA_SCROLL_SET_PB4_SCM_GET(AREA_SCROLL_SET_PB4_WORD)       (((AREA_SCROLL_SET_PB4_WORD) & AREA_SCROLL_SET_PB4_SCM_MASK) >> AREA_SCROLL_SET_PB4_SCM_LSB)
#define AREA_SCROLL_SET_PB4_SCM_PUT(AREA_SCROLL_SET_PB4_WORD)       (((AREA_SCROLL_SET_PB4_WORD) << AREA_SCROLL_SET_PB4_SCM_LSB) & AREA_SCROLL_SET_PB4_SCM_MASK)

// LCD_START_SCROLL_SET Command parameter byte 1
#define START_SCROLL_SET_PB1_SB_LSB                                 0
#define START_SCROLL_SET_PB1_SB_FIELD_LEN                           6
#define START_SCROLL_SET_PB1_SB_MASK                                (((1 << START_SCROLL_SET_PB1_SB_FIELD_LEN) - 1) << START_SCROLL_SET_PB1_SB_LSB)
#define START_SCROLL_SET_PB1_SB_GET(START_SCROLL_SET_PB1_WORD)      (((START_SCROLL_SET_PB1_WORD) & START_SCROLL_SET_PB1_SB_MASK) >> START_SCROLL_SET_PB1_SB_LSB)
#define START_SCROLL_SET_PB1_SB_PUT(START_SCROLL_SET_PB1_WORD)      (((START_SCROLL_SET_PB1_WORD) << START_SCROLL_SET_PB1_SB_LSB) & START_SCROLL_SET_PB1_SB_MASK)

// LCD_POWER_CONTROL Command parameter byte 1
#define VR_OFF  0   // Voltage reference generator
#define VR_ON   1
#define VF_OFF  0   // Voltage follower
#define VF_ON   1
#define VB_OFF  0   // Voltage boost
#define VB_ON   1

#define POWER_CONTROL_PB1_VR_LSB                                    0
#define POWER_CONTROL_PB1_VR_FIELD_LEN                              1
#define POWER_CONTROL_PB1_VR_MASK                                   (((1 << POWER_CONTROL_PB1_VR_FIELD_LEN) - 1) << POWER_CONTROL_PB1_VR_LSB)
#define POWER_CONTROL_PB1_VR_GET(POWER_CONTROL_PB1_WORD)            (((POWER_CONTROL_PB1_WORD) & POWER_CONTROL_PB1_VR_MASK) >> POWER_CONTROL_PB1_VR_LSB)
#define POWER_CONTROL_PB1_VR_PUT(POWER_CONTROL_PB1_WORD)            (((POWER_CONTROL_PB1_WORD) << POWER_CONTROL_PB1_VR_LSB) & POWER_CONTROL_PB1_VR_MASK)

#define POWER_CONTROL_PB1_VF_LSB                                    1
#define POWER_CONTROL_PB1_VF_FIELD_LEN                              1
#define POWER_CONTROL_PB1_VF_MASK                                   (((1 << POWER_CONTROL_PB1_VF_FIELD_LEN) - 1) << POWER_CONTROL_PB1_VF_LSB)
#define POWER_CONTROL_PB1_VF_GET(POWER_CONTROL_PB1_WORD)            (((POWER_CONTROL_PB1_WORD) & POWER_CONTROL_PB1_VF_MASK) >> POWER_CONTROL_PB1_VF_LSB)
#define POWER_CONTROL_PB1_VF_PUT(POWER_CONTROL_PB1_WORD)            (((POWER_CONTROL_PB1_WORD) << POWER_CONTROL_PB1_VF_LSB) & POWER_CONTROL_PB1_VF_MASK)

#define POWER_CONTROL_PB1_VB_LSB                                    3
#define POWER_CONTROL_PB1_VB_FIELD_LEN                              1
#define POWER_CONTROL_PB1_VB_MASK                                   (((1 << POWER_CONTROL_PB1_VB_FIELD_LEN) - 1) << POWER_CONTROL_PB1_VB_LSB)
#define POWER_CONTROL_PB1_VB_GET(POWER_CONTROL_PB1_WORD)            (((POWER_CONTROL_PB1_WORD) & POWER_CONTROL_PB1_VB_MASK) >> POWER_CONTROL_PB1_VB_LSB)
#define POWER_CONTROL_PB1_VB_PUT(POWER_CONTROL_PB1_WORD)            (((POWER_CONTROL_PB1_WORD) << POWER_CONTROL_PB1_VB_LSB) & POWER_CONTROL_PB1_VB_MASK)

// LCD_CONTRAST_CONTROL Command parameter bytes 1 & 2
#define CONTRAST_CONTROL_PB1_VPR_5_0_LSB                            0
#define CONTRAST_CONTROL_PB1_VPR_5_0_FIELD_LEN                      6
#define CONTRAST_CONTROL_PB1_VPR_5_0_MASK                           (((1 << CONTRAST_CONTROL_PB1_VPR_5_0_FIELD_LEN) - 1) << CONTRAST_CONTROL_PB1_VPR_5_0_LSB)
#define CONTRAST_CONTROL_PB1_VPR_5_0_GET(CONTRAST_CONTROL_PB1_WORD) (((CONTRAST_CONTROL_PB1_WORD) & CONTRAST_CONTROL_PB1_VPR_5_0_MASK) >> CONTRAST_CONTROL_PB1_VPR_5_0_LSB)
#define CONTRAST_CONTROL_PB1_VPR_5_0_PUT(CONTRAST_CONTROL_PB1_WORD) (((CONTRAST_CONTROL_PB1_WORD) << CONTRAST_CONTROL_PB1_VPR_5_0_LSB) & CONTRAST_CONTROL_PB1_VPR_5_0_MASK)

#define CONTRAST_CONTROL_PB2_VPR_8_6_LSB                            0
#define CONTRAST_CONTROL_PB2_VPR_8_6_FIELD_LEN                      3
#define CONTRAST_CONTROL_PB2_VPR_8_6_MASK                           (((1 << CONTRAST_CONTROL_PB2_VPR_8_6_FIELD_LEN) - 1) << CONTRAST_CONTROL_PB2_VPR_8_6_LSB)
#define CONTRAST_CONTROL_PB2_VPR_8_6_GET(CONTRAST_CONTROL_PB2_WORD) (((CONTRAST_CONTROL_PB2_WORD) & CONTRAST_CONTROL_PB2_VPR_8_6_MASK) >> CONTRAST_CONTROL_PB2_VPR_8_6_LSB)
#define CONTRAST_CONTROL_PB2_VPR_8_6_PUT(CONTRAST_CONTROL_PB2_WORD) (((CONTRAST_CONTROL_PB2_WORD) << CONTRAST_CONTROL_PB2_VPR_8_6_LSB) & CONTRAST_CONTROL_PB2_VPR_8_6_MASK)

