//
//  test_config.h
//  pcr
//
//  Created by Matthew Fonken on 12/16/18.
//  Copyright © 2018 Matthew Fonken. All rights reserved.
//

#ifndef test_config_h
#define test_config_h

//#define MAIN_FPS                        10//50
//#define TAU_FPS                         MAIN_FPS
//#define COMBINE_FPS                     10 /// NOTE: BNO055 max rate is 200
//
///* GLOBAL SETUP */
////#define __CAM__
////#define HAS_FILE
//#define HAS_GENERATOR
//#ifndef __RHO__
//#define CV_TRACK_BLOBS
//#endif
//
////#define AUTOMATION_RUN
//#define AUTOMATION_SIZES                { 100, 250, 500, 750, 1000, 1250, 1500, 1750, 2000 }
//#define AUTOMATION_INSTRUCTIONS         { 's' };//, 's', 's' }
//#define AUTOMATION_END_AFTER_INSTRUCTIONS
//
//#define IMAGE_SET 0
//#define IMAGE_SINGLE 1
//
//#define IMAGE_TICKS                     (90)
//#define THRESH_IMAGE
//#ifndef __CAM__
////#define ROTATE_IMAGE
//#endif
//
//#define TITLE_STRING                    "Combine Alpha v1.3"
//#define GROUP_NAME                      "/gradient/"// "/misc/""/frames/star/"
//#define FILE_NAME                       "gradient_centralized"//"double_circle_fade""triple_circle_fade"//"double_square"
////#define GROUP_NAME                      "/frames/small/"
////#define FILE_NAME                       "1"
//#define FILE_TYPE                       ".png"
//#define IMAGE_TYPE                      IMAGE_SINGLE//IMAGE_SET
//
//#if IMAGE_TYPE == IMAGE_SINGLE
//#define FRAME_IMAGE_IMAGE_SOURCE_PATH   GROUP_NAME "/" FILE_NAME FILE_TYPE
//#define FRAME_IMAGE_SOURCE_NUM_FRAMES   IMAGE_TICKS
//#elif IMAGE_TYPE == IMAGE_SET
//#define FRAME_IMAGE_IMAGE_SOURCE_PATH   GROUP_NAME
//#define FRAME_IMAGE_SOURCE_NUM_FRAMES   15//26//31
//#endif
//
////#define PRINT_TUNING_STAGES
//#define THRESH_FRAME_PRINT_STEP         30
//
//#ifdef AUTOMATION_RUN
//#define FRAME_WIDTH                     50
//#define FRAME_HEIGHT                    FRAME_WIDTH
//#else
//#ifdef __CAM__
//#define FRAME_WIDTH                     (1920>>1)
//#define FRAME_HEIGHT                    (1080>>1)
//#else
//#define FRAME_WIDTH                     700
//#define FRAME_HEIGHT                    FRAME_WIDTH
//#endif
//#endif
//
///* IMAGE PROCESSING */
#define FILE_ROOT                       "/Users/matthew/Desktop/"
#define FRAME_IMAGE_ROOT                FILE_ROOT "PersonalResources/TestImages/"
//#define FRAME_SAVE_ROOT                 FILE_ROOT "PCRImages/"
//
////#define sleep(X) usleep(X*1E6)
//#define KEY_DELAY (1E3/MAIN_FPS)
//
//#define TX_FILENAME                     FILE_ROOT "psm_perf/p_data.csv"
//#define X_DIM_FILENAME                  FILE_ROOT "psm_perf/x_data.csv"
//#define Y_DIM_FILENAME                  FILE_ROOT "psm_perf/y_data.csv"
//
//#define STR_HELPER(x) #x
//#define STR(x) STR_HELPER(x)
//#define PERF_FILENAME                   FILE_ROOT "psm_perf/perf_data.csv"
//
//#define DASHBOARD_WIDTH                 10000
//#define DASHBOARD_HEIGHT                FRAME_HEIGHT

#endif /* test_config_h */
