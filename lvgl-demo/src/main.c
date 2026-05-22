/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

static uint32_t count;

// Same value as lvgl-demo/boards/native_sim_64.overlay
#define DISPLAY_WIDTH  1280
#define DISPLAY_HEIGHT  720

// Reference: https://lvgl.io/docs/open/api

//=========================================
// global variable
//=========================================
/*
# can data configuration:
#     01: Speed         - 1 byte
#     02: gear          - 1 byte
#     03: handBreak     - 1 byte
#     04: fuel          - 1 byte
#     05: throttle      - 1 byte
#     06: break         - 1 byte
#     07: elappsed_time - 3 byte
*/
uint8_t speed = 0;
uint8_t gear = 0;
uint8_t breakVal = 0;
uint8_t handBreak = 0;
uint8_t fuel = 0;
uint8_t throttleVal = 0;
uint8_t elappsed_time_h = 0;
uint8_t elappsed_time_m = 0;
uint8_t elappsed_time_s = 0;

/*
 * for CAN BUS
 */
#include <zephyr/drivers/can.h>
const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
void rx_callback_function(const struct device *dev, struct can_frame *frame, void *user_data)
{
    int id = frame->id;
    switch (id) {
        case 1: speed = frame->data[0]; break;
        case 2: gear = frame->data[0]; break;
        case 3: handBreak = frame->data[0]; break;
        case 4: fuel = frame->data[0]; break;
        case 5: throttleVal = frame->data[0]; break;
        case 6: breakVal = frame->data[0];break;
        case 7: elappsed_time_s = frame->data[2];
                elappsed_time_m = frame->data[1];
                elappsed_time_h = frame->data[0];
                break;
        default: break;
    }
}
struct can_filter filter = {
    .flags = 0,
    .id = 0,
    .mask = 0,
};


void draw_frame(lv_obj_t* canvas) {
    // clear canvas
    //lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_COVER);
    lv_canvas_fill_bg(canvas, lv_color_black(), LV_OPA_TRANSP);

    static int cnt = 0;
    static int x = 0;
    /* common */
    lv_layer_t layer;

    // Speed Meter
    lv_canvas_init_layer(canvas, &layer);
    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    arc_dsc.color = lv_color_make(50, 100, 255);
    arc_dsc.center.x = 233;
    arc_dsc.center.y = 252;
    arc_dsc.width = 40;
    arc_dsc.radius = 138;
    arc_dsc.start_angle = 135;
    arc_dsc.end_angle = 135+280.0/200.0*speed;
    arc_dsc.opa = LV_OPA_50;
    lv_draw_arc(&layer, &arc_dsc);
    lv_canvas_finish_layer(canvas, &layer);

    /* Hand break */
    lv_canvas_init_layer(canvas, &layer);
    lv_draw_arc_dsc_t handbreak_arc_dsc;
    lv_draw_arc_dsc_init(&handbreak_arc_dsc);
    handbreak_arc_dsc.color = lv_color_make(255, 0, 102);
    handbreak_arc_dsc.center.x = 765;
    handbreak_arc_dsc.center.y = 283;
    handbreak_arc_dsc.width = 6;
    handbreak_arc_dsc.radius = 20;
    handbreak_arc_dsc.start_angle = 0;
    handbreak_arc_dsc.end_angle = 360;
    handbreak_arc_dsc.opa = LV_OPA_20;
    if (handBreak)
        handbreak_arc_dsc.opa = LV_OPA_100;
    lv_draw_arc(&layer, &handbreak_arc_dsc);

    handbreak_arc_dsc.radius = 28;
    handbreak_arc_dsc.width = 4;

    handbreak_arc_dsc.start_angle = 300;
    handbreak_arc_dsc.end_angle = 420;
    lv_draw_arc(&layer, &handbreak_arc_dsc);

    handbreak_arc_dsc.start_angle = 120;
    handbreak_arc_dsc.end_angle = 240;
    lv_draw_arc(&layer, &handbreak_arc_dsc);
    // Draw ! in the circle
    lv_draw_rect_dsc_t handbreak_rect_dsc;
    lv_draw_rect_dsc_init(&handbreak_rect_dsc);
    handbreak_rect_dsc.bg_color = lv_color_make(255, 0, 102);
    handbreak_rect_dsc.bg_opa = LV_OPA_20;
    if (handBreak)
        handbreak_rect_dsc.bg_opa = LV_OPA_100;
    lv_area_t handbreak_coords = {763, 275, 766, 287};
    lv_draw_rect(&layer, &handbreak_rect_dsc, &handbreak_coords);
    handbreak_coords.y1 = 290;
    handbreak_coords.y2 = 293;
    lv_draw_rect(&layer, &handbreak_rect_dsc, &handbreak_coords);

    lv_canvas_finish_layer(canvas, &layer);

    /* Fuel gauge */
#if 0
    lv_canvas_init_layer(canvas, &layer);
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = lv_color_make(50, 100, 255);
    rect_dsc.bg_opa = LV_OPA_50;
    lv_area_t gauge_coords = {1070, (315-2*(fuel)), 1140, 315};  // {1050, 115, 1140, 315}
    lv_draw_rect(&layer, &rect_dsc, &gauge_coords);
    lv_canvas_finish_layer(canvas, &layer);
#endif
    lv_canvas_init_layer(canvas, &layer);
    lv_draw_triangle_dsc_t fuel_dsc;
    lv_draw_triangle_dsc_init(&fuel_dsc);
    fuel_dsc.color = lv_color_make(50,100,255);
    fuel_dsc.opa = LV_OPA_50;
    fuel_dsc.p[0].x = 1073;
    fuel_dsc.p[0].y = 315;
    fuel_dsc.p[1].x = 1110;
    fuel_dsc.p[1].y = 315;
    fuel_dsc.p[2].x = 1110 + (36 * fuel / 100);
    fuel_dsc.p[2].y = 315-2*(fuel);
    lv_draw_triangle(&layer, &fuel_dsc);
    fuel_dsc.p[1].x = 1073 + (36 * fuel / 100);
    fuel_dsc.p[1].y = 315-2*(fuel);
    lv_draw_triangle(&layer, &fuel_dsc);
    // cursor
    fuel_dsc.p[0].x = 1073 + (36 * fuel / 100) -30;
    fuel_dsc.p[0].y = 315-2*(fuel) - 15;
    fuel_dsc.p[1].x = 1073 + (36 * fuel / 100) -5;
    fuel_dsc.p[2].x = 1073 + (36 * fuel / 100) -30;
    fuel_dsc.p[2].y = 315-2*(fuel) + 15;
    lv_draw_triangle(&layer, &fuel_dsc);
    // text
    lv_draw_label_dsc_t fuel_label_dsc;
    lv_draw_label_dsc_init(&fuel_label_dsc);
    fuel_label_dsc.color = lv_color_make(255, 255, 255);
    char fuel_str[6] = {0};
    sprintf(fuel_str, "%3d%%", fuel);
    fuel_label_dsc.text = fuel_str;
    fuel_label_dsc.font = &lv_font_montserrat_32;
    lv_area_t coords_fuel;
    coords_fuel.x1 = fuel_dsc.p[0].x-40;
    coords_fuel.y1 = fuel_dsc.p[0].y-30;
    coords_fuel.x2 = fuel_dsc.p[0].x+40;
    coords_fuel.y2 = fuel_dsc.p[0].y+30;
    lv_draw_label(&layer, &fuel_label_dsc, &coords_fuel);
    lv_canvas_finish_layer(canvas, &layer);

    /* Break gauge */
    const uint16_t break_gauge_points[5][4][2] = {
        { {454, 323}, {535, 323}, {528, 331}, {454, 331}, },
        { {454, 336}, {523, 336}, {518, 346}, {454, 346}, },
        { {454, 349}, {514, 349}, {509, 357}, {454, 357}, },
        { {454, 362}, {506, 362}, {504, 370}, {454, 370}, },
        { {455, 375}, {501, 375}, {500, 383}, {455, 383}, },
    };
    for ( int i = 0; i < breakVal/17; ++i) {
        lv_canvas_init_layer(canvas, &layer);
        lv_draw_triangle_dsc_t break_dsc;
        lv_draw_triangle_dsc_init(&break_dsc);
        break_dsc.color = lv_color_make(255, 155, 195);
        if( i >= 3)
            break_dsc.color = lv_color_make(255, 0, 102);
        break_dsc.opa = LV_OPA_100;
        for ( int n = 0; n<3; ++n) {
            break_dsc.p[n].x = break_gauge_points[i][n][0];
            break_dsc.p[n].y = break_gauge_points[i][n][1];
        }
        lv_draw_triangle(&layer, &break_dsc);

        break_dsc.p[1].x = break_gauge_points[i][3][0];
        break_dsc.p[1].y = break_gauge_points[i][3][1];
        lv_draw_triangle(&layer, &break_dsc);

        lv_canvas_finish_layer(canvas, &layer);
    }

    /* Throttle gauge */
    const uint16_t throttle_gauge_points[10][4][2] = {
        { {485, 300}, {582, 300}, {578, 308}, {482, 308}, },
        { {495, 284}, {592, 284}, {586, 292}, {490, 292}, },
        { {508, 272}, {602, 272}, {594, 280}, {500, 280}, },
        { {526, 258}, {622, 258}, {612, 266}, {516, 266}, },
        { {556, 244}, {648, 244}, {630, 252}, {538, 252}, },
        { {586, 232}, {680, 232}, {660, 240}, {564, 240}, },
        { {628, 218}, {722, 218}, {690, 226}, {598, 226}, },
        { {676, 204}, {780, 204}, {746, 212}, {642, 212}, },
        { {760, 188}, {862, 188}, {806, 196}, {702, 196}, },
        { {878, 178}, {970, 178}, {894, 186}, {798, 186}, },
    };
    for ( int i = 0; i < throttleVal/9; ++i) {
        lv_canvas_init_layer(canvas, &layer);
        lv_draw_triangle_dsc_t throttle_dsc;
        lv_draw_triangle_dsc_init(&throttle_dsc);
        throttle_dsc.color = lv_color_make(50, 100, 200);
        if( i > 6)
            throttle_dsc.color = lv_color_make(179, 214, 255);
        throttle_dsc.opa = LV_OPA_100;
        for ( int n = 0; n<3; ++n) {
            throttle_dsc.p[n].x = throttle_gauge_points[i][n][0];
            throttle_dsc.p[n].y = throttle_gauge_points[i][n][1];
        }
        lv_draw_triangle(&layer, &throttle_dsc);

        throttle_dsc.p[1].x = throttle_gauge_points[i][3][0];
        throttle_dsc.p[1].y = throttle_gauge_points[i][3][1];
        lv_draw_triangle(&layer, &throttle_dsc);

        lv_canvas_finish_layer(canvas, &layer);
    }

    /* Speed text */
    lv_canvas_init_layer(canvas, &layer);
    lv_draw_label_dsc_t speed_dsc;
    lv_draw_label_dsc_init(&speed_dsc);
    speed_dsc.color = lv_color_make(255, 255, 255);
    char speed_str[10] = {0};
    sprintf(speed_str, "%3d", speed);
    speed_dsc.text = speed_str;
    speed_dsc.font = &lv_font_montserrat_48;
    lv_area_t coords_speed;
    coords_speed.x1 = 190;
    coords_speed.y1 = 220;
    coords_speed.x2 = 280;
    coords_speed.y2 = 300;
    lv_draw_label(&layer, &speed_dsc, &coords_speed);

    coords_speed.y1 = 270;
    speed_dsc.text = "km/h";;
    speed_dsc.font = &lv_font_montserrat_32;
    lv_draw_label(&layer, &speed_dsc, &coords_speed);
    lv_canvas_finish_layer(canvas, &layer);

    /* Gear */
    lv_canvas_init_layer(canvas, &layer);
    lv_draw_label_dsc_t gear_dsc;
    lv_draw_label_dsc_init(&gear_dsc);
    gear_dsc.color = lv_color_make(255, 255, 255);
    char gear_str[2] = {'0' + gear, 0};
    gear_dsc.text = gear_str;
    gear_dsc.font = &lv_font_montserrat_48;
    lv_area_t coords_gear;
    coords_gear.x1 = 930;
    coords_gear.y1 = 250;
    coords_gear.x2 = 1000;
    coords_gear.y2 = 350;
    lv_draw_label(&layer, &gear_dsc, &coords_gear);
    lv_canvas_finish_layer(canvas, &layer);

    /* Elapsed Time */
    lv_canvas_init_layer(canvas, &layer);
    lv_draw_label_dsc_t time_dsc;
    lv_draw_label_dsc_init(&time_dsc);
    time_dsc.color = lv_color_make(255, 255, 255);
    char time_str[9] = {0};
    sprintf(time_str, "%02d:%02d:%02d", elappsed_time_h, elappsed_time_m, elappsed_time_s);
    time_dsc.text = time_str;
    time_dsc.font = &lv_font_montserrat_32;
    lv_area_t coords_time;
    coords_time.x1 = 690;
    coords_time.y1 = 105;
    coords_time.x2 = 990;
    coords_time.y2 = 140;
    lv_draw_label(&layer, &time_dsc, &coords_time);
    lv_canvas_finish_layer(canvas, &layer);

    // debug
    x++;
    x %= 100;

    // Update canvas
    lv_obj_invalidate(canvas);
}


int main(void)
{
	const struct device *display_dev;
	int ret;

can_start( can_dev );
int filter_id = can_add_rx_filter(can_dev, rx_callback_function, NULL, &filter);
if (filter_id < 0) {
  LOG_ERR("Unable to add rx filter [%d]", filter_id);
}

	/* Initialize Display */
	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}
	ret = display_blanking_off(display_dev);
	if (ret < 0 && ret != -ENOSYS) {
		LOG_ERR("Failed to turn blanking off (error %d)", ret);
	    return 0;
	}

        // Convert image from https://lvgl.io/tools/imageconverter
        // to src/meter_cluster_bg.c
        LV_IMAGE_DECLARE(meter_cluster_bg);
        lv_obj_t *bg_img = lv_image_create(lv_screen_active());
        lv_image_set_src(bg_img, &meter_cluster_bg);
        //lv_obj_align(bg_img, LV_ALIGN_CENTER, 0, 0);
        lv_obj_align(bg_img, LV_ALIGN_TOP_LEFT, 0, 0);

        LV_DRAW_BUF_DEFINE_STATIC(draw_buf, DISPLAY_WIDTH, DISPLAY_HEIGHT, LV_COLOR_FORMAT_ARGB8888);
        LV_DRAW_BUF_INIT_STATIC(draw_buf);
        lv_obj_t * canvas = lv_canvas_create(lv_screen_active());
        lv_canvas_set_draw_buf(canvas, &draw_buf);

        lv_timer_handler();
	while (1) {
		draw_frame(canvas);
		lv_timer_handler();
		++count;
		k_sleep(K_MSEC(10));
	

        }
can_stop( can_dev );
}
