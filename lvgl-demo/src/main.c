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

    /* Fuel gauge */
    lv_canvas_init_layer(canvas, &layer);
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = lv_color_make(50, 100, 255);
    rect_dsc.bg_opa = LV_OPA_50;
    lv_area_t gauge_coords = {1070, (315-2*(fuel)), 1140, 315};  // {1050, 115, 1140, 315}
    lv_draw_rect(&layer, &rect_dsc, &gauge_coords);
    lv_canvas_finish_layer(canvas, &layer);

    /* Speed text */
    lv_canvas_init_layer(canvas, &layer);
    lv_draw_label_dsc_t speed_dsc;
    lv_draw_label_dsc_init(&speed_dsc);
    speed_dsc.color = lv_palette_main(LV_PALETTE_BLUE);
    char speed_str[10] = {0};
    sprintf(speed_str, "%3d\nkm/h", speed);
    speed_dsc.text = speed_str;
    speed_dsc.font = &lv_font_montserrat_32;
    lv_area_t coords_speed;
    coords_speed.x1 = 190;
    coords_speed.y1 = 200;
    coords_speed.x2 = 280;
    coords_speed.y2 = 300;
    lv_draw_label(&layer, &speed_dsc, &coords_speed);
    lv_canvas_finish_layer(canvas, &layer);

    /* Gear */
    lv_canvas_init_layer(canvas, &layer);
    lv_draw_label_dsc_t gear_dsc;
    lv_draw_label_dsc_init(&gear_dsc);
    gear_dsc.color = lv_palette_main(LV_PALETTE_BLUE);
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
    time_dsc.color = lv_palette_main(LV_PALETTE_BLUE);
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
