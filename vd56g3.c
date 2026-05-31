/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "vd56g3.h"
#include "vd56g3_regs.h"
#include "sdkconfig.h"

typedef struct {
    uint32_t exposure_val;
    uint32_t exposure_max;
    uint32_t gain_index; // current gain index

    uint32_t vflip_en : 1;
    uint32_t hmirror_en : 1;

    uint32_t is_fastboot : 1;

	uint32_t xclk_freq;
	uint32_t pll_prediv;
	uint32_t pll_mult;
	uint32_t pixel_clock;
	uint16_t oif_ctrl;
	uint8_t nb_of_lane;
	uint32_t gpios[VD56G3_NB_GPIOS];
	bool ext_vt_sync;

} vd56g3_para_t;

struct vd56g3_cam {
    vd56g3_para_t vd56g3_para;
};

#define VD56G3_IO_MUX_LOCK(mux)
#define VD56G3_IO_MUX_UNLOCK(mux)
#define VD56G3_ENABLE_OUT_XCLK(pin,clk)
#define VD56G3_DISABLE_OUT_XCLK(pin)
/* Mini exposure time: 1 row period
 * Max exposure time:  frame length(0x380e, 0x380f) - 25 row periods
*/
#define VD56G3_EXP_MAX_OFFSET   25

#define VD56G3_FETCH_EXP_H(val)     (((val) >> 12) & 0xf)
#define VD56G3_FETCH_EXP_M(val)     (((val) >> 4) & 0xff)
#define VD56G3_FETCH_EXP_L(val)     (((val) & 0xf) << 4)

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms)  vTaskDelay((ms > portTICK_PERIOD_MS ? ms/ portTICK_PERIOD_MS : 1))
#define VD56G3_SUPPORT_NUM CONFIG_CAMERA_VD56G3_MAX_SUPPORT

#define HZ_PER_MHZ		1000000UL
#define MEGA			1000000UL

static const uint8_t s_vd56g3_exp_min = 0x05;
static const uint32_t s_vd56g3_limited_gain_ = 200;
static size_t s_vd56g3_limited_gain_index;

static const char *TAG = "vd56g3";

#define EXPOSURE_V4L2_UNIT_US                   100
#define EXPOSURE_V4L2_TO_VD56G3(v, sf)          \
    ((uint32_t)(((double)v) * EXPOSURE_V4L2_UNIT_US * 1000 / (((sf)->isp_info->isp_v1_info.tline_ns)) + 0.5))
#define EXPOSURE_VD56G3_TO_V4L2(v, sf)          \
    ((int32_t)(((double)v) * (((sf)->isp_info->isp_v1_info.tline_ns)) / EXPOSURE_V4L2_UNIT_US / 1000 + 0.5))

static const uint32_t vd56g3_total_gain_val_map[] = {
    1000, 1062, 1125, 1187, 1250, 1312, 1375, 1437, 1500, 1562, 1625, 1687, 1750, 1812, 1875, 1937,
    2000, 2062, 2125, 2187, 2250, 2312, 2375, 2437, 2500, 2562, 2625, 2687, 2750, 2812, 2875, 2937,
    3000, 3062, 3125, 3187, 3250, 3312, 3375, 3437, 3500, 3562, 3625, 3687, 3750, 3812, 3875, 3937,
    4000, 4125, 4250, 4375, 4500, 4625, 4750, 4875, 5000, 5125, 5250, 5375, 5500, 5625, 5750, 5875,
    6000, 6125, 6250, 6375, 6500, 6625, 6750, 6875, 7000, 7125, 7250, 7375, 7500, 7625, 7750, 7875,
    8000, 8250, 8500, 8750, 9000, 9250, 9500, 9750, 10000, 10250, 10500, 10750, 11000, 11250, 11500, 11750,
    12000, 12250, 12500, 12750, 13000, 13250, 13500, 13750, 14000, 14250, 14500, 14750, 15000, 15250, 15500, 15750,
};

#if 0
static const uint8_t vd56g3_gain_reg_map[] = {
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
    0x40, 0x42, 0x44, 0x46, 0x48, 0x4a, 0x4c, 0x4e, 0x50, 0x52, 0x54, 0x56, 0x58, 0x5a, 0x5c, 0x5e,
    0x60, 0x62, 0x64, 0x66, 0x68, 0x6a, 0x6c, 0x6e, 0x70, 0x72, 0x74, 0x76, 0x78, 0x7a, 0x7c, 0x7e,
    0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98, 0x9c, 0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8, 0xbc,
    0xc0, 0xc4, 0xc8, 0xcc, 0xd0, 0xd4, 0xd8, 0xdc, 0xe0, 0xe4, 0xe8, 0xec, 0xf0, 0xf4, 0xf8, 0xfc,
};
#endif

static const esp_cam_sensor_isp_info_t vd56g3_isp_info[] = {
    {
        // MIPI_2lane_12Minput_RAW10_1120x1360_30fps
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 160800000,
            .hts = 1236,
            .vts = 1478,
            .tline_ns = 7687, 
            .gain_def = 8,    
            .exp_def = 0x2a9,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
    {
        // MIPI_2lane_12Minput_RAW8_1120x1360_88fps
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 160800000,
            .hts = 1236,
            .vts = 1478,
            .tline_ns = 7687,
            .gain_def = 8, 
            .exp_def = 0x2a9,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
    {
        // MIPI_2lane_12Minput_RAW8_480x640_88fps (cropped on sensor side)
        .isp_v1_info = {
            .version = SENSOR_ISP_INFO_VERSION_DEFAULT,
            .pclk = 160800000,
            .hts = 1236,
            .vts = 1478,
            .tline_ns = 7687,
            .gain_def = 8, 
            .exp_def = 0x2a9,
            .bayer_type = ESP_CAM_SENSOR_BAYER_BGGR,
        }
    },
};


static const esp_cam_sensor_format_t vd56g3_format_info[] = {
    {
        .name = "MIPI_2lane_12Minput_RAW10_1120x1360_30fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 12000000,
        .width = 1120,
        .height = 1360,
        .regs = NULL,
        .regs_size = 0,
        .fps = 30,
        .isp_info = &vd56g3_isp_info[0],
        .mipi_info = {
            .mipi_clk = VD56G3_LINK_FREQ_DEF_2LANES,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_12Minput_RAW8_1120x1360_88fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW8,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 12000000,
        .width = 1120,
        .height = 1360,
        .regs = NULL,
        .regs_size = 0,
        .fps = 88,
        .isp_info = &vd56g3_isp_info[1],
        .mipi_info = {
            .mipi_clk = VD56G3_LINK_FREQ_DEF_2LANES,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },
    {
        .name = "MIPI_2lane_12Minput_RAW8_480x640_88fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_RAW8,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 12000000,
        .width = 480,
        .height = 640,
        .regs = NULL,
        .regs_size = 0,
        .fps = 88,
        .isp_info = &vd56g3_isp_info[2],
        .mipi_info = {
            .mipi_clk = VD56G3_LINK_FREQ_DEF_2LANES,
            .lane_num = 2,
            .line_sync_en = false,
        },
        .reserved = NULL,
    },};

static esp_err_t vd56g3_read(esp_sccb_io_handle_t sccb_handle, uint32_t reg, uint32_t *read_buf)
{
    esp_err_t err = ESP_FAIL;

    /* register width is encoded in the address */
    unsigned int len = (reg >> CCI_REG_WIDTH_SHIFT) & 7;

    /* width-code not to be sent to sensor */
    reg = reg & CCI_REG_ADDR_MASK;

    switch (len) {
	case 1:
        uint8_t buf8 = 0;
        err = esp_sccb_transmit_receive_reg_a16v8(sccb_handle, reg, &buf8);
        *read_buf = buf8;
        break;
    case 2:
        uint16_t buf16 = 0;
        err = esp_sccb_transmit_receive_reg_a16v16(sccb_handle, reg, &buf16);
        *read_buf = __builtin_bswap16(buf16);
        break;
    case 4:
        uint32_t buf32;
        err = esp_sccb_transmit_receive_reg_a16v32(sccb_handle, reg, &buf32);
        *read_buf = __builtin_bswap32(buf32);
        break;
    default:
        err = ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "vd56g3_read called with invalid length %d for reg 0x%x", len, reg);
        break;
    }

    //ESP_LOGI(TAG, "vd56g3_read 0x%04x 0x%04x", reg, *read_buf);

    return err;
}

static esp_err_t vd56g3_write(esp_sccb_io_handle_t sccb_handle, uint32_t reg, uint32_t data)
{
    esp_err_t ret = ESP_FAIL;
    unsigned int len = (reg >> CCI_REG_WIDTH_SHIFT) & 7;

    reg = reg & CCI_REG_ADDR_MASK;

    //ESP_LOGI(TAG, "vd56g3_write 0x%04x 0x%04x", reg, data);

	switch (len) {
	case 1:
		ret = esp_sccb_transmit_reg_a16v8(sccb_handle, reg, (uint8_t)data);
		break;
	case 2:
        uint16_t buf16 = __builtin_bswap16((uint16_t)data);
		ret = esp_sccb_transmit_reg_a16v16(sccb_handle, reg, buf16);
		break;
	case 4:
        uint32_t buf32 = __builtin_bswap32(data);
		ret = esp_sccb_transmit_reg_a16v32(sccb_handle, reg, buf32);
		break;
	default:
        ret = ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "vd56g3_write called with invalid register length %d for reg 0x%x", len, reg);
        break;
	}

    return ret;
}

#if 0
/* write a array of registers  */
static esp_err_t vd56g3_write_array(esp_sccb_io_handle_t sccb_handle, vd56g3_reginfo_t *regarray)
{
    int i = 0;
    esp_err_t ret = ESP_OK;
    while ((ret == ESP_OK) && regarray[i].reg != VD56G3_REG_END) {
        if (regarray[i].reg != VD56G3_REG_DELAY) {
            ret = vd56g3_write(sccb_handle, regarray[i].reg, regarray[i].val);
        } else {
            delay_ms(regarray[i].val);
        }
        i++;
    }
    ESP_LOGD(TAG, "Set array done[i=%d]", i);
    return ret;
}
    #endif

#if 0
static esp_err_t vd56g3_set_reg_bits(esp_sccb_io_handle_t sccb_handle, uint16_t reg, uint8_t offset, uint8_t length, uint8_t value)
{
    esp_err_t ret = ESP_OK;
    uint32_t reg_data = 0;

    ret = vd56g3_read(sccb_handle, reg, &reg_data);
    if (ret != ESP_OK) {
        return ret;
    }
    uint8_t mask = ((1 << length) - 1) << offset;
    value = (reg_data & ~mask) | ((value << offset) & mask);
    ret = vd56g3_write(sccb_handle, reg, value);
    return ret;
}
#endif

static esp_err_t vd56g3_poll_reg(esp_sccb_io_handle_t sccb_handle, uint32_t reg, uint32_t poll_val)
{
    uint32_t val;
    esp_err_t ret = ESP_OK;
    
    time_t now = time(0);
    time_t timeout = now + 2;

    while(true) {
        ret = vd56g3_read(sccb_handle, reg, &val);
        if (val == poll_val) break;
        if (ret != ESP_OK) break;

        if(time(0) > timeout) {
            ret = ESP_ERR_TIMEOUT;
            ESP_LOGW(TAG, "pollig timeout: reg 0x%x waiting for val 0x%x, got 0x%x", CCI_REG_ADDR(reg), poll_val, val);
            break;
        }
    }
    return ret;
}

static esp_err_t vd56g3_wait_state(esp_sccb_io_handle_t sccb_handle, int state)
{
	return vd56g3_poll_reg(sccb_handle, VD56G3_REG_SYSTEM_FSM, state);
}

static esp_err_t vd56g3_set_test_pattern(esp_cam_sensor_device_t *dev, int enable)
{
    //return vd56g3_set_reg_bits(dev->sccb_handle, VD56G3_REG_TEST_PATTERN, 7, 1,  enable ? 0x01 : 0x00);
    return ESP_FAIL;
}

static esp_err_t vd56g3_hw_reset(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
    }
    return ESP_OK;
}

static esp_err_t vd56g3_soft_reset(esp_cam_sensor_device_t *dev)
{
    //esp_err_t ret = vd56g3_write(dev->sccb_handle, VD56G3_REG_SW_RESET, 0x01);
    //delay_ms(5);
    //return ret;
    return ESP_FAIL;
}

static esp_err_t vd56g3_boot(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_FAIL;

	vd56g3_write(dev->sccb_handle, VD56G3_REG_BOOT, VD56G3_CMD_BOOT);
	vd56g3_poll_reg(dev->sccb_handle, VD56G3_REG_BOOT, VD56G3_CMD_ACK);

    /* ignore errors from write and poll. as long as we end up in SW_STBY, all is well. */

    ret = vd56g3_wait_state(dev->sccb_handle, VD56G3_SYSTEM_FSM_SW_STBY);
    if(ret == ESP_OK) {
        ESP_LOGI(TAG, "sensor in SW_STANDBY state");
    }
    else {
        ESP_LOGE(TAG, "sensor did not reach SW_STANDBY state");
    }

    return ret;
}

static esp_err_t vd56g3_patch(esp_cam_sensor_device_t *dev)
{
#if 0
    struct i2c_client *client = sensor->i2c_client;
	const u8 *patch = patch_cut2;
	int patch_size = sizeof(patch_cut2);
	u8 patch_major;
	u8 patch_minor;
	int cur_patch_rev = 0;
	int ret = 0;

	patch_major = patch[3];
	patch_minor = patch[2];

	vd56g3_write_array(sensor, 0x2000, patch_size, patch);
	vd56g3_write(sensor, VD56G3_REG_BOOT, VD56G3_CMD_PATCH_SETUP);
	vd56g3_poll_reg(sensor, VD56G3_REG_BOOT, VD56G3_CMD_ACK);
	vd56g3_read(sensor, VD56G3_REG_FWPATCH_REVISION, &cur_patch_rev);
	if (ret)
		return ret;

	if (cur_patch_rev != (patch_major << 8) + patch_minor) {
		dev_err(&client->dev,
			"bad patch version expected %d.%d got %d.%d",
			patch_major, patch_minor, cur_patch_rev >> 8,
			cur_patch_rev & 0xff);
		return -ENODEV;
	}
	dev_info(&client->dev, "patch %d.%d applied", cur_patch_rev >> 8,
		 cur_patch_rev & 0xff);
	return ESP_OK;
#endif
	return ESP_FAIL;
}

static esp_err_t vd56g3_vtpatch(esp_cam_sensor_device_t *dev)
{
#if 0
    struct i2c_client *client = sensor->i2c_client;
	int i;
	int vtpatch_offset = 0;
	int cur_vtpatch_rev = 0;
	int ret = 0;

	vd56g3_write(sensor, VD56G3_REG_VTPATCHING,
		     VD56G3_CMD_START_VTRAM_UPDATE);
	vd56g3_poll_reg(sensor, VD56G3_REG_VTPATCHING, VD56G3_CMD_ACK);
	vd56g3_wait_state(sensor, VD56G3_SYSTEM_FSM_SW_STBY);

	for (i = 0; i < vtpatch_area_nb; i++) {
		vd56g3_write_array(sensor, vtpatch_desc[i].offset,
				   vtpatch_desc[i].size,
				   vtpatch + vtpatch_offset);

		vtpatch_offset += vtpatch_desc[i].size;
	}

	vd56g3_write(sensor, VD56G3_REG_VTPATCHING, VD56G3_CMD_END_VTRAM_UPDATE);
	vd56g3_poll_reg(sensor, VD56G3_REG_VTPATCHING, VD56G3_CMD_ACK);
	vd56g3_wait_state(sensor, VD56G3_SYSTEM_FSM_SW_STBY);
	vd56g3_read(sensor, VD56G3_REG_VTPATCH_ID, &cur_vtpatch_rev);
	if (ret)
		return ret;

	if (cur_vtpatch_rev != VT_REVISION) {
		dev_err(&client->dev, "bad vtpatch version, expected %d got %d",
			VT_REVISION, cur_vtpatch_rev);
		return -ENODEV;
	}
	dev_info(&client->dev, "VT patch %d applied", VT_REVISION);
	return ESP_OK;
#endif
	return ESP_FAIL;
}

static esp_err_t vd56g3_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    esp_err_t ret = ESP_FAIL;
    uint32_t pid;

    ret = vd56g3_read(dev->sccb_handle, VD56G3_REG_MODEL_ID, &pid);
    if (ret != ESP_OK) {
        return ret;
    }

    id->pid = (uint16_t)pid;

    return ret;
}

static esp_err_t vd56g3_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    esp_err_t ret = ESP_OK;
    uint32_t val;
    struct vd56g3_cam *cam_vd56g3 = (struct vd56g3_cam *)dev->priv;
    esp_sccb_io_handle_t sccb = dev->sccb_handle;
    vd56g3_para_t *vd56g3_para = &cam_vd56g3->vd56g3_para;

    if (enable == false)
    {
        ESP_LOGI(TAG, "stopping stream");

        // read back status registers
        vd56g3_read(sccb, VD56G3_REG_STAT_ERROR_CODE, &val);
        ESP_LOGI(TAG, "error code = 0x%x", val);
        vd56g3_read(sccb, VD56G3_REG_STAT_FRAME_RATE, &val);
        ESP_LOGI(TAG, "frame rate = %d", val);
        vd56g3_read(sccb, VD56G3_REG_STAT_FRAME_COUNTER, &val);
        ESP_LOGI(TAG, "frame counter = %d", val);


        /* Retrieve Expo cluster to enable coldstart of AE */
        //ret = vd56g3_read_expo_cluster(sensor, true);

        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_STREAMING, VD56G3_CMD_STOP_STREAM));
        ESP_ERROR_CHECK(vd56g3_poll_reg(sccb, VD56G3_REG_STREAMING, VD56G3_CMD_ACK));
        ret = vd56g3_wait_state(sccb, VD56G3_SYSTEM_FSM_SW_STBY);
        if(ret == ESP_OK) {
            ESP_LOGI(TAG, "sensor in SW_STANDBY state");
        }
        else {
            ESP_LOGE(TAG, "sensor did not reach SW_STANDBY state");
        }
    } else {
        ESP_LOGI(TAG, "starting steam...");

        unsigned int csi_mbps = ((vd56g3_para->nb_of_lane == 2) ?
					 VD56G3_LINK_FREQ_DEF_2LANES :
					 VD56G3_LINK_FREQ_DEF_1LANE) * 2 / MEGA;
        unsigned int binning;

        /* configure clocks */
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_EXT_CLOCK, vd56g3_para->xclk_freq));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_CLK_PLL_PREDIV, vd56g3_para->pll_prediv));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_CLK_SYS_PLL_MULT, vd56g3_para->pll_mult));
        ESP_LOGI(TAG, "xclk_freq=%d", vd56g3_para->xclk_freq);
        ESP_LOGI(TAG, "pll_prediv=%d", vd56g3_para->pll_prediv);
        ESP_LOGI(TAG, "pll_mult=%d", vd56g3_para->pll_mult);

        /* configure output */
        uint8_t bpp; uint8_t datatype;
        switch(dev->cur_format->format) {
        case ESP_CAM_SENSOR_PIXFORMAT_RAW10:
            bpp = 10;
            datatype = MIPI_CSI2_DT_RAW10;
            break;
        case ESP_CAM_SENSOR_PIXFORMAT_RAW8:
            bpp = 8;
            datatype = MIPI_CSI2_DT_RAW8;
            break;
        default:
            ESP_LOGE(TAG, "unsupported pixel format %d", dev->cur_format->format);
            return ESP_FAIL;
        }
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_FORMAT_CTRL, bpp));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_OIF_CTRL, vd56g3_para->oif_ctrl));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_OIF_CSI_BITRATE, csi_mbps));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_OIF_IMG_CTRL, datatype));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_ISL_ENABLE, 0));
        ESP_LOGI(TAG, "format_ctrl=0x%x", bpp);
        ESP_LOGI(TAG, "csi_mbps=%d", csi_mbps);
        ESP_LOGI(TAG, "csi datatype=0x%x", datatype);
        ESP_LOGI(TAG, "oif_ctrl=0x%x", vd56g3_para->oif_ctrl);

        // TODO: crop* should be read from somewhere...
        uint32_t crop_top = 0;
        uint32_t crop_height = dev->cur_format->height;
        uint32_t crop_left = 0;
        uint32_t crop_width = dev->cur_format->width; 

        /* configure binning mode */
        int subsampling_factor = 0;
        switch (crop_width / dev->cur_format->width) {
        case 1:
        default:
            binning = READOUT_NORMAL;
            subsampling_factor = 1;
            break;
        case 2:
            binning = READOUT_DIGITAL_BINNING_X2;
            subsampling_factor = 2;
            break;
        }
        vd56g3_write(sccb, VD56G3_REG_READOUT_CTRL, binning);
        ESP_LOGI(TAG, "binning=%d", binning);

        /* configure ROIs */
        //video timing output
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_Y_START, crop_top));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_Y_END, crop_top + crop_height - 1));
        // image roi
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_OUT_ROI_X_START, crop_left));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_OUT_ROI_X_END, crop_left + crop_width - 1));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_OUT_ROI_Y_START, 0));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_OUT_ROI_Y_END, crop_height - 1));
        // auto exposure
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_AE_ROI_START_H, crop_left));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_AE_ROI_END_H, crop_left + crop_width - 1));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_AE_ROI_START_V, 0));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_AE_ROI_END_V, crop_height - 1));

        // set all GPIOs to input
        for (int i=0; i<VD56G3_NB_GPIOS; i++)
        {
            ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_GPIO_0_CTRL+i, VD56G3_GPIOX_GPIO_IN));
        }

        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_ORIENTATION, 0));

        int y_start = crop_top;
        int y_end = crop_top + crop_height - 1;
        int cur_line_len = 1236;
        int min_line_len = 1236;
        int frame_length_offset = 31 + 18 + (20*min_line_len/cur_line_len+0.5);
        uint32_t frame_length_min = (y_end - y_start + 1) / subsampling_factor + frame_length_offset;
        ESP_LOGI(TAG, "minimum frame_length %u", frame_length_min);

        float line_time = cur_line_len / (float)vd56g3_para->pixel_clock;
        uint32_t frame_length = 1 / (line_time * dev->cur_format->fps);
        ESP_LOGI(TAG, "calculated frame_length for %d fps: %u", dev->cur_format->fps, frame_length);

        if (frame_length < frame_length_min)
        {
            ESP_LOGE(TAG, "requested frame_length %u below minimum of %u", frame_length, frame_length_min);
        }
        else
        {
            ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_FRAME_LENGTH, frame_length));
        }

        // logged from linux driver:
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_DUSTER_CTRL                   , 0x0013));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_DARKCAL_CTRL                  , 0x0001));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_PATGEN_CTRL                   , 0x0000));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_EXP_MODE                      , 0x0000));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_AE_COLDSTART_COARSE_EXPOSURE  , 0x058c));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_AE_COLDSTART_ANALOG_GAIN      , 0x0000));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_AE_COLDSTART_DIGITAL_GAIN     , 0x0100));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_EXP_MODE                      , 0x0000));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_AE_COMPENSATION               , 0x0000));
        //ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_FRAME_LENGTH                  , 0x3f86)); // 8 fps
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_AE_TARGET_PERCENTAGE          , 0x001e));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_AE_STEP_PROPORTION            , 0x008c));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_AE_LEAK_PROPORTION            , 0x2ccc));
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_DARKCAL_PEDESTAL              , 0x0040));

        // disable mipi clock output between frames
        // ESP32P4 only recognized a mipi clock in non-continuous clock mode
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_DPHYTX_CTRL, VD56G3_DPHYTX_CONTMODE_DIS));

        uint32_t sync_mode; // 0x0: master, 0x2: i2c follower
        sync_mode = 0;
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_VT_CTRL, sync_mode));

        /* start streaming */
        ESP_ERROR_CHECK(vd56g3_write(sccb, VD56G3_REG_STBY, VD56G3_CMD_START_STREAM));
        ESP_ERROR_CHECK(vd56g3_poll_reg(sccb, VD56G3_REG_STBY, VD56G3_CMD_ACK));
        ret = vd56g3_wait_state(sccb, VD56G3_SYSTEM_FSM_STREAMING);
        if(ret == ESP_OK) {
            ESP_LOGI(TAG, "sensor in STREAMING state");
        }
        else {
            ESP_LOGE(TAG, "sensor did not reach STREAMING state");
        }

        // read back status registers
        vd56g3_read(sccb, VD56G3_REG_STAT_ERROR_CODE, &val);
        ESP_LOGI(TAG, "error code = 0x%x", val);
        vd56g3_read(sccb, VD56G3_REG_STAT_FRAME_RATE, &val);
        ESP_LOGI(TAG, "frame rate = %d", val);
        vd56g3_read(sccb, VD56G3_REG_STAT_FRAME_COUNTER, &val);
        ESP_LOGI(TAG, "frame counter = %d", val);
    
        /*
          VD56G3 mipi clock:
          mipi_freq = sys_pll_clk * clk_mipi_pll_mult / clk_mipi_pll_opdiv / 32
          sys_pll_clk = 804 MHz
        */
        float mipi_freq;
        uint32_t clk_mipi_pll_mult;
        uint32_t clk_mipi_pll_opdiv;
        vd56g3_read(sccb, VD56G3_REG_STAT_CLK_MIPI_PLL_MULTI, &clk_mipi_pll_mult);
        ESP_LOGI(TAG, "mipi_pll_mult = %d", clk_mipi_pll_mult);
        vd56g3_read(sccb, VD56G3_REG_STAT_CLK_MIPI_PLL_OPDIV, &clk_mipi_pll_opdiv);
        ESP_LOGI(TAG, "mipi_pll_opdiv = %d", clk_mipi_pll_opdiv);
        mipi_freq = 804e6 * clk_mipi_pll_mult / clk_mipi_pll_opdiv / 32;
        ESP_LOGI(TAG, "mipi_freq = %.3f MHz", mipi_freq/1e6);

        vd56g3_read(sccb, VD56G3_REG_STAT_LINE_LENGTH, &val);
        ESP_LOGI(TAG, "line_length = %d", val);
        vd56g3_read(sccb, VD56G3_REG_STAT_FRAME_LENGTH, &val);
        ESP_LOGI(TAG, "frame_length = %d", val);
        vd56g3_read(sccb, VD56G3_REG_STAT_X_START, &val);
        ESP_LOGI(TAG, "x_start = %d", val);
        vd56g3_read(sccb, VD56G3_REG_STAT_X_END, &val);
        ESP_LOGI(TAG, "x_end = %d", val);

        vd56g3_read(sccb, VD56G3_REG_STAT_OUT_ROI_X_SIZE, &val);
        ESP_LOGI(TAG, "out_roi_x_size = %d", val);
        vd56g3_read(sccb, VD56G3_REG_STAT_OUT_ROI_Y_SIZE, &val);
        ESP_LOGI(TAG, "out_roi_y_size = %d", val);
    }

    dev->stream_status = enable;
    ESP_LOGI(TAG, "Stream=%d", enable);

    return ret;
}

static esp_err_t vd56g3_set_mirror(esp_cam_sensor_device_t *dev, int enable)
{
    //return vd56g3_set_reg_bits(dev->sccb_handle, 0x3820, 2, 1,  enable ? 0x01 : 0x00);
    return ESP_FAIL;
}

static esp_err_t vd56g3_set_vflip(esp_cam_sensor_device_t *dev, int enable)
{
    //return vd56g3_set_reg_bits(dev->sccb_handle, 0x3821, 2, 1,  enable ? 0x01 : 0x00);
    return ESP_FAIL;
}

static esp_err_t vd56g3_set_exp_val(esp_cam_sensor_device_t *dev, uint32_t u32_val)
{
    esp_err_t ret = ESP_FAIL;
#if 0
    struct vd56g3_cam *cam_vd56g3 = (struct vd56g3_cam *)dev->priv;
    uint32_t value_buf = MAX(u32_val, s_vd56g3_exp_min);
    value_buf = MIN(value_buf, cam_vd56g3->vd56g3_para.exposure_max);

    ESP_LOGD(TAG, "set exposure 0x%" PRIx32, value_buf);
    /* 4 least significant bits of expsoure are fractional part */
    ret = vd56g3_write(dev->sccb_handle,
                       VD56G3_REG_EXPOSURE_H,
                       VD56G3_FETCH_EXP_H(value_buf));
    ret |= vd56g3_write(dev->sccb_handle,
                        VD56G3_REG_EXPOSURE_M,
                        VD56G3_FETCH_EXP_M(value_buf));
    ret |= vd56g3_write(dev->sccb_handle,
                        VD56G3_REG_EXPOSURE_L,
                        VD56G3_FETCH_EXP_L(value_buf));
    if (ret == ESP_OK) {
        cam_vd56g3->vd56g3_para.exposure_val = value_buf;
    }
#endif
    return ret;
}

static esp_err_t vd56g3_set_total_gain_val(esp_cam_sensor_device_t *dev, uint32_t u32_val)
{
    esp_err_t ret = ESP_FAIL;
#if 0
    struct vd56g3_cam *cam_vd56g3 = (struct vd56g3_cam *)dev->priv;
    u32_val = MIN(u32_val, s_vd56g3_limited_gain_index);

    ESP_LOGD(TAG, "dgain %" PRIx8, vd56g3_gain_reg_map[u32_val]);
    ret = vd56g3_write(dev->sccb_handle,
                       VD56G3_REG_GAIN,
                       vd56g3_gain_reg_map[u32_val]);
    if (ret == ESP_OK) {
        cam_vd56g3->vd56g3_para.gain_index = u32_val;
    }
#endif
    return ret;
}

static esp_err_t vd56g3_query_para_desc(esp_cam_sensor_device_t *dev, esp_cam_sensor_param_desc_t *qdesc)
{
    esp_err_t ret = ESP_OK;
    switch (qdesc->id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = s_vd56g3_exp_min;
        qdesc->number.maximum = dev->cur_format->isp_info->isp_v1_info.vts - VD56G3_EXP_MAX_OFFSET; // max = VTS-6 = height+vblank-6, so when update vblank, exposure_max must be updated
        qdesc->number.step = 1;
        qdesc->default_value = dev->cur_format->isp_info->isp_v1_info.exp_def;
        break;
    case ESP_CAM_SENSOR_EXPOSURE_US:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = EXPOSURE_VD56G3_TO_V4L2(s_vd56g3_exp_min, dev->cur_format);
        qdesc->number.maximum = EXPOSURE_VD56G3_TO_V4L2((dev->cur_format->isp_info->isp_v1_info.vts - VD56G3_EXP_MAX_OFFSET), dev->cur_format); // max = VTS-6 = height+vblank-6, so when update vblank, exposure_max must be updated
        qdesc->number.step = MAX(EXPOSURE_VD56G3_TO_V4L2(0x01, dev->cur_format), 1);
        qdesc->default_value = EXPOSURE_VD56G3_TO_V4L2((dev->cur_format->isp_info->isp_v1_info.exp_def), dev->cur_format);
        break;
    case ESP_CAM_SENSOR_GAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_ENUMERATION;
        qdesc->enumeration.count = s_vd56g3_limited_gain_index;
        qdesc->enumeration.elements = vd56g3_total_gain_val_map;
        qdesc->default_value = dev->cur_format->isp_info->isp_v1_info.gain_def; // gain index
        break;
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_U8;
        qdesc->u8.size = sizeof(esp_cam_sensor_gh_exp_gain_t);
        break;
    case ESP_CAM_SENSOR_VFLIP:
    case ESP_CAM_SENSOR_HMIRROR:
        qdesc->type = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum = 0;
        qdesc->number.maximum = 1;
        qdesc->number.step = 1;
        qdesc->default_value = 0;
        break;
    default: {
        ESP_LOGD(TAG, "id=%"PRIx32" is not supported", qdesc->id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    }
    return ret;
}

static esp_err_t vd56g3_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id, void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;
    struct vd56g3_cam *cam_vd56g3 = (struct vd56g3_cam *)dev->priv;
    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL: {
        *(uint32_t *)arg = cam_vd56g3->vd56g3_para.exposure_val;
        break;
    }
    case ESP_CAM_SENSOR_GAIN: {
        *(uint32_t *)arg = cam_vd56g3->vd56g3_para.gain_index;
        break;
    }
    default: {
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }
    }
    return ret;
}

static esp_err_t vd56g3_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id, const void *arg, size_t size)
{
    esp_err_t ret = ESP_OK;

    switch (id) {
    case ESP_CAM_SENSOR_EXPOSURE_VAL: {
        uint32_t u32_val = *(uint32_t *)arg;
        // Todo, Add group hold if fast AEC need
        ret = vd56g3_set_exp_val(dev, u32_val);
        break;
    }
    case ESP_CAM_SENSOR_EXPOSURE_US: {
        uint32_t u32_val = *(uint32_t *)arg;
        uint32_t ori_exp = EXPOSURE_V4L2_TO_VD56G3(u32_val, dev->cur_format);
        ret = vd56g3_set_exp_val(dev, ori_exp);
        break;
    }
    case ESP_CAM_SENSOR_GAIN: {
        uint32_t u32_val = *(uint32_t *)arg;
        ret = vd56g3_set_total_gain_val(dev, u32_val);
        break;
    }
    case ESP_CAM_SENSOR_GROUP_EXP_GAIN: {
        esp_cam_sensor_gh_exp_gain_t *value = (esp_cam_sensor_gh_exp_gain_t *)arg;
        uint32_t ori_exp = 0;
        if (value->exposure_us != 0) {
            ori_exp = EXPOSURE_V4L2_TO_VD56G3(value->exposure_us, dev->cur_format);
        } else if (value->exposure_val != 0) {
            ori_exp = value->exposure_val;
        } else {
            ret = ESP_ERR_INVALID_ARG;
            break;
        }
        ret = vd56g3_set_exp_val(dev, ori_exp);
        ret |= vd56g3_set_total_gain_val(dev, value->gain_index);
        break;
    }
    case ESP_CAM_SENSOR_VFLIP: {
        int *value = (int *)arg;
        ret = vd56g3_set_vflip(dev, *value);
        break;
    }
    case ESP_CAM_SENSOR_HMIRROR: {
        int *value = (int *)arg;
        ret = vd56g3_set_mirror(dev, *value);
        break;
    }
    default: {
        ESP_LOGE(TAG, "set id=%" PRIx32 " is not supported", id);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }
    }

    return ret;
}

static esp_err_t vd56g3_query_support_formats(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);

    formats->count = ARRAY_SIZE(vd56g3_format_info);
    formats->format_array = &vd56g3_format_info[0];
    return ESP_OK;
}

static esp_err_t vd56g3_query_support_capability(esp_cam_sensor_device_t *dev, esp_cam_sensor_capability_t *sensor_cap)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, sensor_cap);

    sensor_cap->fmt_raw = 1;
    return 0;
}

static esp_err_t vd56g3_prepare_clock_tree(esp_cam_sensor_device_t *dev)
{
    struct vd56g3_cam *cam_vd56g3 = (struct vd56g3_cam *)dev->priv;
    vd56g3_para_t *sensor = &cam_vd56g3->vd56g3_para;

	const unsigned int predivs[] = { 1, 2, 4 };
	uint32_t pll_out;
	int i;

	/* External clock must be in [6Mhz-27Mhz] */
	if (sensor->xclk_freq < 6 * HZ_PER_MHZ ||
	    sensor->xclk_freq > 27 * HZ_PER_MHZ) {
		ESP_LOGE(TAG, "Only 6Mhz-27Mhz clock range supported. Provided %lu MHz", sensor->xclk_freq / HZ_PER_MHZ);
		return ESP_FAIL;
	}

	/* PLL input should be in [6Mhz-12Mhz[ */
	for (i = 0; i < ARRAY_SIZE(predivs); i++) {
		sensor->pll_prediv = predivs[i];
		if (sensor->xclk_freq / sensor->pll_prediv < 12 * HZ_PER_MHZ)
			break;
	}

	/* PLL output clock must be as close as possible to 804Mhz */
	sensor->pll_mult = (VD56G3_TARGET_PLL * sensor->pll_prediv +
			    sensor->xclk_freq / 2) / sensor->xclk_freq;
	pll_out = sensor->xclk_freq * sensor->pll_mult / sensor->pll_prediv;
    ESP_LOGI(TAG, "pll_out=%.3f MHz", pll_out/1e6);


	/* Target Pixel Clock for standard 10bit ADC mode : 160.8Mhz */
	sensor->pixel_clock = pll_out / VD56G3_VT_CLOCK_DIV;
    ESP_LOGI(TAG, "vd56g3_prepare_clock_tree: pixel_clock=%.3f MHz", sensor->pixel_clock/1e6);

	return ESP_OK;
}

static esp_err_t vd56g3_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{

    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    struct vd56g3_cam *cam_vd56g3 = (struct vd56g3_cam *)dev->priv;
    esp_err_t ret = ESP_OK;
    /* Depending on the interface type, an available configuration is automatically loaded.
    You can set the output format of the sensor without using query_format().*/
    if (format == NULL) {
        format = &vd56g3_format_info[CONFIG_CAMERA_VD56G3_MIPI_IF_FORMAT_INDEX_DEFAULT];
    }

    ESP_LOGI(TAG, "vd56g3_set_format %s", format->name);

    //ret = vd56g3_write_array(dev->sccb_handle, (vd56g3_reginfo_t *)format->regs);
    ESP_LOGD(TAG, "%s", format->name);

    // FIXME
    // this should set the crop values to tell the sensor how much of the native resolution to output

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set format regs fail");
        return ESP_CAM_SENSOR_ERR_FAILED_SET_FORMAT;
    }

    dev->cur_format = format;
    // init para
    cam_vd56g3->vd56g3_para.exposure_val = dev->cur_format->isp_info->isp_v1_info.exp_def;
    cam_vd56g3->vd56g3_para.gain_index = dev->cur_format->isp_info->isp_v1_info.gain_def;
    cam_vd56g3->vd56g3_para.exposure_max = dev->cur_format->isp_info->isp_v1_info.vts - VD56G3_EXP_MAX_OFFSET;

    cam_vd56g3->vd56g3_para.nb_of_lane = dev->cur_format->mipi_info.lane_num;
    ESP_LOGI(TAG, "%d mipi lanes", cam_vd56g3->vd56g3_para.nb_of_lane);

    /*
	sensor->oif_ctrl = n_lanes |
			   (ep.bus.mipi_csi2.lane_polarities[0] << 3) |
			   ((phy_data_lanes[0]) << 4) |
			   (ep.bus.mipi_csi2.lane_polarities[1] << 6) |
			   ((phy_data_lanes[1]) << 7) |
			   (ep.bus.mipi_csi2.lane_polarities[2] << 9); 
    */

	//cam_vd56g3->vd56g3_para.oif_ctrl = cam_vd56g3->vd56g3_para.nb_of_lane | 0x2c8;
    
    // on the S-board (STEVAL-56G3MAI), P and N of the mipi data and clock lanes are swapped
    uint8_t clklane_swap = 1;
    uint8_t datalane0_mapping = 0;
    uint8_t datalane0_swap = 1;
    uint8_t datalane1_mapping = 1;
    uint8_t datalene1_swap = 1;

    cam_vd56g3->vd56g3_para.oif_ctrl = 
        cam_vd56g3->vd56g3_para.nb_of_lane |
        (clklane_swap << 3) |
        (datalane0_mapping << 4) |
        (datalane0_swap << 6) |
        (datalane1_mapping << 7) |
        (datalene1_swap << 9);

    cam_vd56g3->vd56g3_para.xclk_freq = dev->cur_format->xclk;

    ret = vd56g3_prepare_clock_tree(dev);

    return ret;
}

static esp_err_t vd56g3_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);

    esp_err_t ret = ESP_FAIL;

    if (dev->cur_format != NULL) {
        memcpy(format, dev->cur_format, sizeof(esp_cam_sensor_format_t));
        ret = ESP_OK;
    }
    return ret;
}

static esp_err_t vd56g3_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    esp_err_t ret = ESP_OK;
    uint32_t regval;
    esp_cam_sensor_reg_val_t *sensor_reg;
    VD56G3_IO_MUX_LOCK(mux);

    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_HW_RESET:
        ESP_LOGI(TAG, "ESP_CAM_SENSOR_IOC_HW_RESET");
        ret = vd56g3_hw_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_SW_RESET:
        ESP_LOGI(TAG, "ESP_CAM_SENSOR_IOC_SW_RESET");
        ret = vd56g3_soft_reset(dev);
        break;
    case ESP_CAM_SENSOR_IOC_S_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ESP_LOGI(TAG, "ESP_CAM_SENSOR_IOC_S_REG 0x%06x 0x%04x", sensor_reg->regaddr, sensor_reg->value);
        ret = vd56g3_write(dev->sccb_handle, sensor_reg->regaddr, sensor_reg->value);
        break;
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        ESP_LOGI(TAG, "ESP_CAM_SENSOR_IOC_S_STREAM");
        ret = vd56g3_set_stream(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_S_TEST_PATTERN:
        ESP_LOGI(TAG, "ESP_CAM_SENSOR_IOC_S_TEST_PATTERN");
        ret = vd56g3_set_test_pattern(dev, *(int *)arg);
        break;
    case ESP_CAM_SENSOR_IOC_G_REG:
        sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
        ret = vd56g3_read(dev->sccb_handle, sensor_reg->regaddr, &regval);
        if (ret == ESP_OK) {
            sensor_reg->value = regval;
        }
        ESP_LOGI(TAG, "ESP_CAM_SENSOR_IOC_G_REG 0x%06x 0x%04x", sensor_reg->regaddr, sensor_reg->value);
        break;
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        ESP_LOGI(TAG, "ESP_CAM_SENSOR_IOC_G_CHIP_ID");
        ret = vd56g3_get_sensor_id(dev, arg);
        break;
    default:
        break;
    }

    VD56G3_IO_MUX_UNLOCK(mux);
    return ret;
}

static esp_err_t vd56g3_power_on(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        VD56G3_ENABLE_OUT_XCLK(dev->xclk_pin, dev->xclk_freq_hz);
    }

    if (dev->pwdn_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->pwdn_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        // carefully, logic is inverted compared to reset pin
        gpio_set_level(dev->pwdn_pin, 1);
        delay_ms(10);
        gpio_set_level(dev->pwdn_pin, 0);
        delay_ms(10);
    }

    if (dev->reset_pin >= 0) {
        gpio_config_t conf = { 0 };
        conf.pin_bit_mask = 1LL << dev->reset_pin;
        conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&conf);

        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
    }

    return ret;
}

static esp_err_t vd56g3_power_off(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->xclk_pin >= 0) {
        VD56G3_DISABLE_OUT_XCLK(dev->xclk_pin);
    }

    if (dev->pwdn_pin >= 0) {
        gpio_set_level(dev->pwdn_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->pwdn_pin, 1);
        delay_ms(10);
    }

    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
    }

    return ret;
}

static esp_err_t vd56g3_delete(esp_cam_sensor_device_t *dev)
{
    ESP_LOGD(TAG, "del vd56g3 (%p)", dev);
    if (dev) {
        if (dev->priv) {
            free(dev->priv);
            dev->priv = NULL;
        }
        free(dev);
        dev = NULL;
    }

    return ESP_OK;
}

static const esp_cam_sensor_ops_t vd56g3_ops = {
    .query_para_desc = vd56g3_query_para_desc,
    .get_para_value = vd56g3_get_para_value,
    .set_para_value = vd56g3_set_para_value,
    .query_support_formats = vd56g3_query_support_formats,
    .query_support_capability = vd56g3_query_support_capability,
    .set_format = vd56g3_set_format,
    .get_format = vd56g3_get_format,
    .priv_ioctl = vd56g3_priv_ioctl,
    .del = vd56g3_delete
};

esp_cam_sensor_device_t *vd56g3_detect(esp_cam_sensor_config_t *config)
{
    esp_err_t ret;
    esp_cam_sensor_device_t *dev = NULL;
    struct vd56g3_cam *cam_vd56g3;

    ESP_LOGW(TAG, "vd56g3_detect");

    s_vd56g3_limited_gain_index = ARRAY_SIZE(vd56g3_total_gain_val_map);
    if (config == NULL) {
        return NULL;
    }

    dev = calloc(1, sizeof(esp_cam_sensor_device_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "No memory for camera");
        return NULL;
    }

    cam_vd56g3 = heap_caps_calloc(1, sizeof(struct vd56g3_cam), MALLOC_CAP_DEFAULT);
    if (!cam_vd56g3) {
        ESP_LOGE(TAG, "failed to calloc cam");
        free(dev);
        return NULL;
    }

    dev->name = (char *)VD56G3_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin = config->xclk_pin;
    dev->reset_pin = config->reset_pin;
    dev->pwdn_pin = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops = &vd56g3_ops;
    dev->cur_format = &vd56g3_format_info[CONFIG_CAMERA_VD56G3_MIPI_IF_FORMAT_INDEX_DEFAULT];
    // init para
    cam_vd56g3->vd56g3_para.exposure_val = dev->cur_format->isp_info->isp_v1_info.exp_def;
    cam_vd56g3->vd56g3_para.gain_index = dev->cur_format->isp_info->isp_v1_info.gain_def;
    cam_vd56g3->vd56g3_para.exposure_max = dev->cur_format->isp_info->isp_v1_info.vts - VD56G3_EXP_MAX_OFFSET;
    dev->priv = cam_vd56g3;
    for (size_t i = 0; i < ARRAY_SIZE(vd56g3_total_gain_val_map); i++) {
        if (vd56g3_total_gain_val_map[i] > s_vd56g3_limited_gain_) {
            s_vd56g3_limited_gain_index = i - 1;
            break;
        }
    }

    // Configure sensor power, clock, and SCCB port
    if (vd56g3_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Camera power on failed");
        goto err_free_handler;
    }

    if (vd56g3_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(TAG, "Get sensor ID failed");
        goto err_free_handler;
    } else if (dev->id.pid != VD56G3_MODEL_ID) {
        ESP_LOGE(TAG, "Camera sensor is not VD56G3, PID=0x%x", dev->id.pid);
        goto err_free_handler;
    }
    ESP_LOGI(TAG, "Detected Camera sensor PID=0x%x", dev->id.pid);

    uint32_t device_revision = 0;
    if (vd56g3_read(dev->sccb_handle, VD56G3_REG_REVISION, &device_revision) != ESP_OK)
    {
        ESP_LOGE(TAG, "Get sensor revision failed");
        goto err_free_handler;
    }
    ESP_LOGI(TAG, "Detected Camera sensor revision=0x%x", device_revision);

	if ((device_revision >> 8) == VD56G3_REVISION_CUT2) {
		cam_vd56g3->vd56g3_para.is_fastboot = false;
	} else if ((device_revision >> 8) == VD56G3_REVISION_CUT3) {
		cam_vd56g3->vd56g3_para.is_fastboot = true;
	} else {
        ESP_LOGE(TAG, "Unsupported Cut version %x", device_revision);
        goto err_free_handler;
	}
    ESP_LOGI(TAG, "is_fastboot = %x", cam_vd56g3->vd56g3_para.is_fastboot);

    uint32_t optical_revision = 0;
    if (vd56g3_read(dev->sccb_handle, VD56G3_REG_OPTICAL_REVISION, &optical_revision) != ESP_OK)
    {
        ESP_LOGE(TAG, "Get sensor optical revision failed");
        goto err_free_handler;
    }
    ESP_LOGI(TAG, "Detected Camera sensor optical revision=0x%x", optical_revision);

	if (optical_revision != VD56G3_OPTICAL_REVISION_MONO) {
        ESP_LOGE(TAG, "Incorrect optival revision");
        goto err_free_handler;
    }

    if (!cam_vd56g3->vd56g3_para.is_fastboot) {
		ret = vd56g3_patch(dev);
		if (ret) {
			ESP_LOGE(TAG, "sensor patch failed %d", ret);
			goto err_free_handler;
		}
        ESP_LOGI(TAG, "patch loaded successfully");
	}

	if (vd56g3_boot(dev) != ESP_OK){
			ESP_LOGE(TAG, "sensor boot failed");
			goto err_free_handler;
    }
    ESP_LOGI(TAG, "sensor booted sucessfully");

	if (!cam_vd56g3->vd56g3_para.is_fastboot) {
		ret = vd56g3_vtpatch(dev);
		if (ret) {
			ESP_LOGE(TAG, "sensor VT patch failed %d", ret);
			goto err_free_handler;
		}
        ESP_LOGI(TAG, "vtpatch loaded successfully");
	}

    ESP_LOGI(TAG, "Camera detection complete successfully");
    return dev;

err_free_handler:
    vd56g3_power_off(dev);
    free(dev->priv);
    free(dev);

    return NULL;
}

void vd56g3_testlog(void)
{
    ESP_LOGW(TAG, "vd56g3 library linked correctly");
}

#if CONFIG_CAMERA_VD56G3_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(vd56g3_detect, ESP_CAM_SENSOR_MIPI_CSI, VD56G3_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return vd56g3_detect(config);
}
#endif
