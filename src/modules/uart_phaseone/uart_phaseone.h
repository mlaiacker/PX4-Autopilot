/*
 * uart_phaseone.h
 *
 *  Created on: Oct 15, 2020
 *      Author: mlaiacker
 */

#ifndef MODULES_UART_PHASEONE_UART_PHASEONE_H_
#define MODULES_UART_PHASEONE_UART_PHASEONE_H_

#include <stdint.h>

#define IX_MAX_SIZE 253
#define IX_HEADER 88

typedef enum{
	GET_SYSTEM_INFO=8,
	SET_APERTURE=10,
	GET_APERTURE=11,
	GET_APERTURE_RANGE=12,
	INCREMENT_APERTURE=13,
	SET_ISO=14,
	GET_ISO=15,
	GET_ISO_RANGE=16,
	INCREMENT_ISO=17,
	SET_SHUTTER_SPEED=18,
	GET_SHUTTER_SPEED=19,
	GET_SHUTTER_SPEED_RANGE=20,
	INCREMENT_SHUTTER_SPEED=21,
	SET_BLACK_CALIB_MODE=25,
	SET_EXPOSURE_MODE=27,
	SET_EXPOSURE_COMPENSATION=28,
	GET_EXPOSURE_COMPENSATION=29,
	GET_EXPOSURE_COMPENSATION_RANGE=30,
	INCREMENT_EXPOSURE_COMPENSATION=31,
	SET_FOCUS_DISTANCE=32,
	SET_FOCUS_ENCODER_POSITION=33,
	GET_FOCUS_DISTANCE=34,
	GET_FOCUS_ENCODER_POSITION=35,
	GET_FOCUS_CURRENT_POSITION=36,
	GET_FOCUS_INFO=38,
	SET_GPS_ENABLE=39,
	GET_GPS_ENABLE=40,
	SET_GPS_RECEIVER=41,
	GET_GPS_RECEIVER=42,
	SET_GPS_BAUD_RATE=43,
	GET_GPS_BAUD_RATE=44,
	CAPTURE=110,
	GET_SYSTEM_STATUS=111,
	GET_EXT_SYSTEM_STATUS=112,
	START_LIVE_VIEW=114,
	STOP_LIVE_VIEW=115,
	SET_REGION_OF_INTEREST=116,
	SET_HDMI_EXPOSURE_MODE=117,
	SET_HDMI_LIGHTNESS=118,
	SET_HDMI_ISO=119,
	SET_HDMI_EXPOSURE_TIME=120,
	SET_HDMI_OVERLAY_MODE=127,
	GET_HDMI_OVERLAY_MODE=128,
	GET_LOCAL_STORAGE_STATUS=130,
	SET_LOCAL_STORAGE_ACTION=131,
	UNDEFINED_ID=255
}IX_CMD_ID;


typedef enum{
	NO_ERROR=0,
	ERR_GENERAL=-1,
	ERR_NOT_SUPPORTED=-2,
	ERR_CANNOT_EXECUTE=-3,
	ERR_MSG_SIZE=-4,
	ERR_PROTOCOL_VERSION=-5,
	ERR_INPUT_OUT_OF_RANGE=-6
}IX_CMD_COMPLETION_CODE_ID;

typedef enum{
	STATUS_ERROR=0, /*System is in error state (general error)*/
	STATUS_READY=1, /*System is ready to capture*/
	STATUS_BUSY=2 /*System is not ready to capture*/
}IX_SYSTEM_STATUS;

typedef enum{
	ASYNC_REPLY_MODE=0, /*Reply message is sent to the host right
after receiving the request message and
before completing the requested
command*/
	SYNC_REPLY_MODE=1 /*Reply message is sent to the host after
completing the requested command*/
}IX_REPLY_MODE;

typedef enum{
	MANUAL_MODE=0, /*Exposure parameters in use are the one set manually*/
	AUTO_MODE=1, /* Exposure parameters are set automatically by the system*/
}IX_EXPOSURE_MODE;

typedef enum{
	NMEA=0,
	NOVATEL=1,
	APPLANIX=2,
	IGI=3,
	GGS_OxTS=4,
	VECTORNAV=5,
}IX_GPS_TYPE;

typedef enum{
	BAUD9600=0,
	BAUD19600=1,
	BAUD38400=2,
	BAUD57600=3,
	BAUD115200=4
}IX_GPS_BAUD;

typedef enum{
	LOCAL_STORAGE_XQD=0,
}IX_STORAGE_TYPE;

typedef enum{
	LOCAL_STORAGE_UNAVAILABLE=0,
	LOCAL_STORAGE_READY=1,
	LOCAL_STORAGE_FORMATTING=2,
}IX_STORAGE_STATUS;


typedef enum{
	MASS_STORAGE_OFF=0,
	MASS_STORAGE_READ_ONLY=1,
}IX_STORAGE_MODE;

typedef enum{
	QUICK_FORMAT=0, /*Run quick format of local storage*/
//	FULL_FORMAT=1, /* Currently not supported */
	ENABLE_MASS_STORAGE_MODE=2, /*Enable USB mass storage mode*/
	DISABLE_MASS_STORAGE_MODE=3, /* Disable USB mass storage mode*/
}IX_STORAGE_ACTION;


#pragma pack(push,1)
	typedef struct{
		int parse_state;
		uint8_t msg_size;
		uint8_t	version;
		int8_t	cmd_completion_code;
		uint8_t	cmd_id;
		uint8_t cmd_data[IX_MAX_SIZE];
		uint8_t checksum;
	}ix_reply_t;

	typedef struct{
		uint8_t header;
		uint8_t msg_size;
		uint8_t	version;
		uint8_t	cmd_id;
	}ix_request_header_t;

	typedef struct{ // #112
		uint8_t 	System_status;
		uint32_t	Remaining_captures;
		uint32_t	Successful_captures_counter;
		uint32_t	Missed_captures_counter;
	}ix_ext_sys_status_t;

	typedef struct{ // #8
		uint8_t		Camera_brand_id;
		uint32_t	Camera_model_id;
		uint8_t		Camera_name[16];
		uint8_t		Lens_brand_id;
		uint16_t	Lens_model_id;
		uint16_t	Lens_focal_length;
		uint8_t		Lens_name[32];
	}ix_system_info_t;

	typedef struct{
		uint8_t		Local_storage_type;
		uint8_t		Local_storage_status;
		uint64_t	Local_storage_size;
		uint64_t	Local_storage_capacity;
		uint32_t	Local_storage_image_capacity;
		uint8_t		Mass_storage_mode;
		uint8_t		_res[4];
	}ix_local_storage_status_t;

	typedef struct{ // #19
		int8_t	Shutter_speed_value_num;
		uint8_t	Shutter_speed_value_denom;
	}ix_shutter_speed_t;

	typedef struct{ // #15
		int8_t	ISO_value_num;
		uint8_t	ISO_value_denom;
	}ix_iso_t;

	typedef struct{ // #11
		int8_t	Apeture_value_num;
		uint8_t	Apeture_value_denom;
	}ix_apeture_t;

	typedef struct{ // #29
		int8_t	Exposure_Compensation_value_num;
		uint8_t	Exposure_Compensation_value_denom;
	}ix_exposure_t;

#pragma pack(pop)


#endif /* MODULES_UART_PHASEONE_UART_PHASEONE_H_ */
