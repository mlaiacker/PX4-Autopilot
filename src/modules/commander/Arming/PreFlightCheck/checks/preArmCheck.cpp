/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "../PreFlightCheck.hpp"

#include <ArmAuthorization.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <HealthFlags.h>


#include <uORB/Subscription.hpp>
#include <dataman/dataman.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/vehicle_global_position.h>

bool
checkVTOLLanding(orb_advert_t *mavlink_log_pub, bool gpos_valid)
{
	uORB::Subscription	mission_sub(ORB_ID(mission));		/**< mission subscription */
	mission_s		mission;
	mission_sub.copy(&mission);

	vehicle_global_position_s v_gpos;
	uORB::Subscription v_gpos_sub(ORB_ID(vehicle_global_position));
	v_gpos_sub.copy(&v_gpos);
	/* Go through all mission items and search for a landing waypoint
	 * if landing waypoint is found: the previous waypoint is checked to be at a feasible distance and altitude given the landing slope */

	bool land_at_arming_found = false;

	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(missionitem);

		if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}
			if(missionitem.nav_cmd == NAV_CMD_VTOL_LAND &&
			   fabsf(missionitem.params[1]-1.0f)<FLT_EPSILON && // GD feature: land at take off
			   !land_at_arming_found ) {
				if (!PX4_ISFINITE(v_gpos.lat) ||
					!PX4_ISFINITE(v_gpos.lon) ||
					!PX4_ISFINITE(v_gpos.alt) ||
					!gpos_valid) {
					mavlink_log_critical(mavlink_log_pub, "Mission: cant update landing, no gps position");
				} else {
					missionitem.lat = v_gpos.lat;
					missionitem.lon = v_gpos.lon;

					land_at_arming_found = true; // only update one item
					PX4_INFO("updating landing pos(%d).",(int)i);
					if (dm_write((dm_item_t)mission.dataman_id, i, DM_PERSIST_POWER_ON_RESET, &missionitem, len) != len) {
						/* not supposed to happen unless the datamanager can't access the SD card, etc. */
						PX4_INFO("failed to write mission");
						return false;
					}
					mavlink_log_info(mavlink_log_pub, "Mission: update land(%i) to cur. pos.",(int)i);
				}
			}
	}
	/* No landing waypoints or no waypoints */
	return true;
}


bool PreFlightCheck::preArmCheck(orb_advert_t *mavlink_log_pub, const vehicle_status_flags_s &status_flags,
				 const safety_s &safety, const arm_requirements_t &arm_requirements, vehicle_status_s &status, bool report_fail)
{
	bool prearm_ok = true;

	// USB not connected
	if (!status_flags.circuit_breaker_engaged_usb_check && status_flags.usb_connected) {
		if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Flying with USB is not safe"); }

		prearm_ok = false;
	}

	// battery and system power status
	if (!status_flags.circuit_breaker_engaged_power_check) {

		// Fail transition if power is not good
		if (!status_flags.condition_power_input_valid) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Connect power module"); }

			prearm_ok = false;
		}

		// main battery level
		set_health_flags(subsystem_info_s::SUBSYSTEM_TYPE_SENSORBATTERY, true, true,
				 status_flags.condition_battery_healthy, status);

		// Only arm if healthy
		if (!status_flags.condition_battery_healthy) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Check battery"); }
			}

			prearm_ok = false;
		}
	}

	// Arm Requirements: mission
	if (arm_requirements.mission) {

		if (!status_flags.condition_auto_mission_available) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! No valid mission"); }
			}

			prearm_ok = false;
		}

		if (!status_flags.condition_global_position_valid) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Missions require a global position"); }
			}

			prearm_ok = false;
		}
	}

	if (arm_requirements.global_position) {

		if (!status_flags.condition_global_position_valid) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Global position required"); }
			}

			prearm_ok = false;
		}

		if (!status_flags.condition_home_position_valid) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Home position invalid"); }
			}

			prearm_ok = false;
		}
	}

	// safety button
	if (safety.safety_switch_available && !safety.safety_off) {
		// Fail transition if we need safety switch press
		if (prearm_ok) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Press safety switch first"); }
		}

		prearm_ok = false;
	}

	if (status_flags.avoidance_system_required && !status_flags.avoidance_system_valid) {
		if (prearm_ok) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Avoidance system not ready"); }
		}

		prearm_ok = false;

	}

	if (arm_requirements.esc_check && status_flags.condition_escs_error) {
		if (prearm_ok) {
			if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! One or more ESCs are offline"); }

			prearm_ok = false;
		}
	}

	if (status.is_vtol) {

		if(!checkVTOLLanding(mavlink_log_pub, status_flags.condition_global_position_valid)) {
			prearm_ok = false;
		}

		if (status.in_transition_mode) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Vehicle is in transition state"); }

				prearm_ok = false;
			}
		}

		if (!status_flags.circuit_breaker_vtol_fw_arming_check
		    && status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
			if (prearm_ok) {
				if (report_fail) { mavlink_log_critical(mavlink_log_pub, "Arming denied! Vehicle is not in multicopter mode"); }

				prearm_ok = false;
			}
		}
	}

	// Arm Requirements: authorization
	// check last, and only if everything else has passed
	if (arm_requirements.arm_authorization && prearm_ok) {
		if (arm_auth_check() != vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED) {
			// feedback provided in arm_auth_check
			prearm_ok = false;
		}
	}


	return prearm_ok;
}
