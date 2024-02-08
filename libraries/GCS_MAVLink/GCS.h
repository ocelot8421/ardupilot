/// @file	GCS.h
/// @brief	Interface definition for the various Ground Control System
// protocols.
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 1
#endif

#if HAL_GCS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "GCS_MAVLink.h"
#include <AP_Mission/AP_Mission.h>
#include <stdint.h>
#include "MAVLink_routing.h"
#include <AP_RTC/JitterCorrection.h>
#include <AP_Common/Bitmask.h>
#include <AP_LTM_Telem/AP_LTM_Telem.h>
#include <AP_Devo_Telem/AP_Devo_Telem.h>
#include <AP_Filesystem/AP_Filesystem_Available.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_OpenDroneID/AP_OpenDroneID.h>
#include <AP_Mount/AP_Mount.h>
#include <AC_Fence/AC_Fence.h>
#include <AP_Airspeed/AP_Airspeed_config.h>

#include "ap_message.h"

#define GCS_DEBUG_SEND_MESSAGE_TIMINGS 0

#ifndef HAL_HIGH_LATENCY2_ENABLED
#define HAL_HIGH_LATENCY2_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#ifndef HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
#define HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED (HAVE_FILESYSTEM_SUPPORT && BOARD_FLASH_SIZE > 1024)
#endif

// macros used to determine if a message will fit in the space available.

void gcs_out_of_space_to_send(mavlink_channel_t chan);

// important note: despite the names, these messages do NOT check to
// see if the payload will fit in the buffer.  They check to see if
// the packed message along with any channel overhead will fit.

// PAYLOAD_SIZE returns the amount of space required to send the
// mavlink message with id id on channel chan.  Mavlink2 has higher
// overheads than mavlink1, for example.

// check if a message will fit in the payload space available
#define PAYLOAD_SIZE(chan, id) (unsigned(GCS_MAVLINK::packet_overhead_chan(chan)+MAVLINK_MSG_ID_ ## id ## _LEN))

// HAVE_PAYLOAD_SPACE evaluates to an expression that can be used
// anywhere in the code to determine if the mavlink message with ID id
// can currently fit in the output of _chan.  Note the use of the ","
// operator here to increment a counter.
#define HAVE_PAYLOAD_SPACE(_chan, id) (comm_get_txspace(_chan) >= PAYLOAD_SIZE(_chan, id) ? true : (gcs_out_of_space_to_send(_chan), false))

// CHECK_PAYLOAD_SIZE - macro which may only be used within a
// GCS_MAVLink object's methods.  It inserts code which will
// immediately return false from the current function if there is no
// room to fit the mavlink message with id id on the current object's
// output
#define CHECK_PAYLOAD_SIZE(id) if (txspace() < unsigned(packet_overhead()+MAVLINK_MSG_ID_ ## id ## _LEN)) { gcs_out_of_space_to_send(chan); return false; }

// CHECK_PAYLOAD_SIZE2 - macro which inserts code which will
// immediately return false from the current function if there is no
// room to fit the mavlink message with id id on the mavlink output
// channel "chan".  It is expecting there to be a "chan" variable in
// scope.
#define CHECK_PAYLOAD_SIZE2(id) if (!HAVE_PAYLOAD_SPACE(chan, id)) return false

// CHECK_PAYLOAD_SIZE2_VOID - macro which inserts code which will
// immediately return from the current (void) function if there is no
// room to fit the mavlink message with id id on the mavlink output
// channel "chan".
#define CHECK_PAYLOAD_SIZE2_VOID(chan, id) if (!HAVE_PAYLOAD_SPACE(chan, id)) return

// convenience macros for defining which ap_message ids are in which streams:
#define MAV_STREAM_ENTRY(stream_name)           \
    {                                           \
        GCS_MAVLINK::stream_name,               \
        stream_name ## _msgs,                   \
        ARRAY_SIZE(stream_name ## _msgs)        \
    }
#define MAV_STREAM_TERMINATOR { (streams)0, nullptr, 0 }

#define GCS_MAVLINK_NUM_STREAM_RATES 10
class GCS_MAVLINK_Parameters
{
public:

    GCS_MAVLINK_Parameters();

    static const struct AP_Param::GroupInfo        var_info[];

    // saveable rate of each stream
    AP_Int16        streamRates[GCS_MAVLINK_NUM_STREAM_RATES];
};

#if HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
class DefaultIntervalsFromFiles
{

public:

    DefaultIntervalsFromFiles(uint16_t max_num);
    ~DefaultIntervalsFromFiles();

    void set(ap_message id, uint16_t interval);
    uint16_t num_intervals() const {
        return _num_intervals;
    }
    bool get_interval_for_ap_message_id(ap_message id, uint16_t &interval) const;
    ap_message id_at(uint8_t ofs) const;
    uint16_t interval_at(uint8_t ofs) const;

private:

    struct from_file_default_interval {
        ap_message id;
        uint16_t interval;
    };

    from_file_default_interval *_intervals;

    uint16_t _num_intervals;
    uint16_t _max_intervals;
};
#endif

///
/// @class	GCS_MAVLINK
/// @brief	MAVLink transport control class
///
class GCS_MAVLINK
{
public:
    friend class GCS;

    GCS_MAVLINK(GCS_MAVLINK_Parameters &parameters, AP_HAL::UARTDriver &uart);
    virtual ~GCS_MAVLINK() {}

    void        update_receive(uint32_t max_time_us=1000);
    void        update_send();
    bool        init(uint8_t instance);
    void        send_message(enum ap_message id);
    void        send_text(MAV_SEVERITY severity, const char *fmt, ...) const FMT_PRINTF(3, 4);
    void        queued_param_send();
    void        queued_mission_request_send();

    bool sending_mavlink1() const;

    // returns true if we are requesting any items from the GCS:
    bool requesting_mission_items() const;

    /// Check for available transmit space
    uint16_t txspace() const {
        if (_locked) {
            return 0;
        }
        // there were concerns over return a too-large value for
        // txspace (in case we tried to do too much with the space in
        // a single loop):
        return MIN(_port->txspace(), 8192U);
    }

    // this is called when we discover we'd like to send something but can't:
    void out_of_space_to_send() { out_of_space_to_send_count++; }

    void send_mission_ack(const mavlink_message_t &msg,
                          MAV_MISSION_TYPE mission_type,
                          MAV_MISSION_RESULT result) const {
        CHECK_PAYLOAD_SIZE2_VOID(chan, MISSION_ACK);
        mavlink_msg_mission_ack_send(chan,
                                     msg.sysid,
                                     msg.compid,
                                     result,
                                     mission_type);
    }

    static const MAV_MISSION_TYPE supported_mission_types[3];

    // packetReceived is called on any successful decode of a mavlink message
    virtual void packetReceived(const mavlink_status_t &status,
                                const mavlink_message_t &msg);

    // send a mavlink_message_t out this GCS_MAVLINK connection.
    // Caller is responsible for ensuring space.
    void send_message(uint32_t msgid, const char *pkt) const {
        const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
        if (entry == nullptr) {
            return;
        }
        send_message(pkt, entry);
    }
    void send_message(const char *pkt, const mavlink_msg_entry_t *entry) const {
        _mav_finalize_message_chan_send(chan,
                                        entry->msgid,
                                        pkt,
                                        entry->min_msg_len,
                                        entry->max_msg_len,
                                        entry->crc_extra);
    }

    // accessor for uart
    AP_HAL::UARTDriver *get_uart() { return _port; }

    virtual uint8_t sysid_my_gcs() const = 0;
    virtual bool sysid_enforce() const { return false; }

    void send_parameter_value(const char *param_name,
                              ap_var_type param_type,
                              float param_value);

    // NOTE! The streams enum below and the
    // set of AP_Int16 stream rates _must_ be
    // kept in the same order
    enum streams : uint8_t {
        STREAM_RAW_SENSORS,
        STREAM_EXTENDED_STATUS,
        STREAM_RC_CHANNELS,
        STREAM_RAW_CONTROLLER,
        STREAM_POSITION,
        STREAM_EXTRA1,
        STREAM_EXTRA2,
        STREAM_EXTRA3,
        STREAM_PARAMS,
        STREAM_ADSB,
        NUM_STREAMS
    };

    // streams must be moved out into the top level for
    // GCS_MAVLINK_Parameters to be able to use it.  This is an
    // extensive change, so we 'll just keep them in sync with a
    // static assert for now:
    static_assert(NUM_STREAMS == GCS_MAVLINK_NUM_STREAM_RATES, "num streams must equal num stream rates");

    bool is_high_bandwidth() { return chan == MAVLINK_COMM_0; }
    // return true if this channel has hardware flow control
    bool have_flow_control();

    bool is_active() const {
        return GCS_MAVLINK::active_channel_mask() & (1 << (chan-MAVLINK_COMM_0));
    }
    bool is_streaming() const {
        return sending_bucket_id != no_bucket_to_send;
    }

    mavlink_channel_t get_chan() const { return chan; }
    uint32_t get_last_heartbeat_time() const { return last_heartbeat_time; };

    uint32_t        last_heartbeat_time; // milliseconds

    static uint32_t last_radio_status_remrssi_ms() {
        return last_radio_status.remrssi_ms;
    }
    static float telemetry_radio_rssi(); // 0==no signal, 1==full signal

    // mission item index to be sent on queued msg, delayed or not
    uint16_t mission_item_reached_index = AP_MISSION_CMD_INDEX_NONE;

    // common send functions
    void send_heartbeat(void) const;
    void send_meminfo(void);
    void send_fence_status() const;
    void send_power_status(void);
#if HAL_WITH_MCU_MONITORING
    void send_mcu_status(void);
#endif
    void send_battery_status(const uint8_t instance) const;
    bool send_battery_status();
    void send_distance_sensor();
    // send_rangefinder sends only if a downward-facing instance is
    // found.  Rover overrides this!
    virtual void send_rangefinder() const;
    void send_proximity();
    virtual void send_nav_controller_output() const = 0;
    virtual void send_pid_tuning() = 0;
    void send_ahrs2();
    void send_system_time() const;
    void send_rc_channels() const;
    void send_rc_channels_raw() const;
    void send_raw_imu();

    void send_scaled_pressure_instance(uint8_t instance, void (*send_fn)(mavlink_channel_t chan, uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature, int16_t temperature_press_diff));
    void send_scaled_pressure();
    void send_scaled_pressure2();
    virtual void send_scaled_pressure3(); // allow sub to override this
    void send_simstate() const;
    void send_sim_state() const;
    void send_ahrs();
    void send_battery2();
#if AP_OPTICALFLOW_ENABLED
    void send_opticalflow();
#endif
    virtual void send_attitude() const;
    /**
     * Custom mavlink functions
    */
    void send_abz_status_spray() const;
    void send_abz_start_mission() const;
    void send_abz_update_mission() const;
    void send_abz_plan_progress() const;
    void send_abz_is_return_point() const;
    void send_abz_is_mission_in_progress() const;
    void send_abz_returning_point_cor_action() const;
    void send_abz_mission_params_action() const;
    void send_abz_liter_left_action() const;
    void send_abz_drone_version_action() const;
    void send_abz_firmware_version_action() const;
    void send_abz_empty_point_cor_action() const;
    void send_abz_lemon_point_cor_action() const;
    void send_abz_empty_tank_action() const;

    virtual void send_attitude_quaternion() const;
    void send_autopilot_version() const;
    void send_extended_sys_state() const;
    void send_local_position() const;
    void send_vfr_hud();
    void send_vibration() const;
    void send_gimbal_device_attitude_status() const;
    void send_named_float(const char *name, float value) const;
    void send_home_position() const;
    void send_gps_global_origin() const;
    virtual void send_attitude_target() {};
    virtual void send_position_target_global_int() { };
    virtual void send_position_target_local_ned() { };
    void send_servo_output_raw();
    void send_accelcal_vehicle_position(uint32_t position);
    void send_scaled_imu(uint8_t instance, void (*send_fn)(mavlink_channel_t chan, uint32_t time_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag, int16_t temperature));
    void send_sys_status();
    void send_set_position_target_global_int(uint8_t target_system, uint8_t target_component, const Location& loc);
    void send_rpm() const;
    void send_generator_status() const;
    virtual void send_winch_status() const {};
    void send_water_depth() const;
    int8_t battery_remaining_pct(const uint8_t instance) const;

#if HAL_HIGH_LATENCY2_ENABLED
    void send_high_latency2() const;
#endif // HAL_HIGH_LATENCY2_ENABLED
    void send_uavionix_adsb_out_status() const;
    void send_autopilot_state_for_gimbal_device() const;

    // lock a channel, preventing use by MAVLink
    void lock(bool _lock) {
        _locked = _lock;
    }
    // returns true if this channel isn't available for MAVLink
    bool locked() const {
        return _locked;
    }

    // return a bitmap of active channels. Used by libraries to loop
    // over active channels to send to all active channels    
    static uint8_t active_channel_mask(void) { return mavlink_active; }

    // return a bitmap of streaming channels
    static uint8_t streaming_channel_mask(void) { return chan_is_streaming; }

    // return a bitmap of private channels
    static uint8_t private_channel_mask(void) { return mavlink_private; }

    // set a channel as private. Private channels get sent heartbeats, but
    // don't get broadcast packets or forwarded packets
    static void set_channel_private(mavlink_channel_t chan);

    // return true if channel is private
    static bool is_private(mavlink_channel_t _chan) {
        return (mavlink_private & (1U<<(unsigned)_chan)) != 0;
    }
    
    // return true if channel is private
    bool is_private(void) const { return is_private(chan); }

#if HAL_HIGH_LATENCY2_ENABLED
    // return true if the link should be sending. Will return false if is a high latency link AND is not active
    bool should_send() { return is_high_latency_link ? high_latency_link_enabled : true; }
#endif

    /*
      send a MAVLink message to all components with this vehicle's system id
      This is a no-op if no routes to components have been learned
    */
    static void send_to_components(uint32_t msgid, const char *pkt, uint8_t pkt_len) { routing.send_to_components(msgid, pkt, pkt_len); }

    /*
      allow forwarding of packets / heartbeats to be blocked as required by some components to reduce traffic
    */
    static void disable_channel_routing(mavlink_channel_t chan) { routing.no_route_mask |= (1U<<(chan-MAVLINK_COMM_0)); }
    
    /*
      search for a component in the routing table with given mav_type and retrieve it's sysid, compid and channel
      returns if a matching component is found
     */
    static bool find_by_mavtype(uint8_t mav_type, uint8_t &sysid, uint8_t &compid, mavlink_channel_t &channel) { return routing.find_by_mavtype(mav_type, sysid, compid, channel); }

    /*
      search for the first vehicle or component in the routing table with given mav_type and component id and retrieve its sysid and channel
      returns true if a match is found
     */
    static bool find_by_mavtype_and_compid(uint8_t mav_type, uint8_t compid, uint8_t &sysid, mavlink_channel_t &channel) { return routing.find_by_mavtype_and_compid(mav_type, compid, sysid, channel); }

    // update signing timestamp on GPS lock
    static void update_signing_timestamp(uint64_t timestamp_usec);

    // return current packet overhead for a channel
    static uint8_t packet_overhead_chan(mavlink_channel_t chan);

    // alternative protocol function handler
    FUNCTOR_TYPEDEF(protocol_handler_fn_t, bool, uint8_t, AP_HAL::UARTDriver *);

    struct stream_entries {
        const streams stream_id;
        const ap_message *ap_message_ids;
        const uint8_t num_ap_message_ids;
    };
    // vehicle subclass cpp files should define this:
    static const struct stream_entries all_stream_entries[];

    virtual uint64_t capabilities() const;
    uint16_t get_stream_slowdown_ms() const { return stream_slowdown_ms; }
    uint8_t get_last_txbuf() const { return last_txbuf; }

    MAV_RESULT set_message_interval(uint32_t msg_id, int32_t interval_us);

protected:
   MAV_RESULT handle_command_do_abz_sprayer(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_abz_sprayer_rtm(const mavlink_command_long_t &packet);
    MAV_RESULT handel_command_do_abz_liters_needed(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_abz_calculate_points(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_abz_request_drone_version(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_abz_custom_capacity(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_abz_calculate_liter_need(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_abz_changes(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_abz_request_firmware_version(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_abz_request_mission_params(const mavlink_command_long_t &packet);
    bool mavlink_coordinate_frame_to_location_alt_frame(MAV_FRAME coordinate_frame,
                                                        Location::AltFrame &frame);

    // overridable method to check for packet acceptance. Allows for
    // enforcement of GCS sysid
    bool accept_packet(const mavlink_status_t &status, const mavlink_message_t &msg) const;
    void set_ekf_origin(const Location& loc);

    virtual MAV_MODE base_mode() const = 0;
    MAV_STATE system_status() const;
    virtual MAV_STATE vehicle_system_status() const = 0;

    virtual MAV_VTOL_STATE vtol_state() const { return MAV_VTOL_STATE_UNDEFINED; }
    virtual MAV_LANDED_STATE landed_state() const { return MAV_LANDED_STATE_UNDEFINED; }

    // return a MAVLink parameter type given a AP_Param type
    static MAV_PARAM_TYPE mav_param_type(enum ap_var_type t);

    AP_Param *                  _queued_parameter;      ///< next parameter to
                                                        // be sent in queue
    mavlink_channel_t           chan;
    uint8_t packet_overhead(void) const { return packet_overhead_chan(chan); }

    // saveable rate of each stream
    AP_Int16        *streamRates;

    void handle_heartbeat(const mavlink_message_t &msg) const;

    virtual bool persist_streamrates() const { return false; }
    void handle_request_data_stream(const mavlink_message_t &msg);

    virtual void handle_command_ack(const mavlink_message_t &msg);
    void handle_set_mode(const mavlink_message_t &msg);
    void handle_command_int(const mavlink_message_t &msg);

    MAV_RESULT handle_command_int_do_set_home(const mavlink_command_int_t &packet);
    virtual MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet);

    virtual bool set_home_to_current_location(bool lock) = 0;
    virtual bool set_home(const Location& loc, bool lock) = 0;

    virtual MAV_RESULT handle_command_component_arm_disarm(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_set_home(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_aux_function(const mavlink_command_long_t &packet);
    void handle_mission_request_list(const mavlink_message_t &msg);
    void handle_mission_request(const mavlink_message_t &msg) const;
    void handle_mission_request_int(const mavlink_message_t &msg) const;
    void handle_mission_clear_all(const mavlink_message_t &msg) const;

    // Note that there exists a relatively new mavlink DO command,
    // MAV_CMD_DO_SET_MISSION_CURRENT which provides an acknowledgement
    // that the command has been received, rather than the GCS having to
    // rely on getting back an identical sequence number as some currently
    // do.
    virtual void handle_mission_set_current(AP_Mission &mission, const mavlink_message_t &msg);
    void handle_mission_count(const mavlink_message_t &msg);
    void handle_mission_write_partial_list(const mavlink_message_t &msg);
    void handle_mission_item(const mavlink_message_t &msg);

    void handle_distance_sensor(const mavlink_message_t &msg);
    void handle_obstacle_distance(const mavlink_message_t &msg);
    void handle_obstacle_distance_3d(const mavlink_message_t &msg);

    void handle_adsb_message(const mavlink_message_t &msg);

    void handle_osd_param_config(const mavlink_message_t &msg) const;

    void handle_common_param_message(const mavlink_message_t &msg);
    void handle_param_set(const mavlink_message_t &msg);
    void handle_param_request_list(const mavlink_message_t &msg);
    void handle_param_request_read(const mavlink_message_t &msg);
    virtual bool params_ready() const { return true; }
    void handle_rc_channels_override(const mavlink_message_t &msg);
    void handle_system_time_message(const mavlink_message_t &msg);
    void handle_common_rally_message(const mavlink_message_t &msg);
    void handle_rally_fetch_point(const mavlink_message_t &msg);
    void handle_rally_point(const mavlink_message_t &msg) const;
#if HAL_MOUNT_ENABLED
    virtual void handle_mount_message(const mavlink_message_t &msg);
#endif
    void handle_fence_message(const mavlink_message_t &msg);
    void handle_param_value(const mavlink_message_t &msg);
    void handle_radio_status(const mavlink_message_t &msg, bool log_radio);
    void handle_serial_control(const mavlink_message_t &msg);
    void handle_vision_position_delta(const mavlink_message_t &msg);

    void handle_common_message(const mavlink_message_t &msg);
    void handle_set_gps_global_origin(const mavlink_message_t &msg);
    void handle_setup_signing(const mavlink_message_t &msg) const;
    virtual MAV_RESULT handle_preflight_reboot(const mavlink_command_long_t &packet);

    // reset a message interval via mavlink:
    MAV_RESULT handle_command_set_message_interval(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_get_message_interval(const mavlink_command_long_t &packet);
    bool get_ap_message_interval(ap_message id, uint16_t &interval_ms) const;
    MAV_RESULT handle_command_request_message(const mavlink_command_long_t &packet);

    MAV_RESULT handle_rc_bind(const mavlink_command_long_t &packet);
    virtual MAV_RESULT handle_flight_termination(const mavlink_command_long_t &packet);

    void handle_send_autopilot_version(const mavlink_message_t &msg);
    MAV_RESULT handle_command_request_autopilot_capabilities(const mavlink_command_long_t &packet);

    virtual void send_banner();

    // send a (textual) message to the GCS that a received message has
    // been deprecated
    void send_received_message_deprecation_warning(const char *message);

    void handle_device_op_read(const mavlink_message_t &msg);
    void handle_device_op_write(const mavlink_message_t &msg);

    void send_timesync();
    // returns the time a timesync message was most likely received:
    uint64_t timesync_receive_timestamp_ns() const;
    // returns a timestamp suitable for packing into the ts1 field of TIMESYNC:
    uint64_t timesync_timestamp_ns() const;
    void handle_timesync(const mavlink_message_t &msg);
    struct {
        int64_t sent_ts1;
        uint32_t last_sent_ms;
        const uint16_t interval_ms = 10000;
    }  _timesync_request;

    void handle_statustext(const mavlink_message_t &msg) const;
    void handle_named_value(const mavlink_message_t &msg) const;

    bool telemetry_delayed() const;
    virtual uint32_t telem_delay() const = 0;

    MAV_RESULT handle_command_run_prearm_checks(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_preflight_set_sensor_offsets(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_flash_bootloader(const mavlink_command_long_t &packet);

    // generally this should not be overridden; Plane overrides it to ensure
    // failsafe isn't triggered during calibration
    virtual MAV_RESULT handle_command_preflight_calibration(const mavlink_command_long_t &packet);

    virtual MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_long_t &packet);
    virtual MAV_RESULT _handle_command_preflight_calibration_baro();

    MAV_RESULT handle_command_preflight_can(const mavlink_command_long_t &packet);

    virtual MAV_RESULT handle_command_do_set_mission_current(const mavlink_command_long_t &packet);

    MAV_RESULT handle_command_battery_reset(const mavlink_command_long_t &packet);
    void handle_command_long(const mavlink_message_t &msg);
    MAV_RESULT handle_command_accelcal_vehicle_pos(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_set_roi_sysid(const uint8_t sysid);
    MAV_RESULT handle_command_do_set_roi_sysid(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_do_set_roi_sysid(const mavlink_command_long_t &packet);
    virtual MAV_RESULT handle_command_mount(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_mag_cal(const mavlink_command_long_t &packet);
    virtual MAV_RESULT handle_command_long_packet(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_camera(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_send_banner(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_set_roi(const mavlink_command_int_t &packet);
    MAV_RESULT handle_command_do_set_roi(const mavlink_command_long_t &packet);
    virtual MAV_RESULT handle_command_do_set_roi(const Location &roi_loc);
    MAV_RESULT handle_command_do_gripper(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_sprayer(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_set_mode(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_get_home_position(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_do_fence_enable(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_debug_trap(const mavlink_command_long_t &packet);
    MAV_RESULT handle_command_set_ekf_source_set(const mavlink_command_long_t &packet);

    /*
      handle MAV_CMD_CAN_FORWARD and CAN_FRAME messages for CAN over MAVLink
     */
    void can_frame_callback(uint8_t bus, const AP_HAL::CANFrame &);
    MAV_RESULT handle_can_forward(const mavlink_command_long_t &packet, const mavlink_message_t &msg);
    void handle_can_frame(const mavlink_message_t &msg) const;

#if AP_OPTICALFLOW_ENABLED
    void handle_optical_flow(const mavlink_message_t &msg);
#endif

    MAV_RESULT handle_fixed_mag_cal_yaw(const mavlink_command_long_t &packet);

    // default empty handling of LANDING_TARGET
    virtual void handle_landing_target(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) { }
    // vehicle-overridable message send function
    virtual bool try_send_message(enum ap_message id);
    virtual void send_global_position_int();

    // message sending functions:
    bool try_send_mission_message(enum ap_message id);
    void send_hwstatus();
    void handle_data_packet(const mavlink_message_t &msg);

    // these two methods are called after current_loc is updated:
    virtual int32_t global_position_int_alt() const;
    virtual int32_t global_position_int_relative_alt() const;

    virtual float vfr_hud_climbrate() const;
    virtual float vfr_hud_airspeed() const;
    virtual int16_t vfr_hud_throttle() const { return 0; }
    virtual float vfr_hud_alt() const;

#if HAL_HIGH_LATENCY2_ENABLED
    virtual int16_t high_latency_target_altitude() const { return 0; }
    virtual uint8_t high_latency_tgt_heading() const { return 0; }
    virtual uint16_t high_latency_tgt_dist() const { return 0; }
    virtual uint8_t high_latency_tgt_airspeed() const { return 0; }
    virtual uint8_t high_latency_wind_speed() const { return 0; }
    virtual uint8_t high_latency_wind_direction() const { return 0; }
    int8_t high_latency_air_temperature() const;

    MAV_RESULT handle_control_high_latency(const mavlink_command_long_t &packet);

    // true if this is a high latency link
    bool is_high_latency_link;
    bool high_latency_link_enabled;
#endif // HAL_HIGH_LATENCY2_ENABLED
    
    static constexpr const float magic_force_arm_value = 2989.0f;
    static constexpr const float magic_force_disarm_value = 21196.0f;

    void manual_override(class RC_Channel *c, int16_t value_in, uint16_t offset, float scaler, const uint32_t tnow, bool reversed = false);

    uint8_t receiver_rssi() const;

    /*
      correct an offboard timestamp in microseconds to a local time
      since boot in milliseconds
     */
    uint32_t correct_offboard_timestamp_usec_to_ms(uint64_t offboard_usec, uint16_t payload_size);

    // converts a COMMAND_LONG packet to a COMMAND_INT packet, where
    // the command-long packet is assumed to be in the supplied frame.
    // If location is not present in the command then just omit frame.
    static void convert_COMMAND_LONG_to_COMMAND_INT(const mavlink_command_long_t &in, mavlink_command_int_t &out, MAV_FRAME frame = MAV_FRAME_GLOBAL_RELATIVE_ALT);

    // methods to extract a Location object from a command_long or command_int
    bool location_from_command_t(const mavlink_command_long_t &in, MAV_FRAME in_frame, Location &out);
    bool location_from_command_t(const mavlink_command_int_t &in, Location &out);

private:

    const AP_SerialManager::UARTState *uartstate;

    // last time we got a non-zero RSSI from RADIO_STATUS
    static struct LastRadioStatus {
        uint32_t remrssi_ms;
        uint8_t rssi;
        uint32_t received_ms; // time RADIO_STATUS received
    } last_radio_status;

    void log_mavlink_stats();

    uint32_t last_accel_cal_ms; // used to rate limit accel cals for bad links

    MAV_RESULT _set_mode_common(const MAV_MODE base_mode, const uint32_t custom_mode);

    // send a (textual) message to the GCS that a received message has
    // been deprecated
    uint32_t last_deprecation_warning_send_time_ms;
    const char *last_deprecation_message;

    void service_statustext(void);

    virtual void        handleMessage(const mavlink_message_t &msg) = 0;

    MAV_RESULT handle_servorelay_message(const mavlink_command_long_t &packet);

    static bool command_long_stores_location(const MAV_CMD command);

    bool calibrate_gyros();

    /// The stream we are communicating over
    AP_HAL::UARTDriver *_port;

    /// Perform queued sending operations
    ///
    enum ap_var_type            _queued_parameter_type; ///< type of the next
                                                        // parameter
    AP_Param::ParamToken        _queued_parameter_token; ///AP_Param token for
                                                         // next() call
    uint16_t                    _queued_parameter_index; ///< next queued
                                                         // parameter's index
    uint16_t                    _queued_parameter_count; ///< saved count of
                                                         // parameters for
                                                         // queued send
    uint32_t                    _queued_parameter_send_time_ms;

    // number of extra ms to add to slow things down for the radio
    uint16_t         stream_slowdown_ms;
    // last reported radio buffer percent available
    uint8_t          last_txbuf = 100;

    // outbound ("deferred message") queue.

    // "special" messages such as heartbeat, next_param etc are stored
    // separately to stream-rated messages like AHRS2 etc.  If these
    // were to be stored in buckets then they would be slowed down
    // based on stream_slowdown, which we have not traditionally done.
    struct deferred_message_t {
        const ap_message id;
        uint16_t interval_ms;
        uint16_t last_sent_ms; // from AP_HAL::millis16()
    } deferred_message[3] = {
        { MSG_HEARTBEAT, },
        { MSG_NEXT_PARAM, },
        { MSG_HIGH_LATENCY2, },
    };
    // returns index of id in deferred_message[] or -1 if not present
    int8_t get_deferred_message_index(const ap_message id) const;
    // returns index of a message in deferred_message[] which should
    // be sent (or -1 if none to send at the moment)
    int8_t deferred_message_to_send_index(uint16_t now16_ms);
    // cache of which deferred message should be sent next:
    int8_t next_deferred_message_to_send_cache = -1;

    struct deferred_message_bucket_t {
        Bitmask<MSG_LAST> ap_message_ids;
        uint16_t interval_ms;
        uint16_t last_sent_ms; // from AP_HAL::millis16()
    };
    deferred_message_bucket_t deferred_message_bucket[10];
    static const uint8_t no_bucket_to_send = -1;
    static const ap_message no_message_to_send = (ap_message)-1;
    uint8_t sending_bucket_id = no_bucket_to_send;
    Bitmask<MSG_LAST> bucket_message_ids_to_send;

    ap_message next_deferred_bucket_message_to_send(uint16_t now16_ms);
    void find_next_bucket_to_send(uint16_t now16_ms);
    void remove_message_from_bucket(int8_t bucket, ap_message id);

    // bitmask of IDs the code has spontaneously decided it wants to
    // send out.  Examples include HEARTBEAT (gcs_send_heartbeat)
    Bitmask<MSG_LAST> pushed_ap_message_ids;

    // returns true if it is OK to send a message while we are in
    // delay callback.  In particular, when we are doing sensor init
    // we still send heartbeats.
    bool should_send_message_in_delay_callback(const ap_message id) const;

    // if true is returned, interval will contain the default interval for id
    bool get_default_interval_for_ap_message(const ap_message id, uint16_t &interval) const;
    //  if true is returned, interval will contain the default interval for id
    bool get_default_interval_for_mavlink_message_id(const uint32_t mavlink_message_id, uint16_t &interval) const;
    // returns an interval in milliseconds for any ap_message in stream id
    uint16_t get_interval_for_stream(GCS_MAVLINK::streams id) const;
    // set an inverval for a specific mavlink message.  Returns false
    // on failure (typically because there is no mapping from that
    // mavlink ID to an ap_message)
    bool set_mavlink_message_id_interval(const uint32_t mavlink_id,
                                         const uint16_t interval_ms);
    // map a mavlink ID to an ap_message which, if passed to
    // try_send_message, will cause a mavlink message with that id to
    // be emitted.  Returns MSG_LAST if no such mapping exists.
    ap_message mavlink_id_to_ap_message_id(const uint32_t mavlink_id) const;
    // set the interval at which an ap_message should be emitted (in ms)
    bool set_ap_message_interval(enum ap_message id, uint16_t interval_ms);
    // call set_ap_message_interval for each entry in a stream,
    // the interval being based on the stream's rate
    void initialise_message_intervals_for_stream(GCS_MAVLINK::streams id);
    // call initialise_message_intervals_for_stream on every stream:
    void initialise_message_intervals_from_streamrates();
    // boolean that indicated that message intervals have been set
    // from streamrates:
    bool deferred_messages_initialised;
#if HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
    // read configuration files from (e.g.) SD and ROMFS, set
    // intervals from same
    void initialise_message_intervals_from_config_files();
    // read file, set message intervals from it:
    void get_intervals_from_filepath(const char *path, DefaultIntervalsFromFiles &);
#endif
    // return interval deferred message bucket should be sent after.
    // When sending parameters and waypoints this may be longer than
    // the interval specified in "deferred"
    uint16_t get_reschedule_interval_ms(const deferred_message_bucket_t &deferred) const;

    bool do_try_send_message(const ap_message id);

    // time when we missed sending a parameter for GCS
    static uint32_t reserve_param_space_start_ms;
    
    // bitmask of what mavlink channels are active
    static uint8_t mavlink_active;

    // bitmask of what mavlink channels are private
    static uint8_t mavlink_private;

    // bitmask of what mavlink channels are streaming
    static uint8_t chan_is_streaming;

    // mavlink routing object
    static MAVLink_routing routing;

    struct pending_param_request {
        mavlink_channel_t chan;
        int16_t param_index;
        char param_name[AP_MAX_NAME_SIZE+1];
    };

    struct pending_param_reply {
        mavlink_channel_t chan;        
        float value;
        enum ap_var_type p_type;
        int16_t param_index;
        uint16_t count;
        char param_name[AP_MAX_NAME_SIZE+1];
    };

    // queue of pending parameter requests and replies
    static ObjectBuffer<pending_param_request> param_requests;
    static ObjectBuffer<pending_param_reply> param_replies;

    // have we registered the IO timer callback?
    static bool param_timer_registered;

    // IO timer callback for parameters
    void param_io_timer(void);

    uint8_t send_parameter_async_replies();

    enum class FTP_OP : uint8_t {
        None = 0,
        TerminateSession = 1,
        ResetSessions = 2,
        ListDirectory = 3,
        OpenFileRO = 4,
        ReadFile = 5,
        CreateFile = 6,
        WriteFile = 7,
        RemoveFile = 8,
        CreateDirectory = 9,
        RemoveDirectory = 10,
        OpenFileWO = 11,
        TruncateFile = 12,
        Rename = 13,
        CalcFileCRC32 = 14,
        BurstReadFile = 15,
        Ack = 128,
        Nack = 129,
    };

    enum class FTP_ERROR : uint8_t {
        None = 0,
        Fail = 1,
        FailErrno = 2,
        InvalidDataSize = 3,
        InvalidSession = 4,
        NoSessionsAvailable = 5,
        EndOfFile = 6,
        UnknownCommand = 7,
        FileExists = 8,
        FileProtected = 9,
        FileNotFound = 10,
    };

    struct pending_ftp {
        uint32_t offset;
        mavlink_channel_t chan;        
        uint16_t seq_number;
        FTP_OP opcode;
        FTP_OP req_opcode;
        bool  burst_complete;
        uint8_t size;
        uint8_t session;
        uint8_t sysid;
        uint8_t compid;
        uint8_t data[239];
    };

    enum class FTP_FILE_MODE {
        Read,
        Write,
    };

    struct ftp_state {
        ObjectBuffer<pending_ftp> *requests;
        ObjectBuffer<pending_ftp> *replies;

        // session specific info, currently only support a single session over all links
        int fd = -1;
        FTP_FILE_MODE mode; // work around AP_Filesystem not supporting file modes
        int16_t current_session;
        uint32_t last_send_ms;
        uint8_t need_banner_send_mask;
    };
    static struct ftp_state ftp;

    static void ftp_error(struct pending_ftp &response, FTP_ERROR error); // FTP helper method for packing a NAK
    static int gen_dir_entry(char *dest, size_t space, const char * path, const struct dirent * entry); // FTP helper for emitting a dir response
    static void ftp_list_dir(struct pending_ftp &request, struct pending_ftp &response);

    bool ftp_init(void);
    void handle_file_transfer_protocol(const mavlink_message_t &msg);
    void send_ftp_replies(void);
    void ftp_worker(void);
    void ftp_push_replies(pending_ftp &reply);

    void send_distance_sensor(const class AP_RangeFinder_Backend *sensor, const uint8_t instance) const;

    virtual bool handle_guided_request(AP_Mission::Mission_Command &cmd) = 0;
    virtual void handle_change_alt_request(AP_Mission::Mission_Command &cmd) {};
    void handle_common_mission_message(const mavlink_message_t &msg);

    void handle_vicon_position_estimate(const mavlink_message_t &msg);
    void handle_vision_position_estimate(const mavlink_message_t &msg);
    void handle_global_vision_position_estimate(const mavlink_message_t &msg);
    void handle_att_pos_mocap(const mavlink_message_t &msg);
    void handle_odometry(const mavlink_message_t &msg);
    void handle_common_vision_position_estimate_data(const uint64_t usec,
                                                     const float x,
                                                     const float y,
                                                     const float z,
                                                     const float roll,
                                                     const float pitch,
                                                     const float yaw,
                                                     const float covariance[21],
                                                     const uint8_t reset_counter,
                                                     const uint16_t payload_size);
    void handle_vision_speed_estimate(const mavlink_message_t &msg);
    void handle_landing_target(const mavlink_message_t &msg);

    void lock_channel(const mavlink_channel_t chan, bool lock);

    mavlink_signing_t signing;
    static mavlink_signing_streams_t signing_streams;
    static uint32_t last_signing_save_ms;

    static StorageAccess _signing_storage;
    static bool signing_key_save(const struct SigningKey &key);
    static bool signing_key_load(struct SigningKey &key);
    void load_signing_key(void);
    bool signing_enabled(void) const;
    static void save_signing_timestamp(bool force_save_now);

#if HAL_MAVLINK_INTERVALS_FROM_FILES_ENABLED
    // structure containing default intervals read from files for this
    // link:
    DefaultIntervalsFromFiles *default_intervals_from_files;
#endif

    // alternative protocol handler support
    struct {
        GCS_MAVLINK::protocol_handler_fn_t handler;
        uint32_t last_mavlink_ms;
        uint32_t last_alternate_ms;
        bool active;
    } alternative;

    JitterCorrection lag_correction;
    
    // we cache the current location and send it even if the AHRS has
    // no idea where we are:
    struct Location global_position_current_loc;

    uint8_t last_tx_seq;
    uint16_t send_packet_count;
    uint16_t out_of_space_to_send_count; // number of times HAVE_PAYLOAD_SPACE and friends have returned false

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    struct {
        uint32_t longest_time_us;
        ap_message longest_id;
        uint32_t no_space_for_message;
        uint16_t statustext_last_sent_ms;
        uint32_t behind;
        uint32_t out_of_time;
        uint16_t fnbts_maxtime;
        uint32_t max_retry_deferred_body_us;
        uint8_t max_retry_deferred_body_type;
    } try_send_message_stats;
    uint16_t max_slowdown_ms;
#endif

    uint32_t last_mavlink_stats_logged;

    uint8_t last_battery_status_idx;

    // if we've ever sent a DISTANCE_SENSOR message out of an
    // orientation we continue to send it out, even if it is not
    // longer valid.
    uint8_t proximity_ever_valid_bitmask;

    // true if we should NOT do MAVLink on this port (usually because
    // someone's doing SERIAL_CONTROL over mavlink)
    bool _locked;
};

/// @class GCS
/// @brief global GCS object
class GCS
{

public:

    GCS() {
        if (_singleton  == nullptr) {
            _singleton = this;
        } else {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            // this is a serious problem, but we don't need to kill a
            // real vehicle
            AP_HAL::panic("GCS must be singleton");
#endif
        }
    };

    static class GCS *get_singleton() {
        return _singleton;
    }

    virtual uint32_t custom_mode() const = 0;
    virtual MAV_TYPE frame_type() const = 0;
    virtual const char* frame_string() const { return nullptr; }

    struct statustext_t {
        mavlink_statustext_t    msg;
        uint16_t                entry_created_ms;
        uint8_t                 bitmask;
    };
    class StatusTextQueue : public ObjectArray<statustext_t> {
    public:
        using ObjectArray::ObjectArray;
        HAL_Semaphore &semaphore() { return _sem; }
        void prune();
    private:
        // a lock for the statustext queue, to make it safe to use send_text()
        // from multiple threads
        HAL_Semaphore _sem;

        uint32_t last_prune_ms;
    };

    StatusTextQueue &statustext_queue() {
        return _statustext_queue;
    }

    // last time traffic was seen from my designated GCS.  traffic
    // includes heartbeats and some manual control messages.
    uint32_t sysid_myggcs_last_seen_time_ms() const {
        return _sysid_mygcs_last_seen_time_ms;
    }
    // called when valid traffic has been seen from our GCS
    void sysid_myggcs_seen(uint32_t seen_time_ms) {
        _sysid_mygcs_last_seen_time_ms = seen_time_ms;
    }

    void send_to_active_channels(uint32_t msgid, const char *pkt);

    void send_text(MAV_SEVERITY severity, const char *fmt, ...) FMT_PRINTF(3, 4);
    void send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list);
    virtual void send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list, uint8_t mask);
    uint8_t statustext_send_channel_mask() const;

    virtual GCS_MAVLINK *chan(const uint8_t ofs) = 0;
    virtual const GCS_MAVLINK *chan(const uint8_t ofs) const = 0;
    // return the number of valid GCS objects
    uint8_t num_gcs() const { return _num_gcs; };
    void send_message(enum ap_message id);
    void send_mission_item_reached_message(uint16_t mission_index);
    void send_named_float(const char *name, float value) const;

    void send_parameter_value(const char *param_name,
                              ap_var_type param_type,
                              float param_value);

    static class MissionItemProtocol_Waypoints *_missionitemprotocol_waypoints;
    static class MissionItemProtocol_Rally *_missionitemprotocol_rally;
#if AP_FENCE_ENABLED
    static class MissionItemProtocol_Fence *_missionitemprotocol_fence;
#endif
    class MissionItemProtocol *get_prot_for_mission_type(const MAV_MISSION_TYPE mission_type) const;
    void try_send_queued_message_for_type(MAV_MISSION_TYPE type) const;

    void update_send();
    void update_receive();

    // minimum amount of time (in microseconds) that must remain in
    // the main scheduler loop before we are allowed to send any
    // mavlink messages.  We want to prioritise the main flight
    // control loop over communications
    virtual uint16_t min_loop_time_remaining_for_message_send_us() const {
        return 200;
    }

    void init();
    void setup_console();
    void setup_uarts();

    bool out_of_time() const;

    // frsky backend
    class AP_Frsky_Telem *frsky;

#if AP_LTM_TELEM_ENABLED
    // LTM backend
    AP_LTM_Telem ltm_telemetry;
#endif

#if AP_DEVO_TELEM_ENABLED
    // Devo backend
    AP_DEVO_Telem devo_telemetry;
#endif

    // install an alternative protocol handler
    bool install_alternative_protocol(mavlink_channel_t chan, GCS_MAVLINK::protocol_handler_fn_t handler);

    // get the VFR_HUD throttle
    int16_t get_hud_throttle(void) const { return num_gcs()>0?chan(0)->vfr_hud_throttle():0; }

    // update uart pass-thru
    void update_passthru();

    void get_sensor_status_flags(uint32_t &present, uint32_t &enabled, uint32_t &health);
    virtual bool vehicle_initialised() const { return true; }

    virtual bool simple_input_active() const { return false; }
    virtual bool supersimple_input_active() const { return false; }

    // set message interval for a given serial port and message id
    // this function is for use by lua scripts, most consumers should use the channel level function
    MAV_RESULT set_message_interval(uint8_t port_num, uint32_t msg_id, int32_t interval_us);

    uint8_t get_channel_from_port_number(uint8_t port_num);

#if HAL_HIGH_LATENCY2_ENABLED
    void enable_high_latency_connections(bool enabled);
#endif // HAL_HIGH_LATENCY2_ENABLED

protected:

    virtual uint8_t sysid_this_mav() const = 0;

    virtual GCS_MAVLINK *new_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                                 AP_HAL::UARTDriver &uart) = 0;

    uint32_t control_sensors_present;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;
    virtual void update_vehicle_sensor_status_flags() {}

    GCS_MAVLINK_Parameters chan_parameters[MAVLINK_COMM_NUM_BUFFERS];
    uint8_t _num_gcs;
    GCS_MAVLINK *_chan[MAVLINK_COMM_NUM_BUFFERS];

private:

    static GCS *_singleton;

    void create_gcs_mavlink_backend(GCS_MAVLINK_Parameters &params,
                                    AP_HAL::UARTDriver &uart);

    char statustext_printf_buffer[256+1];

    virtual AP_GPS::GPS_Status min_status_for_gps_healthy() const {
        // NO_FIX simply excludes NO_GPS
        return AP_GPS::GPS_Status::NO_FIX;
    }

    void update_sensor_status_flags();

    // time we last saw traffic from our GCS
    uint32_t _sysid_mygcs_last_seen_time_ms;

    void service_statustext(void);
#if HAL_MEM_CLASS <= HAL_MEM_CLASS_192 || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static const uint8_t _status_capacity = 7;
#else
    static const uint8_t _status_capacity = 30;
#endif

    // queue of outgoing statustext messages.  Each entry consumes 58
    // bytes of RAM on stm32
    StatusTextQueue _statustext_queue{_status_capacity};

    // true if we have already allocated protocol objects:
    bool initialised_missionitemprotocol_objects;

    // true if update_send has ever been called:
    bool update_send_has_been_called;

    // handle passthru between two UARTs
    struct {
        bool enabled;
        bool timer_installed;
        AP_HAL::UARTDriver *port1;
        AP_HAL::UARTDriver *port2;
        uint32_t start_ms;
        uint32_t last_ms;
        uint32_t last_port1_data_ms;
        uint32_t baud1;
        uint32_t baud2;
        uint8_t timeout_s;
        HAL_Semaphore sem;
    } _passthru;

    // timer called to implement pass-thru
    void passthru_timer();

    // this contains the index of the GCS_MAVLINK backend we will
    // first call update_send on.  It is incremented each time
    // GCS::update_send is called so we don't starve later links of
    // time in which they are permitted to send messages.
    uint8_t first_backend_to_send;
};

GCS &gcs();

// send text when we do have a GCS
#if !defined(HAL_BUILD_AP_PERIPH)
#define GCS_SEND_TEXT(severity, format, args...) gcs().send_text(severity, format, ##args)
#else
extern "C" {
void can_printf(const char *fmt, ...);
}
#define GCS_SEND_TEXT(severity, format, args...) (void)severity; can_printf(format, ##args)
#endif

#elif defined(HAL_BUILD_AP_PERIPH) && !defined(STM32F1)

// map send text to can_printf() on larger AP_Periph boards
extern "C" {
void can_printf(const char *fmt, ...);
}
#define GCS_SEND_TEXT(severity, format, args...) can_printf(format, ##args)

#else // HAL_GCS_ENABLED
// empty send text when we have no GCS
#define GCS_SEND_TEXT(severity, format, args...)

#endif // HAL_GCS_ENABLED
/// @file   ABZ_Sprayer.h
/// @brief  Crop sprayer library

/**
    The crop spraying functionality can be enabled in ArduCopter by doing the following:
        - set RC7_OPTION or RC8_OPTION parameter to 15 to allow turning the sprayer on/off from one of these channels
        - set SERVO10_FUNCTION to 22 to enable the servo output controlling the pump speed on servo-out 10
        - set SERVO11_FUNCTION to 23 to enable the servo output controlling the spinner on servo-out 11
        - ensure the RC10_MIN, RC10_MAX, RC11_MIN, RC11_MAX accurately hold the min and maximum servo values you could possibly output to the pump and spinner
        - set the SPRAY_SPINNER to the pwm value the spinner should spin at when on
        - set the SPRAY_PUMP_RATE to the value the pump servo should move to when the vehicle is travelling 1m/s expressed as a percentage (i.e. 0 ~ 100) of the full servo range.  I.e. 0 = the pump will not operate, 100 = maximum speed at 1m/s.  50 = 1/2 speed at 1m/s, full speed at 2m/s
        - set the SPRAY_PUMP_MIN to the minimum value that the pump servo should move to while engaged expressed as a percentage (i.e. 0 ~ 100) of the full servo range
        - set the SPRAY_SPEED_MIN to the minimum speed (in cm/s) the vehicle should be moving at before the pump and sprayer are turned on.  0 will mean the pump and spinner will always be on when the system is enabled with ch7/ch8 switch
**/
//TODO make it custom lib
#pragma once
#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
/**ABZ parameters*/
#define ABZ_SPRAYER_DEFAULT_START_PWM_SEC    1               ///< The default time that is used at the beginning of a spraying section for high pwm
#define ABZ_SPRAYER_DEFAULT_START_PWM        1800            ///< The default pwm that is used at the beginning of a spraying section
#define ABZ_SPRAYER_FIRM_VERSION             1.0             ///< firmware version deprecated
#define ABZ_SPRAYER_DEFAULT_LN               1.0             ///< Default liters need, used to come from QGC now  it is from calculation
#define ABZ_SPRAYER_DEFAULT_TS               0.0             ///< Total sprayed liquid during the mission
#define ABZ_SPRAYER_FLIGHT_MOD               0               ///< flight mod  0 isterrain follow 1 is relative to home
#define ABZ_SPRAYER_DEFAULT_A                36922.0         ///< A param of PWM calculation (magic number) 
#define ABZ_SPRAYER_DEFAULT_B                8048.5          ///< B param of PWM calculation (magic number) 
#define ABZ_SPRAYER_DEFAULT_C                1053.1          ///< C param of PWM calculation (magic number)

/// @class  ABZ_Sprayer
/// @brief  Object managing a crop sprayer comprised of a spinner and a pump both controlled by pwm
class ABZ_Sprayer {
public:
    //TODO consider more private
    ABZ_Sprayer();
    int returning_point =   -7;         /**The optimal returning point of the drone in mission. Calculated by firmware -7 never be a valid mission item*/
    int count_sec   =   0;              /*Not used, or reference errror*/
    int delay   =   1000;               /**Milisec no results*/
    int start_time;                     /**time stamp of spraying start*/
    int now;                            /**Not used or reference errror*/

    uint32_t    last_time   =   AP_HAL::millis();   /**Not used or reference errror*/
    uint32_t    start_pwm_time   =   0;                  /**Stores time of High PWM start. @see update() in GCS.cpp*/

    double  coverage_percentage  =   0.0;       /**Percentage of the spraying. @see APMISSION::calculatePoints() and update() in GCS.cpp*/
    double  tartaly_liter    =   -10.0;         /**Liquid loaded to the tank in liters*/
    double  liter_per_sec    =   0.0;           /**Not used, or reference errror*/
    double  round_sprayed    =   0.0;           /**Amount of sprayed liquid in the round (since AP_MISSION::start()) in liquid*/
    double  remaining_fluid  =   0.0;           /**The total amount left to spray in liters*/
    double  remaining_starts =   0.0;           /**The amount of takeoffs remaining to finish misssion*/
    double  remaining_time   =   0.0;           /**The reamining time to finish mission*/
    double  liter_per_second =   0.0;           /**Liter out per second during spraying based on spacing and coverage*/
    double  total_meter  =   0.0;               /**The length of the remaining mission in meters*/
    double  max_speed    =   0.0;               /**The speed seted in the waypoint is the max speed the dron will aim*/
    double  rlat = -1.0;                        /**Returning point latitude send with MSG_ABZ_RETURNING_POINT_COR @see gcs_common.cpp*/
    double  rlng = -1.0;                        /**Returning point longitude send with MSG_ABZ_RETURNING_POINT_COR @see gcs_common.cpp*/
    double  tlat = -1.0;                        /**Tank empty point latitude send with MSG_ABZ_EMPTY_POINT_COR @see gcs_common.cpp*/
    double  tlng = -1.0;                        /**Tank empty point longitude send with MSG_ABZ_EMPTY_POINT_COR @see gcs_common.cpp*/

    float   return_to_mission_like_rtl_speed    =   0;          /**The speed the drone aim during RTM*/
    float   _testOrNot  =   false;                              /**Is it test or not test*/
    float   liter_left;                                         /**The liter left in the tank*/
    float   total_sprayed;                                      /**Total amount sprayed out in liter*/
    
    bool    send_tank_is_empty  =   true;                       /**It's true If tank level below threshold*/
    bool    spraying_abz    =   false;                          /**True if the drone is spraying*/
    bool    firstcheck  =   true;                               /**Basic checks*/

    AP_Int16    start_pwm;              /**The pwm that is used at the beginning of a spraying sectin @see ABZ_Sprayer::update() in gcs.cpp*/
    AP_Int16    start_pwm_sec;          /**The time of the higher pwm at the beginning of a spraying section in seconds @see ABZ_Sprayer::update() in gcs.cpp*/
    AP_Int8     F_MOD;                  /**Flight mod param if 0 terrain follow if 1 relativ to alt @see surface_tracking.cpp*/
    AP_Int8     is_start_pwm;           /**Param wich turns on and off the pre pwm function*/
    AP_Int8     drone_type;             /**Type of the drone. If 1 is L10 2 is L30*/
    AP_Float    pwm_a;                  /**A param of PWM calculatin @see ABZ_Sprayer::update() in gcs.cpp*/
    AP_Float    pwm_b;                  /**B param of PWM calculatin @see ABZ_Sprayer::update() in gcs.cpp*/
    AP_Float    pwm_c;                  /**C param of PWM calculatin @see ABZ_Sprayer::update() in gcs.cpp*/
    AP_Float    T_sprayed;              /**Total amount of sprayed liquid in liters stored in param used for calculations @see gcs.spp and AP_mission.cpp*/
    AP_Float    L_Need;                 /**Total liter need in mission stored in param used for calculations @see gcs.spp and AP_mission.cpp*/
    AP_Float    firm_version;           /**Version of firmware not used anymore*/
    AP_Float    liters_left;            /**The amuont of liquid in the tank stored as a param send with MSG_ABZ_LITER_LEFT @see ABZ_Sprayer::sendCapacity() in gcs.cpp*/
    /** Do not allow copies */
    ABZ_Sprayer(const ABZ_Sprayer &other)   =   delete;
    ABZ_Sprayer &operator=(const ABZ_Sprayer&)  =   delete;

    static  ABZ_Sprayer *get_singleton();                               /**Singelton of ABZ_Sprayer class*/
    static  ABZ_Sprayer *_singleton;                                    /**Singelton of ABZ_Sprayer class*/
    static const struct  AP_Param::GroupInfo var_info[];                /**Param group for ABZ_Sprayer class*/
    /**setters @see gcs.cpp for details*/
    void    set_Calculating_Values(const mavlink_command_long_t &packet);
    void    set_Calculating_Values(const mavlink_mission_item_int_t& packet);
    void    setCustomTartaly(const mavlink_command_long_t &packet);
    void    set_Calculating_Values_command(const AP_Mission::Mission_Command& cmd);
    void    setmissionpoints(int pointsnumber) {_number_of_mission_points=pointsnumber;}
    void    setSpacing(float spacing);
    void    setCoverage(float coverage);
    void    setlitersNeeded(float need);
    void    setSpinnerPwm(float spinnerwm);
    void    set_rtm_value(float rtm_value,float rtm_value_alt,float rtm_value_speed);
    void    set_pump_rate(float pct_at_1ms) { _pump_pct_1ms.set(pct_at_1ms); }
    void    SetSpraying_speed_adaptive(bool spraying);
    void    setDontSprayInMission();
    /**getters @see gcs.cpp for details*/
    int     getmissionpoints() {return _number_of_mission_points;}

    int32_t    getRTMAlt()    {return return_to_mission_like_rtl_altitude;}

    float   getSpacing();
    float   getCustomTartaly();
    float   getCoverage();
    float   getlitersNeeded();

    bool    getReturn_To_Mission_Like_Rtl() {return return_to_mission_like_rtl;}
    /**do*/
    void    resetDontSprayInMission();
    /// run - allow or disallow spraying to occur
    void    run();
    /// running - returns true if spraying is currently permitted
    bool    running() const { return _flags.running; }
    /// spraying - returns true if spraying is actually happening
    bool    spraying() const { return _flags.spraying; }
    /// test_pump - set to true to turn on pump as if travelling at 1m/s as a test
    void    test_pump(bool true_false) { _flags.testing = true_false; }
    /// set_pump_rate - sets desired quantity of spray when travelling at 1m/s as a percentage of the pumps maximum rate
    void    sendCapacity();
    /// update - adjusts servo positions based on speed and requested quantity
    void    update();
    void    run_mission(bool true_false);

private:

    int    _number_of_mission_points;

    uint16_t    _spinnerpwm;                                        /**PWM of sprayer head*/
    int32_t     return_to_mission_like_rtl_altitude    =   1500;    /**Altitude the drone aims when doing RTM*/
    uint32_t    _speed_under_min_time;                              /**Not used or reference errror*/

    float   _liters_needed;                     /**Liter needed for mission not used*/
    float   _customTartaly;                     /**The amount of liquid loaded to the tank in liters*/
    float   _coverage;                          /**The coverage value seted in Mission spryer command*/
    float   _spacing;                           /**The coverage value seted in Mission spryer command*/
    float   _testPumpPWM;                       /**Liquid pump pwm during test*/
    float   _testSpinnerPWM;                    /**Spinner pwm during test*/

    bool    dont_spray                  =   false;      /**Spraye or not*/
    bool    return_to_mission_like_rtl  =   false;      /**If RTM like RTL is in its true*/
             
    AP_Int8     _enabled;               ///< top level enable/disable control
    AP_Int8     _pump_min_pct;          ///< minimum pump rate (expressed as a percentage from 0 to 100)

    AP_Int16    _spinner_pwm;           ///< pwm rate of spinner

    AP_Float    _pump_pct_1ms;          ///< desired pump rate (expressed as a percentage of top rate) when travelling at 1m/s
    AP_Float    _speed_min;    ///< minimum speed in cm/s above which the sprayer will be started
    /// flag bitmask
    struct sprayer_flags_type {
        uint8_t spraying    : 1;            ///< 1 if we are currently spraying
        uint8_t testing     : 1;            ///< 1 if we are testing the sprayer and should output a minimum value
        uint8_t running     : 1;            ///< 1 if we are permitted to run sprayer
    } _flags;

    void stop_spraying();
};

namespace ABZ {
    ABZ_Sprayer * get_singleton();
};



