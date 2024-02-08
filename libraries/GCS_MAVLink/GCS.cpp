#include "GCS.h"

#include <AC_Fence/AC_Fence.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_Notify/AP_Notify.h>

#include "MissionItemProtocol_Waypoints.h"
#include "MissionItemProtocol_Rally.h"
#include "MissionItemProtocol_Fence.h"

extern const AP_HAL::HAL& hal;

// if this assert fails then fix it and the comment in GCS.h whe
// _statustext_queue is declared
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
assert_storage_size<GCS::statustext_t, 58> _assert_statustext_t_size;
#endif

void GCS::get_sensor_status_flags(uint32_t &present,
                                  uint32_t &enabled,
                                  uint32_t &health)
{
    update_sensor_status_flags();

    present = control_sensors_present;
    enabled = control_sensors_enabled;
    health = control_sensors_health;
}

MissionItemProtocol_Waypoints *GCS::_missionitemprotocol_waypoints;
MissionItemProtocol_Rally *GCS::_missionitemprotocol_rally;
#if AP_FENCE_ENABLED
MissionItemProtocol_Fence *GCS::_missionitemprotocol_fence;
#endif

const MAV_MISSION_TYPE GCS_MAVLINK::supported_mission_types[] = {
    MAV_MISSION_TYPE_MISSION,
    MAV_MISSION_TYPE_RALLY,
    MAV_MISSION_TYPE_FENCE,
};

void GCS::init()
{
    mavlink_system.sysid = sysid_this_mav();
}

/*
 * returns a mask of channels that statustexts should be sent to
 */
uint8_t GCS::statustext_send_channel_mask() const
{
    uint8_t ret = 0;
    ret |= GCS_MAVLINK::active_channel_mask();
    ret |= GCS_MAVLINK::streaming_channel_mask();
    ret &= ~GCS_MAVLINK::private_channel_mask();
    return ret;
}

/*
  send a text message to all GCS
 */
void GCS::send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list)
{
    uint8_t mask = statustext_send_channel_mask();
    if (!update_send_has_been_called) {
        // we have not yet initialised the streaming-channel-mask,
        // which is done as part of the update() call.  So just send
        // it to all channels:
        mask = (1<<_num_gcs)-1;
    }
    send_textv(severity, fmt, arg_list, mask);
}

void GCS::send_text(MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    send_textv(severity, fmt, arg_list);
    va_end(arg_list);
}

void GCS::send_to_active_channels(uint32_t msgid, const char *pkt)
{
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
    if (entry == nullptr) {
        return;
    }
    for (uint8_t i=0; i<num_gcs(); i++) {
        GCS_MAVLINK &c = *chan(i);
        if (c.is_private()) {
            continue;
        }
        if (!c.is_active()) {
            continue;
        }
        if (entry->max_msg_len + c.packet_overhead() > c.txspace()) {
            // no room on this channel
            continue;
        }
        c.send_message(pkt, entry);
    }
}

void GCS::send_named_float(const char *name, float value) const
{

    mavlink_named_value_float_t packet {};
    packet.time_boot_ms = AP_HAL::millis();
    packet.value = value;
    memcpy(packet.name, name, MIN(strlen(name), (uint8_t)MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN));

    gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
                                  (const char *)&packet);
}

#if HAL_HIGH_LATENCY2_ENABLED
void GCS::enable_high_latency_connections(bool enabled)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        GCS_MAVLINK &c = *chan(i);
        c.high_latency_link_enabled = enabled && c.is_high_latency_link;
    } 
    gcs().send_text(MAV_SEVERITY_NOTICE, "High Latency %s", enabled ? "enabled" : "disabled");
}
#endif // HAL_HIGH_LATENCY2_ENABLED

/*
  install an alternative protocol handler. This allows another
  protocol to take over the link if MAVLink goes idle. It is used to
  allow for the AP_BLHeli pass-thru protocols to run on hal.serial(0)
 */
bool GCS::install_alternative_protocol(mavlink_channel_t c, GCS_MAVLINK::protocol_handler_fn_t handler)
{
    if (c >= num_gcs()) {
        return false;
    }
    if (chan(c)->alternative.handler && handler) {
        // already have one installed - we may need to add support for
        // multiple alternative handlers
        return false;
    }
    chan(c)->alternative.handler = handler;
    return true;
}

void GCS::update_sensor_status_flags()
{
    control_sensors_present = 0;
    control_sensors_enabled = 0;
    control_sensors_health = 0;

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_AHRS)
    AP_AHRS &ahrs = AP::ahrs();
    const AP_InertialSensor &ins = AP::ins();

    control_sensors_present |= MAV_SYS_STATUS_AHRS;
    if (ahrs.initialised()) {
        control_sensors_enabled |= MAV_SYS_STATUS_AHRS;
        if (ahrs.healthy()) {
            if (!ahrs.have_inertial_nav() || ins.accel_calibrated_ok_all()) {
                control_sensors_health |= MAV_SYS_STATUS_AHRS;
            }
        }
    }
#endif

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_MAG)
    const Compass &compass = AP::compass();
    if (AP::compass().available()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (compass.available() && compass.healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
#endif

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_BARO)
    const AP_Baro &barometer = AP::baro();
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    if (barometer.all_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
#endif

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_GPS)
    const AP_GPS &gps = AP::gps();
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if (gps.is_healthy() && gps.status() >= min_status_for_gps_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#endif

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_BATTERY)
    const AP_BattMonitor &battery = AP::battery();
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_BATTERY;
    if (battery.num_instances() > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
    if (battery.healthy() && !battery.has_failsafed()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#endif

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_AHRS)
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    if (!ins.calibrating()) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
        if (ins.get_accel_health_all()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        }
        if (ins.get_gyro_health_all() && ins.gyro_calibrated_ok_all()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
        }
    }
#endif

#if HAL_LOGGING_ENABLED
    const AP_Logger &logger = AP::logger();
    bool logging_present = logger.logging_present();
    bool logging_enabled = logger.logging_enabled();
    bool logging_healthy = !logger.logging_failed();
#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_GPS)
    // some GPS units do logging, so they have to be healthy too:
    logging_present |= gps.logging_present();
    logging_enabled |= gps.logging_enabled();
    logging_healthy &= !gps.logging_failed();
#endif
    if (logging_present) {
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }
    if (logging_enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }
    if (logging_healthy) {
        control_sensors_health |= MAV_SYS_STATUS_LOGGING;
    }
#endif  // HAL_LOGGING_ENABLED

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
#if !defined(HAL_BUILD_AP_PERIPH)
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }
    control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (ahrs.get_ekf_type() == 10) {
        // always show EKF type 10 as healthy. This prevents spurious error
        // messages in xplane and other simulators that use EKF type 10
        control_sensors_health |= MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_SENSOR_GPS | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
#endif

#if !defined(HAL_BUILD_AP_PERIPH) && AP_FENCE_ENABLED
    const AC_Fence *fence = AP::fence();
    if (fence != nullptr) {
        if (fence->sys_status_enabled()) {
            control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
        }
        if (fence->sys_status_present()) {
            control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
        }
        if (!fence->sys_status_failed()) {
            control_sensors_health |= MAV_SYS_STATUS_GEOFENCE;
        }
    }
#endif

    // airspeed
#if AP_AIRSPEED_ENABLED
    const AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed && airspeed->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        const bool use = airspeed->use();
        const bool enabled = AP::ahrs().airspeed_sensor_enabled();
        if (use) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        }
        if (airspeed->all_healthy() && (!use || enabled)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        }
    }
#endif

#if HAL_VISUALODOM_ENABLED
    const AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom && visual_odom->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        if (visual_odom->healthy()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        }
    }
#endif

    // give GCS status of prearm checks. This is enabled if any arming checks are enabled.
    // it is healthy if armed or checks are passing
#if !defined(HAL_BUILD_AP_PERIPH)
    control_sensors_present |= MAV_SYS_STATUS_PREARM_CHECK;
    if (AP::arming().get_enabled_checks()) {
        control_sensors_enabled |= MAV_SYS_STATUS_PREARM_CHECK;
        if (hal.util->get_soft_armed() || AP_Notify::flags.pre_arm_check) {
            control_sensors_health |= MAV_SYS_STATUS_PREARM_CHECK;
        }
    }
#endif

    update_vehicle_sensor_status_flags();
}

bool GCS::out_of_time() const
{
#if defined(HAL_BUILD_AP_PERIPH)
    // we are never out of time for AP_Periph
    // as we don't have concept of AP_Scheduler in AP_Periph
    return false;
#endif
    // while we are in the delay callback we are never out of time:
    if (hal.scheduler->in_delay_callback()) {
        return false;
    }

    // we always want to be able to send messages out while in the error loop:
    if (AP_BoardConfig::in_config_error()) {
        return false;
    }

    if (min_loop_time_remaining_for_message_send_us() <= AP::scheduler().time_available_usec()) {
        return false;
    }

    return true;
}

void gcs_out_of_space_to_send(mavlink_channel_t chan)
{
    gcs().chan(chan)->out_of_space_to_send();
}


#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// ------------------------------
/**
 * @brief Parameters avaible in the settings menu at QGC. Use GroupInfo to group them into abz
*/
const AP_Param::GroupInfo ABZ_Sprayer::var_info[] = {
    // @Param: PWM_A
    // @DisplayName: PWM A param
    // @Description: PWM equation A param
    // @Units: s
    // @Range: -10000 10000
    // @User: Standard
    AP_GROUPINFO("PWM_A",   0, ABZ_Sprayer, pwm_a, ABZ_SPRAYER_DEFAULT_A),
    
    // @Param: PWM_B
    // @DisplayName: PWM A param
    // @Description: PWM equation A param
    // @Units: s
    // @Range: -10000 10000
    // @User: Standard
    AP_GROUPINFO("PWM_B",  1, ABZ_Sprayer, pwm_b, ABZ_SPRAYER_DEFAULT_B),
    
    // @Param: PWM_C
    // @DisplayName: PWM C param
    // @Description: PWM equation C param
    // @Units: s
    // @Range: -10000 10000
    // @User: Standard
    AP_GROUPINFO("PWM_C",   2, ABZ_Sprayer, pwm_c, ABZ_SPRAYER_DEFAULT_C),

    // @Param: L_NEED
    // @DisplayName: Needed liters
    // @Description: Needed liters for the entire mission given from QGC
    // @Units: s
    // @Range: -10000 10000
    // @User: Standard
    AP_GROUPINFO("L_NEED",   3, ABZ_Sprayer, L_Need, ABZ_SPRAYER_DEFAULT_LN),

    // @Param: T_SPRAYED
    // @DisplayName: Total Sprayed
    // @Description: Total Sprayed fluid during the entire mission
    // @Units: s
    // @Range: -10000 10000
    // @User: Standard
    AP_GROUPINFO("T_SPRAYED",   4, ABZ_Sprayer, T_sprayed, ABZ_SPRAYER_DEFAULT_TS),

    // @Param: FLIGHT_MOD
    // @DisplayName: Flight  Mod
    // @Description: set flight mod
    // @User: Standard
    AP_GROUPINFO("FLIGHT_MOD",   5, ABZ_Sprayer, F_MOD, ABZ_SPRAYER_FLIGHT_MOD),

    // @Param: FIRM_VERSION
    // @DisplayName: Firmware version
    // @Description: the actual version of the firmware
    // @User: Standard
    // @ReadOnly: True
    AP_GROUPINFO("Firm_VERSION",6, ABZ_Sprayer, firm_version, ABZ_SPRAYER_FIRM_VERSION),

    // @Param: LITERS_LEFT
    // @DisplayName: Liters left
    // @Description: liters left in the tank
    // @User: Standard
    // @ReadOnly: True
    AP_GROUPINFO("LITERS_LEFT",7, ABZ_Sprayer, liters_left, 10.0),

    // @Param: START_PWM
    // @DisplayName: Start PWM
    // @Description: PWM used at spraying start.
    // @User: Standard
    AP_GROUPINFO("START_PWM",8, ABZ_Sprayer, start_pwm, ABZ_SPRAYER_DEFAULT_START_PWM ),

    // @Param: START_PWM_S
    // @DisplayName: Start PWM SEC
    // @Description: Time in sec of higher pwm at spraying start
    // @User: Standard
    AP_GROUPINFO("START_PWM_S",9, ABZ_Sprayer, start_pwm_sec,ABZ_SPRAYER_DEFAULT_START_PWM_SEC),

    // @Param: IS_START_PWM
    // @DisplayName: IS Start PWM active
    // @Description: Time in sec of higher pwm at spraying start
    // @User: Standard
    AP_GROUPINFO("IS_S_PWM",10, ABZ_Sprayer, is_start_pwm,0),

    // @Param: DTYPE
    // @DisplayName: Drone type
    // @Description: Type of the drone. 1 is L10 2 is L30
    // @User: Standard
    AP_GROUPINFO("DTYPE",11, ABZ_Sprayer, drone_type,1),

    AP_GROUPEND
};
/**Singelton for sprayer*/
ABZ_Sprayer::ABZ_Sprayer()
{
    if(_singleton)
    {
        return;
    }
    _singleton = this;
    
    AP_Param::setup_object_defaults(this, var_info);

    // check for silly parameter values
    

    // To-Do: ensure that the pump and spinner servo channels are enabled
}
/*
 * Get the ABZ_Sprayer singleton
 */
ABZ_Sprayer *ABZ_Sprayer::_singleton;
ABZ_Sprayer *ABZ_Sprayer::get_singleton()
{
    return _singleton;
}
/**
 * @brief resets the dont_spray to its default
 *  */
void ABZ_Sprayer::resetDontSprayInMission()
{
    dont_spray=false;
}
/**
 * @brief  sets the dont_spray to its oppoosite
 *  */
void ABZ_Sprayer::setDontSprayInMission()
{
    if(dont_spray)
    {
        dont_spray=false;
    }
    else
    {
        dont_spray=true;
    }
}
/**
 * @brief sets spraying_abz to True or False
 * @param spraying  is  a bool type that sets the spraying_abz
 *  */
void ABZ_Sprayer::SetSpraying_speed_adaptive(bool spraying)
{

    spraying_abz=spraying;
}
/**
 * @brief Runs the sprayer action.
 *
 * This function toggles the spraying state of the ABZ_Sprayer object.
 * If spraying is currently active, it sends a message indicating that
 * ABZ is spraying and then sets the spraying state to false. If spraying
 * is not active, it checks the _testOrNot flag. If this flag is set to 1.0f,
 * it sends a text message to the GCS (Ground Control Station) with an error.
 * Otherwise, it sets the spraying state to true.
 */
void ABZ_Sprayer::run(){

    if(spraying_abz){
        gcs().send_message(MSG_ABZ_IS_SPRAYING);
        spraying_abz=false;
    }else{

        if (is_equal(_testOrNot,1.0f)){
             gcs().send_text(MAV_SEVERITY_INFO, "Problem in GCS at ABZ__Sprayer::run(): %d",spraying_abz);
        }else{
            spraying_abz=true;
        }
    }
}
/**
 * @brief Controls the spraying action based on a given condition.
 * 
 * This function sets the spraying state according to the true_false argument.
 * If _testOrNot is equal to 1.0f, it reports a problem to the GCS (Ground Control Station).
 * If no problem is reported and spraying_abz is true, it marks the start time,
 * sends a message to indicate that ABZ is spraying, and logs "Spraying ON".
 * If spraying_abz is false, it simply logs "Spraying OFF".
 *
 * @param true_false A boolean flag to determine the spraying state; true to start spraying and false to stop.
 * @see AP_Mission::start_command_abz_sprayer(const AP_Mission::Mission_Command& cmd) in AP_Mission_Commands.cpp
 */
void ABZ_Sprayer::run_mission(bool true_false){
    
    spraying_abz=true_false;

    if (is_equal(_testOrNot,1.0f)){
             gcs().send_text(MAV_SEVERITY_INFO, "Problem in GCS at ABZ__Sprayer::run_mission(): %d",spraying_abz);
    }else{

        if(spraying_abz){
            start_time = AP_HAL::millis();
            gcs().send_message(MSG_ABZ_IS_SPRAYING);
            gcs().send_text(MAV_SEVERITY_INFO, "Spraying ON");
        }else{
            gcs().send_text(MAV_SEVERITY_INFO, "Spraying: OFF");
        }

    }

}
/**
 * @brief Stops the spraying process.
 *
 * This function sets the spraying flag to false, effectively
 * indicating that the ABZ_Sprayer should cease spraying operations.
 */
void ABZ_Sprayer::stop_spraying(){

    _flags.spraying = false;
}
/**
 * @brief sends the liter left message 
 * called at 10hz and send the current amount of fluid in the tank
 * 
*/
void ABZ_Sprayer::sendCapacity(){
    gcs().send_message(MSG_ABZ_LITER_LEFT);
}
/**
 * @brief Updates the sprayer state and controls the sprayer's PWM outputs based on current conditions and settings.
 * 
 * The function first checks if spraying is disabled by the 'dont_spray' flag. If not, it proceeds to check
 * if the sprayer is in test mode (when '_testOrNot' is 1.0f). In test mode, the function will adjust the PWM
 * values of the pump and spinner to ensure they are within the operational range and then update the outputs.
 * 
 * If not in test mode but spraying is active ('spraying_abz' is true), it calculates the sprayer output based on
 * the current ground speed and sprayer configuration parameters such as spacing and coverage. It also updates the 
 * total amount sprayed, coverage percentage, remaining fluid, and remaining time estimates. If the tank's fluid level 
 * falls below a critical threshold, it sends a critical message and updates the relevant status message.
 * 
 * If spraying is not active, it resets the PWM values for both the pump and the spinner to their default idle state.
 * 
 * If 'dont_spray' is set to true, it ensures both the pump and spinner are set to idle regardless of other conditions.
 * 
 * If we start spraying for 1000 ms it send a high pwm signal.
 */
void ABZ_Sprayer::update(){
    
    if(!dont_spray){

        if(is_equal(_testOrNot,1.0f)){

            if(_testPumpPWM<1050){
                _testPumpPWM=1050;
            }

            if(_testSpinnerPWM<1050){
                _testSpinnerPWM=500;
            }

            if(_testPumpPWM>1900){
                _testPumpPWM=1050;
            }

            if(_testSpinnerPWM>1900){
                _testSpinnerPWM=1050;
            }

            SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump,_testPumpPWM);
            SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner,_testSpinnerPWM);
            gcs().send_text(MAV_SEVERITY_ALERT, "_testPumpPWM: %f", _testPumpPWM); //hajni task#5
            gcs().send_text(MAV_SEVERITY_ALERT, "_testSpinnerPWM: %f", _testSpinnerPWM); //hajni task#5
            return;
        }else if(spraying_abz){   

            _testOrNot=0.0f;
            Vector3f velocity; 
            if (!AP::ahrs().get_velocity_NED(velocity)) {
            // treat unknown velocity as zero which should lead to pump stopping
            // velocity will already be zero but this avoids a coverity warning
                velocity.zero();
            }

            float ground_speed = norm(velocity.x , velocity.y );

            if(ground_speed<0.0001){

                return;
            }
            
            float how_many_second=_spacing*ground_speed;
            how_many_second=10000/how_many_second;
            
            liter_per_second=_coverage/how_many_second;
            float pwm;
            //if we want to use different pwm at start 
            if (is_start_pwm > 0){
                //if we started to spray
                if (start_pwm_time>0.0){
                    //if the time lower then max time we use starting pwm else we calculate the pwm
                    if ((AP_HAL::millis()-start_pwm_time)<static_cast<uint32_t>(start_pwm_sec*1000)){
                        pwm = start_pwm;
                    }else{
                        //if drone is l10 els l30 or custom
                        if(drone_type==1){
                            //gcs().send_text(MAV_SEVERITY_INFO,"L10");
                            pwm = (pwm_a*(liter_per_second*liter_per_second))+(pwm_b*liter_per_second)+pwm_c;
                        }else{
                            //gcs().send_text(MAV_SEVERITY_INFO,"L30");
                            pwm = (pwm_a*(liter_per_second*liter_per_second))-(pwm_b*liter_per_second)+pwm_c; 
                        }
                    }
                //if it is the start of the spraying we use starting pwm
                }else{
                    start_pwm_time = AP_HAL::millis();
                    pwm = start_pwm;
                }
            //if different pwm at start is off we calculate the pwm
            }else{
                //if drone is l10 els l30 or custom
                if(drone_type==1){
                    //gcs().send_text(MAV_SEVERITY_INFO,"L10");
                    pwm = (pwm_a*(liter_per_second*liter_per_second))+(pwm_b*liter_per_second)+pwm_c;
                }else{
                    //gcs().send_text(MAV_SEVERITY_INFO,"L30");
                    pwm = (pwm_a*(liter_per_second*liter_per_second))-(pwm_b*liter_per_second)+pwm_c;
                }

            }
            
            if(1050<=pwm){

                if(pwm<=1900){
                    round_sprayed += liter_per_second/10;
                    T_sprayed.set_and_save(T_sprayed+liter_per_second/10);
                    coverage_percentage = (T_sprayed/L_Need)*100;
                    remaining_fluid = L_Need-T_sprayed;
                    remaining_starts = remaining_fluid/tartaly_liter;

                    if((int)remaining_starts < remaining_starts){
                        remaining_starts = (int)remaining_starts+1;
                    }

                    total_meter = total_meter-ground_speed/10;
                    remaining_time = total_meter/max_speed;
                    liter_left = tartaly_liter-round_sprayed;
                    liters_left.set_and_save(liter_left);

                    if(liter_left<0.20){   

                        if(send_tank_is_empty){
                            gcs().send_text(MAV_SEVERITY_CRITICAL,"Tank is empty!");
                            gcs().send_message(MSG_ABZ_EMPTY_TANK_ACTION);
                            send_tank_is_empty=false;
                        }

                    }

                    gcs().send_message(MSG_ABZ_PLAN_PROGRESS);
                    SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump,int(pwm));
                    SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner,int(_spinnerpwm));

                }else{
                    SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump,1050);
                    SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner,int(_spinnerpwm));
                }
            
            
            }
        }else if(!spraying_abz){
            start_pwm_time = 0.0;
            SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner,1050);
            SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump,1050);
            _testPumpPWM=0;
            _testSpinnerPWM=0;
        }

    }else{
        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_spinner,1050);
        SRV_Channels::set_output_pwm(SRV_Channel::k_sprayer_pump,1050);
    }
}
/**
 * @brief sets value from packet
 * @param packet a short mav command
*/
void ABZ_Sprayer::set_Calculating_Values(const mavlink_mission_item_int_t& packet){
    _coverage=packet.param2;
    _spacing=packet.param3;
    _spinnerpwm=packet.param4;
}
/**
 * @brief sets value from packet
 * @param packet a long mav command
*/
void ABZ_Sprayer::set_Calculating_Values (const mavlink_command_long_t &packet){
    _coverage=packet.param2;
    _spacing=packet.param3;
    _spinnerpwm=packet.param4;
    _testOrNot=packet.param5;
    _testSpinnerPWM=packet.param6;
    _testPumpPWM=packet.param7;
}
/**
 * @brief sets value from command
 * @param packet is a mission spraying command
*/
void ABZ_Sprayer::set_Calculating_Values_command(const AP_Mission::Mission_Command& cmd){
    _coverage=cmd.content.sprayer.coverage;
    _spacing=cmd.content.sprayer.spacing;
    _spinnerpwm=cmd.content.sprayer.spinnerpwm*100;
}
/**
 * @brief setting rtm related values
 * If we use RTM wich returns to mission like RTL, we need  to set alt and speed and turn on the optioon
 * @param rtm_value is rtm on or off
 * @param rtm_value_alt is the returning altitude
 * @param rtm_value_speed is the returning speed
*/
void ABZ_Sprayer::set_rtm_value(float rtm_value,float rtm_value_alt,float rtm_value_speed){
   
    if( is_equal(rtm_value,1.0f)){
        return_to_mission_like_rtl=true;
    }else{
        return_to_mission_like_rtl=false;
    }

    return_to_mission_like_rtl_altitude=rtm_value_alt;
    return_to_mission_like_rtl_speed=rtm_value_speed;
}
/**
 * @brief setter for spacing
 * @param spacing is float type the value we want  to set
*/
void ABZ_Sprayer::setSpacing(float spacing){
    _spacing=spacing;
}
/**
 * @brief setter for coverage
 * @param coverage is float type the value we want  to set
*/
void ABZ_Sprayer::setCoverage(float coverage){
    _coverage=coverage;
}
/**
 * @brief setter for Literrs_needed
 * @param need is float type the value we want  to set
*/
void ABZ_Sprayer::setlitersNeeded(float need){
    _liters_needed = need;
}
/**
 * @brief setter for customtartaly
 * @param packet is a mavling command
*/
void ABZ_Sprayer::setCustomTartaly(const mavlink_command_long_t &packet){
    _customTartaly = packet.param1;
}
/**
 * @brief setter for SpinnerPwm
 * @param spinnerpwm is float type the value we want  to set
*/
void ABZ_Sprayer::setSpinnerPwm(float spinnerpwm)
{
    _spinnerpwm=spinnerpwm;
}
/**
 * @brief getter for spacing
 * @return _spacing as float value
*/
float ABZ_Sprayer::getSpacing(){
    return _spacing;
}
/**
 * @brief getter for covverage
 * @return _coverage as float value
*/
float ABZ_Sprayer::getCoverage(){
    return _coverage;
}
/**
 * @brief getter for literrs needed
 * @return _liters_needed as float value
*/
float ABZ_Sprayer::getlitersNeeded(){
    return _liters_needed;
}
/**
 * @brief getter for customTartaly
 * @return _customTartaly as float value
*/
float ABZ_Sprayer::getCustomTartaly(){
    return _customTartaly;
}
/**
 * namespace definition
*/
namespace ABZ {

    ABZ_Sprayer * get_singleton(){
        return ABZ_Sprayer::get_singleton();
    }

};



