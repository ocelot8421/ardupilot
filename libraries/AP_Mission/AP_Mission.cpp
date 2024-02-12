/// @file    AP_Mission.cpp
/// @brief   Handles the MAVLINK command mission stack.  Reads and writes mission to storage.

#include "AP_Mission.h"
#include "AP_Mission_MiddelPoint.hpp"
#include <AP_Terrain/AP_Terrain.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <cmath>
#define ALLOW_DOUBLE_MATH_FUNCTIONS
const AP_Param::GroupInfo AP_Mission::var_info[] = {

    // @Param: TOTAL
    // @DisplayName: Total mission commands
    // @Description: The number of mission mission items that has been loaded by the ground station. Do not change this manually.
    // @Range: 0 32766
    // @Increment: 1
    // @User: Advanced
    // @ReadOnly: True
    AP_GROUPINFO_FLAGS("TOTAL",  0, AP_Mission, _cmd_total, 0, AP_PARAM_FLAG_INTERNAL_USE_ONLY),

    // @Param: RESTART
    // @DisplayName: Mission Restart when entering Auto mode
    // @Description: Controls mission starting point when entering Auto mode (either restart from beginning of mission or resume from last command run)
    // @Values: 0:Resume Mission, 1:Restart Mission
    // @User: Advanced
    AP_GROUPINFO("RESTART",  1, AP_Mission, _restart, AP_MISSION_RESTART_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: Mission options bitmask
    // @Description: Bitmask of what options to use in missions.
    // @Bitmask: 0:Clear Mission on reboot, 1:Use distance to land calc on battery failsafe,2:ContinueAfterLand
    // @Bitmask{Copter}: 0:Clear Mission on reboot, 2:ContinueAfterLand
    // @Bitmask{Rover, Sub}: 0:Clear Mission on reboot
    // @User: Advanced
    AP_GROUPINFO("OPTIONS",  2, AP_Mission, _options, AP_MISSION_OPTIONS_DEFAULT),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

// storage object
StorageAccess AP_Mission::_storage(StorageManager::StorageMission);

HAL_Semaphore AP_Mission::_rsem;

///
/// public mission methods
///

/// init - initialises this library including checks the version in eeprom matches this library
void AP_Mission::init()
{
    // check_eeprom_version - checks version of missions stored in eeprom matches this library
    // command list will be cleared if they do not match
    check_eeprom_version();

    // initialize the jump tracking array
    init_jump_tracking();

    // If Mission Clear bit is set then it should clear the mission, otherwise retain the mission.
    if (AP_MISSION_MASK_MISSION_CLEAR & _options) {
        gcs().send_text(MAV_SEVERITY_INFO, "Clearing Mission");
        clear();
    }

    _last_change_time_ms = AP_HAL::millis();
}
/**
 * @brief Calculates the distance to a waypoint
 * if the distance from the drone to the waypoint is bigger then 1000 meter it returns true
 * Used to check the distance to the first waypoint.
 * This way if the first waypoint is too far the drone won't start the mission automaticaly
 * Needed for avoiding user mistakes like uploading wrong mission
 * called in gcs_common.cpp
 * \callgraph
 * \callergraph
 * @return true if distance > 1000 meter
 * 
*/
// TO DO use the dist of two in function should be in ABZ lib
bool AP_Mission::waypoint_one_kilometer_away_atleast(){

    AP_GPS * gps=AP_GPS::get_singleton();
    
    if(gps==nullptr){
        return false;
    }

    Mission_Command first_waypoint_one_kilometer;
    read_cmd_from_storage(2,first_waypoint_one_kilometer);

    if(is_nav_cmd(first_waypoint_one_kilometer)){
        int radius=6371000;
        double gps_lat=(double)gps->location().lat/(double)10000000;
        double gps_lon=(double)gps->location().lng/(double)10000000;
        double fwp_lat=(double)first_waypoint_one_kilometer.content.location.lat/(double)10000000;
        double fwp_lon=(double)first_waypoint_one_kilometer.content.location.lng/(double)10000000;
        double omega_1=fwp_lat*3.14159265/180;
        double omega_2=gps_lat*3.14159265/180;
        double delta_omega=(gps_lat-fwp_lat)*3.14159265/180;
        double delta_alpha=(gps_lon-fwp_lon)*3.14159265/180;
        double a=( (sinf(delta_omega/2) * sinf(delta_omega/2)) +cosf(omega_1) * cosf(omega_2) * (sinf(delta_alpha/2) * sinf(delta_alpha/2)));
        double c=2*atan2f(sqrtf(a),sqrtf(1-a));
        double d=radius*c;

        if(d>1000){
           return true;
        }

    }

    return false;
}

/**
 * @brief Calculates distance between two Mission waypoints
 * Uses lat,lon of the points to calculate the distance
 * Haversine Formula:
 * The Haversine formula calculates the shortest distance between two points on the surface of a sphere, given their latitude and longitude.
 * It is particularly useful in navigation and geospatial computations, such as calculating distances between GPS coordinates
 * The order of the points is important
 * The function you provided calculates this distance based on the inputted latitudes and longitudes of the two points. Here's a step-by-step breakdown in simpler terms:
 * Earth's Radius: The Earth isn't flat! It's a big ball (well, more like an oblong sphere). We need to know its size to calculate distances on it. So, this function uses an average value for Earth's radius, which is about 6,371 kilometers (or 6,371,000 meters).
 * Degrees to Radians: Latitude and longitude values are usually given in degrees, like on a compass or map. But the math we're using works with another unit called "radians." So, first, we convert degrees to radians.
 * The Haversine Formula: This is a special formula to find the shortest distance between two points on a sphere. It involves some trigonometry and uses the converted latitude and longitude values.
 * Calculate the Distance: Multiply the Earth's radius by the result from the Haversine formula. This gives the distance between the two points in meters.
 * Return the Result: The function then tells us the calculated distance.
 * @param P1 is the first point
 * @param P2 is the second point
 * \callgraph
 * \callergraph
 * @return distance P1P2
*/
//TO DO make it accept mission items not lat and long. Also should be in ABZ lib
double AP_Mission::get_dist_of_two_points(double latp1,double lngp1,double latp2,double lngp2){

    int radius=6371000;/*earth radius in meter*/
    /*
     * calculating distance between two gps points.
    */
    double omega_1=latp2*3.14159265/180;
    double omega_2=latp1*3.14159265/180;
    double delta_omega=(latp1-latp2)*3.14159265/180;
    double delta_alpha=(lngp1-lngp2)*3.14159265/180;
    double a=( (sinf(delta_omega/2) * sinf(delta_omega/2)) +cosf(omega_1) * cosf(omega_2) * (sinf(delta_alpha/2) * sinf(delta_alpha/2)));
    double c=2*atan2f(sqrtf(a),sqrtf(1-a));
    double d=radius*c;/*distance between the two points*/

    return d;
}
/**
 * @brief calculates radians from a degree
 * degrees: This is the angle in degrees you want to convert.
 * M_PI: This is a constant that represents the value of π (pi), which is approximately 3.14159265.
 * 180.0: There are 180 degrees in π radians.
 * @param degrees is the value we want to calculate to radians
 * @return value in radians
*/
//TO DO ABZ lib
double AP_Mission::degreesToRadians(double degrees) {

    return degrees * M_PI / 180.0; 
} 
/**
 * @brief calculates degree from radian
 * radians: This is the angle in radian you want to convert.
 * M_PI: This is a constant that represents the value of π (pi), which is approximately 3.14159265.
 * 180.0: There are 180 degrees in π radians.
 * @param radians is the value we want to calculate to degrees
 * @return value in degrees
*/
//TO DO ABZlib
double AP_Mission::radiansToDegrees(double radians) {

    return radians * 180.0 / M_PI; 
} 
/**
 * @brief Calculates the latitude and longitude of a third point (Point 3) given two known points (Point 1 and Point 2) 
 *        and the distance from Point 1 to Point 3 along the great-circle path.
 * 
 * The function determines the position of Point 3, which is located at a specified distance from Point 1 and 
 * in the direction of Point 2 from Point 1.
 * 
 * Conversion to Radians: Before conducting trigonometric calculations, the given latitudes and longitudes (in degrees) are converted to radians,
 *                        since trigonometric functions operate on radian values.
 * 
 * Difference in Longitude: The difference in longitudes between the two given points is calculated.
 * 
 * Bearing Angle (θ): The function calculates the initial bearing or forward azimuth from Point 1 to Point 2.
 *                    This is the angle between the line from Point 1 to the North pole and the line from Point 1 to Point 2.
 *                    The bearing is computed using the atan2 function, which returns the angle between the positive X-axis and the point (x, y), making it suitable for our needs.
 * 
 * Latitude of Point 3: Using the Haversine formula and the calculated bearing, the latitude of Point 3 is computed. 
 *                      This involves using the asin function and other trigonometric relations that factor in the distance from Point 1 to Point 3 and the Earth's radius.
 * 
 * Longitude of Point 3: Similarly, the longitude of Point 3 is calculated using the bearing and the known latitudes.
 *                       This is done using the atan2 function and relationships derived from spherical trigonometry.
 * 
 * Conversion Back to Degrees: The resulting latitude and longitude of Point 3, obtained in radians, are converted back to degrees for a more interpretable result.
 *
 * @param lat1 Latitude of Point 1 in degrees.
 * @param lon1 Longitude of Point 1 in degrees.
 * @param lat2 Latitude of Point 2 in degrees.
 * @param lon2 Longitude of Point 2 in degrees.
 * @param dist13 Distance from Point 1 to Point 3 in meters.
 * 
 * @return A pair (std::pair) where the first element is the latitude of Point 3 in degrees and 
 *         the second element is the longitude of Point 3 in degrees.
 *
 * @note This function assumes a spherical Earth and uses the Earth's average radius of 6371 kilometers. 
 *       The results might not account for the ellipsoidal shape of the Earth.
 */
//TO Do Mission item not lat lon, ABZ lib
std::pair<double,double> AP_Mission::calculatePoint3(double lat1, double lon1, double lat2, double lon2, double dist13) { 

    double EarthRadius =6371000.0;
    double lat3;
    double lon3;
    double lat1_rad = degreesToRadians(lat1);
    double lon1_rad = degreesToRadians(lon1);
    double lat2_rad = degreesToRadians(lat2);
    double lon2_rad = degreesToRadians(lon2);
    double delta_lon = lon2_rad - lon1_rad;
    double theta = atan2(sin(delta_lon) * cos(lat2_rad),cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon));  
    double lat3_rad = asin(sin(lat1_rad) * cos(dist13 / EarthRadius) +  cos(lat1_rad) * sin(dist13 / EarthRadius) * cos(theta)); 
    double lon3_rad = lon1_rad + atan2(sin(theta) * sin(dist13 / EarthRadius) * cos(lat1_rad),cos(dist13 / EarthRadius) - sin(lat1_rad) * sin(lat3_rad)); 
    lat3 = radiansToDegrees(lat3_rad);     
    lon3 = radiansToDegrees(lon3_rad); 

    return std::make_pair(lat3,lon3);

}
/**
 * @brief Calculates the amount of liters required for a mission based on the given mission commands.
 * 
 * The function estimates the amount of liquid needed for spraying throughout a mission. It processes
 * the mission commands, determines the relevant waypoints that define spray sections, calculates the 
 * distance of these sections, and then multiplies by the coverage and spacing parameters to determine 
 * the total liters required.
 *
 * @note This function assumes a series of conditions:
 *       - A singleton pattern for accessing the sprayer and GPS.
 *       - Mission commands have specific IDs that indicate their type.
 *       - The Earth is treated as a sphere for distance calculations.
 *       - If RTM is done to the mission it shifts according
 * 
 * @warning If the mission commands do not provide the necessary spacing or coverage information,
 *          the function will return early and send a relevant message. Similarly, if the mission 
 *          is deemed too short or if spacing or coverage is zero, the function will report and exit.
 * 
 * @post After successful execution, the calculated liters needed will be saved to the sprayer's `L_Need`
 *       and a message will be sent with the required liters.
 * @param None
 * @return None
 */
void AP_Mission::CalculateLiterNeed(){

    ABZ_Sprayer * sprayer = ABZ::get_singleton();
    if(sprayer==nullptr)
    {
        return;
    }

    AP_GPS * gps=AP_GPS::get_singleton();
    if(gps==nullptr)
    {
        return;
    }
    //create variable
    int rtm = 0;
    int j = 1;
    int first = 0;
    bool is_space_and_coverage = false;
    double cov;
    double space;
    double meter =0.0;
    float liter_need = 0.0;
    double s1w1_lat;
    double s1w1_lng;
    double s1w2_lat;
    double s1w2_lng;
    //create mission commands
    Mission_Command p1;
    Mission_Command p2;
    Mission_Command sp;
    Mission_Command cmd;
    Mission_Command fw;
    //conditional initializations
    if (isRTMDone){
        gcs().send_text(MAV_SEVERITY_INFO, "RTM");
        rtm = 4; //if rtm is shifted than shift everything by 4
    }
    //if mission is missing
    if(_cmd_total<4){
        gcs().send_text(MAV_SEVERITY_INFO, "Mission is too short");
        return;
    }
    //find first waypoint
    j = 1+rtm;

    while (j < _cmd_total){
        read_cmd_from_storage(j,fw);

        if (fw.id == 16){
            first = j;
            break;
        }

        j = j+1;
    }
    //get space and coverage
    j = 1+rtm;

    while (j < _cmd_total && is_space_and_coverage == false){
        read_cmd_from_storage(j,cmd);

        if (cmd.id == 1500){
            space = cmd.content.sprayer.spacing;
            cov = cmd.content.sprayer.coverage;
            is_space_and_coverage = true;
        }

        j = j+1;
    }
    //if we can't get spacing or coverage information
    if(is_space_and_coverage==false){
        gcs().send_text(MAV_SEVERITY_INFO, "Missing spacing and coverage");
        return;
    }
    //check is coverage or space is zero
    if(is_equal((float)space,0.0f) || is_equal((float)cov,0.0f)){
        gcs().send_text(MAV_SEVERITY_INFO, "Space or coverage is zero");
        return;
    }
    //count liter need for entire mission
    for(int i = first; i < _cmd_total;i+=3){
        read_cmd_from_storage(i,p1);
        read_cmd_from_storage(i+1,sp);
        //if p1 is a spraying section start we add the liter to the section
        if(sp.p1 == 1){
            read_cmd_from_storage(i+3,p2);
            s1w1_lat=(double)p1.content.location.lat/(double)10000000;
            s1w1_lng=(double)p1.content.location.lng/(double)10000000;
            s1w2_lat=(double)p2.content.location.lat/(double)10000000;
            s1w2_lng=(double)p2.content.location.lng/(double)10000000;
            meter = get_dist_of_two_points(s1w1_lat,s1w1_lng,s1w2_lat,s1w2_lng);
            //sprayer->total_meter=sprayer->total_meter+meter;
            meter = meter*space;
            meter = meter/10000;
            liter_need += meter*cov;
        }else{}//if p1 is a spraying stop we skip section
        
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Liter need: %f",liter_need);
    sprayer->L_Need.set_and_save(liter_need);
}
/**
 * @brief Calculates the total distance (in meters) of the sections in the mission.
 * 
 * This function computes the total distance to be covered in a spraying mission by processing the mission
 * commands and determining the waypoints that define a sections. Each section's length is 
 * computed and accumulated to give the total distance.
 * 
 * @note This function assumes a series of conditions:
 *       - The function utilizes a singleton pattern for accessing the sprayer and GPS instances.
 *       - Mission commands are expected to have specific IDs indicating their type. The Earth's 
 *       - geometry is assumed to be spherical for distance calculations.
 *       - If RTM is done to the mission it shifts according
 * 
 * @warning The function will exit early and send a relevant message if:
 *          - The sprayer or GPS singletons cannot be accessed.
 *          - The mission is deemed too short.
 * 
 * @post After successful execution, the calculated total distance will be saved to the sprayer's `total_meter`
 *       attribute and a message with the total distance in meters will be sent.
 * 
 * @param None
 * @return None
 */
void AP_Mission::calculateTotalMeter(){

    ABZ_Sprayer * sprayer = ABZ::get_singleton();
    if(sprayer==nullptr)
    {
        return;
    }

    AP_GPS * gps=AP_GPS::get_singleton();
    if(gps==nullptr)
    {
        return;
    }
    //create variable
    int rtm = 0;
    int j = 1;
    int first = 0;
    double meter =0.0;
    double s1w1_lat;
    double s1w1_lng;
    double s1w2_lat;
    double s1w2_lng;
    //create mission commands
    Mission_Command p1;
    Mission_Command p2;
    Mission_Command fw;


    //conditional initializations
    if (isRTMDone){
        gcs().send_text(MAV_SEVERITY_INFO, "RTM");
        rtm = 4; //if rtm is shifted than shift everything by 4
    }
    //if mission is missing
    if(_cmd_total<4){
        gcs().send_text(MAV_SEVERITY_INFO, "Mission is too short");
        return;
    }
    //find first waypoint
    j = 1+rtm; 
    while (j < _cmd_total){
        read_cmd_from_storage(j,fw);

        if (fw.id == 16){
            first = j;
            break;
        }

        j = j+1;
    }
    //count liter need for entire mission
    sprayer->total_meter = 0.0;
    for(int i = first; i < _cmd_total;i+=3){
        read_cmd_from_storage(i,p1);
        read_cmd_from_storage(i+3,p2);
        s1w1_lat=(double)p1.content.location.lat/(double)10000000;
        s1w1_lng=(double)p1.content.location.lng/(double)10000000;
        s1w2_lat=(double)p2.content.location.lat/(double)10000000;
        s1w2_lng=(double)p2.content.location.lng/(double)10000000;
        meter = get_dist_of_two_points(s1w1_lat,s1w1_lng,s1w2_lat,s1w2_lng);
        sprayer->total_meter=sprayer->total_meter+meter;
        
        
    }
    //we will set L-need but now we just print it.
    gcs().send_text(MAV_SEVERITY_INFO, "Total meter: %f",(float)sprayer->total_meter);
    
    
}

/**
 * @brief Calculates the key points in the mission such as tank empty and returning point.
 * 
 * This function evaluates the waypoints in the mission, determines where the sprayer's tank
 * will run empty, and decides on the optimal return point based on various conditions
 * such as distance from home, remaining liquid, and speed.
 * 
 * @note This function assumes the mission waypoints are stored in a particular order, 
 * changspeed,waypoint,sprayer. This structure is crucial for proper calculations since we increment by 3 to jump to the next waypoint.
 * 
 * Key steps:
 * 1. Initialization of key objects like sprayer, GPS, and mission commands.
 * 2. Reading home location and potential RTM (Return To Mission) shifts.
 * 3. Extracting key mission parameters like spacing, coverage, and speed from waypoints.
 * 4. Calculating key metrics like coverage percentage, remaining fluid, time, and starts.
 * 5. Evaluating each section (from start point to end point) to determine:
 *     - If the section is non spraying it jumps to the next.
 *     - If the section can be fully sprayed with the remaining fluid it jumps to next.
 *     - If not, calculating the point where the tank will run empty.
 *     - Determining the optimal return point based on distances.
 * 6. Sending key points and progress messages to the ground control station.
 * 
 * @warning This function heavily relies on the mission structure and waypoint commands. 
 * Changes in waypoint structure or command IDs can break the calculations.
 * 
 * @param None
 * @return None
 * 
 * @see read_cmd_from_storage(), get_dist_of_two_points(), calculatePoint3()
 */
//TO Do make sub functions and ABZ lib
void AP_Mission::calculatePoints(){

    ABZ_Sprayer * sprayer = ABZ::get_singleton();

    if(sprayer==nullptr)
    {
        return;
    }

    AP_GPS * gps=AP_GPS::get_singleton();

    if(gps==nullptr)
    {
        return;
    }
    
    calculateTotalMeter();
    //declare mission commands
    Mission_Command fw;
    Mission_Command home;
    Mission_Command p1;
    Mission_Command sp;
    Mission_Command p2;
    Mission_Command cmd;
    Mission_Command speed;

    //declare variables
    int rtm = 0;
    int j = 1;
    int first = 0;
    bool is_space_and_coverage = false;
    bool is_speed = false;
    double cov;
    double space;
    double meter;
    double local_tartaly = 0.0;
    double liter_need_to_next_wp = 0.0;
    double progress_percent = 1.0;
    double home_lat;
    double home_lng;
    double w1_lat;
    double w1_lng;
    double w2_lat;
    double w2_lng;
    double dist13;
    float d1;
    float d2;
    float LN;
    float TL;
    float MS;
    //declare custom lng,lat paire
    std::pair<double,double> point3;
    //initialize sprayer values
    sprayer->rlat=-1;
    sprayer->rlng=-1;
    sprayer->tlat=-1;
    sprayer->tlng=-1;
    sprayer->returning_point=-7;
    sprayer->send_tank_is_empty=true;
    //get home point coordinate
    read_cmd_from_storage(0,home);
    home_lat=(double)home.content.location.lat/(double)10000000;
    home_lng=(double)home.content.location.lng/(double)10000000;
    //if rtm is shifted than we shift by 4
    if (isRTMDone){
        //gcs().send_text(MAV_SEVERITY_INFO, "RTM");
        rtm = 4;
    }
    //if mission is missing
    if(_cmd_total<4){
        gcs().send_text(MAV_SEVERITY_INFO, "Mission is too short");
        gcs().send_message(MSG_ABZ_RETURNING_POINT_COR);                //TODO make a method instead of repeating lines
        gcs().send_message(MSG_ABZ_EMPTY_POINT_COR);
        gcs().send_message(MSG_ABZ_LEMON_POINT_COR);
        gcs().send_message(MSG_ABZ_HI_POINT_COR);
        return;
    }
    //set pwm based on drone type
    if (sprayer->drone_type==2){
        sprayer->pwm_a.set_and_save(13064.0);
        sprayer->pwm_b.set_and_save(1028.5);
        sprayer->pwm_c.set_and_save(1239.0);
    }else if(sprayer->drone_type==1){
        sprayer->pwm_a.set_and_save(36922.0);
        sprayer->pwm_b.set_and_save(8048.5);
        sprayer->pwm_c.set_and_save(1053.1);
    }
    //get spac and coverage:
    j=j+rtm;

    while (j < _cmd_total && is_space_and_coverage == false){
        read_cmd_from_storage(j,cmd);

        if (cmd.id == 1500){
            space = cmd.content.sprayer.spacing;
            cov = cmd.content.sprayer.coverage;
            is_space_and_coverage = true;
        }

        j = j+1;
    }
    
    //if we can't get spacing or coverage information
    if(is_space_and_coverage==false){
        gcs().send_text(MAV_SEVERITY_INFO, "Missing spacing and coverage");
        gcs().send_message(MSG_ABZ_RETURNING_POINT_COR);
        gcs().send_message(MSG_ABZ_EMPTY_POINT_COR);
        gcs().send_message(MSG_ABZ_LEMON_POINT_COR);
        gcs().send_message(MSG_ABZ_HI_POINT_COR);
        return;
    }
    //check is coverage or space is zero
    if(is_equal((float)space,0.0f) || is_equal((float)cov,0.0f)){
        gcs().send_text(MAV_SEVERITY_INFO, "Space or coverage is zero");
        gcs().send_message(MSG_ABZ_RETURNING_POINT_COR);
        gcs().send_message(MSG_ABZ_EMPTY_POINT_COR);
        gcs().send_message(MSG_ABZ_LEMON_POINT_COR);
        gcs().send_message(MSG_ABZ_HI_POINT_COR);
        return;
    }

    //if tank empty or in negative
    if(sprayer->liters_left <= 0){
        gcs().send_text(MAV_SEVERITY_INFO, "Tank value is low");
        gcs().send_message(MSG_ABZ_RETURNING_POINT_COR);
        gcs().send_message(MSG_ABZ_EMPTY_POINT_COR);
        gcs().send_message(MSG_ABZ_LEMON_POINT_COR);
        gcs().send_message(MSG_ABZ_HI_POINT_COR);
        return;
    }
    //get speed
    j = 1+rtm;
    while(j<_cmd_total){
        
        read_cmd_from_storage(j,speed);

        if (speed.id == 178){
            sprayer->max_speed = speed.content.speed.target_ms;
            is_speed=true;
            break;
        }

        j = j+1;
    }
    //if can't get speed or speed zero
    if(is_speed==false){
        gcs().send_text(MAV_SEVERITY_INFO, "No change speed found!");
        gcs().send_message(MSG_ABZ_RETURNING_POINT_COR);
        gcs().send_message(MSG_ABZ_EMPTY_POINT_COR);
        gcs().send_message(MSG_ABZ_LEMON_POINT_COR);
        gcs().send_message(MSG_ABZ_HI_POINT_COR);

        return;
    }

    if(is_equal((float)sprayer->max_speed,0.0f)){
        gcs().send_text(MAV_SEVERITY_INFO, "Speed is zero");
        gcs().send_message(MSG_ABZ_RETURNING_POINT_COR);
        gcs().send_message(MSG_ABZ_EMPTY_POINT_COR);
        gcs().send_message(MSG_ABZ_LEMON_POINT_COR);
        gcs().send_message(MSG_ABZ_HI_POINT_COR);

        return;
    }
    //check is Liters need is not 0
    LN = sprayer->L_Need;

    if (is_equal(LN,0.0f) == false){
        sprayer->coverage_percentage = (sprayer->T_sprayed/sprayer->L_Need)*100;
    }else{
        gcs().send_text(MAV_SEVERITY_INFO, "Zero att 885");
    }
    //check is tank capacity is not 0
    TL = sprayer->tartaly_liter;
    
    if(is_equal(TL,0.0f) == false){
        sprayer->remaining_fluid = sprayer->L_Need-sprayer->T_sprayed;
        sprayer->remaining_starts = sprayer->remaining_fluid/sprayer->tartaly_liter;
    }else{
        gcs().send_text(MAV_SEVERITY_INFO, "Zero att 894");
    }
    //set remaining starts
    if((int)sprayer->remaining_starts < sprayer->remaining_starts){
        sprayer->remaining_starts = (int)sprayer->remaining_starts+1;
    }
    //check is speed is not 0
    MS = sprayer->max_speed;

    if (is_equal(MS,0.0f) == false){
        sprayer->remaining_time = sprayer->total_meter/sprayer->max_speed;
        gcs().send_text(MAV_SEVERITY_INFO, "time: %f",(float)sprayer->remaining_time);
    }else{
        gcs().send_text(MAV_SEVERITY_INFO, "Zero att 906");
    }
    //find first non rtm waypoint
    j = 1+rtm;
    while (j < _cmd_total){
        read_cmd_from_storage(j,fw);

        if (fw.id == 16){
            first = j;
            break;
        }
        j = j+1;
    }
    //set local tartaly for calculation
    if (sprayer->liters_left < sprayer->tartaly_liter){
        local_tartaly = sprayer->liters_left;
    }else{
        local_tartaly = sprayer->tartaly_liter;
    }
    //if we can sprayer the entire mission we don't need to calculate
    if(sprayer->remaining_fluid < sprayer->tartaly_liter-0.25){
        gcs().send_message(MSG_ABZ_PLAN_PROGRESS);
        gcs().send_message(MSG_ABZ_RETURNING_POINT_COR);
        gcs().send_message(MSG_ABZ_EMPTY_POINT_COR);
        gcs().send_message(MSG_ABZ_LEMON_POINT_COR);
        gcs().send_message(MSG_ABZ_HI_POINT_COR);
        return;
    }
    //find tank empty and returning point
    for(int i = first; i < _cmd_total;i+=3){

        //read start point(p1),read sprayer for start point(sp), read end point(p2)
        read_cmd_from_storage(i,p1);
        read_cmd_from_storage(i+1,sp);   
        
        //if we are at the last point we sprayed everything. last point is the return point
        if (i+3 >= _cmd_total){
            w1_lat=(double)p1.content.location.lat/(double)10000000;
            w1_lng=(double)p1.content.location.lng/(double)10000000;
            sprayer->rlat=w1_lat;
            sprayer->rlng=w1_lng;
            //sprayer->tlat=w1_lat;
            //sprayer->tlng=w1_lng;
            sprayer->returning_point = i;
            break;
        }
        //if start point is a spraying start point we calculate if we can spray the section p1-p2
        if(sp.p1 == 1){
            //calculate liter need of section p1-p2
            read_cmd_from_storage(i+3,p2);
            w1_lat=(double)p1.content.location.lat/(double)10000000;
            w1_lng=(double)p1.content.location.lng/(double)10000000;
            w2_lat=(double)p2.content.location.lat/(double)10000000;
            w2_lng=(double)p2.content.location.lng/(double)10000000;
            meter = get_dist_of_two_points(w1_lat,w1_lng,w2_lat,w2_lng);
            liter_need_to_next_wp = meter*space;
            liter_need_to_next_wp = liter_need_to_next_wp/10000;
            liter_need_to_next_wp = liter_need_to_next_wp*cov;
            //it should not be zero but check just to be sure
            if (is_equal((float)liter_need_to_next_wp,0.0f)){
                gcs().send_text(MAV_SEVERITY_INFO, "Zero att 954");
                return;
            }else{
                progress_percent = (local_tartaly-0.2)/liter_need_to_next_wp;
            }
            //how many times we can spray with what we have in the tank
            if (progress_percent > 1.0){
                progress_percent = 1.0;
            }
            //finding closer side of the section
            d1 = get_dist_of_two_points(home_lat,home_lng,w1_lat,w1_lng);
            d2 = get_dist_of_two_points(home_lat,home_lng,w2_lat,w2_lng);
            //if we can't spraye the section calculate the returning point and tank empty point
            if(progress_percent<1.0){
                //find tank empty point distance from p1
                dist13 = meter*progress_percent;
                //determine lat and lng of tank empty point in the p1-p2 line 
                point3 = calculatePoint3(w1_lat, w1_lng, w2_lat, w2_lng, dist13);
                //if p2 is closer than we should spray until tank is empty. return = tank empty
                if(d2<d1){
                    sprayer->rlat=w2_lat;
                    sprayer->rlng=w2_lng;
                    sprayer->tlat=point3.first;
                    sprayer->tlng=point3.second;
                    sprayer->returning_point = i+3;
                }else{
                    //if p1 is closer than we should return at p1
                    sprayer->rlat=w1_lat;
                    sprayer->rlng=w1_lng;
                    sprayer->tlat=point3.first;
                    sprayer->tlng=point3.second;
                    sprayer->returning_point = i;  
                }
                //found point let's stop
                break;
            }else{
                //if we can spray 100% of section then lower the tank and continou
                local_tartaly=local_tartaly-liter_need_to_next_wp;
                //sync with tank is empty message

            }

        }//if p1 is stop spraying we just skip the section
        
    }
    //send points
    gcs().send_message(MSG_ABZ_PLAN_PROGRESS);
    gcs().send_message(MSG_ABZ_RETURNING_POINT_COR);
    gcs().send_message(MSG_ABZ_EMPTY_POINT_COR);
    gcs().send_message(MSG_ABZ_LEMON_POINT_COR);
    gcs().send_message(MSG_ABZ_HI_POINT_COR);
    
}
/**
 * @brief Handle Return To Mission (RTM) operation.
 *
 * This function checks if the RTM operation is enabled and is not done previously.
 * If the conditions are met, it modifies the mission commands to include RTM functionality.
 * The primary objective is to set intermediate waypoints and change speed for the UAV 
 * to return to its mission after certain conditions are met.
 *
 * @note The RTM is based on the UAV's current GPS location, a predetermined altitude,
 *       and the target speed (from the sprayer module). The function re-arranges the mission commands 
 *       to include these new instructions and then saves the updated total number of commands.
 *
 * @pre A valid instance of `ABZ_Sprayer` and `AP_GPS` should be available.
 *      The `isRTMDone` flag should be false to indicate RTM has not been done previously.
 *
 * @post The `_cmd_total` is updated with the new total number of commands.
 *       The `isRTMDone` flag is set to true, indicating RTM is done.
 *       A message is sent to the ground control station indicating an update in the mission.
 */
void AP_Mission::RTM(){

    ABZ_Sprayer * sprayer = ABZ::get_singleton();

    if(sprayer==nullptr){
        return;
    }

    AP_GPS * gps=AP_GPS::get_singleton();

    if(gps==nullptr){
        return;
    }

    if(sprayer->getReturn_To_Mission_Like_Rtl() && isRTMDone == false){

        Mission_Command changespeed;
        Mission_Command command;
        Mission_Command temp_command;
        Mission_Command first_waypoint;
        Mission_Command cmd;
        int total;

        changespeed.id=MAV_CMD_DO_CHANGE_SPEED;
        changespeed.content.speed.speed_type=1;
        changespeed.content.speed.target_ms=sprayer->return_to_mission_like_rtl_speed;
        changespeed.content.speed.throttle_pct=-1;

        for(int i=_cmd_total;i>0;i--){
            read_cmd_from_storage(i,command);
            write_cmd_to_storage(i+4, command);
        }

        for(int i=1;i<5;i++){
            temp_command.p1=0;
            write_cmd_to_storage(i ,temp_command);
        }

        
        read_cmd_from_storage(6,first_waypoint);
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.content.location.lat=gps->location().lat;
        cmd.content.location.lng=gps->location().lng;
        cmd.content.location.alt=sprayer->getRTMAlt();
        cmd.content.location.change_alt_frame(Location::AltFrame::ABOVE_TERRAIN);
        cmd.content.location.alt=sprayer->getRTMAlt();
        write_cmd_to_storage(1,changespeed);
        write_cmd_to_storage(2,cmd);
        cmd.content.location.lat=first_waypoint.content.location.lat;
        cmd.content.location.lng=first_waypoint.content.location.lng;
        cmd.content.location.alt=(int32_t)sprayer->getRTMAlt();
        write_cmd_to_storage(3,changespeed);
        write_cmd_to_storage(4,cmd);
        total=_cmd_total+4;
        _cmd_total.set_and_save(total);
        isRTMDone = true;
        gcs().send_message(MSG_ABZ_UPDATE_MISSION);
    }
}
/**
 * @brief Starts the mission process.
 *
 * This method initializes the GPS and sprayer singletons, sets up initial states for the sprayer,
 * and prepares the mission by advancing to the first navigation command. It also calculates certain
 * points for the mission and sends a message indicating the mission's progress. Runs when copter turned to auto mode.
 * It calls RTM and calculatePoints() also sets the tartaly_liter and round sprayed.
 * 
 * @pre Assumes that the singletons for AP_GPS and ABZ_Sprayer have been properly instantiated.
 * 
 * @post If successful, the mission state is set to MISSION_RUNNING. If there's a failure in advancing
 *       the navigation command, the mission is marked as complete.
 *
 * @note This function returns early if either the GPS or sprayer singletons are not available.
 * @param None
 * @return None
 * 
 * @see RTM(),calculatePoints()
 */
void AP_Mission::start(){

    AP_GPS * gps=AP_GPS::get_singleton();
    if(gps==nullptr)
    {
        return;
    }
    ABZ_Sprayer * sprayer = ABZ::get_singleton();
    if(sprayer==nullptr)
    {
        return;
    }
    
    sprayer->tartaly_liter = sprayer->liters_left; /*the liquid in the tank equals the liquid left in the tank. */
    sprayer->round_sprayed = 0.0; /*New start means new spraying round make sprayed in rround zero*/
    RTM();
    //sending info
    _flags.state = MISSION_RUNNING;
    sprayer->SetSpraying_speed_adaptive(false);
    reset(); // reset mission to the first command, resets jump tracking
    // advance to the first command
    if (!advance_current_nav_cmd()) {
        // on failure set mission complete
        complete();
    }
    //Calculate the Returning and the Tank is empty point
    calculatePoints();
    gcs().send_message(MSG_ABZ_IS_MISSION_IN_PROGRESS);/*This message tells QGC if the mission is running or not*/
}

/// stop - stops mission execution.  subsequent calls to update() will have no effect until the mission is started or resumed
/**
 * @brief Stops the current mission and performs necessary updates and configurations.
 *
 * This method updates the mission state to indicate that the mission has stopped. Additionally,
 * it retrieves and configures associated components like the sprayer and GPS. Depending on the
 * state and configuration, mission commands may be adjusted, especially when considering the
 * "return to mission" functionality. If RTM is on then it reverses the RTM effect
 *
 * @pre Assumes the singletons for AP_GPS, ABZ_Sprayer, and AP_Mission have been properly instantiated.
 *
 * @post The mission state is updated to MISSION_STOPPED. Messages are sent to notify about the 
 *       mission's and sprayer's status. Based on the configurations and states, mission commands
 *       may be adjusted or updated, and points for the mission might be recalculated.
 *
 * @note This function has multiple return points based on the availability of certain singletons.
 *       Additionally, it handles cases both when the "return to mission" feature is enabled and 
 *       when it is not.
 */
void AP_Mission::stop()
{
    _flags.state = MISSION_STOPPED;
    gcs().send_message(MSG_ABZ_IS_MISSION_IN_PROGRESS);

    AP_Mission *mission = AP_Mission::get_singleton();
    if (mission == nullptr){
        return;
    }
   
    ABZ_Sprayer * sprayer = ABZ::get_singleton();
    if(sprayer==nullptr){
        return;
    }
    gcs().send_message(MSG_ABZ_IS_SPRAYING);
    sprayer->SetSpraying_speed_adaptive(false);
    sprayer->setSpinnerPwm(1050);
    sprayer->resetDontSprayInMission();

    AP_GPS * gps=AP_GPS::get_singleton();
    if(gps==nullptr)
    {
        return;
    }
    bool changed=false;
   if(sprayer->getReturn_To_Mission_Like_Rtl())
    {
        isRTMDone = false;
       
        if(_nav_cmd.index!=AP_MISSION_CMD_INDEX_NONE)
        {
           
            if(_nav_cmd.index<8)
            {
                
                
                int j=1;
                for (int i=5;i<_cmd_total;i++)
                {
                     Mission_Command cmd;
                     read_cmd_from_storage(i,cmd);
                     write_cmd_to_storage(j ,cmd);
                     j++;

                
                }
                changed=true;
                _cmd_total.set_and_save(j);
            }
        }
    }
    if(!changed)
    {
    if(_prev_nav_cmd_index!=AP_MISSION_CMD_INDEX_NONE)
        {
        _prev_nav_cmd.content.location.lat=gps->location().lat;
        _prev_nav_cmd.content.location.lng=gps->location().lng;
        Mission_Command changespeed;
        if(sprayer->getReturn_To_Mission_Like_Rtl()){
            changespeed.id=MAV_CMD_DO_CHANGE_SPEED;
            changespeed.content.speed.speed_type=1;
            changespeed.content.speed.target_ms=sprayer->return_to_mission_like_rtl_speed;
            changespeed.content.speed.throttle_pct=-1;
        }else{
            int j = 0;
            while (j < _cmd_total){
                read_cmd_from_storage(j,changespeed);
                if (changespeed.id == MAV_CMD_DO_CHANGE_SPEED){
                    break;
                }else{
                    j = j+1;
                }
            }
        }
        
        write_cmd_to_storage(1,changespeed);
        write_cmd_to_storage(2, _prev_nav_cmd);

        int cmd_total_local=2;
        for(int i=_prev_nav_cmd_index+1;i<_cmd_total;i++) 
        {
            Mission_Command cmd;
            
            read_cmd_from_storage(i,cmd);
            write_cmd_to_storage(cmd_total_local+1, cmd);

        
            cmd_total_local++;
        
        }
        
        
        _cmd_total.set_and_save(cmd_total_local+1);
        }
    }
    gcs().send_message(MSG_ABZ_UPDATE_MISSION);

    reset();
    calculatePoints();
}
/// resume - continues the mission execution from where we last left off
/// previous running commands will be re-initialized
void AP_Mission::resume()
{
    // if mission had completed then start it from the first command
    if (_flags.state == MISSION_COMPLETE) {
        start();
        return;
    }

    // if mission had stopped then restart it
    if (_flags.state == MISSION_STOPPED) {
        _flags.state = MISSION_RUNNING;

        // if no valid nav command index restart from beginning
        if (_nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
            start();
            return;
        }
    }

    // ensure cache coherence
    if (!read_cmd_from_storage(_nav_cmd.index, _nav_cmd)) {
        // if we failed to read the command from storage, then the command may have
        // been from a previously loaded mission it is illogical to ever resume
        // flying to a command that has been excluded from the current mission
        start();
        return;
    }

    // rewind the mission wp if the repeat distance has been set via MAV_CMD_DO_SET_RESUME_REPEAT_DIST
    if (_repeat_dist > 0 && _wp_index_history[LAST_WP_PASSED] != AP_MISSION_CMD_INDEX_NONE) {
        // if not already in a resume state calculate the position to rewind to
        Mission_Command tmp_cmd;
        if (!_flags.resuming_mission && calc_rewind_pos(tmp_cmd)) {
            _resume_cmd = tmp_cmd;
        }

        // resume mission to rewound position
        if (_resume_cmd.index != AP_MISSION_CMD_INDEX_NONE && start_command(_resume_cmd)) {
            _nav_cmd = _resume_cmd;
            _flags.nav_cmd_loaded = true;
            // set flag to prevent history being re-written
            _flags.resuming_mission = true;
            return;
        }
    }

    // restart active navigation command. We run these on resume()
    // regardless of whether the mission was stopped, as we may be
    // re-entering AUTO mode and the nav_cmd callback needs to be run
    // to setup the current target waypoint

    if (_flags.do_cmd_loaded && _do_cmd.index != AP_MISSION_CMD_INDEX_NONE) {
        // restart the active do command, which will also load the nav command for us
        set_current_cmd(_do_cmd.index);
    } else if (_flags.nav_cmd_loaded) {
        // restart the active nav command
        set_current_cmd(_nav_cmd.index);
    }

    // Note: if there is no active command then the mission must have been stopped just after the previous nav command completed
    //      update will take care of finding and starting the nav command
}

/// check if the next nav command is a takeoff, skipping delays
bool AP_Mission::is_takeoff_next(uint16_t cmd_index)
{
    Mission_Command cmd = {};
    // check a maximum of 16 items, remembering that missions can have
    // loops in them
    for (uint8_t i=0; i<16; i++, cmd_index++) {
        if (!get_next_nav_cmd(cmd_index, cmd)) {
            return false;
        }
        switch (cmd.id) {
        // any of these are considered a takeoff command:
        case MAV_CMD_NAV_VTOL_TAKEOFF:
        case MAV_CMD_NAV_TAKEOFF:
        case MAV_CMD_NAV_TAKEOFF_LOCAL:
            return true;
        // any of these are considered "skippable" when determining if
        // we "start with a takeoff command"
        case MAV_CMD_DO_AUX_FUNCTION:
        case MAV_CMD_NAV_DELAY:
            continue;
        default:
            return false;
        }
    }
    return false;
}

/// check mission starts with a takeoff command
bool AP_Mission::starts_with_takeoff_cmd()
{
    uint16_t cmd_index = _restart ? AP_MISSION_CMD_INDEX_NONE : _nav_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    }
    return is_takeoff_next(cmd_index);
}

/*
    return true if MIS_OPTIONS is set to allow continue of mission
    logic after a land and the next waypoint is a takeoff. If this
    is false then after a landing is complete the vehicle should 
    disarm and mission logic should stop
*/
bool AP_Mission::continue_after_land_check_for_takeoff()
{
    if (!continue_after_land()) {
        return false;
    }
    if (_nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
        return false;
    }
    return is_takeoff_next(_nav_cmd.index+1);
}

/// start_or_resume - if MIS_AUTORESTART=0 this will call resume(), otherwise it will call start()
void AP_Mission::start_or_resume()
{
    if (_restart == 1 && !_force_resume) {
        start();
    } else {
        resume();
        _force_resume = false;
    }
}

/// reset - reset mission to the first command
void AP_Mission::reset()
{
    _flags.nav_cmd_loaded  = false;
    _flags.do_cmd_loaded   = false;
    _flags.do_cmd_all_done = false;
    _flags.in_landing_sequence = false;
    _nav_cmd.index         = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index          = AP_MISSION_CMD_INDEX_NONE;
    _prev_nav_cmd_index    = AP_MISSION_CMD_INDEX_NONE;
    _prev_nav_cmd_wp_index = AP_MISSION_CMD_INDEX_NONE;
    _prev_nav_cmd_id       = AP_MISSION_CMD_ID_NONE;
    init_jump_tracking();
    reset_wp_history();
}

/// clear - clears out mission
///     returns true if mission was running so it could not be cleared
bool AP_Mission::clear()
{
    // do not allow clearing the mission while it is running
    if (_flags.state == MISSION_RUNNING) {
        return false;
    }

    // remove all commands
    truncate(0);

    // clear index to commands
    _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.nav_cmd_loaded = false;
    _flags.do_cmd_loaded = false;

    // return success
    return true;
}


/// trucate - truncate any mission items beyond index
void AP_Mission::truncate(uint16_t index)
{
    if ((unsigned)_cmd_total > index) {
        _cmd_total.set_and_save(index);
        _last_change_time_ms = AP_HAL::millis();
    }
}

/// update - ensures the command queues are loaded with the next command and calls main programs command_init and command_verify functions to progress the mission
///     should be called at 10hz or higher
void AP_Mission::update()
{
    // exit immediately if not running or no mission commands
    if (_flags.state != MISSION_RUNNING || _cmd_total == 0) {
        return;
    }

    update_exit_position();

    // save persistent waypoint_num for watchdog restore
    hal.util->persistent_data.waypoint_num = _nav_cmd.index;

    // check if we have an active nav command
    if (!_flags.nav_cmd_loaded || _nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
        // advance in mission if no active nav command
        if (!advance_current_nav_cmd()) {
            // failure to advance nav command means mission has completed
            complete();
            return;
        }
    } else {
        // run the active nav command
        if (verify_command(_nav_cmd)) {
            // market _nav_cmd as complete (it will be started on the next iteration)
            _flags.nav_cmd_loaded = false;
            // immediately advance to the next mission command
            if (!advance_current_nav_cmd()) {
                // failure to advance nav command means mission has completed
                complete();
                return;
            }
        }
    }

    // check if we have an active do command
    if (!_flags.do_cmd_loaded) {
        advance_current_do_cmd();
    } else {
        // check the active do command
        if (verify_command(_do_cmd)) {
            // mark _do_cmd as complete
            _flags.do_cmd_loaded = false;
        }
    }
}
/*ABZ sprayer added*/
bool AP_Mission::verify_command(const Mission_Command& cmd)
{
    switch (cmd.id) {
    // do-commands always return true for verify:
    case MAV_CMD_DO_GRIPPER:
    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_DO_SET_RELAY:
    case MAV_CMD_DO_REPEAT_SERVO:
    case MAV_CMD_DO_REPEAT_RELAY:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_PARACHUTE:
    case MAV_CMD_DO_SEND_SCRIPT_MESSAGE:
    case MAV_CMD_DO_SPRAYER:
    case MAV_CMD_DO_AUX_FUNCTION:
    case MAV_CMD_DO_SET_RESUME_REPEAT_DIST:
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
    case MAV_CMD_ABZ_SPRAYER: /*added thee  ABZ Sprayer class*/
        return true;
    default:
        return _cmd_verify_fn(cmd);
    }
}
/*ABZ sprayer added*/
bool AP_Mission::start_command(const Mission_Command& cmd)
{
    // check for landing related commands and set in_landing_sequence flag
    if (is_landing_type_cmd(cmd.id) || cmd.id == MAV_CMD_DO_LAND_START) {
        set_in_landing_sequence_flag(true);
    } else if (is_takeoff_type_cmd(cmd.id)) {
        set_in_landing_sequence_flag(false);
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Mission: %u %s", cmd.index, cmd.type());
    switch (cmd.id) {
    case MAV_CMD_DO_AUX_FUNCTION:
        return start_command_do_aux_function(cmd);
    case MAV_CMD_DO_GRIPPER:
        return start_command_do_gripper(cmd);
    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_DO_SET_RELAY:
    case MAV_CMD_DO_REPEAT_SERVO:
    case MAV_CMD_DO_REPEAT_RELAY:
        return start_command_do_servorelayevents(cmd);
    case MAV_CMD_DO_CONTROL_VIDEO:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        return start_command_camera(cmd);
    case MAV_CMD_DO_PARACHUTE:
        return start_command_parachute(cmd);
    case MAV_CMD_DO_SEND_SCRIPT_MESSAGE:
        return start_command_do_scripting(cmd);
    case MAV_CMD_DO_SPRAYER:
        return start_command_do_sprayer(cmd);
    case MAV_CMD_DO_SET_RESUME_REPEAT_DIST:
        return command_do_set_repeat_dist(cmd);
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        return start_command_do_gimbal_manager_pitchyaw(cmd);
    case MAV_CMD_ABZ_SPRAYER:
        return start_command_abz_sprayer(cmd);
    default:
        return _cmd_start_fn(cmd);
    }
}

///
/// public command methods
///

/// add_cmd - adds a command to the end of the command list and writes to storage
///     returns true if successfully added, false on failure
///     cmd.index is updated with it's new position in the mission
bool AP_Mission::add_cmd(Mission_Command& cmd)
{
    // attempt to write the command to storage
    bool ret = write_cmd_to_storage(_cmd_total, cmd);

    if (ret) {
        // update command's index
        cmd.index = _cmd_total;
        // increment total number of commands
        _cmd_total.set_and_save(_cmd_total + 1);
    }

    return ret;
}

/// replace_cmd - replaces the command at position 'index' in the command list with the provided cmd
///     replacing the current active command will have no effect until the command is restarted
///     returns true if successfully replaced, false on failure
bool AP_Mission::replace_cmd(uint16_t index, const Mission_Command& cmd)
{
    // sanity check index
    if (index >= (unsigned)_cmd_total) {
        return false;
    }

    // attempt to write the command to storage
    return write_cmd_to_storage(index, cmd);
}

/// is_nav_cmd - returns true if the command's id is a "navigation" command, false if "do" or "conditional" command
bool AP_Mission::is_nav_cmd(const Mission_Command& cmd)
{
    // NAV commands all have ids below MAV_CMD_NAV_LAST, plus some exceptions
    return (cmd.id <= MAV_CMD_NAV_LAST ||
            cmd.id == MAV_CMD_NAV_SET_YAW_SPEED ||
            cmd.id == MAV_CMD_NAV_SCRIPT_TIME ||
            cmd.id == MAV_CMD_NAV_ATTITUDE_TIME);
}

/// get_next_nav_cmd - gets next "navigation" command found at or after start_index
///     returns true if found, false if not found (i.e. reached end of mission command list)
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_nav_cmd is responsible for this)
bool AP_Mission::get_next_nav_cmd(uint16_t start_index, Mission_Command& cmd)
{
    // search until the end of the mission command list
    for (uint16_t cmd_index = start_index; cmd_index < (unsigned)_cmd_total; cmd_index++) {
        // get next command
        if (!get_next_cmd(cmd_index, cmd, false)) {
            // no more commands so return failure
            return false;
        }
        // if found a "navigation" command then return it
        if (is_nav_cmd(cmd)) {
            return true;
        }
    }

    // if we got this far we did not find a navigation command
    return false;
}

/// get the ground course of the next navigation leg in centidegrees
/// from 0 36000. Return default_angle if next navigation
/// leg cannot be determined
int32_t AP_Mission::get_next_ground_course_cd(int32_t default_angle)
{
    Mission_Command cmd;
    if (!get_next_nav_cmd(_nav_cmd.index+1, cmd)) {
        return default_angle;
    }
    // special handling for nav commands with no target location
    if (cmd.id == MAV_CMD_NAV_GUIDED_ENABLE ||
        cmd.id == MAV_CMD_NAV_DELAY) {
        return default_angle;
    }
    if (cmd.id == MAV_CMD_NAV_SET_YAW_SPEED) {
        return (_nav_cmd.content.set_yaw_speed.angle_deg * 100);
    }
    return _nav_cmd.content.location.get_bearing_to(cmd.content.location);
}

// set_current_cmd - jumps to command specified by index
bool AP_Mission::set_current_cmd(uint16_t index, bool rewind)
{
    // read command to check for DO_LAND_START
    Mission_Command cmd;
    if (!read_cmd_from_storage(index, cmd) || (cmd.id != MAV_CMD_DO_LAND_START)) {
        _flags.in_landing_sequence = false;
    }

    // mission command has been set and not as rewind command, don't track history.
    if (!rewind) {
        reset_wp_history();
    }

    // sanity check index and that we have a mission
    if (index >= (unsigned)_cmd_total || _cmd_total == 1) {
        return false;
    }

    // stop the current running do command
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.do_cmd_loaded = false;
    _flags.do_cmd_all_done = false;

    // stop current nav cmd
    _flags.nav_cmd_loaded = false;

    // if index is zero then the user wants to completely restart the mission
    if (index == 0 || _flags.state == MISSION_COMPLETE) {
        _prev_nav_cmd_id    = AP_MISSION_CMD_ID_NONE;
        _prev_nav_cmd_index = AP_MISSION_CMD_INDEX_NONE;
        _prev_nav_cmd_wp_index = AP_MISSION_CMD_INDEX_NONE;
        // reset the jump tracking to zero
        init_jump_tracking();
        if (index == 0) {
            index = 1;
        }
    }

    // if the mission is stopped or completed move the nav_cmd index to the specified point and set the state to stopped
    // so that if the user resumes the mission it will begin at the specified index
    if (_flags.state != MISSION_RUNNING) {
        // search until we find next nav command or reach end of command list
        while (!_flags.nav_cmd_loaded) {
            // get next command
            if (!get_next_cmd(index, cmd, true)) {
                _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
                return false;
            }

            // check if navigation or "do" command
            if (is_nav_cmd(cmd)) {
                // set current navigation command
                _nav_cmd = cmd;
                _flags.nav_cmd_loaded = true;
            } else {
                // set current do command
                if (!_flags.do_cmd_loaded) {
                    _do_cmd = cmd;
                    _flags.do_cmd_loaded = true;
                }
            }
            // move onto next command
            index = cmd.index+1;
        }

        // if we have not found a do command then set flag to show there are no do-commands to be run before nav command completes
        if (!_flags.do_cmd_loaded) {
            _flags.do_cmd_all_done = true;
        }

        // if we got this far then the mission can safely be "resumed" from the specified index so we set the state to "stopped"
        _flags.state = MISSION_STOPPED;
        return true;
    }

    // the state must be MISSION_RUNNING, allow advance_current_nav_cmd() to manage starting the item
    if (!advance_current_nav_cmd(index)) {
        // on failure set mission complete
        complete();
        return false;
    }

    // if we got this far we must have successfully advanced the nav command
    return true;
}

// restart current navigation command.  Used to handle external changes to mission
// returns true on success, false if mission is not running or current nav command is invalid
bool AP_Mission::restart_current_nav_cmd()
{
    // return immediately if mission is not running
    if (_flags.state != MISSION_RUNNING) {
        return false;
    }

    // return immediately if nav command index is invalid
    const uint16_t nav_cmd_index = get_current_nav_index();
    if ((nav_cmd_index == 0) || (nav_cmd_index >= num_commands())) {
        return false;
    }

    return set_current_cmd(_nav_cmd.index);
}

// returns false on any issue at all.
bool AP_Mission::set_item(uint16_t index, mavlink_mission_item_int_t& src_packet)
{
    // this is the on-storage format
    AP_Mission::Mission_Command cmd;

    // can't handle request for anything bigger than the mission size+1...
    if (index > num_commands()) {
        return false;
    }

    // convert from mavlink-ish format to storage format, if we can.
    if (mavlink_int_to_mission_cmd(src_packet, cmd) != MAV_MISSION_ACCEPTED) {
        return false;
    }

    // A request to set the 'next' item after the end is how we add an extra
    //  item to the list, thus allowing us to write entire missions if needed.
    if (index == num_commands()) {
        return add_cmd(cmd);
    }

    // replacing an existing mission item...
    return AP_Mission::replace_cmd(index, cmd);
}

bool AP_Mission::get_item(uint16_t index, mavlink_mission_item_int_t& ret_packet) const
{
    // setting ret_packet.command = -1  and/or returning false
    //  means it contains invalid data after it leaves here.

    // this is the on-storage format
    AP_Mission::Mission_Command cmd;

    // can't handle request for anything bigger than the mission size...
    if (index >= num_commands()) {
        ret_packet.command = -1;
        return false;
    }

    // minimal placeholder values during read-from-storage
    ret_packet.target_system = 1;     // unused sysid
    ret_packet.target_component =  1; // unused compid

    // 0=home, higher number/s = mission item number.
    ret_packet.seq = index;

    // retrieve mission from eeprom
    if (!read_cmd_from_storage(ret_packet.seq, cmd)) {
        ret_packet.command = -1;
        return false;
    }
    // convert into mavlink-ish format for lua and friends.
    if (!mission_cmd_to_mavlink_int(cmd, ret_packet)) {
        ret_packet.command = -1;
        return false;
    }

    // set packet's current field to 1 if this is the command being executed
    if (cmd.id == (uint16_t)get_current_nav_cmd().index) {
        ret_packet.current = 1;
    } else {
        ret_packet.current = 0;
    }

    // set auto continue to 1, becasue that's what's done elsewhere.
    ret_packet.autocontinue = 1;     // 1 (true), 0 (false)
    ret_packet.command = cmd.id;

    return true;
}


struct PACKED Packed_Location_Option_Flags {
    uint8_t relative_alt : 1;           // 1 if altitude is relative to home
    uint8_t unused1      : 1;           // unused flag (defined so that loiter_ccw uses the correct bit)
    uint8_t loiter_ccw   : 1;           // 0 if clockwise, 1 if counter clockwise
    uint8_t terrain_alt  : 1;           // this altitude is above terrain
    uint8_t origin_alt   : 1;           // this altitude is above ekf origin
    uint8_t loiter_xtrack : 1;          // 0 to crosstrack from center of waypoint, 1 to crosstrack from tangent exit location
    uint8_t type_specific_bit_0 : 1;    // each mission item type can use this for storing 1 bit of extra data
};

struct PACKED PackedLocation {
    union {
        Packed_Location_Option_Flags flags;                    ///< options bitmask (1<<0 = relative altitude)
        uint8_t options;                                /// allows writing all flags to eeprom as one byte
    };
    // by making alt 24 bit we can make p1 in a command 16 bit,
    // allowing an accurate angle in centi-degrees. This keeps the
    // storage cost per mission item at 15 bytes, and allows mission
    // altitudes of up to +/- 83km
    int32_t alt:24;                                     ///< param 2 - Altitude in centimeters (meters * 100) see LOCATION_ALT_MAX_M
    int32_t lat;                                        ///< param 3 - Latitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
};

union PackedContent {
    // location
    PackedLocation location;      // Waypoint location

    // raw bytes, for reading/writing to eeprom. Note that only 10
    // bytes are available if a 16 bit command ID is used
    uint8_t bytes[12];

};

assert_storage_size<PackedContent, 12> assert_storage_size_PackedContent;

/// load_cmd_from_storage - load command from storage
///     true is return if successful
/**
 * @brief Returns command from storage at index
 * This is a basic function. It takes an index ad a Mission command variable.
 * Reads the mission item from the given index and puts it to the given variable.
 * @param index is the index of the  mission item in the command storage
 * @param cmd is the Missioon commmand type variable that will hold the data red from storage
 * @return True on succes
*/
bool AP_Mission::read_cmd_from_storage(uint16_t index, Mission_Command& cmd) const
{
    WITH_SEMAPHORE(_rsem);

    // special handling for command #0 which is home
    if (index == 0) {
        cmd = {};
        cmd.id = MAV_CMD_NAV_WAYPOINT;
        cmd.content.location = AP::ahrs().get_home();
        return true;
    }

    if (index >= (unsigned)_cmd_total) {
        return false;
    }

    // ensure all bytes of cmd are zeroed
    cmd = {};

    // Find out proper location in memory by using the start_byte position + the index
    // we can load a command, we don't process it yet
    // read WP position
    const uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

    PackedContent packed_content {};

    const uint8_t b1 = _storage.read_byte(pos_in_storage);
    if (b1 == 0) {
        cmd.id = _storage.read_uint16(pos_in_storage+1);
        cmd.p1 = _storage.read_uint16(pos_in_storage+3);
        _storage.read_block(packed_content.bytes, pos_in_storage+5, 10);
    } else {
        cmd.id = b1;
        cmd.p1 = _storage.read_uint16(pos_in_storage+1);
        _storage.read_block(packed_content.bytes, pos_in_storage+3, 12);
    }

    if (stored_in_location(cmd.id)) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        // NOTE!  no 16-bit command may be stored_in_location as only
        // 10 bytes are available for storage and lat/lon/alt required
        // 4*sizeof(float) == 12 bytes of storage.
        if (b1 == 0) {
            AP_HAL::panic("May not store location for 16-bit commands");
        }
#endif
        // Location is not PACKED; field-wise copy it:
        cmd.content.location.relative_alt = packed_content.location.flags.relative_alt;
        cmd.content.location.loiter_ccw = packed_content.location.flags.loiter_ccw;
        cmd.content.location.terrain_alt = packed_content.location.flags.terrain_alt;
        cmd.content.location.origin_alt = packed_content.location.flags.origin_alt;
        cmd.content.location.loiter_xtrack = packed_content.location.flags.loiter_xtrack;
        cmd.content.location.alt = packed_content.location.alt;
        cmd.content.location.lat = packed_content.location.lat;
        cmd.content.location.lng = packed_content.location.lng;

        if (packed_content.location.flags.type_specific_bit_0) {
            cmd.type_specific_bits = 1U << 0;
        }
    } else {
        // all other options in Content are assumed to be packed:
        static_assert(sizeof(cmd.content) >= 12,
                      "content is big enough to take bytes");
        // (void *) cast to specify gcc that we know that we are copy byte into a non trivial type and leaving 4 bytes untouched
        memcpy((void *)&cmd.content, packed_content.bytes, 12);
    }

    // set command's index to it's position in eeprom
    cmd.index = index;

    // return success
    return true;
}

bool AP_Mission::stored_in_location(uint16_t id)
{
    switch (id) {
    case MAV_CMD_NAV_WAYPOINT:
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_NAV_TAKEOFF:
    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
    case MAV_CMD_NAV_LOITER_TO_ALT:
    case MAV_CMD_NAV_SPLINE_WAYPOINT:
    case MAV_CMD_NAV_GUIDED_ENABLE:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_LAND_START:
    case MAV_CMD_DO_GO_AROUND:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        return true;
    default:
        return false;
    }
}

/// write_cmd_to_storage - write a command to storage
///     index is used to calculate the storage location
///     true is returned if successful
/**
 * @brief Writes command to storage to a given index
 * This is a basic function. It takes an index ad a Mission command variable.
 * Writes the mission item to the given index.
 * @param index is the index of the  mission item in the command storage
 * @param cmd is the Missioon commmand type variable that holds the mmission item data 
 * @return True on succes
*/
bool AP_Mission::write_cmd_to_storage(uint16_t index, const Mission_Command& cmd)
{
    WITH_SEMAPHORE(_rsem);

    // range check cmd's index
    if (index >= num_commands_max()) {
        return false;
    }

    PackedContent packed {};
    if (stored_in_location(cmd.id)) {
        // Location is not PACKED; field-wise copy it:
        packed.location.flags.relative_alt = cmd.content.location.relative_alt;
        packed.location.flags.loiter_ccw = cmd.content.location.loiter_ccw;
        packed.location.flags.terrain_alt = cmd.content.location.terrain_alt;
        packed.location.flags.origin_alt = cmd.content.location.origin_alt;
        packed.location.flags.loiter_xtrack = cmd.content.location.loiter_xtrack;
        packed.location.flags.type_specific_bit_0 = cmd.type_specific_bits & (1U<<0);
        packed.location.alt = cmd.content.location.alt;
        packed.location.lat = cmd.content.location.lat;
        packed.location.lng = cmd.content.location.lng;
    } else {
        // all other options in Content are assumed to be packed:
        static_assert(sizeof(packed.bytes) >= 12,
                      "packed.bytes is big enough to take content");
        memcpy(packed.bytes, &cmd.content, 12);
    }

    // calculate where in storage the command should be placed
    uint16_t pos_in_storage = 4 + (index * AP_MISSION_EEPROM_COMMAND_SIZE);

    if (cmd.id < 256) {
        _storage.write_byte(pos_in_storage, cmd.id);
        _storage.write_uint16(pos_in_storage+1, cmd.p1);
        _storage.write_block(pos_in_storage+3, packed.bytes, 12);
    } else {
        // if the command ID is above 256 we store a 0 followed by the 16 bit command ID
        _storage.write_byte(pos_in_storage, 0);
        _storage.write_uint16(pos_in_storage+1, cmd.id);
        _storage.write_uint16(pos_in_storage+3, cmd.p1);
        _storage.write_block(pos_in_storage+5, packed.bytes, 10);
    }

    // remember when the mission last changed
    _last_change_time_ms = AP_HAL::millis();

    // return success
    return true;
}

/// write_home_to_storage - writes the special purpose cmd 0 (home) to storage
///     home is taken directly from ahrs
void AP_Mission::write_home_to_storage()
{
    Mission_Command home_cmd = {};
    home_cmd.id = MAV_CMD_NAV_WAYPOINT;
    home_cmd.content.location = AP::ahrs().get_home();
    write_cmd_to_storage(0,home_cmd);
}

MAV_MISSION_RESULT AP_Mission::sanity_check_params(const mavlink_mission_item_int_t& packet)
{
    uint8_t nan_mask;
    switch (packet.command) {
    case MAV_CMD_NAV_WAYPOINT:
        nan_mask = ~(1 << 3); // param 4 can be nan
        break;
    case MAV_CMD_NAV_LAND:
        nan_mask = ~(1 << 3); // param 4 can be nan
        break;
    case MAV_CMD_NAV_TAKEOFF:
        nan_mask = ~(1 << 3); // param 4 can be nan
        break;
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        nan_mask = ~(1 << 3); // param 4 can be nan
        break;
    case MAV_CMD_NAV_VTOL_LAND:
        nan_mask = ~((1 << 2) | (1 << 3)); // param 3 and 4 can be nan
        break;
    default:
        nan_mask = 0xff;
        break;
    }

    if (((nan_mask & (1 << 0)) && isnan(packet.param1)) ||
        isinf(packet.param1)) {
        return MAV_MISSION_INVALID_PARAM1;
    }
    if (((nan_mask & (1 << 1)) && isnan(packet.param2)) ||
        isinf(packet.param2)) {
        return MAV_MISSION_INVALID_PARAM2;
    }
    if (((nan_mask & (1 << 2)) && isnan(packet.param3)) ||
        isinf(packet.param3)) {
        return MAV_MISSION_INVALID_PARAM3;
    }
    if (((nan_mask & (1 << 3)) && isnan(packet.param4)) ||
        isinf(packet.param4)) {
        return MAV_MISSION_INVALID_PARAM4;
    }
    return MAV_MISSION_ACCEPTED;
}

// mavlink_int_to_mission_cmd - converts mavlink message to an AP_Mission::Mission_Command object which can be stored to eeprom
//  return MAV_MISSION_ACCEPTED on success, MAV_MISSION_RESULT error on failure
MAV_MISSION_RESULT AP_Mission::mavlink_int_to_mission_cmd(const mavlink_mission_item_int_t& packet, AP_Mission::Mission_Command& cmd)
{
    // command's position in mission list and mavlink id
    cmd.index = packet.seq;
    cmd.id = packet.command;
    cmd.content.location = {};

    MAV_MISSION_RESULT param_check = sanity_check_params(packet);
    if (param_check != MAV_MISSION_ACCEPTED) {
        return param_check;
    }

    // command specific conversions from mavlink packet to mission command
    switch (cmd.id) {

    case 0:
        // this is reserved for storing 16 bit command IDs
        return MAV_MISSION_INVALID;

    case MAV_CMD_NAV_WAYPOINT: {                        // MAV ID: 16
        /*
          the 15 byte limit means we can't fit both delay and radius
          in the cmd structure. When we expand the mission structure
          we can do this properly
         */
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        // acceptance radius in meters and pass by distance in meters
        uint16_t acp = packet.param2;           // param 2 is acceptance radius in meters is held in low p1
        uint16_t passby = packet.param3;        // param 3 is pass by distance in meters is held in high p1

        // limit to 255 so it does not wrap during the shift or mask operation
        passby = MIN(0xFF,passby);
        acp = MIN(0xFF,acp);

        cmd.p1 = (passby << 8) | (acp & 0x00FF);
#else
        // delay at waypoint in seconds (this is for copters???)
        cmd.p1 = packet.param1;
#endif
    }
    break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        cmd.p1 = fabsf(packet.param3);                  // store radius as 16bit since no other params are competing for space
        cmd.content.location.loiter_ccw = (packet.param3 < 0);    // -1 = counter clockwise, +1 = clockwise
        break;

    case MAV_CMD_NAV_LOITER_TURNS: {                    // MAV ID: 18
        // number of turns is stored in the lowest bits.  radii below
        // 255m are stored in the top 8 bits as an 8-bit integer.
        // Radii above 255m are stored divided by 10 and a bit set in
        // storage so that on retrieval they are multiplied by 10
        cmd.p1 = MIN(255, packet.param1); // store number of times to circle in low p1
        uint8_t radius_m;
        const float abs_radius = fabsf(packet.param3);
        if (abs_radius <= 255) {
            radius_m = abs_radius;
        } else {
            radius_m = MIN(255, abs_radius * 0.1);
            cmd.type_specific_bits = 1U << 0;
        }
        cmd.p1 |= (radius_m<<8);   // store radius in high byte of p1
        cmd.content.location.loiter_ccw = (packet.param3 < 0);
        cmd.content.location.loiter_xtrack = (packet.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
    }
    break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        cmd.p1 = packet.param1;                         // loiter time in seconds uses all 16 bits, 8bit seconds is too small. No room for radius.
        cmd.content.location.loiter_ccw = (packet.param3 < 0);
        cmd.content.location.loiter_xtrack = (packet.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        cmd.p1 = packet.param1;                         // abort target altitude(m)  (plane only)
        if (!isnan(packet.param4)) {
            cmd.content.location.loiter_ccw = is_negative(packet.param4); // yaw direction, (plane deepstall only)
        }
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        cmd.p1 = packet.param1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:           // MAV ID: 30
        cmd.p1 = packet.param1;                         // Climb/Descend
        // 0 = Neutral, cmd complete at +/- 5 of indicated alt.
        // 1 = Climb, cmd complete at or above indicated alt.
        // 2 = Descend, cmd complete at or below indicated alt.
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:                     // MAV ID: 31
        cmd.p1 = fabsf(packet.param2);                  // param2 is radius in meters
        cmd.content.location.loiter_ccw = (packet.param2 < 0);
        cmd.content.location.loiter_xtrack = (packet.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:                   // MAV ID: 82
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        return MAV_MISSION_UNSUPPORTED;
#else
        cmd.p1 = packet.param1;                         // delay at waypoint in seconds
        break;
#endif

    case MAV_CMD_NAV_GUIDED_ENABLE:                     // MAV ID: 92
        cmd.p1 = packet.param1;                         // on/off. >0.5 means "on", hand-over control to external controller
        break;

    case MAV_CMD_NAV_DELAY:                            // MAV ID: 93
        cmd.content.nav_delay.seconds = packet.param1; // delay in seconds
        cmd.content.nav_delay.hour_utc = packet.param2;// absolute time's hour (utc)
        cmd.content.nav_delay.min_utc = packet.param3;// absolute time's min (utc)
        cmd.content.nav_delay.sec_utc = packet.param4; // absolute time's second (utc)
        break;

    case MAV_CMD_CONDITION_DELAY:                       // MAV ID: 112
        cmd.content.delay.seconds = packet.param1;      // delay in seconds
        break;

    case MAV_CMD_CONDITION_DISTANCE:                    // MAV ID: 114
        cmd.content.distance.meters = packet.param1;    // distance in meters from next waypoint
        break;

    case MAV_CMD_CONDITION_YAW:                         // MAV ID: 115
        cmd.content.yaw.angle_deg = packet.param1;      // target angle in degrees
        cmd.content.yaw.turn_rate_dps = packet.param2;  // 0 = use default turn rate otherwise specific turn rate in deg/sec
        cmd.content.yaw.direction = packet.param3;      // -1 = ccw, +1 = cw
        cmd.content.yaw.relative_angle = packet.param4; // lng=0: absolute angle provided, lng=1: relative angle provided
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        cmd.content.jump.target = packet.param1;        // jump-to command number
        cmd.content.jump.num_times = packet.param2;     // repeat count
        break;

    case MAV_CMD_DO_CHANGE_SPEED:                       // MAV ID: 178
        cmd.content.speed.speed_type = packet.param1;   // 0 = airspeed, 1 = ground speed
        cmd.content.speed.target_ms = packet.param2;    // target speed in m/s
        cmd.content.speed.throttle_pct = packet.param3; // throttle as a percentage from 1 ~ 100%
        break;

    case MAV_CMD_DO_SET_HOME:
        cmd.p1 = packet.param1;                         // p1=0 means use current location, p=1 means use provided location
        break;

    case MAV_CMD_DO_SET_RELAY:                          // MAV ID: 181
        cmd.content.relay.num = packet.param1;          // relay number
        cmd.content.relay.state = packet.param2;        // 0:off, 1:on
        break;

    case MAV_CMD_DO_REPEAT_RELAY:                       // MAV ID: 182
        cmd.content.repeat_relay.num = packet.param1;           // relay number
        cmd.content.repeat_relay.repeat_count = packet.param2;  // count
        cmd.content.repeat_relay.cycle_time = packet.param3;    // time converted from seconds to milliseconds
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        cmd.content.servo.channel = packet.param1;      // channel
        cmd.content.servo.pwm = packet.param2;          // PWM
        break;

    case MAV_CMD_DO_REPEAT_SERVO:                       // MAV ID: 184
        cmd.content.repeat_servo.channel = packet.param1;      // channel
        cmd.content.repeat_servo.pwm = packet.param2;          // PWM
        cmd.content.repeat_servo.repeat_count = packet.param3; // count
        cmd.content.repeat_servo.cycle_time = packet.param4;   // time in seconds
        break;

    case MAV_CMD_DO_LAND_START:                         // MAV ID: 189
        break;

    case MAV_CMD_DO_GO_AROUND:                          // MAV ID: 191
        break;

    case MAV_CMD_DO_SET_ROI:                            // MAV ID: 201
        cmd.p1 = packet.param1;                         // 0 = no roi, 1 = next waypoint, 2 = waypoint number, 3 = fixed location, 4 = given target (not supported)
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // MAV ID: 202
        cmd.content.digicam_configure.shooting_mode = packet.param1;
        cmd.content.digicam_configure.shutter_speed = packet.param2;
        cmd.content.digicam_configure.aperture = packet.param3;
        cmd.content.digicam_configure.ISO = packet.param4;
        cmd.content.digicam_configure.exposure_type = packet.x;
        cmd.content.digicam_configure.cmd_id = packet.y;
        cmd.content.digicam_configure.engine_cutoff_time = packet.z;
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // MAV ID: 203
        cmd.content.digicam_control.session = packet.param1;
        cmd.content.digicam_control.zoom_pos = packet.param2;
        cmd.content.digicam_control.zoom_step = packet.param3;
        cmd.content.digicam_control.focus_lock = packet.param4;
        cmd.content.digicam_control.shooting_cmd = packet.x;
        cmd.content.digicam_control.cmd_id = packet.y;
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // MAV ID: 205
        cmd.content.mount_control.pitch = packet.param1;
        cmd.content.mount_control.roll = packet.param2;
        cmd.content.mount_control.yaw = packet.param3;
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        cmd.content.cam_trigg_dist.meters = packet.param1;  // distance between camera shots in meters
        cmd.content.cam_trigg_dist.trigger = packet.param3; // when enabled, camera triggers once immediately
        break;

    case MAV_CMD_DO_FENCE_ENABLE:                       // MAV ID: 207
        cmd.p1 = packet.param1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_AUX_FUNCTION:
        cmd.content.auxfunction.function = packet.param1;
        cmd.content.auxfunction.switchpos = packet.param2;
        break;

    case MAV_CMD_DO_PARACHUTE:                         // MAV ID: 208
        cmd.p1 = packet.param1;                        // action 0=disable, 1=enable, 2=release.  See PARACHUTE_ACTION enum
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:                    // MAV ID: 210
        cmd.p1 = packet.param1;                         // normal=0 inverted=1
        break;

    case MAV_CMD_DO_GRIPPER:                            // MAV ID: 211
        cmd.content.gripper.num = packet.param1;        // gripper number
        cmd.content.gripper.action = packet.param2;     // action 0=release, 1=grab.  See GRIPPER_ACTION enum
        break;

    case MAV_CMD_DO_GUIDED_LIMITS:                      // MAV ID: 222
        cmd.p1 = packet.param1;                         // max time in seconds the external controller will be allowed to control the vehicle
        cmd.content.guided_limits.alt_min = packet.param2;  // min alt below which the command will be aborted.  0 for no lower alt limit
        cmd.content.guided_limits.alt_max = packet.param3;  // max alt above which the command will be aborted.  0 for no upper alt limit
        cmd.content.guided_limits.horiz_max = packet.param4;// max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:                    // MAV ID: 211
        cmd.p1 = packet.param1;                         // disable=0 enable=1
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:                     // MAV ID: 83
        cmd.content.altitude_wait.altitude = packet.param1;
        cmd.content.altitude_wait.descent_rate = packet.param2;
        cmd.content.altitude_wait.wiggle_time = packet.param3;
        break;

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        break;

    case MAV_CMD_NAV_VTOL_LAND:
        cmd.p1 = (NAV_VTOL_LAND_OPTIONS)packet.param1;
        break;

    case MAV_CMD_DO_VTOL_TRANSITION:
        cmd.content.do_vtol_transition.target_state = packet.param1;
        break;

    case MAV_CMD_DO_SET_REVERSE:
        cmd.p1 = packet.param1; // 0 = forward, 1 = reverse
        break;

    case MAV_CMD_DO_ENGINE_CONTROL:
        cmd.content.do_engine_control.start_control = (packet.param1>0);
        cmd.content.do_engine_control.cold_start = (packet.param2>0);
        cmd.content.do_engine_control.height_delay_cm = packet.param3*100;
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:
        cmd.p1 = packet.param1*100; // copy max-descend parameter (m->cm)
        break;

    case MAV_CMD_NAV_SET_YAW_SPEED:
        cmd.content.set_yaw_speed.angle_deg = packet.param1;        // target angle in degrees
        cmd.content.set_yaw_speed.speed = packet.param2;            // speed in meters/second
        cmd.content.set_yaw_speed.relative_angle = packet.param3;   // 0 = absolute angle, 1 = relative angle
        break;

    case MAV_CMD_DO_WINCH:                              // MAV ID: 42600
        cmd.content.winch.num = packet.param1;          // winch number
        cmd.content.winch.action = packet.param2;       // action (0 = relax, 1 = length control, 2 = rate control).  See WINCH_ACTION enum
        cmd.content.winch.release_length = packet.param3;   // cable distance to unwind in meters, negative numbers to wind in cable
        cmd.content.winch.release_rate = packet.param4; // release rate in meters/second
        break;

    case MAV_CMD_DO_SET_RESUME_REPEAT_DIST:
        cmd.p1 = packet.param1; // Resume repeat distance (m)
        break;

    case MAV_CMD_DO_SPRAYER:
        cmd.p1 = packet.param1;                        // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_SEND_SCRIPT_MESSAGE:
        cmd.p1 = packet.param1;
        cmd.content.scripting.p1 = packet.param2;
        cmd.content.scripting.p2 = packet.param3;
        cmd.content.scripting.p3 = packet.param4;
        break;

    case MAV_CMD_NAV_SCRIPT_TIME:
        cmd.content.nav_script_time.command = packet.param1;
        cmd.content.nav_script_time.timeout_s = packet.param2;
        cmd.content.nav_script_time.arg1 = packet.param3;
        cmd.content.nav_script_time.arg2 = packet.param4;
        break;

    case MAV_CMD_NAV_ATTITUDE_TIME:
        cmd.content.nav_attitude_time.time_sec = constrain_float(packet.param1, 0, UINT16_MAX);
        cmd.content.nav_attitude_time.roll_deg = (fabsf(packet.param2) <= 180) ? packet.param2 : 0;
        cmd.content.nav_attitude_time.pitch_deg = (fabsf(packet.param3) <= 90) ? packet.param3 : 0;
        cmd.content.nav_attitude_time.yaw_deg = ((packet.param4 >= -180) && (packet.param4 <= 360)) ? packet.param4 : 0;
        cmd.content.nav_attitude_time.climb_rate = packet.x;
        break;

    case MAV_CMD_DO_PAUSE_CONTINUE:
        cmd.p1 = packet.param1;
        break;

    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        cmd.content.gimbal_manager_pitchyaw.pitch_angle_deg = packet.param1;
        cmd.content.gimbal_manager_pitchyaw.yaw_angle_deg = packet.param2;
        cmd.content.gimbal_manager_pitchyaw.pitch_rate_degs = packet.param3;
        cmd.content.gimbal_manager_pitchyaw.yaw_rate_degs = packet.param4;
        cmd.content.gimbal_manager_pitchyaw.flags = packet.x;
        cmd.content.gimbal_manager_pitchyaw.gimbal_id = packet.z;
        break;
    case MAV_CMD_ABZ_SPRAYER:
        cmd.p1=packet.param1;
        cmd.content.sprayer.coverage=packet.param2;
        cmd.content.sprayer.spacing=packet.param3;
        cmd.content.sprayer.spinnerpwm=(uint16_t)(packet.param4/100);
       /* gcs().send_text(MAV_SEVERITY_INFO,"content spinner:%d", cmd.content.sprayer.spinnerpwm);
        gcs().send_text(MAV_SEVERITY_INFO,"param spinner:%f", packet.param4 );

         gcs().send_text(MAV_SEVERITY_INFO,"content coverage:%f", cmd.content.sprayer.coverage);
        gcs().send_text(MAV_SEVERITY_INFO,"param coverage:%f", packet.param2 );

         gcs().send_text(MAV_SEVERITY_INFO,"content spacing:%f", cmd.content.sprayer.spacing);
        gcs().send_text(MAV_SEVERITY_INFO,"param coverage:%f", packet.param3 );*/
       
        break;
    default:
        // unrecognised command
        return MAV_MISSION_UNSUPPORTED;
    }

    // copy location from mavlink to command
    if (stored_in_location(cmd.id)) {

        // sanity check location
        if (!check_lat(packet.x)) {
            return MAV_MISSION_INVALID_PARAM5_X;
        }
        if (!check_lng(packet.y)) {
            return MAV_MISSION_INVALID_PARAM6_Y;
        }
        if (isnan(packet.z) || fabsf(packet.z) >= LOCATION_ALT_MAX_M) {
            return MAV_MISSION_INVALID_PARAM7;
        }

        cmd.content.location.lat = packet.x;
        cmd.content.location.lng = packet.y;

        cmd.content.location.alt = packet.z * 100.0f;       // convert packet's alt (m) to cmd alt (cm)

        switch (packet.frame) {

        case MAV_FRAME_MISSION:
        case MAV_FRAME_GLOBAL:
        case MAV_FRAME_GLOBAL_INT:
            cmd.content.location.relative_alt = 0;
            break;

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            cmd.content.location.relative_alt = 1;
            break;

#if AP_TERRAIN_AVAILABLE
        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
        case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
            // we mark it as a relative altitude, as it doesn't have
            // home alt added
            cmd.content.location.relative_alt = 1;
            // mark altitude as above terrain, not above home
            cmd.content.location.terrain_alt = 1;
            break;
#endif

        default:
            return MAV_MISSION_UNSUPPORTED_FRAME;
        }
    }

    // if we got this far then it must have been successful
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT AP_Mission::convert_MISSION_ITEM_to_MISSION_ITEM_INT(const mavlink_mission_item_t &packet,
        mavlink_mission_item_int_t &mav_cmd)
{
    // TODO: rename mav_cmd to mission_item_int
    // TODO: rename packet to mission_item
    mav_cmd.param1 = packet.param1;
    mav_cmd.param2 = packet.param2;
    mav_cmd.param3 = packet.param3;
    mav_cmd.param4 = packet.param4;
    mav_cmd.z = packet.z;
    mav_cmd.seq = packet.seq;
    mav_cmd.command = packet.command;
    mav_cmd.target_system = packet.target_system;
    mav_cmd.target_component = packet.target_component;
    mav_cmd.frame = packet.frame;
    mav_cmd.current = packet.current;
    mav_cmd.autocontinue = packet.autocontinue;
    mav_cmd.mission_type = packet.mission_type;

    /*
      the strategy for handling both MISSION_ITEM and MISSION_ITEM_INT
      is to pass the lat/lng in MISSION_ITEM_INT straight through, and
      for MISSION_ITEM multiply by 1e7 here. We need an exception for
      any commands which use the x and y fields not as
      latitude/longitude.
     */
    switch (packet.command) {
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_NAV_ATTITUDE_TIME:
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        mav_cmd.x = packet.x;
        mav_cmd.y = packet.y;
        break;

    default:
        // all other commands use x and y as lat/lon. We need to
        // multiply by 1e7 to convert to int32_t
        if (!check_lat(packet.x)) {
            return MAV_MISSION_INVALID_PARAM5_X;
        }
        if (!check_lng(packet.y)) {
            return MAV_MISSION_INVALID_PARAM6_Y;
        }
        mav_cmd.x = packet.x * 1.0e7f;
        mav_cmd.y = packet.y * 1.0e7f;
        break;
    }

    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT AP_Mission::convert_MISSION_ITEM_INT_to_MISSION_ITEM(const mavlink_mission_item_int_t &item_int,
        mavlink_mission_item_t &item)
{
    item.param1 = item_int.param1;
    item.param2 = item_int.param2;
    item.param3 = item_int.param3;
    item.param4 = item_int.param4;
    item.z = item_int.z;
    item.seq = item_int.seq;
    item.command = item_int.command;
    item.target_system = item_int.target_system;
    item.target_component = item_int.target_component;
    item.frame = item_int.frame;
    item.current = item_int.current;
    item.autocontinue = item_int.autocontinue;
    item.mission_type = item_int.mission_type;

    switch (item_int.command) {
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_NAV_ATTITUDE_TIME:
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        item.x = item_int.x;
        item.y = item_int.y;
        break;

    default:
        // all other commands use x and y as lat/lon. We need to
        // multiply by 1e-7 to convert to float
        item.x = item_int.x * 1.0e-7f;
        item.y = item_int.y * 1.0e-7f;
        if (!check_lat(item.x)) {
            return MAV_MISSION_INVALID_PARAM5_X;
        }
        if (!check_lng(item.y)) {
            return MAV_MISSION_INVALID_PARAM6_Y;
        }
        break;
    }

    return MAV_MISSION_ACCEPTED;
}

// mavlink_cmd_long_to_mission_cmd - converts a mavlink cmd long to an AP_Mission::Mission_Command object which can be stored to eeprom
// return MAV_MISSION_ACCEPTED on success, MAV_MISSION_RESULT error on failure
MAV_MISSION_RESULT AP_Mission::mavlink_cmd_long_to_mission_cmd(const mavlink_command_long_t& packet, AP_Mission::Mission_Command& cmd)
{
    mavlink_mission_item_int_t miss_item = {0};

    miss_item.param1 = packet.param1;
    miss_item.param2 = packet.param2;
    miss_item.param3 = packet.param3;
    miss_item.param4 = packet.param4;

    miss_item.command = packet.command;
    miss_item.target_system = packet.target_system;
    miss_item.target_component = packet.target_component;

    return mavlink_int_to_mission_cmd(miss_item, cmd);
}

// mission_cmd_to_mavlink_int - converts an AP_Mission::Mission_Command object to a mavlink message which can be sent to the GCS
//  return true on success, false on failure
//  NOTE: callers to this method current fill parts of "packet" in before calling this method, so do NOT attempt to zero the entire packet in here
/**
 * @brief This is a basic function to handel mission items
 * This is a basic function. We adeed Sparyer command
*/
bool AP_Mission::mission_cmd_to_mavlink_int(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_int_t& packet)
{
    // command's position in mission list and mavlink id
    packet.seq = cmd.index;
    packet.command = cmd.id;

    // set defaults
    packet.current = 0;     // 1 if we are passing back the mission command that is currently being executed
    packet.param1 = 0;
    packet.param2 = 0;
    packet.param3 = 0;
    packet.param4 = 0;
    packet.frame = 0;
    packet.autocontinue = 1;

    // command specific conversions from mission command to mavlink packet
    switch (cmd.id) {
    case 0:
        // this is reserved for 16 bit command IDs
        return false;

    case MAV_CMD_NAV_WAYPOINT:                          // MAV ID: 16
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        // acceptance radius in meters

        packet.param2 = LOWBYTE(cmd.p1);        // param 2 is acceptance radius in meters is held in low p1
        packet.param3 = HIGHBYTE(cmd.p1);       // param 3 is pass by distance in meters is held in high p1
#else
        // delay at waypoint in seconds
        packet.param1 = cmd.p1;
#endif
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        packet.param3 = (float)cmd.p1;
        if (cmd.content.location.loiter_ccw) {
            packet.param3 *= -1;
        }
        break;

    case MAV_CMD_NAV_LOITER_TURNS:                      // MAV ID: 18
        packet.param1 = LOWBYTE(cmd.p1);                // number of times to circle is held in low byte of p1
        packet.param3 = HIGHBYTE(cmd.p1);               // radius is held in high byte of p1
        if (cmd.content.location.loiter_ccw) {
            packet.param3 = -packet.param3;
        }
        if (cmd.type_specific_bits & (1U<<0)) {
            packet.param3 *= 10;
        }
        packet.param4 = cmd.content.location.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        packet.param1 = cmd.p1;                         // loiter time in seconds
        if (cmd.content.location.loiter_ccw) {
            packet.param3 = -1;
        } else {
            packet.param3 = 1;
        }
        packet.param4 = cmd.content.location.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        packet.param1 = cmd.p1;                        // abort target altitude(m)  (plane only)
        packet.param4 = cmd.content.location.loiter_ccw ? -1 : 1; // yaw direction, (plane deepstall only)
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        packet.param1 = cmd.p1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:           // MAV ID: 30
        packet.param1 = cmd.p1;                         // Climb/Descend
        // 0 = Neutral, cmd complete at +/- 5 of indicated alt.
        // 1 = Climb, cmd complete at or above indicated alt.
        // 2 = Descend, cmd complete at or below indicated alt.
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:                     // MAV ID: 31
        packet.param2 = cmd.p1;                        // loiter radius(m)
        if (cmd.content.location.loiter_ccw) {
            packet.param2 = -packet.param2;
        }
        packet.param4 = cmd.content.location.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:                   // MAV ID: 82
        packet.param1 = cmd.p1;                         // delay at waypoint in seconds
        break;

    case MAV_CMD_NAV_GUIDED_ENABLE:                     // MAV ID: 92
        packet.param1 = cmd.p1;                         // on/off. >0.5 means "on", hand-over control to external controller
        break;

    case MAV_CMD_NAV_DELAY:                            // MAV ID: 93
        packet.param1 = cmd.content.nav_delay.seconds; // delay in seconds
        packet.param2 = cmd.content.nav_delay.hour_utc; // absolute time's day of week (utc)
        packet.param3 = cmd.content.nav_delay.min_utc; // absolute time's hour (utc)
        packet.param4 = cmd.content.nav_delay.sec_utc; // absolute time's min (utc)
        break;

    case MAV_CMD_CONDITION_DELAY:                       // MAV ID: 112
        packet.param1 = cmd.content.delay.seconds;      // delay in seconds
        break;

    case MAV_CMD_CONDITION_DISTANCE:                    // MAV ID: 114
        packet.param1 = cmd.content.distance.meters;    // distance in meters from next waypoint
        break;

    case MAV_CMD_CONDITION_YAW:                         // MAV ID: 115
        packet.param1 = cmd.content.yaw.angle_deg;      // target angle in degrees
        packet.param2 = cmd.content.yaw.turn_rate_dps;  // 0 = use default turn rate otherwise specific turn rate in deg/sec
        packet.param3 = cmd.content.yaw.direction;      // -1 = ccw, +1 = cw
        packet.param4 = cmd.content.yaw.relative_angle; // 0 = absolute angle provided, 1 = relative angle provided
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        packet.param1 = cmd.content.jump.target;        // jump-to command number
        packet.param2 = cmd.content.jump.num_times;     // repeat count
        break;

    case MAV_CMD_DO_CHANGE_SPEED:                       // MAV ID: 178
        packet.param1 = cmd.content.speed.speed_type;   // 0 = airspeed, 1 = ground speed
        packet.param2 = cmd.content.speed.target_ms;    // speed in m/s
        packet.param3 = cmd.content.speed.throttle_pct; // throttle as a percentage from 1 ~ 100%
        break;

    case MAV_CMD_DO_SET_HOME:                           // MAV ID: 179
        packet.param1 = cmd.p1;                         // p1=0 means use current location, p=1 means use provided location
        break;

    case MAV_CMD_DO_SET_RELAY:                          // MAV ID: 181
        packet.param1 = cmd.content.relay.num;          // relay number
        packet.param2 = cmd.content.relay.state;        // 0:off, 1:on
        break;

    case MAV_CMD_DO_REPEAT_RELAY:                       // MAV ID: 182
        packet.param1 = cmd.content.repeat_relay.num;           // relay number
        packet.param2 = cmd.content.repeat_relay.repeat_count;  // count
        packet.param3 = cmd.content.repeat_relay.cycle_time;    // time in seconds
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        packet.param1 = cmd.content.servo.channel;      // channel
        packet.param2 = cmd.content.servo.pwm;          // PWM
        break;

    case MAV_CMD_DO_REPEAT_SERVO:                       // MAV ID: 184
        packet.param1 = cmd.content.repeat_servo.channel;       // channel
        packet.param2 = cmd.content.repeat_servo.pwm;           // PWM
        packet.param3 = cmd.content.repeat_servo.repeat_count;  // count
        packet.param4 = cmd.content.repeat_servo.cycle_time;    // time in milliseconds converted to seconds
        break;

    case MAV_CMD_DO_LAND_START:                         // MAV ID: 189
        break;

    case MAV_CMD_DO_GO_AROUND:                          // MAV ID: 191
        break;

    case MAV_CMD_DO_SET_ROI:                            // MAV ID: 201
        packet.param1 = cmd.p1;                         // 0 = no roi, 1 = next waypoint, 2 = waypoint number, 3 = fixed location, 4 = given target (not supported)
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // MAV ID: 202
        packet.param1 = cmd.content.digicam_configure.shooting_mode;
        packet.param2 = cmd.content.digicam_configure.shutter_speed;
        packet.param3 = cmd.content.digicam_configure.aperture;
        packet.param4 = cmd.content.digicam_configure.ISO;
        packet.x = cmd.content.digicam_configure.exposure_type;
        packet.y = cmd.content.digicam_configure.cmd_id;
        packet.z = cmd.content.digicam_configure.engine_cutoff_time;
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // MAV ID: 203
        packet.param1 = cmd.content.digicam_control.session;
        packet.param2 = cmd.content.digicam_control.zoom_pos;
        packet.param3 = cmd.content.digicam_control.zoom_step;
        packet.param4 = cmd.content.digicam_control.focus_lock;
        packet.x = cmd.content.digicam_control.shooting_cmd;
        packet.y = cmd.content.digicam_control.cmd_id;
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // MAV ID: 205
        packet.param1 = cmd.content.mount_control.pitch;
        packet.param2 = cmd.content.mount_control.roll;
        packet.param3 = cmd.content.mount_control.yaw;
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        packet.param1 = cmd.content.cam_trigg_dist.meters;  // distance between camera shots in meters
        packet.param3 = cmd.content.cam_trigg_dist.trigger; // when enabled, camera triggers once immediately
        break;

    case MAV_CMD_DO_FENCE_ENABLE:                       // MAV ID: 207
        packet.param1 = cmd.p1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_PARACHUTE:                          // MAV ID: 208
        packet.param1 = cmd.p1;                         // action 0=disable, 1=enable, 2=release.  See PARACHUTE_ACTION enum
        break;

    case MAV_CMD_DO_SPRAYER:
        packet.param1 = cmd.p1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_AUX_FUNCTION:
        packet.param1 = cmd.content.auxfunction.function;
        packet.param2 = cmd.content.auxfunction.switchpos;
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:                    // MAV ID: 210
        packet.param1 = cmd.p1;                         // normal=0 inverted=1
        break;

    case MAV_CMD_DO_GRIPPER:                            // MAV ID: 211
        packet.param1 = cmd.content.gripper.num;        // gripper number
        packet.param2 = cmd.content.gripper.action;     // action 0=release, 1=grab.  See GRIPPER_ACTION enum
        break;

    case MAV_CMD_DO_GUIDED_LIMITS:                      // MAV ID: 222
        packet.param1 = cmd.p1;                         // max time in seconds the external controller will be allowed to control the vehicle
        packet.param2 = cmd.content.guided_limits.alt_min;  // min alt below which the command will be aborted.  0 for no lower alt limit
        packet.param3 = cmd.content.guided_limits.alt_max;  // max alt above which the command will be aborted.  0 for no upper alt limit
        packet.param4 = cmd.content.guided_limits.horiz_max;// max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        packet.param1 = cmd.p1;                         // disable=0 enable=1
        break;

    case MAV_CMD_DO_SET_REVERSE:
        packet.param1 = cmd.p1;   // 0 = forward, 1 = reverse
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:                     // MAV ID: 83
        packet.param1 = cmd.content.altitude_wait.altitude;
        packet.param2 = cmd.content.altitude_wait.descent_rate;
        packet.param3 = cmd.content.altitude_wait.wiggle_time;
        break;

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        break;

    case MAV_CMD_NAV_VTOL_LAND:
        packet.param1 = cmd.p1;
        break;

    case MAV_CMD_DO_VTOL_TRANSITION:
        packet.param1 = cmd.content.do_vtol_transition.target_state;
        break;

    case MAV_CMD_DO_ENGINE_CONTROL:
        packet.param1 = cmd.content.do_engine_control.start_control?1:0;
        packet.param2 = cmd.content.do_engine_control.cold_start?1:0;
        packet.param3 = cmd.content.do_engine_control.height_delay_cm*0.01f;
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:
        packet.param1 = cmd.p1/100.0f; // copy max-descend parameter (cm->m)
        break;

    case MAV_CMD_NAV_SET_YAW_SPEED:
        packet.param1 = cmd.content.set_yaw_speed.angle_deg;        // target angle in degrees
        packet.param2 = cmd.content.set_yaw_speed.speed;            // speed in meters/second
        packet.param3 = cmd.content.set_yaw_speed.relative_angle;   // 0 = absolute angle, 1 = relative angle
        break;

    case MAV_CMD_DO_WINCH:
        packet.param1 = cmd.content.winch.num;              // winch number
        packet.param2 = cmd.content.winch.action;           // action (0 = relax, 1 = length control, 2 = rate control).  See WINCH_ACTION enum
        packet.param3 = cmd.content.winch.release_length;   // cable distance to unwind in meters, negative numbers to wind in cable
        packet.param4 = cmd.content.winch.release_rate;     // release rate in meters/second
        break;

    case MAV_CMD_DO_SET_RESUME_REPEAT_DIST:
        packet.param1 = cmd.p1; // Resume repeat distance (m)
        break;

    case MAV_CMD_DO_SEND_SCRIPT_MESSAGE:
        packet.param1 = cmd.p1;
        packet.param2 = cmd.content.scripting.p1;
        packet.param3 = cmd.content.scripting.p2;
        packet.param4 = cmd.content.scripting.p3;
        break;

    case MAV_CMD_NAV_SCRIPT_TIME:
        packet.param1 = cmd.content.nav_script_time.command;
        packet.param2 = cmd.content.nav_script_time.timeout_s;
        packet.param3 = cmd.content.nav_script_time.arg1;
        packet.param4 = cmd.content.nav_script_time.arg2;
        break;

    case MAV_CMD_NAV_ATTITUDE_TIME:
        packet.param1 = cmd.content.nav_attitude_time.time_sec;
        packet.param2 = cmd.content.nav_attitude_time.roll_deg;
        packet.param3 = cmd.content.nav_attitude_time.pitch_deg;
        packet.param4 = cmd.content.nav_attitude_time.yaw_deg;
        packet.x = cmd.content.nav_attitude_time.climb_rate;
        break;

    case MAV_CMD_DO_PAUSE_CONTINUE:
        packet.param1 = cmd.p1;
        break;

    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        packet.param1 = cmd.content.gimbal_manager_pitchyaw.pitch_angle_deg;
        packet.param2 = cmd.content.gimbal_manager_pitchyaw.yaw_angle_deg;
        packet.param3 = cmd.content.gimbal_manager_pitchyaw.pitch_rate_degs;
        packet.param4 = cmd.content.gimbal_manager_pitchyaw.yaw_rate_degs;
        packet.x = cmd.content.gimbal_manager_pitchyaw.flags;
        packet.z = cmd.content.gimbal_manager_pitchyaw.gimbal_id;
        break;
    case MAV_CMD_ABZ_SPRAYER:
        packet.param1=cmd.p1;
        packet.param4=cmd.content.sprayer.spinnerpwm*100; /**0 disable 1 enable*/
        packet.param2=cmd.content.sprayer.coverage;
        packet.param3=cmd.content.sprayer.spacing;
       // packet.param4=cmd.p1;
        break;
    default:
        // unrecognised command
        return false;
    }

    // copy location from mavlink to command
    if (stored_in_location(cmd.id)) {
        packet.x = cmd.content.location.lat;
        packet.y = cmd.content.location.lng;

        packet.z = cmd.content.location.alt * 0.01f;   // cmd alt in cm to m
        if (cmd.content.location.relative_alt) {
            packet.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        } else {
            packet.frame = MAV_FRAME_GLOBAL;
        }
#if AP_TERRAIN_AVAILABLE
        if (cmd.content.location.terrain_alt) {
            // this is a above-terrain altitude
            if (!cmd.content.location.relative_alt) {
                // refuse to return non-relative terrain mission
                // items. Internally we do have these, and they
                // have home.alt added, but we should never be
                // returning them to the GCS, as the GCS doesn't know
                // our home.alt, so it would have no way to properly
                // interpret it
                return false;
            }
            packet.z = cmd.content.location.alt * 0.01f;
            packet.frame = MAV_FRAME_GLOBAL_TERRAIN_ALT;
        }
#else
        // don't ever return terrain mission items if no terrain support
        if (cmd.content.location.terrain_alt) {
            return false;
        }
#endif
    }

    // if we got this far then it must have been successful
    return true;
}

///
/// private methods
///

/// complete - mission is marked complete and clean-up performed including calling the mission_complete_fn
/**
 * @brief Runs whe mission is ended
 * This is a basic function. We set the flag, and send the las progress message.
 * Reads the mission item from the given index and puts it to the given variable.
 * Also we turn off spraying and reset the sprayin logic to basic
*/
void AP_Mission::complete()
{
    isRTMDone = false;
    // flag mission as complete
    _flags.state = MISSION_COMPLETE;
    //_flags.state = MISSION_STOPPED;
    gcs().send_message(MSG_ABZ_IS_MISSION_IN_PROGRESS);

    AP_Mission *mission = AP_Mission::get_singleton();
    if (mission == nullptr) {
        return;
    }
   // gcs().send_text(MAV_SEVERITY_INFO,"state complete: %d",mission->state());
    _flags.in_landing_sequence = false;
 ABZ_Sprayer *sprayer = ABZ::get_singleton();
    if (sprayer == nullptr) {
        return;
    }
    sprayer->SetSpraying_speed_adaptive(false);
    sprayer->setSpinnerPwm(1050);
    sprayer->T_sprayed.set_and_save(0);
    gcs().send_message(MSG_ABZ_IS_SPRAYING);
    gcs().send_text(MAV_SEVERITY_ALERT, "Spraying: OFF");
    gcs().send_text(MAV_SEVERITY_INFO, "Spraying: OFF");
    //gcs().send_message(MSG_ABZ_IS_RETURN_POINT);

   
    // callback to main program's mission complete function
    _mission_complete_fn();
}

/// advance_current_nav_cmd - moves current nav command forward
///     do command will also be loaded
///     accounts for do-jump commands
//      returns true if command is advanced, false if failed (i.e. mission completed)
bool AP_Mission::advance_current_nav_cmd(uint16_t starting_index)
{
    // exit immediately if we're not running
    if (_flags.state != MISSION_RUNNING) {
        return false;
    }

    // exit immediately if current nav command has not completed
    if (_flags.nav_cmd_loaded) {
        return false;
    }

    // stop the current running do command
    _do_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.do_cmd_loaded = false;
    _flags.do_cmd_all_done = false;

    // get starting point for search
    uint16_t cmd_index = starting_index > 0 ? starting_index - 1 : _nav_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        // start from beginning of the mission command list
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    } else {
        // start from one position past the current nav command
        cmd_index++;
    }

    // avoid endless loops
    uint8_t max_loops = 255;

    // search until we find next nav command or reach end of command list
    while (!_flags.nav_cmd_loaded && max_loops-- > 0) {
        // get next command
        Mission_Command cmd;
        if (!get_next_cmd(cmd_index, cmd, true)) {
            return false;
        }

        // check if navigation or "do" command
        if (is_nav_cmd(cmd)) {
            // save previous nav command index
            _prev_nav_cmd_id = _nav_cmd.id;
            _prev_nav_cmd_index = _nav_cmd.index;
            _prev_nav_cmd=_nav_cmd;
            
            ABZ_Sprayer * sprayer = ABZ::get_singleton();
            if(sprayer==nullptr)
            {
                return false;
            }

            if(_nav_cmd.index == sprayer->returning_point){
                gcs().send_text(MAV_SEVERITY_ALERT, "Please refill");
                //gcs().send_message(MSG_ABZ_IS_RETURN_POINT);
            }
            // save separate previous nav command index if it contains lat,long,alt
            if (!(cmd.content.location.lat == 0 && cmd.content.location.lng == 0)) {
                _prev_nav_cmd_wp_index = _nav_cmd.index;
            }
            // set current navigation command and start it
            _nav_cmd = cmd;
            if (start_command(_nav_cmd)) {
                _flags.nav_cmd_loaded = true;
            }
            // save a loaded wp index in history array for when _repeat_dist is set via MAV_CMD_DO_SET_RESUME_REPEAT_DIST
            // and prevent history being re-written until vehicle returns to interrupted position
            if (_repeat_dist > 0 && !_flags.resuming_mission && _nav_cmd.index != AP_MISSION_CMD_INDEX_NONE && !(_nav_cmd.content.location.lat == 0 && _nav_cmd.content.location.lng == 0)) {
                // update mission history. last index position is always the most recent wp loaded.
                for (uint8_t i=0; i<AP_MISSION_MAX_WP_HISTORY-1; i++) {
                    _wp_index_history[i] = _wp_index_history[i+1];
                }
                _wp_index_history[AP_MISSION_MAX_WP_HISTORY-1] = _nav_cmd.index;
            }
            // check if the vehicle is resuming and has returned to where it was interrupted
            if (_flags.resuming_mission && _nav_cmd.index == _wp_index_history[AP_MISSION_MAX_WP_HISTORY-1]) {
                // vehicle has resumed previous position
                gcs().send_text(MAV_SEVERITY_INFO, "Mission: Returned to interrupted WP");
                _flags.resuming_mission = false;
            }

        } else {
            // set current do command and start it (if not already set)
            if (!_flags.do_cmd_loaded) {
                _do_cmd = cmd;
                _flags.do_cmd_loaded = true;
                start_command(_do_cmd);
            }
        }
        // move onto next command
        cmd_index = cmd.index+1;
    }

    if (max_loops == 0) {
        // infinite loop.  This can happen if there's a loop involving
        // only nav commands (no DO commands) which won't start()
        return false;
    }

    // if we have not found a do command then set flag to show there are no do-commands to be run before nav command completes
    if (!_flags.do_cmd_loaded) {
        _flags.do_cmd_all_done = true;
    }

    // if we got this far we must have successfully advanced the nav command
    return true;
}

/// advance_current_do_cmd - moves current do command forward
///     accounts for do-jump commands
void AP_Mission::advance_current_do_cmd()
{
    // exit immediately if we're not running or we've completed all possible "do" commands
    if (_flags.state != MISSION_RUNNING || _flags.do_cmd_all_done) {
        return;
    }

    // get starting point for search
    uint16_t cmd_index = _do_cmd.index;
    if (cmd_index == AP_MISSION_CMD_INDEX_NONE) {
        cmd_index = AP_MISSION_FIRST_REAL_COMMAND;
    } else {
        // start from one position past the current do command
        cmd_index = _do_cmd.index + 1;
    }

    // find next do command
    Mission_Command cmd;
    if (!get_next_do_cmd(cmd_index, cmd)) {
        // set flag to stop unnecessarily searching for do commands
        _flags.do_cmd_all_done = true;
        return;
    }

    // set current do command and start it
    _do_cmd = cmd;
    _flags.do_cmd_loaded = true;
    start_command(_do_cmd);
}

/// get_next_cmd - gets next command found at or after start_index
///     returns true if found, false if not found (i.e. mission complete)
///     accounts for do_jump commands
///     increment_jump_num_times_if_found should be set to true if advancing the active navigation command
bool AP_Mission::get_next_cmd(uint16_t start_index, Mission_Command& cmd, bool increment_jump_num_times_if_found, bool send_gcs_msg)
{
    uint16_t cmd_index = start_index;
    Mission_Command temp_cmd;
    uint16_t jump_index = AP_MISSION_CMD_INDEX_NONE;

    // search until the end of the mission command list
    uint8_t max_loops = 64;
    while (cmd_index < (unsigned)_cmd_total) {
        // load the next command
        if (!read_cmd_from_storage(cmd_index, temp_cmd)) {
            // this should never happen because of check above but just in case
            return false;
        }

        // check for do-jump command
        if (temp_cmd.id == MAV_CMD_DO_JUMP) {

            if (max_loops-- == 0) {
                return false;
            }

            // check for invalid target
            if ((temp_cmd.content.jump.target >= (unsigned)_cmd_total) || (temp_cmd.content.jump.target == 0)) {
                // To-Do: log an error?
                return false;
            }

            // check for endless loops
            if (!increment_jump_num_times_if_found && jump_index == cmd_index) {
                // we have somehow reached this jump command twice and there is no chance it will complete
                // To-Do: log an error?
                return false;
            }

            // record this command so we can check for endless loops
            if (jump_index == AP_MISSION_CMD_INDEX_NONE) {
                jump_index = cmd_index;
            }

            // check if jump command is 'repeat forever'
            if (temp_cmd.content.jump.num_times == AP_MISSION_JUMP_REPEAT_FOREVER) {
                // continue searching from jump target
                cmd_index = temp_cmd.content.jump.target;
            } else {
                // get number of times jump command has already been run
                int16_t jump_times_run = get_jump_times_run(temp_cmd);
                if (jump_times_run < temp_cmd.content.jump.num_times) {
                    // update the record of the number of times run
                    if (increment_jump_num_times_if_found && !_flags.resuming_mission) {
                        increment_jump_times_run(temp_cmd, send_gcs_msg);
                    }
                    // continue searching from jump target
                    cmd_index = temp_cmd.content.jump.target;
                } else {
                    // jump has been run specified number of times so move search to next command in mission
                    cmd_index++;
                }
            }
        } else {
            // this is a non-jump command so return it
            cmd = temp_cmd;
            return true;
        }
    }

    // if we got this far we did not find a navigation command
    return false;
}

/// get_next_do_cmd - gets next "do" or "conditional" command after start_index
///     returns true if found, false if not found
///     stops and returns false if it hits another navigation command before it finds the first do or conditional command
///     accounts for do_jump commands but never increments the jump's num_times_run (advance_current_nav_cmd is responsible for this)
bool AP_Mission::get_next_do_cmd(uint16_t start_index, Mission_Command& cmd)
{
    Mission_Command temp_cmd;

    // check we have not passed the end of the mission list
    if (start_index >= (unsigned)_cmd_total) {
        return false;
    }

    // get next command
    if (!get_next_cmd(start_index, temp_cmd, false)) {
        // no more commands so return failure
        return false;
    } else if (is_nav_cmd(temp_cmd)) {
        // if it's a "navigation" command then return false because we do not progress past nav commands
        return false;
    } else {
        // this must be a "do" or "conditional" and is not a do-jump command so return it
        cmd = temp_cmd;
        return true;
    }
}

///
/// jump handling methods
///

// init_jump_tracking - initialise jump_tracking variables
void AP_Mission::init_jump_tracking()
{
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        _jump_tracking[i].index = AP_MISSION_CMD_INDEX_NONE;
        _jump_tracking[i].num_times_run = 0;
    }
}

/// get_jump_times_run - returns number of times the jump command has been run
int16_t AP_Mission::get_jump_times_run(const Mission_Command& cmd)
{
    // exit immediately if cmd is not a do-jump command or target is invalid
    if ((cmd.id != MAV_CMD_DO_JUMP) || (cmd.content.jump.target >= (unsigned)_cmd_total) || (cmd.content.jump.target == 0)) {
        // To-Do: log an error?
        return AP_MISSION_JUMP_TIMES_MAX;
    }

    // search through jump_tracking array for this cmd
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        if (_jump_tracking[i].index == cmd.index) {
            return _jump_tracking[i].num_times_run;
        } else if (_jump_tracking[i].index == AP_MISSION_CMD_INDEX_NONE) {
            // we've searched through all known jump commands and haven't found it so allocate new space in _jump_tracking array
            _jump_tracking[i].index = cmd.index;
            _jump_tracking[i].num_times_run = 0;
            return 0;
        }
    }

    // if we've gotten this far then the _jump_tracking array must be full
    // To-Do: log an error?
    return AP_MISSION_JUMP_TIMES_MAX;
}

/// increment_jump_times_run - increments the recorded number of times the jump command has been run
void AP_Mission::increment_jump_times_run(Mission_Command& cmd, bool send_gcs_msg)
{
    // exit immediately if cmd is not a do-jump command
    if (cmd.id != MAV_CMD_DO_JUMP) {
        // To-Do: log an error?
        return;
    }

    // search through jump_tracking array for this cmd
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        if (_jump_tracking[i].index == cmd.index) {
            _jump_tracking[i].num_times_run++;
            if (send_gcs_msg) {
                gcs().send_text(MAV_SEVERITY_INFO, "Mission: %u Jump %i/%i", _jump_tracking[i].index, _jump_tracking[i].num_times_run, cmd.content.jump.num_times);
            }
            return;
        } else if (_jump_tracking[i].index == AP_MISSION_CMD_INDEX_NONE) {
            // we've searched through all known jump commands and haven't found it so allocate new space in _jump_tracking array
            _jump_tracking[i].index = cmd.index;
            _jump_tracking[i].num_times_run = 1;
            return;
        }
    }

    // if we've gotten this far then the _jump_tracking array must be full
    // To-Do: log an error
    return;
}

// check_eeprom_version - checks version of missions stored in eeprom matches this library
// command list will be cleared if they do not match
void AP_Mission::check_eeprom_version()
{
    uint32_t eeprom_version = _storage.read_uint32(0);

    // if eeprom version does not match, clear the command list and update the eeprom version
    if (eeprom_version != AP_MISSION_EEPROM_VERSION) {
        if (clear()) {
            _storage.write_uint32(0, AP_MISSION_EEPROM_VERSION);
        }
    }
}

/*
  return total number of commands that can fit in storage space
 */
uint16_t AP_Mission::num_commands_max(void) const
{
    // -4 to remove space for eeprom version number
    return (_storage.size() - 4) / AP_MISSION_EEPROM_COMMAND_SIZE;
}

// find the nearest landing sequence starting point (DO_LAND_START) and
// return its index.  Returns 0 if no appropriate DO_LAND_START point can
// be found.
uint16_t AP_Mission::get_landing_sequence_start()
{
    struct Location current_loc;

    if (!AP::ahrs().get_location(current_loc)) {
        return 0;
    }

    uint16_t landing_start_index = 0;
    float min_distance = -1;

    // Go through mission looking for nearest landing start command
    for (uint16_t i = 1; i < num_commands(); i++) {
        Mission_Command tmp;
        if (!read_cmd_from_storage(i, tmp)) {
            continue;
        }
        if (tmp.id == MAV_CMD_DO_LAND_START) {
            if (!tmp.content.location.initialised() && !get_next_nav_cmd(i, tmp)) {
                // command does not have a valid location and cannot get next valid
                continue;
            }
            float tmp_distance = tmp.content.location.get_distance(current_loc);
            if (min_distance < 0 || tmp_distance < min_distance) {
                min_distance = tmp_distance;
                landing_start_index = i;
            }
        }
    }

    return landing_start_index;
}

/*
   find the nearest landing sequence starting point (DO_LAND_START) and
   switch to that mission item.  Returns false if no DO_LAND_START
   available.
 */
bool AP_Mission::jump_to_landing_sequence(void)
{
    uint16_t land_idx = get_landing_sequence_start();
    if (land_idx != 0 && set_current_cmd(land_idx)) {

        //if the mission has ended it has to be restarted
        if (state() == AP_Mission::MISSION_STOPPED) {
            resume();
        }

        gcs().send_text(MAV_SEVERITY_INFO, "Landing sequence start");
        _flags.in_landing_sequence = true;
        return true;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "Unable to start landing sequence");
    return false;
}

// jumps the mission to the closest landing abort that is planned, returns false if unable to find a valid abort
bool AP_Mission::jump_to_abort_landing_sequence(void)
{
    struct Location current_loc;

    uint16_t abort_index = 0;
    if (AP::ahrs().get_location(current_loc)) {
        float min_distance = FLT_MAX;

        for (uint16_t i = 1; i < num_commands(); i++) {
            Mission_Command tmp;
            if (!read_cmd_from_storage(i, tmp)) {
                continue;
            }
            if (tmp.id == MAV_CMD_DO_GO_AROUND) {
                float tmp_distance = tmp.content.location.get_distance(current_loc);
                if (tmp_distance < min_distance) {
                    min_distance = tmp_distance;
                    abort_index = i;
                }
            }
        }
    }

    if (abort_index != 0 && set_current_cmd(abort_index)) {

        //if the mission has ended it has to be restarted
        if (state() == AP_Mission::MISSION_STOPPED) {
            resume();
        }

        _flags.in_landing_sequence = false;

        gcs().send_text(MAV_SEVERITY_INFO, "Landing abort sequence start");
        return true;
    }

    gcs().send_text(MAV_SEVERITY_WARNING, "Unable to start find a landing abort sequence");
    return false;
}

// check which is the shortest route to landing an RTL via a DO_LAND_START or continuing on the current mission plan
bool AP_Mission::is_best_land_sequence(void)
{
    // check if there is even a running mission to interupt
    if (_flags.state != MISSION_RUNNING) {
        return false;
    }

    // check if aircraft has already jumped to a landing sequence
    if (_flags.in_landing_sequence) {
        return true;
    }

    // check if MIS_OPTIONS bit set to allow distance calculation to be done
    if (!(_options & AP_MISSION_MASK_DIST_TO_LAND_CALC)) {
        return false;
    }

    // The decision to allow a failsafe to interupt a potential landing approach
    // is a distance travelled minimization problem.  Look forward in
    // mission to evaluate the shortest remaining distance to land.

    // go through the mission for the nearest DO_LAND_START first as this is the most probable route
    // to a landing with the minimum number of WP.
    uint16_t do_land_start_index = get_landing_sequence_start();
    if (do_land_start_index == 0) {
        // then no DO_LAND_START commands are in mission and normal failsafe behaviour should be maintained
        return false;
    }

    // get our current location
    Location current_loc;
    if (!AP::ahrs().get_location(current_loc)) {
        // we don't know where we are!!
        return false;
    }

    // get distance to landing if travelled to nearest DO_LAND_START via RTL
    float dist_via_do_land;
    if (!distance_to_landing(do_land_start_index, dist_via_do_land, current_loc)) {
        // cant get a valid distance to landing
        return false;
    }

    // get distance to landing if continue along current mission path
    float dist_continue_to_land;
    if (!distance_to_landing(_nav_cmd.index, dist_continue_to_land, current_loc)) {
        // cant get a valid distance to landing
        return false;
    }

    // compare distances
    if (dist_via_do_land >= dist_continue_to_land) {
        // then the mission should carry on uninterrupted as that is the shorter distance
        gcs().send_text(MAV_SEVERITY_NOTICE, "Rejecting RTL: closer land if mis continued");
        return true;
    } else {
        // allow failsafes to interrupt the current mission
        return false;
    }
}

// Approximate the distance travelled to get to a landing.  DO_JUMP commands are observed in look forward.
bool AP_Mission::distance_to_landing(uint16_t index, float &tot_distance, Location prev_loc)
{
    Mission_Command temp_cmd;
    tot_distance = 0.0f;
    bool ret;

    // back up jump tracking to reset after distance calculation
    jump_tracking_struct _jump_tracking_backup[AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS];
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        _jump_tracking_backup[i] = _jump_tracking[i];
    }

    // run through remainder of mission to approximate a distance to landing
    for (uint8_t i=0; i<255; i++) {
        // search until the end of the mission command list
        for (uint16_t cmd_index = index; cmd_index < (unsigned)_cmd_total; cmd_index++) {
            // get next command
            if (!get_next_cmd(cmd_index, temp_cmd, true, false)) {
                // we got to the end of the mission
                ret = false;
                goto reset_do_jump_tracking;
            }
            if (temp_cmd.id == MAV_CMD_NAV_WAYPOINT || temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT || is_landing_type_cmd(temp_cmd.id)) {
                break;
            } else if (is_nav_cmd(temp_cmd) || temp_cmd.id == MAV_CMD_CONDITION_DELAY) {
                // if we receive a nav command that we dont handle then give up as cant measure the distance e.g. MAV_CMD_NAV_LOITER_UNLIM
                ret = false;
                goto reset_do_jump_tracking;
            }
        }
        index = temp_cmd.index+1;

        if (!(temp_cmd.content.location.lat == 0 && temp_cmd.content.location.lng == 0)) {
            // add distance to running total
            float disttemp = prev_loc.get_distance(temp_cmd.content.location);
            tot_distance = tot_distance + disttemp;

            // store wp location as previous
            prev_loc = temp_cmd.content.location;
        }

        if (is_landing_type_cmd(temp_cmd.id)) {
            // reached a landing!
            ret = true;
            goto reset_do_jump_tracking;
        }
    }
    // reached end of loop without getting to a landing
    ret = false;

reset_do_jump_tracking:
    for (uint8_t i=0; i<AP_MISSION_MAX_NUM_DO_JUMP_COMMANDS; i++) {
        _jump_tracking[i] = _jump_tracking_backup[i];
    }

    return ret;
}

// check if command is a landing type command.
bool AP_Mission::is_landing_type_cmd(uint16_t id) const
{
    switch (id) {
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_DO_PARACHUTE:
        return true;
    default:
        return false;
    }
}

// check if command is a takeoff type command.
bool AP_Mission::is_takeoff_type_cmd(uint16_t id) const
{
    switch (id) {
    case MAV_CMD_NAV_TAKEOFF:
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return true;
    default:
        return false;
    }
}

const char *AP_Mission::Mission_Command::type() const
{
    switch (id) {
    case MAV_CMD_NAV_WAYPOINT:
        return "WP";
    case MAV_CMD_NAV_SPLINE_WAYPOINT:
        return "SplineWP";
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return "RTL";
    case MAV_CMD_NAV_LOITER_UNLIM:
        return "LoitUnlim";
    case MAV_CMD_NAV_LOITER_TIME:
        return "LoitTime";
    case MAV_CMD_NAV_GUIDED_ENABLE:
        return "GuidedEnable";
    case MAV_CMD_NAV_LOITER_TURNS:
        return "LoitTurns";
    case MAV_CMD_NAV_LOITER_TO_ALT:
        return "LoitAltitude";
    case MAV_CMD_NAV_SET_YAW_SPEED:
        return "SetYawSpd";
    case MAV_CMD_CONDITION_DELAY:
        return "CondDelay";
    case MAV_CMD_CONDITION_DISTANCE:
        return "CondDist";
    case MAV_CMD_DO_CHANGE_SPEED:
        return "ChangeSpeed";
    case MAV_CMD_DO_SET_HOME:
        return "SetHome";
    case MAV_CMD_DO_SET_SERVO:
        return "SetServo";
    case MAV_CMD_DO_SET_RELAY:
        return "SetRelay";
    case MAV_CMD_DO_REPEAT_SERVO:
        return "RepeatServo";
    case MAV_CMD_DO_REPEAT_RELAY:
        return "RepeatRelay";
    case MAV_CMD_DO_CONTROL_VIDEO:
        return "CtrlVideo";
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
        return "DigiCamCfg";
    case MAV_CMD_DO_DIGICAM_CONTROL:
        return "DigiCamCtrl";
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        return "SetCamTrigDst";
    case MAV_CMD_DO_SET_ROI:
        return "SetROI";
    case MAV_CMD_DO_SET_REVERSE:
        return "SetReverse";
    case MAV_CMD_DO_SET_RESUME_REPEAT_DIST:
        return "SetRepeatDist";
    case MAV_CMD_DO_GUIDED_LIMITS:
        return "GuidedLimits";
    case MAV_CMD_NAV_TAKEOFF:
        return "Takeoff";
    case MAV_CMD_NAV_LAND:
        return "Land";
    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
        return "ContinueAndChangeAlt";
    case MAV_CMD_NAV_ALTITUDE_WAIT:
        return "AltitudeWait";
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return "VTOLTakeoff";
    case MAV_CMD_NAV_VTOL_LAND:
        return "VTOLLand";
    case MAV_CMD_DO_INVERTED_FLIGHT:
        return "InvertedFlight";
    case MAV_CMD_DO_FENCE_ENABLE:
        return "FenceEnable";
    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        return "AutoTuneEnable";
    case MAV_CMD_DO_VTOL_TRANSITION:
        return "VTOLTransition";
    case MAV_CMD_DO_ENGINE_CONTROL:
        return "EngineControl";
    case MAV_CMD_CONDITION_YAW:
        return "CondYaw";
    case MAV_CMD_DO_LAND_START:
        return "LandStart";
    case MAV_CMD_NAV_DELAY:
        return "Delay";
    case MAV_CMD_DO_GRIPPER:
        return "Gripper";
    case MAV_CMD_NAV_PAYLOAD_PLACE:
        return "PayloadPlace";
    case MAV_CMD_DO_PARACHUTE:
        return "Parachute";
    case MAV_CMD_DO_SPRAYER:
        return "Sprayer";
    case MAV_CMD_DO_AUX_FUNCTION:
        return "AuxFunction";
    case MAV_CMD_DO_MOUNT_CONTROL:
        return "MountControl";
    case MAV_CMD_DO_WINCH:
        return "Winch";
    case MAV_CMD_DO_SEND_SCRIPT_MESSAGE:
        return "Scripting";
    case MAV_CMD_DO_JUMP:
        return "Jump";
    case MAV_CMD_DO_GO_AROUND:
        return "Go Around";
    case MAV_CMD_NAV_SCRIPT_TIME:
        return "NavScriptTime";
    case MAV_CMD_NAV_ATTITUDE_TIME:
        return "NavAttitudeTime";
    case MAV_CMD_DO_PAUSE_CONTINUE:
        return "PauseContinue";
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
        return "GimbalPitchYaw";
    case MAV_CMD_ABZ_SPRAYER:
        return "ABZSprayer";
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Mission command with ID %u has no string", id);
#endif
        return "?";
    }
}

bool AP_Mission::contains_item(MAV_CMD command) const
{
    for (int i = 1; i < num_commands(); i++) {
        Mission_Command tmp;
        if (!read_cmd_from_storage(i, tmp)) {
            continue;
        }
        if (tmp.id == command) {
            return true;
        }
    }
    return false;
}

/*
  return true if the mission has a terrain relative item.  ~2200us for 530 items on H7
 */
bool AP_Mission::contains_terrain_alt_items(void)
{
    if (_last_contains_relative_calculated_ms != _last_change_time_ms) {
        _contains_terrain_alt_items = calculate_contains_terrain_alt_items();
        _last_contains_relative_calculated_ms = _last_change_time_ms;
    }
    return _contains_terrain_alt_items;
}

bool AP_Mission::calculate_contains_terrain_alt_items(void) const
{
    for (int i = 1; i < num_commands(); i++) {
        Mission_Command tmp;
        if (!read_cmd_from_storage(i, tmp)) {
            continue;
        }
        if (stored_in_location(tmp.id) && tmp.content.location.terrain_alt) {
            return true;
        }
    }
    return false;
}

// reset the mission history to prevent recalling previous mission histories after a mission restart.
void AP_Mission::reset_wp_history(void)
{
    for (uint8_t i = 0; i<AP_MISSION_MAX_WP_HISTORY; i++) {
        _wp_index_history[i] = AP_MISSION_CMD_INDEX_NONE;
    }
    _resume_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _flags.resuming_mission = false;
    _repeat_dist = 0;
}
/**
 * @brief sets and save mission iteems in cmd. Give the number of items you want to save usually it's the
 *         length of the cmd if you added then len cmd +  len added 
*/
void AP_Mission::setSave(int number){
    _cmd_total.set_and_save(number);
}
// store the latest reported position incase of mission exit and rewind resume
void AP_Mission::update_exit_position(void)
{
    if (!AP::ahrs().get_location(_exit_position)) {
        _exit_position.lat = 0;
        _exit_position.lng = 0;
    }
}

// calculate the location of the _resume_cmd wp and set as current
bool AP_Mission::calc_rewind_pos(Mission_Command& rewind_cmd)
{
    // check for a recent history
    if (_wp_index_history[LAST_WP_PASSED] == AP_MISSION_CMD_INDEX_NONE) {
        // no saved history so can't rewind
        return false;
    }

    // check that we have a valid exit position
    if (_exit_position.lat == 0 && _exit_position.lng == 0) {
        return false;
    }

    Mission_Command temp_cmd;
    float rewind_distance = _repeat_dist; //(m)
    uint16_t resume_index;
    Location prev_loc = _exit_position;

    for (int8_t i = (LAST_WP_PASSED); i>=0; i--) {

        // to get this far there has to be at least one 'passed wp' stored in history.  This is to check incase
        // of history array no being completely filled with valid waypoints upon resume.
        if (_wp_index_history[i] == AP_MISSION_CMD_INDEX_NONE) {
            // no more stored history
            resume_index = i+1;
            break;
        }

        if (!read_cmd_from_storage(_wp_index_history[i], temp_cmd)) {
            // if read from storage failed then don't use rewind
            return false;
        }

        // calculate distance
        float disttemp = prev_loc.get_distance(temp_cmd.content.location); //(m)
        rewind_distance -= disttemp;
        resume_index = i;

        if (rewind_distance <= 0.0f) {
            // history rewound enough distance to meet _repeat_dist requirement
            rewind_cmd = temp_cmd;
            break;
        }

        // store wp location as previous
        prev_loc = temp_cmd.content.location;
    }

    if (rewind_distance > 0.0f) {
        // then the history array was rewound all of the way without finding a wp distance > _repeat_dist distance.
        // the last read temp_cmd will be the furthest cmd back in the history array so resume to that.
        rewind_cmd = temp_cmd;
        return true;
    }

    // if we have got this far the desired rewind distance lies between two waypoints stored in history array.
    // calculate the location for the mission to resume

    // the last wp read from storage is the wp that is before the resume wp in the mission order
    Location passed_wp_loc = temp_cmd.content.location;

    // fetch next destination wp
    if (!read_cmd_from_storage(_wp_index_history[resume_index+1], temp_cmd)) {
        // if read from storage failed then don't use rewind
        return false;
    }

    // determine the length of the mission leg that the resume wp lies in
    float leg_length = passed_wp_loc.get_distance(temp_cmd.content.location); //(m)

    // calculate the percentage along the leg that resume wp will be positioned
    float leg_percent = fabsf(rewind_distance)/leg_length;

    // calculate difference vector of mission leg
    Vector3f dist_vec = passed_wp_loc.get_distance_NED(temp_cmd.content.location);

    // calculate the resume wp position
    rewind_cmd.content.location.offset(dist_vec.x * leg_percent, dist_vec.y * leg_percent);
    rewind_cmd.content.location.alt -= dist_vec.z * leg_percent * 100; //(cm)

    // The rewind_cmd.index has the index of the 'last passed wp' from the history array.  This ensures that the mission order
    // continues as planned without further intervention.  The resume wp is not written to memory so will not perminantely change the mission.

    // if we got this far then mission rewind position was successfully calculated
    return true;
}

// singleton instance
AP_Mission *AP_Mission::_singleton;

namespace AP
{

AP_Mission *mission()
{
    return AP_Mission::get_singleton();
}

}
