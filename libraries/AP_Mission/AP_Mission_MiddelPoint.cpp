#include "AP_Mission_MiddelPoint.hpp"

AP_Mission::Mission_Command calculateMiddlePoint(AP_Mission::Mission_Command start_point, AP_Mission::Mission_Command end_point){

        AP_Mission::Mission_Command trainingPointMiddle;
        trainingPointMiddle = start_point; //upload with mock starter data
        trainingPointMiddle.index = 5000; // just need enough large index avoid index overwrite

        Location middle_location = trainingPointMiddle.content.location;
        Location end_location = end_point.content.location;
        Location start_location = start_point.content.location;


        middle_location.lat = (end_location.lat - start_location.lat) * 0.75 + start_location.lat;
        double lat = (double)middle_location.lat / (double)10000000; //to print
        middle_location.lng = (end_location.lng - start_location.lng) * 0.75 + start_location.lng;
        double lng = (double)middle_location.lng / (double)10000000; //to print
        gcs().send_text(MAV_SEVERITY_ALERT, "middle.id: %i,idx: %i,lat: %.4f,lng: %.4f", trainingPointMiddle.id, trainingPointMiddle.index, lat, lng);

        return trainingPointMiddle;
}