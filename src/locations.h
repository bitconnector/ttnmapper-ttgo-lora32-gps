//locations excludet for ttnmapper-upload

struct Geofence
{
    double lat_min;
    double lat_max;
    double lng_min;
    double lng_max;
};

static Geofence geofence[] = {
    {51.99935, 52.03295, 8.50634, 8.55485} //Bielefeld :P
};
