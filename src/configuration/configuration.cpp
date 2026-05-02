#include "configuration.h"

#define PREF_NAMESPACE     "config"
#define PREF_NAMESPACE_ALG "config_alg"

static Preferences prefs;

FileResult InitConfiguration() {
    if (!prefs.begin(PREF_NAMESPACE, false)) {
        return INIT_FAILED;
    }
    prefs.end();
    return SUCCESS;
}

FileResult SaveConfiguration() {
    if (!prefs.begin(PREF_NAMESPACE, false)) {
        return INIT_FAILED;
    }

    prefs.putBool  ("SETUP_DONE",    SETUP_COMPLETED);
    prefs.putInt   ("MOWER_OVERLAP", MOWER_OVERLAP);
    prefs.putInt   ("MAX_DEVIATION", MAX_DEVIATION);
    prefs.putDouble("BASE_LON",      BASE_LON);
    prefs.putDouble("BASE_LAT",      BASE_LAT);
    prefs.putDouble("BASE_EXIT_LON", BASE_EXIT_LON);
    prefs.putDouble("BASE_EXIT_LAT", BASE_EXIT_LAT);
    prefs.putInt   ("GPS_ACC_THR",   GPS_ACC_THRESHOLD);
    prefs.putInt   ("GPS_STAB_DUR",  GPS_STABILITY_CHECK_DURATION_SECONDS);
    prefs.putFloat ("BAT_LVL_MIN",   BATTERY_LEVEL_MIN);
    prefs.putFloat ("BAT_LVL_MAX",   BATTERY_LEVEL_MAX);
    prefs.putBool  ("IMU_INVERT",    IMU_INVERT);
    prefs.putFloat ("MOT_ACC_FACT",  MOTION_ACC_FACTOR);
    prefs.putBool  ("CFG_PATH",      CONFIG_PATH);
    prefs.putBool  ("CFG_BATTERY",   CONFIG_BATTERY);
    prefs.putBool  ("CFG_MOTORS",    CONFIG_MOTORS);
    prefs.putBool  ("CFG_GYRO",      CONFIG_GYRO);
    prefs.putBool  ("CFG_RAIN",      CONFIG_RAIN_SENSOR);
    prefs.putBool  ("CFG_DATUM",     CONFIG_DATUM);

    prefs.end();

    if (!prefs.begin(PREF_NAMESPACE_ALG, false)) {
        return INIT_FAILED;
    }
    prefs.putString("ALG_PATH", AlgorithmGetPathString());
    prefs.end();

    return SUCCESS;
}

FileResult LoadConfiguration() {
    if (!prefs.begin(PREF_NAMESPACE, true)) {
        return INIT_FAILED;
    }

    if (!prefs.isKey("SETUP_DONE")) {
        prefs.end();
        return FAILED_OPEN;
    }

    SETUP_COMPLETED                  = prefs.getBool  ("SETUP_DONE",    SETUP_COMPLETED);
    MOWER_OVERLAP                    = prefs.getInt   ("MOWER_OVERLAP", MOWER_OVERLAP);
    MAX_DEVIATION                    = prefs.getInt   ("MAX_DEVIATION", MAX_DEVIATION);
    BASE_LON                         = prefs.getDouble("BASE_LON",      BASE_LON);
    BASE_LAT                         = prefs.getDouble("BASE_LAT",      BASE_LAT);
    BASE_EXIT_LON                    = prefs.getDouble("BASE_EXIT_LON", BASE_EXIT_LON);
    BASE_EXIT_LAT                    = prefs.getDouble("BASE_EXIT_LAT", BASE_EXIT_LAT);
    GPS_ACC_THRESHOLD                = prefs.getInt   ("GPS_ACC_THR",   GPS_ACC_THRESHOLD);
    GPS_STABILITY_CHECK_DURATION_SECONDS = prefs.getInt("GPS_STAB_DUR", GPS_STABILITY_CHECK_DURATION_SECONDS);
    BATTERY_LEVEL_MIN                = prefs.getFloat ("BAT_LVL_MIN",   BATTERY_LEVEL_MIN);
    BATTERY_LEVEL_MAX                = prefs.getFloat ("BAT_LVL_MAX",   BATTERY_LEVEL_MAX);
    IMU_INVERT                       = prefs.getBool  ("IMU_INVERT",    IMU_INVERT);
    MOTION_ACC_FACTOR                = prefs.getFloat ("MOT_ACC_FACT",  MOTION_ACC_FACTOR);
    CONFIG_PATH                      = prefs.getBool  ("CFG_PATH",      CONFIG_PATH);
    CONFIG_BATTERY                   = prefs.getBool  ("CFG_BATTERY",   CONFIG_BATTERY);
    CONFIG_MOTORS                    = prefs.getBool  ("CFG_MOTORS",    CONFIG_MOTORS);
    CONFIG_GYRO                      = prefs.getBool  ("CFG_GYRO",      CONFIG_GYRO);
    CONFIG_RAIN_SENSOR               = prefs.getBool  ("CFG_RAIN",      CONFIG_RAIN_SENSOR);
    CONFIG_DATUM                     = prefs.getBool  ("CFG_DATUM",     CONFIG_DATUM);

    prefs.end();

    if (!prefs.begin(PREF_NAMESPACE_ALG, true)) {
        return INIT_FAILED;
    }
    String algPath = prefs.getString("ALG_PATH", "");
    prefs.end();

    if (algPath.length() > 0) {
        if (!AlgorithmPopulatePathFromString(algPath)) {
            return ALGORITHM_FAILED;
        }
    }

    return SUCCESS;
}