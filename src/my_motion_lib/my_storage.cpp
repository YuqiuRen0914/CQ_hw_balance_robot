#include "my_storage.h"
#include <Preferences.h>

namespace
{
constexpr const char *NVS_NAMESPACE = "balbot";
constexpr const char *NVS_KEY_CALOK = "cal_ok";
constexpr const char *NVS_KEY_DZL = "dzL";
constexpr const char *NVS_KEY_DZR = "dzR";
constexpr const char *NVS_KEY_GOK = "g_ok";
constexpr const char *NVS_KEY_GX = "gx";
constexpr const char *NVS_KEY_GY = "gy";
constexpr const char *NVS_KEY_GZ = "gz";

Preferences prefs;
bool ready = false;

bool ensure_ready()
{
    if (ready)
        return true;
    ready = prefs.begin(NVS_NAMESPACE, false);
    return ready;
}
} // namespace

bool storage_load_calib(float &dzL, float &dzR)
{
    if (!ensure_ready())
        return false;
    const bool ok = prefs.getBool(NVS_KEY_CALOK, false);
    if (!ok)
        return false;
    dzL = prefs.getFloat(NVS_KEY_DZL, dzL);
    dzR = prefs.getFloat(NVS_KEY_DZR, dzR);
    return true;
}

void storage_save_calib(float dzL, float dzR)
{
    if (!ensure_ready())
        return;
    prefs.putBool(NVS_KEY_CALOK, true);
    prefs.putFloat(NVS_KEY_DZL, dzL);
    prefs.putFloat(NVS_KEY_DZR, dzR);
}

bool storage_load_gyro_bias(float &gx, float &gy, float &gz)
{
    if (!ensure_ready())
        return false;
    const bool ok = prefs.getBool(NVS_KEY_GOK, false);
    if (!ok)
        return false;
    gx = prefs.getFloat(NVS_KEY_GX, gx);
    gy = prefs.getFloat(NVS_KEY_GY, gy);
    gz = prefs.getFloat(NVS_KEY_GZ, gz);
    return true;
}

void storage_save_gyro_bias(float gx, float gy, float gz)
{
    if (!ensure_ready())
        return;
    prefs.putBool(NVS_KEY_GOK, true);
    prefs.putFloat(NVS_KEY_GX, gx);
    prefs.putFloat(NVS_KEY_GY, gy);
    prefs.putFloat(NVS_KEY_GZ, gz);
}
