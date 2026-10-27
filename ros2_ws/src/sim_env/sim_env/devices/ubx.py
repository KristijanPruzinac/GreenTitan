"""u-blox UBX frame encoding (protocol 15+, i.e. M8 and later)."""
import struct

NAV_PVT_FMT = '<IHBBBBBBIiBBBBiiiiIIiiiiiIIH6BihH'  # 92-byte UBX-NAV-PVT payload
assert struct.calcsize(NAV_PVT_FMT) == 92


def frame(cls, msg_id, payload):
    body = struct.pack('<BBH', cls, msg_id, len(payload)) + payload
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b'\xb5\x62' + body + bytes([ck_a, ck_b])


def nav_pvt(*, itow_ms, utc, fix_type, carr_soln, num_sv, lat, lon, height_m, hmsl_m, h_acc_m, v_acc_m,
            vel_ned, g_speed, head_mot_deg, s_acc, head_acc_deg, p_dop=1.2):
    """``utc`` is a datetime; velocities in m/s; returns the complete 100-byte frame."""
    fix_ok = fix_type >= 2
    flags = (1 if fix_ok else 0) | (2 if carr_soln > 0 else 0) | ((carr_soln & 0x3) << 6)
    valid = 0x07 if fix_ok else 0x00  # validDate | validTime | fullyResolved
    payload = struct.pack(
        NAV_PVT_FMT,
        int(itow_ms) & 0xFFFFFFFF, utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second, valid,
        50, int(utc.microsecond * 1000), int(fix_type), flags, 0, int(num_sv),
        int(round(lon * 1e7)), int(round(lat * 1e7)), int(round(height_m * 1e3)), int(round(hmsl_m * 1e3)),
        max(1, int(round(h_acc_m * 1e3))), max(1, int(round(v_acc_m * 1e3))),
        *[int(round(v * 1e3)) for v in vel_ned], int(round(g_speed * 1e3)),
        int(round((head_mot_deg % 360.0) * 1e5)), max(1, int(round(s_acc * 1e3))),
        int(round(min(head_acc_deg, 180.0) * 1e5)), int(round(p_dop * 100)),
        0, 0, 0, 0, 0, 0, 0, 0, 0)
    return frame(0x01, 0x07, payload)
