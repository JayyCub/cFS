#ifndef SIMLINK_ICD_H
#define SIMLINK_ICD_H

/*
** SimLink ICD — wire format between the Unity simulation (the "vehicle
** hardware") and cFS (SIM_IO app). Mirrors Assets/SimLinkProtocol.cs; see
** Docs/SIMLINK_ICD.md for the full interface description.
**
** Every frame is:
**
**     SIMLINK_Header_t | payload | SIMLINK_Trailer_t
**
** little-endian, no padding (every field is naturally aligned — the static
** asserts below enforce it so the layout can't silently drift).
**
** Lock-step: Unity sends one SIM_STATE frame per GNC cycle with an
** incrementing Seq, then blocks until a THRUSTER_CMD frame echoing that same
** Seq comes back. cFS is therefore scheduled by simulation time, not by a
** wall-clock timer, and every run with the same inputs is repeatable.
**
** v3 (realism phase 2): cFS now does thruster allocation and PWM itself, so
** the command is per-thruster valve on-times, not a wrench; SIM_STATE carries
** the crew hand-controller deflections so manual flight also goes through FSW.
**
** v4 (realism phase 4): SIM_STATE carries raw sensor readings (IMU, star
** tracker, relative-pose LIDAR, docking-mechanism switches) instead of
** truth-derived navigation. The cFS NAV app does the navigation.
*/

#include "common_types.h"

#define SIMLINK_SYNC_WORD 0x324B4C53u /* bytes 'S','L','K','2' on the wire (LE) */
#define SIMLINK_VERSION   4u

#define SIMLINK_TYPE_SIM_STATE    1u /* Unity -> cFS, once per GNC cycle            */
#define SIMLINK_TYPE_THRUSTER_CMD 3u /* cFS -> Unity, echoes the SIM_STATE Seq      */
                                     /* (type 2 = v2 WRENCH_CMD, retired)            */

#define SIMLINK_NUM_THRUSTERS 16u

#define SIMLINK_MAX_FRAME_BYTES 512u

typedef struct
{
    uint32 Sync;      /* SIMLINK_SYNC_WORD                                         */
    uint16 Version;   /* SIMLINK_VERSION                                           */
    uint16 Type;      /* SIMLINK_TYPE_*                                            */
    uint32 Seq;       /* Unity GNC-cycle counter; commands echo the state's Seq     */
    uint32 Length;    /* total frame bytes, header + payload + trailer             */
    double SimTime_s; /* simulation time at the cycle boundary (s)                 */
} SIMLINK_Header_t;

typedef struct
{
    uint16 Crc;   /* CRC-16/CCITT-FALSE over every byte before this field */
    uint16 Spare;
} SIMLINK_Trailer_t;

/* SensorValid bits in SIMLINK_SimState_t: the device produced a reading this cycle */
#define SIMLINK_SENSOR_IMU 0x1u /* GyroRate_B and DeltaV_B                                */
#define SIMLINK_SENSOR_ST  0x2u /* StQuat                                                 */
#define SIMLINK_SENSOR_RPS 0x4u /* RpsRange/Az/El — target in field of view and in range */

/* MechFlags bits in SIMLINK_SimState_t: docking-mechanism switches */
#define SIMLINK_MECH_CAPTURE 0x1u /* soft-capture latches engaged */

/*
** SIM_STATE payload — raw sensor readings, sampled at the cycle boundary.
**
** Frames:
**   body (B)   chaser vehicle axes (+Z toward the docking port, +X right, +Y up),
**              origin at the centre of mass — the same frame as the RCS table.
**   LVLH (L)   Unity world axes: +Y radial (up), -Z along-track (velocity
**              direction; the chaser approaches the ISS forward port from
**              +V-bar), +X cross-track.
**   sensor (S) the relative-pose sensor's own axes, boresight +Z.
**
** Quaternions are (x, y, z, w) and rotate vectors from the second-named frame
** into the first: StQuat = q_L_B, so v_L = q ⊗ v_B ⊗ q*.
*/
typedef struct
{
    float  CycleDt_s;   /* sim time covered by one GNC cycle (s)                         */
    uint32 SensorValid; /* SIMLINK_SENSOR_*                                              */

    /* IMU (at the CoM) */
    float GyroRate_B[3]; /* body angular rate (rad/s)                                     */
    float DeltaV_B[3];   /* non-gravitational velocity change over the cycle that just
                         ** ended (m/s) — thrust and contact; free fall reads zero        */

    /* Star tracker */
    float StQuat[4]; /* q_L_B, body attitude in LVLH (x, y, z, w)                         */

    /* Relative-pose sensor (LIDAR): line of sight to the target docking port's
    ** reflector, sensor frame. Az = atan2(x, z), El = asin(y / range). */
    float RpsRange_m;
    float RpsAz_rad;
    float RpsEl_rad;

    uint32 MechFlags; /* SIMLINK_MECH_*                                                    */

    float Thc[3]; /* crew translation hand controller, body X/Y/Z, -1..+1                */
    float Rhc[3]; /* crew rotation hand controller, body X/Y/Z (pitch/yaw/roll), -1..+1  */
} SIMLINK_SimState_t;

/*
** THRUSTER_CMD payload — valve on-times computed by the cFS RCS app (allocation +
** PWM). Every thruster opens at the start of the cycle and closes after its own
** on-time; Unity applies full rated thrust while open.
*/
typedef struct
{
    uint32 NumThrusters;                     /* SIMLINK_NUM_THRUSTERS             */
    int32  GncPhase;                         /* GNC_Phase_t, for HUD display      */
    float  OnTime_s[SIMLINK_NUM_THRUSTERS];  /* 0 = closed this cycle             */
} SIMLINK_ThrusterCmd_t;

#define SIMLINK_SIM_STATE_FRAME_BYTES \
    (sizeof(SIMLINK_Header_t) + sizeof(SIMLINK_SimState_t) + sizeof(SIMLINK_Trailer_t))
#define SIMLINK_THRUSTER_CMD_FRAME_BYTES \
    (sizeof(SIMLINK_Header_t) + sizeof(SIMLINK_ThrusterCmd_t) + sizeof(SIMLINK_Trailer_t))

_Static_assert(sizeof(SIMLINK_Header_t) == 24, "SimLink header layout changed");
_Static_assert(sizeof(SIMLINK_Trailer_t) == 4, "SimLink trailer layout changed");
_Static_assert(sizeof(SIMLINK_SimState_t) == 88, "SimLink SIM_STATE layout changed");
_Static_assert(sizeof(SIMLINK_ThrusterCmd_t) == 72, "SimLink THRUSTER_CMD layout changed");

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR. */
static inline uint16 SIMLINK_Crc16(const uint8 *Data, uint32 Len)
{
    uint16 Crc = 0xFFFF;
    uint32 i;
    int    b;

    for (i = 0; i < Len; i++)
    {
        Crc ^= (uint16)((uint16)Data[i] << 8);
        for (b = 0; b < 8; b++)
        {
            Crc = (Crc & 0x8000) ? (uint16)((Crc << 1) ^ 0x1021) : (uint16)(Crc << 1);
        }
    }
    return Crc;
}

#endif /* SIMLINK_ICD_H */
