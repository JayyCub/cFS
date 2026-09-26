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
*/

#include "common_types.h"

#define SIMLINK_SYNC_WORD 0x324B4C53u /* bytes 'S','L','K','2' on the wire (LE) */
#define SIMLINK_VERSION   3u

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

/* Flags bits in SIMLINK_SimState_t */
#define SIMLINK_FLAG_IN_CORRIDOR 0x1u
#define SIMLINK_FLAG_DOCKED      0x2u

/*
** SIM_STATE payload (Phase 1: still truth-derived relative nav — Phase 4 of the
** realism roadmap replaces this with raw sensor measurements).
**
** Frames: LVLH/world axes are Unity world axes — +Y radial (up), -Z along-track
** (velocity direction; the chaser approaches the ISS forward port from +V-bar),
** +X cross-track. Attitude errors and AngVel are in the chaser docking-port frame.
*/
typedef struct
{
    float  CycleDt_s;         /* sim time covered by one GNC cycle (s)                 */
    float  Range_m;           /* port-face to port-face distance                       */
    float  ClosingSpeed_ms;   /* + = closing                                           */
    float  LateralOffset_m;   /* port distance from the docking axis                   */
    float  AttitudeError_deg; /* cone angle between port axes                          */
    float  RelPos_X;          /* chaser origin - target origin, world frame (m)        */
    float  RelPos_Y;
    float  RelPos_Z;
    float  RelVel_X;          /* chaser - target velocity, world frame (m/s)           */
    float  RelVel_Y;
    float  RelVel_Z;
    float  AngVel_X;          /* chaser angular rate, chaser-port frame (rad/s)        */
    float  AngVel_Y;
    float  AngVel_Z;
    uint32 Flags;             /* SIMLINK_FLAG_*                                        */
    float  PitchError_deg;    /* per-axis attitude error [-180,180], 0 = aligned       */
    float  YawError_deg;
    float  RollError_deg;
    float  LatOffset_X;       /* signed port lateral offset, world X/Y (m)             */
    float  LatOffset_Y;
    float  Thc[3];            /* crew translation hand controller, body X/Y/Z, -1..+1  */
    float  Rhc[3];            /* crew rotation hand controller, body X/Y/Z (pitch/yaw/
                              ** roll), -1..+1                                          */
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
_Static_assert(sizeof(SIMLINK_SimState_t) == 104, "SimLink SIM_STATE layout changed");
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
