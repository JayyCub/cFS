#ifndef NAV_STATUS_H
#define NAV_STATUS_H

/*
** NAV status bits, carried in the solution and HK packets (nav_msg.h). Kept in
** their own header, free of cFE includes, so the estimation core
** (nav_filter.c) and its stand-alone test harness can use them.
*/
#define NAV_STATUS_ATT_VALID    0x01u /* attitude known: star tracker, or gyro-propagated from a recent fix */
#define NAV_STATUS_RATE_VALID   0x02u /* body rate measured by the gyro this cycle                         */
#define NAV_STATUS_RELNAV_VALID 0x04u /* relative position + velocity good enough to steer on              */
#define NAV_STATUS_FILTER_INIT  0x08u /* translation filter initialised                                    */
#define NAV_STATUS_RPS_USED     0x10u /* LIDAR fix accepted this cycle                                     */
#define NAV_STATUS_RPS_REJECTED 0x20u /* LIDAR fix failed the innovation gate this cycle                   */
#define NAV_STATUS_DV_VALID     0x40u /* DeltaV_B is a measurement (IMU valid) this cycle                  */
#define NAV_STATUS_ST_USED      0x80u /* attitude came from the star tracker this cycle (not propagated)   */

#endif /* NAV_STATUS_H */
