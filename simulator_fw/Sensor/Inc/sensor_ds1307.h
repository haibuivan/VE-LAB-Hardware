#ifndef SENSOR_DS1307_H
#define SENSOR_DS1307_H

#ifdef __cplusplus
extern "C" {
#endif

/* API dong bo cho layer App */
void sensor_ds1307_init(void);
void sensor_ds1307_run(void);

/* Backward-compatible API cu */
void ds1307_sim_init(void);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_DS1307_H
