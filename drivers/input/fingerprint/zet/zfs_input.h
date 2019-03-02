/* ZFS1096 fingerprint sensor driver
 *
 * Copyright (c) 2017 powered by albert <albert.lin@zeitecsemi>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#ifndef ZFS_SPI_INPUT_H
#define ZFS_SPI_INPUT_H


extern int zfs_input_init(zfs_data_t *zfs_dev);

extern void zfs_input_destroy(zfs_data_t *zfs_dev);

extern int zfs_input_enable(zfs_data_t *zfs_dev , bool enabled);

extern int zfs_input_set_event(zfs_data_t *zfs_dev , int key_code, int on_off);

extern void zfs_input_snyc(zfs_data_t *zfs_dev);

#ifdef ZET_CAPTURE_SUPPORT

extern int zfs1096_input_task(zfs_data_t *zfs_dev);

extern int zfs1096_input_report(zfs_data_t *zfs_dev, u8 *key_status);

#endif /* ZET_CAPTURE_SUPPORT */

#endif /* ZFS_SPI_INPUT_H */

