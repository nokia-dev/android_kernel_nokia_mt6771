/* ZFS1096 fingerprint sensor driver
 *
 * Copyright (c) 2017 powered by albert <albert.lin@zeitecsemi>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#ifndef LINUX_ZFS_SPI_CAPTURE_H
#define LINUX_ZFS_SPI_CAPTURE_H

extern int zfs_init_capture(zfs_data_t *zfs_dev);

extern bool zfs_capture_check_ready(zfs_data_t *zfs_dev);

#ifdef ZET_CAPTURE_SUPPORT

extern int zfs1096_capture_task(zfs_data_t *zfs_dev);

extern int zfs1096_capture_finger_key_navi_task(zfs_data_t *zfs_dev);

#endif /* ZET_CAPTURE_SUPPORT */

#endif /* LINUX_ZFS_SPI_CAPTURE_H */

