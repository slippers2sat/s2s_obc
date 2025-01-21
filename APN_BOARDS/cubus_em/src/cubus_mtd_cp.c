#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <../../../nuttx/fs/littlefs/lfs_vfs.h>
#include <sys/mount.h>
#include <nuttx/spi/spi.h>

#include <inttypes.h>
#include <errno.h>
#include <stdbool.h>
#include "cubus_mtd.h"

extern struct mtd_dev_s *mtd_partition(FAR struct mtd_dev_s *mtd,
                                    off_t firstblock, off_t nblocks);
static int num_instances = 0;
static int total_blocks = 0;
static mtd_instance_s *instances[MAX_MTD_INSTANCES]={};

static const cubus_mft_device_t spi3_dev = {             // MT25QL on FMUM 1Gb 2048 X 64K
	.type = SPI,
  .bus_id   = 3,
	.devid    = SPIDEV_FLASH(0)
};

static const cubus_mft_device_t spi4_dev = {             // MT25QL on FMUM 1Gb 2048 X 64K
  .type = SPI,
  .bus_id   = 4,
  .devid    = SPIDEV_FLASH(0)
};

static const cubus_mtd_entry_t cubus_mfm = {
	.device = &spi3_dev,
	.npart = 2,
	.partd = {
		{
			.type = MTD_MAINSTORAGE,		// storage space for  HK data logging, flag data storage, reservation table.
			.path = "/fs/mfm/mtd_mainstorage",
			// .nblocks = 51200				// 12.5MB in no of pages, each page having 256 bytes
			.nblocks = 262144				// 64 MB in no of pages
		},
		{					
			.type = MTD_MISSION,			// storage space for missions
			.path = "/fs/mfm/mtd_mission",	
			.nblocks = 262144				// 64 MB in no of pages
		}
	},
};

static const cubus_mtd_entry_t cubus_sfm = {
	.device = &spi4_dev,
	.npart = 2,
	.partd = {
		{
			.type = MTD_MAINSTORAGE,			// Partition for storing MSN data
			.path = "/fs/sfm/mtd_mainstorage",
			.nblocks = 262144			// 128 MB in no of pages, each pages having 256 bytes
		},
		{					
			.type = MTD_MISSION,			// storage space for missions
			.path = "/fs/sfm/mtd_mission",	
			.nblocks = 262144				// 64 MB in no of pages
		}
	},
};

static const cubus_mtd_manifest_t board_mfm_mtd_config = {
	.nconfigs   = 1,
	.entries = {
		&cubus_mfm
	}
};

static const cubus_mft_entry_s mtd_mft = {
	.type = MTD,
	.pmft = (void *) &board_mfm_mtd_config,
};

static const cubus_mft_s mft = {
	.nmft = 1,
	.mfts = {
		&mtd_mft
	}
};

const cubus_mft_s* board_mfm_get_manifest(void)
{
	return &mft;
}

static const cubus_mtd_manifest_t board_sfm_mtd_config = {
	.nconfigs   = 1,
	.entries = {
		&cubus_sfm
	}
};

static const cubus_mft_entry_s mtd_mft_sfm = {
	.type = MTD,
	.pmft = (void *) &board_sfm_mtd_config,
};

static const cubus_mft_s mft_sfm = {
	.nmft = 1,
	.mfts = {
		&mtd_mft_sfm
	}
};

const cubus_mft_s* board_sfm_get_manifest(void)
{
	return &mft_sfm;
}


int mt25ql_attach(mtd_instance_s *instance)
{
	syslog(LOG_INFO,"Starting MTD, MT25QL driver\n");

	/* start the MT25QL driver, attempt 10 times */

	int spi_speed_mhz = 21;

	for (int i = 0; i < 21; i++) {
		/* initialize the right spi */
		struct spi_dev_s *spi = stm32_spibus_initialize(instance->bus_id);

		if (spi == NULL) {
			syslog(LOG_ERR,"failed to locate spi bus");
			return ENXIO;
		}

		/* this resets the spi bus, set correct bus speed again */
		SPI_LOCK(spi, true);
		SPI_SETFREQUENCY(spi, spi_speed_mhz * 1000 * 1000);
		SPI_SETBITS(spi, 8);
		SPI_SETMODE(spi, SPIDEV_MODE0);
		SPI_SELECT(spi, instance->devid, false);
		SPI_LOCK(spi, false);

		instance->mtd_dev = mt25ql_initialize(spi);

		if (instance->mtd_dev) {
			/* abort on first valid result */
			if (i > 0) {
				syslog(LOG_WARNING,"mtd needed %d attempts to attach", i + 1);
			}
			break;
		}

		// try reducing speed for next attempt
		spi_speed_mhz--;
		usleep(10000);
	}

	/* if last attempt is still unsuccessful, abort */
	if (instance->mtd_dev == NULL) {
		syslog(LOG_ERR,"failed to initialize mtd driver");
		return -EIO;
	}

	int ret = instance->mtd_dev->ioctl(instance->mtd_dev, MTDIOC_SETSPEED, (unsigned long)spi_speed_mhz * 1000 * 1000);

	if (ret != OK) {
		// FIXME: From the previous warning call, it looked like this should have been fatal error instead. Tried
		// that but setting the bus speed does fail all the time. Which was then exiting and the board would
		// not run correctly. So changed to CUBUS_WARN.
		syslog(LOG_WARNING,"failed to set bus speed");
	}

	return 0;
}

int cubus_mtd_get_geometry(const mtd_instance_s *instance, unsigned long *blocksize, unsigned long *erasesize,\
                            unsigned long *neraseblocks, 
                            unsigned *blkpererase, unsigned *nblocks, unsigned *partsize)
{
    FAR struct mtd_geometry_s geo;

    int ret = instance->mtd_dev->ioctl(instance->mtd_dev, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));

	if (ret < 0) {
		syslog(LOG_ERR,"mtd->ioctl failed: %d", ret);
		return ret;
	}

	*blocksize = geo.blocksize;
	*erasesize = geo.erasesize;
	*neraseblocks = geo.neraseblocks;

	/* Determine the size of each partition.  Make each partition an even
	 * multiple of the erase block size (perhaps not using some space at the
	 * end of the FLASH).
	 */

	*blkpererase = geo.erasesize / geo.blocksize;
	*nblocks     = (geo.neraseblocks / instance->n_partitions_current) * *blkpererase;
	*partsize    = *nblocks * geo.blocksize;

	return ret;
}


/*
  get partition size in bytes
 */
ssize_t cubus_mtd_get_partition_size(const mtd_instance_s *instance, const char *partname)
{
	unsigned long blocksize, erasesize, neraseblocks;
	unsigned blkpererase, nblocks, partsize = 0;

	int ret = cubus_mtd_get_geometry(instance, &blocksize, &erasesize, &neraseblocks, &blkpererase, &nblocks, &partsize);

	if (ret != OK) {
		syslog(LOG_ERR,"Failed to get geometry");
		return 0;
	}

	unsigned partn = 0;

	for (unsigned n = 0; n < instance->n_partitions_current; n++) {
		if (instance->partition_names[n] != NULL &&
		    partname != NULL &&
		    strcmp(instance->partition_names[n], partname) == 0) {
			partn = n;
			break;
		}
	}

	return instance->partition_block_counts[partn] * blocksize;
}

mtd_instance_s **cubus_mtd_get_instances(unsigned int *count)
{
	*count = num_instances;
	return instances;
}

int cubus_unmount(const cubus_mtd_manifest_t *mft_mtd) {
    if (mft_mtd == NULL) {
        syslog(LOG_ERR, "Invalid MTD configuration provided!\n");
        return -EINVAL;
    }

    if (mft_mtd->nconfigs == 0) {
        syslog(LOG_INFO, "No MTD configurations to unmount.\n");
        return 0;
    }

    for (uint8_t config_idx = 0; config_idx < mft_mtd->nconfigs; ++config_idx) {
        const cubus_mtd_entry_t *entry = mft_mtd->entries[config_idx];
        if (entry == NULL) {
            syslog(LOG_ERR, "Invalid MTD entry at index %d. Skipping...\n", config_idx);
            continue;
        }

        for (uint32_t part_idx = 0; part_idx < entry->npart; ++part_idx) {
            const char *mount_point = entry->partd[part_idx].path;
            if (mount_point == NULL) {
                syslog(LOG_ERR, "Partition mount point is NULL for config %d, partition %d. Skipping...\n", config_idx, part_idx);
                continue;
            }

            syslog(LOG_INFO, "Unmounting partition: %s\n", mount_point);

            int ret = nx_mount(mount_point);
            if (ret < 0) {
                syslog(LOG_ERR, "Failed to unmount %s. Error: %d\n", mount_point, ret);
                return ret; // Return early on failure
            } else {
                syslog(LOG_INFO, "Successfully unmounted %s\n", mount_point);
            }
        }
    }

    syslog(LOG_INFO, "All MTD partitions unmounted successfully.\n");
    return 0;
}



// int cubus_mtd_config(const cubus_mtd_manifest_t *mft_mtd)
// {
// 	int rv = unregister_mtddriver("/mnt/fs/sfm");

// 	return rv;
// }

int cubus_mft_configure(const cubus_mft_s *mft_p)
{

	if (mft_p != NULL) {
		for (uint32_t m = 0; m < mft_p->nmft; m++) {
			switch (mft_p->mfts[m]->type) {
			case MTD:
				cubus_mtd_config((const cubus_mtd_manifest_t *)mft_p->mfts[m]->pmft);
				break;

			case MFT:
			default:
				break;
			}
		}
	}

	return 0;
}
