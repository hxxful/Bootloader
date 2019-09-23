/****************************************************************************
 *
 * Copyright (c) 2018 UAVRS. All rights reserved.
 *
 ****************************************************************************/

/*******************************************************************************
 * Included Files
 ******************************************************************************/


#include "uavrs_v2_flexspi_nor_flash.h"
#include "board.h"

/*******************************************************************************
 * Public Data
 ******************************************************************************/

#if defined (CONFIG_IMXRT1052_HYPER_FLASH)

__attribute__((section(".boot_hdr.conf")))
const struct flexspi_nor_config_s g_flash_config =
{
  .mem_config               =
  {
    .tag                    = FLEXSPI_CFG_BLK_TAG,
    .version                = FLEXSPI_CFG_BLK_VERSION,
    .read_sample_clksrc     = FLASH_READ_SAMPLE_CLK_EXTERNALINPUT_FROM_DQSPAD,
    .cs_hold_time           = 3u,
    .cs_setup_time          = 3u,
    .column_address_width   = 3u,

    /* Enable DDR mode, Word addassable, Safe configuration, Differential clock */

    .controller_misc_option = (1u << FLEXSPIMISC_OFFSET_DDR_MODE_EN) |
                              (1u << FLEXSPIMISC_OFFSET_WORD_ADDRESSABLE_EN) |
                              (1u << FLEXSPIMISC_OFFSET_SAFECONFIG_FREQ_EN) |
                              (1u << FLEXSPIMISC_OFFSET_DIFFCLKEN),
    .sflash_pad_type        = SERIAL_FLASH_8PADS,
    .serial_clk_freq        = FLEXSPI_SERIAL_CLKFREQ_166MHz,
    .sflash_a1size          = 64u * 1024u * 1024u,
    .data_valid_time        = {16u, 16u},
    .lookup_table           =
    {
      /* Read LUTs */

      FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xA0, RADDR_DDR, FLEXSPI_8PAD, 0x18),
      FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10, DUMMY_DDR, FLEXSPI_8PAD, 0x06),
      FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04, STOP, FLEXSPI_1PAD, 0x0),
    },
  },
  .page_size                = 512u,
  .sector_size              = 256u * 1024u,
  .blocksize                = 256u * 1024u,
  .is_uniform_blocksize     = 1,
};

#elif defined (CONFIG_IMXRT1052_QSPI_FLASH)

__attribute__((section(".boot_hdr.conf")))
const struct flexspi_nor_config_s flash_config =
{
	.mem_config =
	{
		.tag = FLEXSPI_CFG_BLK_TAG,/*标志：FCFB*/
		.version = FLEXSPI_CFG_BLK_VERSION,/*版本：V1.4.0*/
		.read_sample_clksrc = FLASH_READ_SAMPLE_CLK_LOOPBACK_INTERNELLY,/*内部环回*/
		.cs_hold_time = 3u,/*保持时间*/
		.cs_setup_time = 3u,/*建立时间*/
		.column_address_width = 0u,/*列地址宽度*/
		.device_mode_cfg_enable = 1u,/*设备模式配置使能*/
		.device_mode_type = 1u,/*Quad 使能命令*/
		.device_mode_seq.reserved = 0u,
		.device_mode_seq.seq_num = 1u,/*LUT序列号*/
		.device_mode_seq.seq_id = 4u,/*LUT序列索引*/
		.device_mode_arg = 0x000200,/*设置 QE=1（S9）*/
		.device_type = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,/*设备类型为nor flash*/
		.sflash_pad_type = SERIAL_FLASH_4PADS,/*设备数据总线为4*/
		.serial_clk_freq = FLEXSPI_SERIAL_CLKFREQ_133MHz,/*flash 时钟*/
		.sflash_a1size = 32u * 1024u * 1024u,/*flash 大小32MBytes*/
		//.data_valid_time = {16u, 16u},
		.lookup_table =
			{
				/*快速读命令（四线）*/
				[0] 	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xEB, RADDR_SDR, FLEXSPI_4PAD, 0x18),
				[1] 	= FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06, READ_SDR, FLEXSPI_4PAD, 0x04),
				

				/*读状态命令*/
				[1*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04),
				/*写使能命令*/
				[3*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0),		
				/*擦除扇区命令*/
				[5*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 0x04),
				/*页编程命令（四线）*/
				[9*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x32, RADDR_SDR, FLEXSPI_1PAD, 0x18),	
				[9*4+1] = FLEXSPI_LUT_SEQ(WRITE_SDR,FLEXSPI_4PAD , 0x04, STOP, FLEXSPI_1PAD, 0),				  
				/*整片擦除*/
				[11*4]	= FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xc7, STOP, FLEXSPI_1PAD, 0),					
			},
	},
	.page_size = 256u,/*页大小为256字节*/
	.sector_size = 4u * 1024u,/*扇区大小为4k字节*/
};

#elif defined(CONFIG_IMXRT1064_QSPI_FLASH)

__attribute__((section(".boot_hdr.conf")))
const struct flexspi_nor_config_s flash_config =
{
    .mem_config =
        {
            .tag              = FLEXSPI_CFG_BLK_TAG,
            .version          = FLEXSPI_CFG_BLK_VERSION,
            .read_sample_clksrc = FLASH_READ_SAMPLE_CLK_LOOPBACK_FROM_DQSPAD,
            .cs_hold_time       = 3u,
            .cs_setup_time      = 3u,
            // Enable DDR mode, Wordaddassable, Safe configuration, Differential clock
            .sflash_pad_type = SERIAL_FLASH_4PADS,
            .serial_clk_freq = FLEXSPI_SERIAL_CLKFREQ_133MHz,
            .sflash_a1size  = 4u * 1024u * 1024u,
            .lookup_table =
                {
                    // Read LUTs
                    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xEB, RADDR_SDR, FLEXSPI_4PAD, 0x18),
                    FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06, READ_SDR, FLEXSPI_4PAD, 0x04),
                },
        },
    .page_size           = 256u,
    .sector_size         = 4u * 1024u,
    .blocksize          = 256u * 1024u,
    .is_uniform_blocksize = false,
};

#else
# error Boot Flash type not chosen!
#endif
