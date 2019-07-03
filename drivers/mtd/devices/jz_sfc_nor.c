/*
 * SFC controller for SPI protocol, use FIFO and DMA;
 *
 * Copyright (c) 2015 Ingenic
 * Author: <xiaoyang.fu@ingenic.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/kernel.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/page.h>

#include <mach/sfc.h>
#include <mach/sfc_params.h>
#include <mach/spi_nor.h>
#include "jz_sfc_common.h"




//#define DEBUG_CLONER_PARAMS

static int L2CACHE_ALIGN_SIZE;

#define STATUS_SUSPND	(1<<0)


struct sfc_flash *to_jz_spi_norflash(struct mtd_info *mtd_info)
{
	return container_of(mtd_info, struct sfc_flash, mtd);
}

int sfc_nor_reset(struct sfc_flash *flash)
{
	struct sfc_transfer transfer;
	struct sfc_message message;
	struct cmd_info cmd;
	int ret;

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	cmd.cmd = SPINOR_OP_RSTEN;
	cmd.dataen = DISABLE;
	transfer.cmd_info = &cmd;
	transfer.sfc_mode = 0;
	sfc_message_add_tail(&transfer, &message);
	ret = sfc_sync(flash->sfc, &message);
	if (ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}

	cmd.cmd = SPINOR_OP_RST;
	cmd.dataen = DISABLE;
	transfer.cmd_info = &cmd;
	transfer.sfc_mode = 0;
	sfc_message_add_tail(&transfer, &message);

	ret = sfc_sync(flash->sfc, &message);
	if (ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}
	udelay(100);
	return ret;
}

int sfc_nor_read_id(struct sfc_flash *flash, u8 command, unsigned int addr, int addr_len, size_t len, int dummy_byte)
{

	struct sfc_transfer transfer;
	struct sfc_message message;
	struct cmd_info cmd;
	int ret;
	unsigned int chip_id = 0;

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	cmd.cmd = command;
	cmd.dataen = ENABLE;

	transfer.addr_len = addr_len;
	transfer.data_dummy_bits = dummy_byte;
	transfer.addr = addr;
	transfer.len = len;
	transfer.data =(unsigned char *)&chip_id;
	transfer.ops_mode = CPU_OPS;
	transfer.sfc_mode = TM_STD_SPI;
	transfer.direction = GLB_TRAN_DIR_READ;
	transfer.cmd_info = &cmd;
	sfc_message_add_tail(&transfer, &message);

	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}

	chip_id =  chip_id & 0x00ffffff;
	chip_id = ((chip_id & 0xff) << 16) | (((chip_id >> 8) & 0xff) << 8) | ((chip_id >> 16) & 0xff);
	return chip_id;
}

static unsigned int sfc_do_read(struct sfc_flash *flash, unsigned int addr, unsigned char *buf, size_t len)
{
	unsigned char command;
	int dummy_byte;
	int addr_size;
	int transfer_mode;
	struct sfc_transfer transfer;
	struct sfc_message message;
	struct cmd_info cmd;
	int ret;

	command = flash->cur_r_cmd->cmd;
	dummy_byte = flash->cur_r_cmd->dummy_byte;
	addr_size = flash->cur_r_cmd->addr_nbyte;
	transfer_mode = flash->cur_r_cmd->transfer_mode;

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	cmd.cmd = command;
	cmd.dataen = ENABLE;
	transfer.addr_len = addr_size;
	transfer.data_dummy_bits = dummy_byte;
	transfer.addr = addr;
	transfer.len = len;
	transfer.data = buf;
	transfer.cur_len = 0;
	if (len >= L2CACHE_ALIGN_SIZE)
		transfer.ops_mode = DMA_OPS;
	else
		transfer.ops_mode = CPU_OPS;
	transfer.sfc_mode = transfer_mode;
	transfer.direction = GLB_TRAN_DIR_READ;
	transfer.cmd_info = &cmd;
	sfc_message_add_tail(&transfer, &message);

	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}
	/*fix the cache line problem,when use jffs2 filesystem must be flush cache twice*/
	if(transfer.ops_mode == DMA_OPS)
		dma_cache_sync(NULL, (void *)buf, len, DMA_FROM_DEVICE);

	return message.actual_length;
}
static unsigned  int sfc_do_write(struct sfc_flash *flash, unsigned int addr, const unsigned char *buf, size_t len)
{
	unsigned char command;
	int dummy_byte;
	int transfer_mode;
	int addr_size;
	struct sfc_transfer transfer[3];
	struct sfc_message message;
	struct cmd_info cmd[3];
	int ret;
	struct spi_nor_info *spi_nor_info;
	struct spi_nor_cmd_info *wr_en;
	struct spi_nor_st_info *busy;

	spi_nor_info = flash->g_nor_info;
	wr_en = &spi_nor_info->wr_en;
	busy = &spi_nor_info->busy;

	command = flash->cur_w_cmd->cmd;
	dummy_byte = flash->cur_w_cmd->dummy_byte;
	addr_size = flash->cur_w_cmd->addr_nbyte;
	transfer_mode = flash->cur_w_cmd->transfer_mode;

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	/* write enable */
	cmd[0].cmd = wr_en->cmd;
	cmd[0].dataen = DISABLE;
	transfer[0].cmd_info = &cmd[0];
	transfer[0].addr_len = wr_en->addr_nbyte;
	transfer[0].sfc_mode = transfer_mode;
	transfer[0].data_dummy_bits = wr_en->dummy_byte;
	sfc_message_add_tail(&transfer[0], &message);

	/* write ops */
	cmd[1].cmd = command;
	cmd[1].dataen = ENABLE;
	transfer[1].addr = addr;
	transfer[1].addr_len = addr_size;
	transfer[1].len = len;
	transfer[1].cur_len = 0;
	transfer[1].data_dummy_bits = dummy_byte;
	transfer[1].data = buf;
	if(len >= L2CACHE_ALIGN_SIZE)
		transfer[1].ops_mode = DMA_OPS;
	else
		transfer[1].ops_mode = CPU_OPS;
	transfer[1].sfc_mode = transfer_mode;
	transfer[1].direction = GLB_TRAN_DIR_WRITE;
	transfer[1].cmd_info = &cmd[1];
	sfc_message_add_tail(&transfer[1], &message);

	cmd[2].cmd = busy->cmd;
	cmd[2].dataen = DISABLE;
	cmd[2].sta_exp = busy->val << busy->bit_shift;
	cmd[2].sta_msk = busy->mask << busy->bit_shift;
	transfer[2].cmd_info = &cmd[2];
	sfc_message_add_tail(&transfer[2], &message);

	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}

	return message.actual_length;
}

static int sfc_do_erase(struct sfc_flash *flash, uint32_t addr)
{
	struct sfc_transfer transfer[3];
	struct sfc_message message;
	struct cmd_info cmd[3];
	int ret;

	struct spi_nor_info *spi_nor_info;
	struct spi_nor_cmd_info *sector_erase;
	struct spi_nor_cmd_info *wr_en;
	struct spi_nor_st_info *busy;
	int addr_size;


	spi_nor_info = flash->g_nor_info;
	sector_erase = &spi_nor_info->sector_erase;
	wr_en = &spi_nor_info->wr_en;
	busy = &spi_nor_info->busy;
	addr_size = sector_erase->addr_nbyte;

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	/* write enable */
	cmd[0].cmd = wr_en->cmd;
	cmd[0].dataen = DISABLE;
	transfer[0].cmd_info = &cmd[0];
	transfer[0].addr_len = wr_en->addr_nbyte;
	transfer[0].sfc_mode = wr_en->transfer_mode;
	transfer[0].data_dummy_bits = wr_en->dummy_byte;
	sfc_message_add_tail(&transfer[0], &message);

	/* erase ops */
	cmd[1].cmd = sector_erase->cmd;
	cmd[1].dataen = DISABLE;
	transfer[1].addr_len = addr_size;
	transfer[1].data_dummy_bits = sector_erase->dummy_byte;
	transfer[1].addr = addr;
	transfer[1].sfc_mode = TM_STD_SPI;
	transfer[1].direction = GLB_TRAN_DIR_WRITE;
	transfer[1].cmd_info = &cmd[1];
	sfc_message_add_tail(&transfer[1], &message);

	cmd[2].cmd = busy->cmd;
	cmd[2].dataen = DISABLE;
	cmd[2].sta_exp = busy->val << busy->bit_shift;
	cmd[2].sta_msk = busy->mask << busy->bit_shift;
	transfer[2].cmd_info = &cmd[2];
	sfc_message_add_tail(&transfer[2], &message);

	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}

	return ret;
}

static int sfc_write(struct sfc_flash *flash, loff_t to, size_t len, const unsigned char *buf)
{
	unsigned int s_len = 0, f_len = 0, a_len = 0;

	if (len > L2CACHE_ALIGN_SIZE) {
		s_len = ALIGN((unsigned int )buf, L2CACHE_ALIGN_SIZE) - (unsigned int)buf;
		if (s_len) {
			sfc_do_write(flash, (unsigned int)to, buf, s_len);
		}

		a_len = (len - s_len) - (len - s_len) % L2CACHE_ALIGN_SIZE;
		if (a_len) {
			sfc_do_write(flash, (unsigned int)to + s_len , &buf[s_len], a_len);
		}

		f_len = len - s_len - a_len;
		if (f_len) {
			sfc_do_write(flash, (unsigned int)to + s_len + a_len , &buf[s_len + a_len], f_len);
		}
	} else {
		sfc_do_write(flash, (unsigned int)to, buf, len);
	}

	return len;
}


static int sfc_read_cacheline_align(struct sfc_flash *flash, unsigned int addr, unsigned char *buf, size_t len)
{
	unsigned int ret = 0;
	unsigned int s_len = 0, f_len = 0, a_len = 0;

	/**
	 * s_len : start not align length
	 * a_len : middle align length
	 * f_len : end not align length
	 */
	if (len > L2CACHE_ALIGN_SIZE) {
		s_len = ALIGN((unsigned int )buf, L2CACHE_ALIGN_SIZE) - (unsigned int)buf;
		if (s_len) {
			ret +=  sfc_do_read(flash, (unsigned int)addr, buf, s_len);
		}

		a_len = (len - s_len) - (len - s_len) % L2CACHE_ALIGN_SIZE;
		if (a_len) {
			ret +=  sfc_do_read(flash, (unsigned int)addr + s_len , &buf[s_len], a_len);
		}

		f_len = len - s_len - a_len;
		if (f_len) {
			ret +=  sfc_do_read(flash, (unsigned int)addr + s_len + a_len , &buf[s_len + a_len], f_len);
		}
	} else {
		ret = sfc_do_read(flash, (unsigned int)addr, buf, len);
	}

	return ret;
}


static unsigned int sfc_read_continue(struct sfc_flash *flash, unsigned int addr, unsigned char *buf, size_t len)
{
	unsigned char *vaddr_start = NULL;
	unsigned char *vaddr_next = NULL;
	int pfn = 0, pfn_next = 0;
	int off_len = 0, transfer_len = 0;
	int page_num = 0, page_offset = 0;
	int continue_times = 0;
	int ret = 0;

	if (is_vmalloc_addr(buf)) {
		vaddr_start = buf;
		pfn = vmalloc_to_pfn(vaddr_start);
		page_offset = (unsigned int)vaddr_start & 0xfff;

		off_len = PAGE_SIZE - page_offset;

		if (off_len >= len) {		//all in one page
			ret = sfc_read_cacheline_align(flash, addr, buf, len);
			return off_len;
		} else {
			page_num = (len - off_len) / PAGE_SIZE;		//more than one page
		}

		vaddr_next = vaddr_start + off_len;
		while (page_num) {		//find continue page
			pfn_next = vmalloc_to_pfn((unsigned char *)((unsigned int)vaddr_next));
			if (pfn + 1 == pfn_next) {
				page_num --;
				continue_times++;
				pfn++;
				vaddr_next += PAGE_SIZE;
			} else {
				break;
			}
		}

		transfer_len = PAGE_SIZE * continue_times + off_len;
		ret += sfc_read_cacheline_align(flash, addr, buf, transfer_len);	//transfer continue page

		if (page_num == 0) {		//last not continue page
			if(transfer_len < len) {
				ret += sfc_read_cacheline_align(flash, addr + transfer_len, &buf[transfer_len], len - transfer_len);
			}
		}
	} else {
		ret = sfc_read_cacheline_align(flash, addr, buf, len);
	}

	return ret;

}

static int sfc_read(struct sfc_flash *flash, loff_t from, size_t len, unsigned char *buf)
{
	int tmp_len = 0, current_len = 0;

	while (len) {
		tmp_len = sfc_read_continue(flash, (unsigned int)from + current_len, &buf[current_len], len);
		current_len += tmp_len;
		len -= tmp_len;
	}

	return current_len;
}

static int jz_spi_norflash_read(struct mtd_info *mtd, loff_t from, size_t len,size_t *retlen, unsigned char *buf)
{
	struct sfc_flash *flash;
	flash = to_jz_spi_norflash(mtd);

	mutex_lock(&flash->lock);
	*retlen = sfc_read(flash, from, len, buf);
	mutex_unlock(&flash->lock);

	return 0;
}

static int jz_spi_norflash_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const unsigned char *buf)
{
	u32 page_offset, actual_len;
	struct sfc_flash *flash;
	int ret;
	struct spi_nor_info *spi_nor_info;

	flash = to_jz_spi_norflash(mtd);
	spi_nor_info = flash->g_nor_info;

	mutex_lock(&flash->lock);

	page_offset = to & (spi_nor_info->page_size - 1);
	/* do all the bytes fit onto one page? */
	if (page_offset + len <= spi_nor_info->page_size) {
		ret = sfc_write(flash, to, len, buf);
		*retlen = ret;
	} else {
		u32 i;

		/* the size of data remaining on the first page */
		actual_len = spi_nor_info->page_size - page_offset;
		ret = sfc_write(flash, to, actual_len, buf);
		*retlen += ret;

		/* write everything in flash->page_size chunks */
		for (i = actual_len; i < len; i += mtd->writesize) {
			actual_len = len - i;
			if (actual_len >= mtd->writesize)
				actual_len = mtd->writesize;

			ret = sfc_write(flash, to + i, actual_len, buf + i);
			*retlen += ret;
		}
	}
	mutex_unlock(&flash->lock);
	return 0;
}


static int jz_spi_norflash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int ret;
	uint32_t addr, end;
	struct sfc_flash *flash;
	struct spi_nor_info *spi_nor_info;

	flash = to_jz_spi_norflash(mtd);
	spi_nor_info = flash->g_nor_info;
	mutex_lock(&flash->lock);

	addr = (instr->addr & (mtd->erasesize - 1));
	if (addr) {
		dev_err(flash->dev, "%s eraseaddr no align\n", __func__);
		mutex_unlock(&flash->lock);
		return -EINVAL;
	}
	end = (instr->len & (mtd->erasesize - 1));
	if (end) {
		dev_err(flash->dev,"%s erasesize no align\n", __func__);
		mutex_unlock(&flash->lock);
		return -EINVAL;
	}
	addr = (uint32_t)instr->addr;
	end = addr + (uint32_t)instr->len;

	while (addr < end) {
		ret = sfc_do_erase(flash, addr);
		if (ret) {
			dev_err(flash->dev,"erase error !\n");
			mutex_unlock(&flash->lock);
			instr->state = MTD_ERASE_FAILED;
			return ret;
		}
		addr += spi_nor_info->erase_size;
	}
	mutex_unlock(&flash->lock);
	instr->state = MTD_ERASE_DONE;

	mtd_erase_callback(instr);
	return 0;
}

static int get_current_operate_cmd(struct sfc_flash *flash)
{

	struct spi_nor_info *spi_nor_info;
	struct spi_nor_cmd_info *read_standard;
	struct spi_nor_cmd_info *read_quad;
	struct spi_nor_cmd_info *write_standard;
	struct spi_nor_cmd_info *write_quad;

	spi_nor_info = flash->g_nor_info;
	read_standard = &spi_nor_info->read_standard;
	read_quad = &spi_nor_info->read_quad;
	write_standard = &spi_nor_info->write_standard;
	write_quad = &spi_nor_info->write_quad;

	if (flash->quad_succeed) {
		flash->cur_r_cmd = read_quad;
		flash->cur_w_cmd = write_quad;
	} else {
		flash->cur_r_cmd = read_standard;
		flash->cur_w_cmd = write_standard;
	}

	return 0;
}

static int jz_spi_norflash_read_params(struct sfc_flash *flash, loff_t from, size_t len, unsigned char *buf)
{
	struct sfc_transfer transfer;
	struct sfc_message message;
	struct cmd_info cmd;
	unsigned char command;
	int dummy_byte = 0;
	int ret = 0;

	command = SPINOR_OP_READ;
	mutex_lock(&flash->lock);

	sfc_message_init(&message);
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));

	cmd.cmd = command;
	cmd.dataen = ENABLE;
	transfer.addr = (unsigned int)from;
	transfer.len = len;
	transfer.data = buf;
	transfer.addr_len = DEFAULT_ADDRSIZE;
	transfer.data_dummy_bits = dummy_byte;
	transfer.ops_mode = CPU_OPS;
	transfer.direction = GLB_TRAN_DIR_READ;
	transfer.sfc_mode = TM_STD_SPI;
	transfer.cmd_info = &cmd;
	sfc_message_add_tail(&transfer, &message);

	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}

	mutex_unlock(&flash->lock);
	return ret;
}

#ifdef DEBUG_CLONER_PARAMS
static void dump_cloner_params(struct burner_params *params)
{
	struct spi_nor_info *spi_nor_info;

	spi_nor_info = &params->spi_nor_info;

	printk("name=%s\n", spi_nor_info->name);
	printk("id=0x%x\n", spi_nor_info->id);

	printk("read_standard->cmd=0x%x\n",		spi_nor_info->read_standard.cmd);
	printk("read_standard->dummy=0x%x\n",		spi_nor_info->read_standard.dummy_byte);
	printk("read_standard->addr_nbyte=0x%x\n",	spi_nor_info->read_standard.addr_nbyte);
	printk("read_standard->transfer_mode=0x%x\n",	spi_nor_info->read_standard.transfer_mode);

	printk("read_quad->cmd=0x%x\n",			spi_nor_info->read_quad.cmd);
	printk("read_quad->dummy=0x%x\n",		spi_nor_info->read_quad.dummy_byte);
	printk("read_quad->addr_nbyte=0x%x\n",		spi_nor_info->read_quad.addr_nbyte);
	printk("read_quad->transfer_mode=0x%x\n",	spi_nor_info->read_quad.transfer_mode);

	printk("write_standard->cmd=0x%x\n",		spi_nor_info->write_standard.cmd);
	printk("write_standard->dummy=0x%x\n",		spi_nor_info->write_standard.dummy_byte);
	printk("write_standard->addr_nbyte=0x%x\n",	spi_nor_info->write_standard.addr_nbyte);
	printk("write_standard->transfer_mode=0x%x\n",	spi_nor_info->write_standard.transfer_mode);

	printk("write_quad->cmd=0x%x\n",		spi_nor_info->write_quad.cmd);
	printk("write_quad->dummy=0x%x\n",		spi_nor_info->write_quad.dummy_byte);
	printk("write_quad->addr_nbyte=0x%x\n",		spi_nor_info->write_quad.addr_nbyte);
	printk("write_quad->transfer_mode=0x%x\n",	spi_nor_info->write_quad.transfer_mode);

	printk("sector_erase->cmd=0x%x\n",		spi_nor_info->sector_erase.cmd);
	printk("sector_erase->dummy=0x%x\n",		spi_nor_info->sector_erase.dummy_byte);
	printk("sector_erase->addr_nbyte=0x%x\n",	spi_nor_info->sector_erase.addr_nbyte);
	printk("sector_erase->transfer_mode=0x%x\n",	spi_nor_info->sector_erase.transfer_mode);

	printk("wr_en->cmd=0x%x\n",		spi_nor_info->wr_en.cmd);
	printk("wr_en->dummy=0x%x\n",		spi_nor_info->wr_en.dummy_byte);
	printk("wr_en->addr_nbyte=0x%x\n",	spi_nor_info->wr_en.addr_nbyte);
	printk("wr_en->transfer_mode=0x%x\n",	spi_nor_info->wr_en.transfer_mode);

	printk("en4byte->cmd=0x%x\n",		spi_nor_info->en4byte.cmd);
	printk("en4byte->dummy=0x%x\n",		spi_nor_info->en4byte.dummy_byte);
	printk("en4byte->addr_nbyte=0x%x\n",	spi_nor_info->en4byte.addr_nbyte);
	printk("en4byte->transfer_mode=0x%x\n",	spi_nor_info->en4byte.transfer_mode);

	printk("quad_set->cmd=0x%x\n",		spi_nor_info->quad_set.cmd);
	printk("quad_set->bit_shift=0x%x\n",		spi_nor_info->quad_set.bit_shift);
	printk("quad_set->mask=0x%x\n",		spi_nor_info->quad_set.mask);
	printk("quad_set->val=0x%x\n",		spi_nor_info->quad_set.val);
	printk("quad_set->len=0x%x\n",		spi_nor_info->quad_set.len);
	printk("quad_set->dummy=0x%x\n",	spi_nor_info->quad_set.dummy);

	printk("quad_get->cmd=0x%x\n",		spi_nor_info->quad_get.cmd);
	printk("quad_get->bit_shift=0x%x\n",		spi_nor_info->quad_get.bit_shift);
	printk("quad_get->mask=0x%x\n",		spi_nor_info->quad_get.mask);
	printk("quad_get->val=0x%x\n",		spi_nor_info->quad_get.val);
	printk("quad_get->len=0x%x\n",		spi_nor_info->quad_get.len);
	printk("quad_get->dummy=0x%x\n",	spi_nor_info->quad_get.dummy);

	printk("busy->cmd=0x%x\n",		spi_nor_info->busy.cmd);
	printk("busy->bit_shift=0x%x\n",		spi_nor_info->busy.bit_shift);
	printk("busy->mask=0x%x\n",		spi_nor_info->busy.mask);
	printk("busy->val=0x%x\n",		spi_nor_info->busy.val);
	printk("busy->len=0x%x\n",		spi_nor_info->busy.len);
	printk("busy->dummy=0x%x\n",		spi_nor_info->busy.dummy);

	printk("quad_ops_mode=%d\n",	spi_nor_info->quad_ops_mode);
	printk("addr_ops_mode=%d\n",	spi_nor_info->addr_ops_mode);

	printk("tCHSH=%d\n",	spi_nor_info->tCHSH);
	printk("tSLCH=%d\n",	spi_nor_info->tSLCH);
	printk("tSHSL_RD=%d\n", spi_nor_info->tSHSL_RD);
	printk("tSHSL_WR=%d\n", spi_nor_info->tSHSL_WR);

	printk("chip_size=%d\n",	spi_nor_info->chip_size);
	printk("page_size=%d\n",	spi_nor_info->page_size);
	printk("erase_size=%d\n",	spi_nor_info->erase_size);

	printk("chip_erase_cmd=0x%x\n",	spi_nor_info->chip_erase_cmd);
}
#endif


static int jz_spi_norflash_match_device(struct sfc_flash *flash)
{
	int ret;
	int err = 0;

	flash->params = (struct burner_params*)kzalloc(sizeof(struct burner_params), GFP_KERNEL);
	if (flash->params == NULL) {
		dev_err(flash->dev, "Failed to alloc mem for params\n");
		err = -ENOMEM;
		goto err_params;
	}

	ret = jz_spi_norflash_read_params(flash, SPIFLASH_PARAMER_OFFSET, sizeof(struct burner_params), (unsigned char *)flash->params);
	if (ret) {
		dev_err(flash->dev, "Failed to read params (burned by Burner)\n");
		err = -EINVAL;
		goto err_read_params;
	}
	dev_info(flash->dev, "magic is 0x%x  version is 0x%x\n", flash->params->magic, flash->params->version);
	if (flash->params->magic == NOR_MAGIC) {
		if (flash->params->version == NOR_VERSION) {
			flash->g_nor_info = &flash->params->spi_nor_info;
			flash->norflash_partitions = &flash->params->norflash_partitions;
		} else {
			flash->g_nor_info = NULL;
			dev_err(flash->dev,"norflash version miss match,the current version is %d.%d.%d,but the burner version is %d.%d.%d\n" \
					,NOR_MAJOR_VERSION_NUMBER, NOR_MINOR_VERSION_NUMBER, NOR_REVERSION_NUMBER \
					,flash->params->version & 0xff, (flash->params->version & 0xff00) >> 8, (flash->params->version & 0xff0000) >> 16);

			err = -EINVAL;
			goto err_version;
		}
	} else {
		flash->g_nor_info = NULL;
		dev_err(flash->dev, "xxxxxxxxx warning : can not read params from flash ! xxxxxxxxxxx\n");
		err = -EINVAL;
		goto err_magic;

	}
#ifdef DEBUG_CLONER_PARAMS
	dump_cloner_params(flash->params);
#endif
	return 0;
err_version:
err_magic:
err_read_params:
	kfree(flash->params);
err_params:
	return err;
}
static void jz_spi_norflash_remove_device(struct sfc_flash *flash)
{
	if(flash->params)
		kfree(flash->params);
}

static ssize_t sfc_nor_partition_offset_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf,"0x%x\n", SPIFLASH_PARAMER_OFFSET + sizeof(int) * 2 + sizeof(struct spi_nor_info));
}

static DEVICE_ATTR(sfc_nor_partition_offset, S_IRUGO | S_IWUSR,
		sfc_nor_partition_offset_show,
		NULL);

static ssize_t sfc_nor_params_offset_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf,"0x%x\n",SPIFLASH_PARAMER_OFFSET);
}

static DEVICE_ATTR(sfc_nor_params_offset, S_IRUGO | S_IWUSR,
		sfc_nor_params_offset_show,
		NULL);

/*add your attr in here*/
static struct attribute *sfc_norflash_info_attributes[] = {
	&dev_attr_sfc_nor_partition_offset.attr,
	&dev_attr_sfc_nor_params_offset.attr,
	NULL
};

static const struct attribute_group sfc_norflash_info_attr_group = {
	.attrs = sfc_norflash_info_attributes
};


static int __init jz_sfc_probe(struct platform_device *pdev)
{
	struct sfc_flash *flash;
	struct mtd_partition *jz_mtd_partition;
	const char *jz_probe_types[] = {"cmdlinepart",NULL};
	int num_partition_info = 0;
	int err = 0,ret = 0;
	struct spi_nor_info *spi_nor_info;
	int i;
	int tchsh;
	int tslch;
	int tshsl_rd;
	int tshsl_wr;

	L2CACHE_ALIGN_SIZE = 128;

	flash = kzalloc(sizeof(struct sfc_flash), GFP_KERNEL);
	if (flash == NULL) {
		dev_err(&pdev->dev, "Failed to alloc mem for flash\n");
		return -ENOMEM;
	}

	flash->dev = &pdev->dev;

	flash->sfc = sfc_res_init(pdev);
	if(IS_ERR_OR_NULL(flash->sfc)) {
		ret = -ENOMEM;
		goto err_sfc_res_init;
	}

	platform_set_drvdata(pdev, flash);
	spin_lock_init(&flash->lock_status);
	mutex_init(&flash->lock);

	ret = sfc_nor_reset(flash);
	if(ret) {
		dev_warn(flash->dev, "Failed to reset nor flash, Try to go on\n");
	}

	set_flash_timing(flash->sfc, DEF_TCHSH, DEF_TSLCH, DEF_TSHSL_R, DEF_TSHSL_W);
	ret = jz_spi_norflash_match_device(flash);
	if (ret) {
		ret = -ENODEV;
		dev_err(flash->dev, "Failed to match correct nor flash device!\n");
		goto err_match_device;
	}

	spi_nor_info = flash->g_nor_info;

	flash->mtd.name     = "sfc_mtd";
	flash->mtd.owner    = THIS_MODULE;
	flash->mtd.type     = MTD_NORFLASH;
	flash->mtd.flags    = MTD_CAP_NORFLASH;
	flash->mtd.erasesize    = flash->params->fs_erase_size;
	flash->mtd.writesize    = spi_nor_info->page_size;
	flash->mtd.size     = spi_nor_info->chip_size;
	flash->mtd._erase   = jz_spi_norflash_erase;
	flash->mtd._read    = jz_spi_norflash_read;
	flash->mtd._write   = jz_spi_norflash_write;


	num_partition_info = flash->norflash_partitions->num_partition_info;
	jz_mtd_partition = (struct mtd_partition*)kzalloc(sizeof(struct mtd_partition) * num_partition_info,GFP_KERNEL);
	if (!jz_mtd_partition) {
		ret = -ENOMEM;
		dev_err(flash->dev, "Failed to alloc mem for jz_mtd_partition\n");
		goto err_alloc_partition;
	}


	for (i = 0; i < num_partition_info; i++) {
		jz_mtd_partition[i].ecclayout = NULL;
		jz_mtd_partition[i].name = flash->norflash_partitions->nor_partition[i].name;
		jz_mtd_partition[i].offset = flash->norflash_partitions->nor_partition[i].offset;

		if (flash->norflash_partitions->nor_partition[i].size == -1) {
			jz_mtd_partition[i].size = MTDPART_SIZ_FULL;
		} else {
			jz_mtd_partition[i].size = flash->norflash_partitions->nor_partition[i].size;
		}

		if (flash->norflash_partitions->nor_partition[i].mask_flags & NORFLASH_PART_RO) {
			jz_mtd_partition[i].mask_flags = MTD_CAP_RAM;
		} else {
			jz_mtd_partition[i].mask_flags = MTD_CAP_ROM;
		}

	}

	tchsh = spi_nor_info->tCHSH;
	tslch = spi_nor_info->tSLCH;
	tshsl_rd = spi_nor_info->tSHSL_RD;
	tshsl_wr = spi_nor_info->tSHSL_WR;
	set_flash_timing(flash->sfc, tchsh, tslch, tshsl_rd, tshsl_wr);

	sfc_nor_get_special_ops(flash);

	if (flash->params->uk_quad) {
		if (flash->nor_flash_ops->set_quad_mode) {
			ret = flash->nor_flash_ops->set_quad_mode(flash);
			if (ret < 0) {
				flash->quad_succeed = 0;
				dev_info(&pdev->dev, "set quad mode error !\n");
			} else {
				flash->quad_succeed = 1;
				dev_info(&pdev->dev, "nor flash quad mode is set, now use quad mode!\n");
			}
		}
	}


	/* if nor flash size is greater than 16M, use 4byte mode */
	if(flash->mtd.size > NOR_SIZE_16M) {
		if (flash->nor_flash_ops->set_4byte_mode) {
			flash->nor_flash_ops->set_4byte_mode(flash);
		}
	}

	get_current_operate_cmd(flash);

	ret = mtd_device_parse_register(&flash->mtd, jz_probe_types, NULL, jz_mtd_partition, num_partition_info);
	if (ret) {
		ret = -ENODEV;
		dev_err(flash->dev, "Failed to parse register!\n");
		goto err_parse_register;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &sfc_norflash_info_attr_group);
	if (err) {
		dev_err(&pdev->dev, "failed to register sysfs\n");
		ret = -EIO;
		goto err_create_group;
	}

	dev_info(&pdev->dev,"SPI NOR MTD LOAD OK\n");
	return 0;

err_create_group:
	mtd_device_unregister(&flash->mtd);
err_parse_register:
	kfree(jz_mtd_partition);
err_alloc_partition:
	jz_spi_norflash_remove_device(flash);
err_match_device:
	sfc_res_deinit(flash->sfc);
err_sfc_res_init:
	kfree(flash);
	return ret;

}

static int __exit jz_sfc_remove(struct platform_device *pdev)
{
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;


	clk_disable(sfc->clk_gate);
	clk_put(sfc->clk_gate);

	clk_disable(sfc->clk);
	clk_put(sfc->clk);

	free_irq(sfc->irq, flash);

	iounmap(sfc->iomem);

	release_mem_region(flash->resource->start, resource_size(flash->resource));

	platform_set_drvdata(pdev, NULL);

	sysfs_remove_group(&pdev->dev.kobj, &sfc_norflash_info_attr_group);
	return 0;
}

static int jz_sfc_suspend(struct platform_device *pdev, pm_message_t msg)
{
	unsigned long flags;
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	spin_lock_irqsave(&flash->lock_status, flags);
	flash->status |= STATUS_SUSPND;
	disable_irq(sfc->irq);
	spin_unlock_irqrestore(&flash->lock_status, flags);

	clk_disable(sfc->clk_gate);

	clk_disable(sfc->clk);

	return 0;
}

static int jz_sfc_resume(struct platform_device *pdev)
{
	unsigned long flags;
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	clk_enable(sfc->clk);

	clk_enable(sfc->clk_gate);

	spin_lock_irqsave(&flash->lock_status, flags);
	flash->status &= ~STATUS_SUSPND;
	enable_irq(sfc->irq);
	spin_unlock_irqrestore(&flash->lock_status, flags);

	return 0;
}

void jz_sfc_shutdown(struct platform_device *pdev)
{
	unsigned long flags;
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	spin_lock_irqsave(&flash->lock_status, flags);
	flash->status |= STATUS_SUSPND;
	disable_irq(sfc->irq);
	spin_unlock_irqrestore(&flash->lock_status, flags);

	clk_disable(sfc->clk_gate);

	clk_disable(sfc->clk);

	return ;
}

static struct platform_driver jz_sfcdrv = {
	.driver		= {
		.name	= "jz-sfc",
		.owner	= THIS_MODULE,
	},
	.remove		= jz_sfc_remove,
	.suspend	= jz_sfc_suspend,
	.resume		= jz_sfc_resume,
//	.shutdown	= jz_sfc_shutdown,
};

static int __init jz_sfc_init(void)
{
	return platform_driver_probe(&jz_sfcdrv, jz_sfc_probe);
}

static void __exit jz_sfc_exit(void)
{
    platform_driver_unregister(&jz_sfcdrv);
}

module_init(jz_sfc_init);
module_exit(jz_sfc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JZ SFC Driver");
