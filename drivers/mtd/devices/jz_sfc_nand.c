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
#include <linux/mtd/nand.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include"./jz_sfc_nand.h"
#include "sfc_nand.h"

#define CONFIG_SPI_QUAD

#define STATUS_SUSPND	(1<<0)
#define to_jz_spi_nand(mtd_info) container_of(mtd_info, struct sfc_flash, mtd)
#define	tCHSH	5	//hold
#define tSLCH	5	//setup
#define tSHSL_RD	20	//interval
#define tSHSL_WR	50
/*
 * below is the informtion about nand
 * that user should modify according to nand spec
 * */
int column_cmdaddr_bits = 0;
/*nand time delay information*/
int tRD = 0;
int tPROG = 0;
int tBERS = 0;
/*nand ecc informtion*/
int ecc_shift = 4;
char ecc_mask = 0x3;//how many bits ecc has (ECCS1,ECCS0)
int ecc_max = 0x3;//ECC capability(status reg's value: ECCS1,ECCS0)
int EccErrValue = 0x1;// the status value when ecc err(Value : ECCS1,ECCS0)

static unsigned int id_table[2] = {0xc8b148, 0xc212};
int manufacturer_id = 0;

#define MANU_GD 0xc8
#define MANU_MX 0xc2


static int jz_sfc_nand_set_feature(struct sfc_flash *flash, unsigned char addr, unsigned char val)
{
	int ret;
	struct sfc_transfer transfer;
	struct sfc_message message;
	struct cmd_info cmd;
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));
	sfc_message_init(&message);
	cmd.cmd = SPINAND_CMD_SET_FEATURE;

	transfer.addr = addr;
	transfer.addr_len = 1;
	transfer.addr_dummy_bits = 0;

	cmd.dataen = ENABLE;
	transfer.data = &val;
	transfer.len = 1;
	transfer.direction = GLB_TRAN_DIR_WRITE;
	transfer.data_dummy_bits = 0;

	transfer.cmd_info = &cmd;
	transfer.ops_mode = CPU_OPS;
	transfer.sfc_mode = TM_STD_SPI;
	sfc_message_add_tail(&transfer, &message);
	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}
	return ret;
}

static int jz_sfc_nand_get_feature(struct sfc_flash *flash, unsigned char addr, unsigned char *val)
{
	int ret;
	struct sfc_transfer transfer;
	struct sfc_message message;
	struct cmd_info cmd;
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));
	sfc_message_init(&message);
	cmd.cmd = SPINAND_CMD_GET_FEATURE;

	transfer.addr = addr;
	transfer.addr_len = 1;
	transfer.addr_dummy_bits = 0;

	cmd.dataen = ENABLE;
	transfer.data = val;
	transfer.len = 1;
	transfer.direction = GLB_TRAN_DIR_READ;
	transfer.data_dummy_bits = 0;

	transfer.cmd_info = &cmd;
	transfer.ops_mode = CPU_OPS;
	transfer.sfc_mode = TM_STD_SPI;
	sfc_message_add_tail(&transfer, &message);
	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
	}
#if 0
	printk("NAND:feacture addr:%x value:%x\n",addr, *feacture);
#endif
	return ret;
}

static int jz_sfc_nand_read(struct sfc_flash *flash, int page, int column, char* buffer, size_t len, u8 *status)
{
	int ret;
	struct sfc_transfer transfer[3];
	struct sfc_message message;
	struct cmd_info cmd[3];

	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));
	sfc_message_init(&message);
	cmd[0].cmd = SPINAND_CMD_PARD;
	transfer[0].addr = page;
	transfer[0].addr_len = 3;
	cmd[0].dataen = DISABLE;
	transfer[0].data_dummy_bits = 0;
	transfer[0].cmd_info = &cmd[0];
	transfer[0].ops_mode = CPU_OPS;
	transfer[0].sfc_mode = TM_STD_SPI;
	sfc_message_add_tail(&transfer[0], &message);
	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}
	udelay(tRD);

	cmd[1].cmd = SPINAND_CMD_GET_FEATURE;
	transfer[1].addr = SPINAND_ADDR_STATUS;
	transfer[1].addr_len = 1;
	transfer[1].addr_dummy_bits = 0;
	cmd[1].dataen = DISABLE;
	transfer[1].data_dummy_bits = 0;
	cmd[1].sta_exp = (0 << 0);
	cmd[1].sta_msk = SPINAND_IS_BUSY;
	transfer[1].cmd_info = &cmd[1];
	transfer[1].ops_mode = CPU_OPS;
	transfer[1].sfc_mode = TM_STD_SPI;
	sfc_message_add_tail(&transfer[1], &message);
	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	*status = (u8)sfc_get_sta_rt(flash->sfc);
	*status = ((*status)>>ecc_shift)&(ecc_mask);
	if((*status) == (EccErrValue)) {
		pr_info("spi nand read error page %d ret = %02x !!! %s %s %d \n",page,ret,__FILE__,__func__,__LINE__);
		*status = -EBADMSG;
	}


#ifdef CONFIG_SPI_QUAD
	cmd[2].cmd = SPINAND_CMD_RDCH_X4;
	if (manufacturer_id == MANU_MX) {
		transfer[2].addr_len = 2;	//mx
		transfer[2].data_dummy_bits = 8;
	} else if (manufacturer_id == MANU_GD) {
		transfer[2].addr_len = 3;	//gd
		transfer[2].data_dummy_bits = 8;
	} else {
		printk("unsupport nand flash\n");
	}

#else
	cmd[2].cmd = SPINAND_CMD_RDCH;
	if (manufacturer_id == MANU_MX) {
		transfer[2].addr_len = 2;	//mx
		transfer[2].data_dummy_bits = 8;
	} else if (manufacturer_id == MANU_GD) {
		if(column_cmdaddr_bits == 24)
			transfer[2].addr_len = 2;
		else if(column_cmdaddr_bits == 32) {
			transfer[2].addr_len = 3;	//gd
			transfer[2].data_dummy_bits = 0;
		} else {
			printk("unsupport nand flash\n");
		}
	}
#endif
	transfer[2].addr = column;
	cmd[2].dataen = ENABLE;
	transfer[2].data = buffer;
	transfer[2].len = len;
	transfer[2].direction = GLB_TRAN_DIR_READ;
	transfer[2].cmd_info = &cmd[2];
	transfer[2].ops_mode = DMA_OPS;
#ifdef CONFIG_SPI_QUAD
	transfer[2].sfc_mode = TM_QI_QO_SPI;
#else
	transfer[2].sfc_mode = TM_STD_SPI;
#endif
	sfc_message_add_tail(&transfer[2], &message);
	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}
	return 0;
}


static void convert_burner_to_driver_use(struct jz_spi_support *spinand,struct jz_spi_support_from_burner *burner,int param_num)
{
	int i=0;
	for(i=0;i<param_num;i++){
		memcpy(spinand[i].name,burner[i].name,SIZEOF_NAME);
		spinand[i].page_size=burner[i].page_size;
		spinand[i].oobsize=burner[i].oobsize;
		spinand[i].sector_size=burner[i].sector_size;
		spinand[i].block_size=burner[i].block_size;
		spinand[i].size=burner[i].size;
		spinand[i].page_num=burner[i].page_num;
		spinand[i].tRD_maxbusy=burner[i].tRD_maxbusy;
		spinand[i].tPROG_maxbusy=burner[i].tPROG_maxbusy;
		spinand[i].tBERS_maxbusy=burner[i].tBERS_maxbusy;
		spinand[i].column_cmdaddr_bits=burner[i].column_cmdaddr_bits;
	}
}

static int transfer_to_mtddriver_struct(struct get_chip_param *param,struct jz_spi_nand_platform_data **change)
{
	int i=0;
	*change=kzalloc(sizeof(struct jz_spi_nand_platform_data),GFP_KERNEL);
	if(!*change)
		return -ENOMEM;
	(*change)->num_spi_flash=param->para_num;
	(*change)->jz_spi_support=kzalloc(param->para_num*sizeof(struct jz_spi_support),GFP_KERNEL);
	if(!(*change)->jz_spi_support)
		return -ENOMEM;
	memcpy((*change)->jz_spi_support,param->addr,param->para_num*sizeof(struct jz_spi_support));
	convert_burner_to_driver_use((*change)->jz_spi_support,param->addr,param->para_num);

	(*change)->num_partitions=param->partition_num;
	(*change)->mtd_partition=kzalloc(param->partition_num*sizeof(struct mtd_partition),GFP_KERNEL);
	if(!(*change)->mtd_partition){
		return -ENOMEM;
	}
	for(i=0;i<(*change)->num_partitions;i++)
	{
		(*change)->mtd_partition[i].name=kzalloc(32*sizeof(char),GFP_KERNEL);
		if(!(*change)->mtd_partition[i].name)
			return -ENOMEM;
		memcpy((*change)->mtd_partition[i].name,param->partition[i].name,32);
		(*change)->mtd_partition[i].size=param->partition[i].size;
		(*change)->mtd_partition[i].offset=param->partition[i].offset;
		(*change)->mtd_partition[i].mask_flags=param->partition[i].mask_flags;

	}
	return 0;
}

static int get_pagesize_from_nand(struct sfc_flash *flash)
{
	char buffer[16];
	int page_size=0;
	u8 status;
	int ret = 0;
	ret = jz_sfc_nand_read(flash, 0, 0, buffer, 16, &status);
	if(ret)
		return ret;
	page_size = buffer[SPL_TYPE_FLAG_LEN+5]*1024;
	return page_size;
}

static int jz_sfc_nand_read_param(struct sfc_flash *flash, struct jz_spi_nand_platform_data **param, int *nand_magic)
{
	/*first get pagesize*/
	int page_size;
	struct get_chip_param param_from_burner;
	char *buffer=NULL;
	char *member_addr=NULL;
	int trytime = 0;
	int ret = 0;
	u8 status;
	column_cmdaddr_bits = 24;
try:
	page_size = get_pagesize_from_nand(flash);
	trytime++;
	if((page_size>0) && (page_size<4096)){
		buffer = kzalloc(page_size, GFP_KERNEL);
		if(!buffer){
			printk("failed to kzalloc for page_size\n");
			return -ENOMEM;
		}
	}else if(trytime < 2){
		column_cmdaddr_bits = 32;
		goto try;
	}else{
		printk("failed to get pagesize from nand\n");
		return -1;
	}
	ret = jz_sfc_nand_read(flash, SPIFLASH_PARAMER_OFFSET/page_size, SPIFLASH_PARAMER_OFFSET%page_size,
			buffer, 1024, &status);
	if(ret)
	{
		kfree(buffer);
		return ret;
	}

	*nand_magic=*(int32_t *)(buffer);
	printk("nand_magic=0x%x\n",*nand_magic);
	if(*nand_magic!=0x6e616e64){
		printk("read sfc nand magic error!");
		kfree(buffer);
		return 0;
	}
	member_addr=buffer+sizeof(int32_t);
	param_from_burner.version=*(int *)member_addr;
	member_addr+=sizeof(param_from_burner.version);
	param_from_burner.flash_type=*(int *)member_addr;
	member_addr+=sizeof(param_from_burner.flash_type);
	param_from_burner.para_num=*(int *)member_addr;
	member_addr+=sizeof(param_from_burner.para_num);
	param_from_burner.addr=(struct jz_spi_support_from_burner *)member_addr;
	member_addr+=param_from_burner.para_num*sizeof(struct jz_spi_support_from_burner);
	param_from_burner.partition_num=*(int *)(member_addr);
	member_addr+=sizeof(param_from_burner.partition_num);
	param_from_burner.partition=(struct jz_spinand_partition *)member_addr;
	transfer_to_mtddriver_struct(&param_from_burner,param);
	return 0;
}


static int sfc_nand_read_id(struct sfc_flash *flash, int id_dummy_nbits, int id_nbytes)
{
	struct sfc_transfer transfer;
	struct sfc_message message;
	struct cmd_info cmd;
	unsigned char id_buf[32];
	unsigned int chip_id = 0;
	int ret, i;
	int mask = 0;

	if (id_nbytes >= 4) {
		printk("warnning : id is too long, error maybe occur!\n");
		return -1;
	}

	mutex_lock(&flash->lock);

	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));
	sfc_message_init(&message);

	cmd.cmd = SPINAND_CMD_RDID;
	cmd.dataen = ENABLE;
	transfer.data_dummy_bits = id_dummy_nbits;
	transfer.data = id_buf;
	transfer.len = id_nbytes;
	transfer.direction = GLB_TRAN_DIR_READ;
	transfer.cmd_info = &cmd;
	transfer.ops_mode = CPU_OPS;
	transfer.sfc_mode = TM_STD_SPI;

	sfc_message_add_tail(&transfer, &message);
	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		printk("sfc nand read id error\n");
		return -EIO;
	}


	for(i = 0; i < (id_nbytes); i++) {
		mask |= (0xff << (i * 8));
	}
	for(i = 0; i < (id_nbytes); i++) {
		chip_id |= (id_buf[i] << ((id_nbytes - i - 1) * 8));
	}
//	printk("chip_id=%x mask=%x\n", chip_id, mask);

	mutex_unlock(&flash->lock);

	return chip_id & mask;
}
struct jz_spi_support *jz_sfc_nand_probe_id(struct sfc_flash *flash)
{
	struct jz_spi_support *jz_spi_nand_support_table;
	struct jz_spi_nand_platform_data *nand_info;
	struct jz_spi_support *params;
	int number_spi_flash;
	int i;
	nand_info = (struct jz_spi_nand_platform_data *)(flash->flash_info);
	number_spi_flash = nand_info->num_spi_flash;
	jz_spi_nand_support_table = nand_info->jz_spi_support;

	for (i = 0; i < number_spi_flash; i++) {
		params = &jz_spi_nand_support_table[i];
		if ((params->id_manufactory == flash->chip_id))
			break;
	}
	if (i >= number_spi_flash) {
		printk("can not find params\n");
		return -1;
	}
#if 0
	printk("**************************************************************\n");
	printk("id_manufactory=0x%08x\n",params->id_manufactory);
	printk("id_device=0x%08x\n",params->id_device);
	printk("name=%s\n",params->name);
	printk("page_size=%d\n",params->page_size);
	printk("oob_size=%d\n",params->oobsize);
	printk("sector_size=%d\n",params->sector_size);
	printk("block_size=%d\n",params->block_size);
	printk("size=%d\n",params->size);
	printk("page_num=%d\n",params->page_num);
	printk("tRD_maxbusy=%d\n",params->tRD_maxbusy);
	printk("tPROG_maxbusy=%d\n",params->tPROG_maxbusy);
	printk("tBERS_maxbusy=%d\n",params->tBERS_maxbusy);
	printk("column_cmdaddr_bits=%d\n",params->column_cmdaddr_bits);
	printk("**************************************************************\n");
#endif
	return params;
}

static int jz_sfcnand_read_oob(struct mtd_info *mtd,loff_t addr,struct mtd_oob_ops *ops)
{
	struct sfc_flash *flash;
	int ret;
	u8 status;
	int page = ((unsigned int)addr)/mtd->writesize;
	flash = to_jz_spi_nand(mtd);

	mutex_lock(&flash->lock);
	if(ops->datbuf){
		ret = jz_sfc_nand_read(flash, page, 0, ops->datbuf, mtd->writesize, &status);
		if(ret<0)
		{
			pr_info("spi nand read error %s %s %d \n",__FILE__,__func__,__LINE__);
			mutex_unlock(&flash->lock);
			return ret;
		}
	}
	ret = jz_sfc_nand_read(flash, page, mtd->writesize+ops->ooboffs, ops->oobbuf, ops->ooblen, &status);
	mutex_unlock(&flash->lock);
	if(ret<0)
	{
		pr_info("spi nand read error %s %s %d \n",__FILE__,__func__,__LINE__);
		return ret;
	}
	return (int)status;
}

static int badblk_check(int len,unsigned char *buf)
{
	int i,bit0_cnt = 0;
	unsigned short *check_buf = (unsigned short *)buf;

	if(check_buf[0] != 0xffff){
		for(i = 0; i < len * 8; i++){
			if(!((check_buf[0] >> i) & 0x1))
				bit0_cnt++;
		}
	}
	if(bit0_cnt > 6 * len)
		return 1; // is bad blk
	return 0;
}

static int jz_sfcnand_block_bad_check(struct mtd_info *mtd, loff_t ofs,int getchip)
{
	int check_len = 2;
	unsigned char check_buf[] = {0xaa,0xaa};
	struct mtd_oob_ops ops;
	ops.oobbuf = check_buf;
	ops.ooblen = check_len;
	jz_sfcnand_read_oob(mtd,ofs,&ops);
	if(badblk_check(check_len,check_buf))
		return 1;
	return 0;
}

static int jz_sfc_nand_write(struct sfc_flash *flash, const u_char *buffer, int page, int column, size_t len, u8 *status)
{
	int ret;
	struct sfc_transfer transfer[4];
	struct sfc_message message;
	struct cmd_info cmd[4];
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));
	sfc_message_init(&message);

	/*2. write enable*/
	cmd[0].cmd = SPINAND_CMD_WREN;
	transfer[0].addr = 0;
	transfer[0].addr_len = 0;
	transfer[0].addr_dummy_bits = 0;
	cmd[0].dataen = DISABLE;
	transfer[0].data_dummy_bits = 0;
	transfer[0].cmd_info = &cmd[0];
	transfer[0].ops_mode = CPU_OPS;
	transfer[0].sfc_mode = TM_STD_SPI;
	sfc_message_add_tail(&transfer[0], &message);

	/*1. write to cache*/
#ifdef CONFIG_SPI_QUAD
	cmd[1].cmd = SPINAND_CMD_PRO_LOAD_X4;
#else
	cmd[1].cmd = SPINAND_CMD_PRO_LOAD;
#endif
	transfer[1].addr = column;
	transfer[1].addr_len = 2;
	transfer[1].addr_dummy_bits = 0;
	cmd[1].dataen = ENABLE;
	transfer[1].data = buffer;
	transfer[1].len = len;
	transfer[1].direction = GLB_TRAN_DIR_WRITE;
	transfer[1].data_dummy_bits = 0;
	transfer[1].cmd_info = &cmd[1];
	transfer[1].ops_mode = DMA_OPS;
#ifdef CONFIG_SPI_QUAD
	transfer[1].sfc_mode = TM_QI_QO_SPI;
#else
	transfer[1].sfc_mode = TM_STD_SPI;
#endif
	sfc_message_add_tail(&transfer[1], &message);
	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
		return ret;
	}
/*3. program enable*/
	cmd[2].cmd = SPINAND_CMD_PRO_EN;
	transfer[2].addr = page;
	transfer[2].addr_len = 3;
	transfer[2].addr_dummy_bits = 0;
	cmd[2].dataen = DISABLE;
	transfer[2].data_dummy_bits = 0;
	transfer[2].cmd_info = &cmd[2];
	transfer[2].ops_mode = CPU_OPS;
	transfer[2].sfc_mode = TM_STD_SPI;
	sfc_message_add_tail(&transfer[2], &message);
	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
		return ret;
	}
/*4. delay*/
	udelay(tPROG);
/*5. get status to be sure nand wirte completed*/
	cmd[3].cmd = SPINAND_CMD_GET_FEATURE;
	transfer[3].addr = SPINAND_ADDR_STATUS;
	transfer[3].addr_len = 1;
	transfer[3].addr_dummy_bits = 0;
	cmd[3].dataen = DISABLE;
	transfer[3].data_dummy_bits = 0;
	cmd[3].sta_exp = (0 << 0);
	cmd[3].sta_msk = SPINAND_IS_BUSY;
	transfer[3].cmd_info = &cmd[3];
	transfer[3].ops_mode = CPU_OPS;
	transfer[3].sfc_mode = TM_STD_SPI;
	sfc_message_add_tail(&transfer[3], &message);
	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
		return ret;
	}

	*status = sfc_get_sta_rt(flash->sfc);
	return 0;
}

static int jz_sfcnand_write_oob(struct mtd_info *mtd,loff_t addr,struct mtd_oob_ops *ops)
{
	struct sfc_flash *flash;
	int page = ((unsigned int)addr)/mtd->writesize;
	int column = mtd->writesize;
	u8 status;
	int ret;
	flash = to_jz_spi_nand(mtd);
	mutex_lock(&flash->lock);
	ret = jz_sfc_nand_write(flash, ops->oobbuf, page, column, ops->ooblen, &status);
	if(ret)
	{
		pr_info("spi nand write error %s %s %d \n",__FILE__,__func__,__LINE__);
		goto write_oob_exit;
	}
	if(status & P_FAIL){
		pr_info("spi nand write oob fail %s %s %d\n",__FILE__,__func__,__LINE__);
		ret = -EIO;
		goto write_oob_exit;
	}
	ops->retlen = ops->ooblen;
write_oob_exit:
	mutex_unlock(&flash->lock);
	return ret;
}

static int jz_sfcnand_chip_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	uint8_t buf[2] = { 0, 0 };
	int  res, ret = 0, i = 0;
	int write_oob = !(chip->bbt_options & NAND_BBT_NO_OOB_BBM);

	/* Write bad block marker to OOB */
	if (write_oob) {
		struct mtd_oob_ops ops;
		loff_t wr_ofs = ofs;
		ops.datbuf = NULL;
		ops.oobbuf = buf;
		ops.ooboffs = chip->badblockpos;
		if (chip->options & NAND_BUSWIDTH_16) {
			ops.ooboffs &= ~0x01;
			ops.len = ops.ooblen = 2;
		} else {
			ops.len = ops.ooblen = 1;
		}
		ops.mode = MTD_OPS_PLACE_OOB;

		/* Write to first/last page(s) if necessary */
		if (chip->bbt_options & NAND_BBT_SCANLASTPAGE)
			wr_ofs += mtd->erasesize - mtd->writesize;
		do {
			res = jz_sfcnand_write_oob(mtd, wr_ofs, &ops);
			if (!ret)
				wr_ofs += mtd->writesize;
		} while ((chip->bbt_options & NAND_BBT_SCAN2NDPAGE) && i < 2);
	}
	/* Update flash-based bad block table */
	if (chip->bbt_options & NAND_BBT_USE_FLASH) {
		res = nand_update_bbt(mtd, ofs);
		if (!ret)
			ret = res;
	}
	if (!ret)
		mtd->ecc_stats.badblocks++;

	return ret;
}

static int jz_sfc_nand_erase_blk(struct sfc_flash *flash,uint32_t addr)
{
	int status=0, ret;
	int page = addr/flash->mtd.writesize;
	struct sfc_transfer transfer[3];
	struct sfc_message message;
	struct cmd_info cmd[3];
	memset(&transfer, 0, sizeof(transfer));
	memset(&cmd, 0, sizeof(cmd));
	sfc_message_init(&message);

	cmd[0].cmd = SPINAND_CMD_WREN;
	transfer[0].addr = 0;
	transfer[0].addr_len = 0;
	transfer[0].addr_dummy_bits = 0;
	cmd[0].dataen = DISABLE;
	transfer[0].data_dummy_bits = 0;
	transfer[0].cmd_info = &cmd[0];
	transfer[0].ops_mode = CPU_OPS;
	transfer[0].sfc_mode = TM_STD_SPI;
	sfc_message_add_tail(&transfer[0], &message);

	switch(flash->mtd.erasesize){
		case SPINAND_OP_BL_128K:
			cmd[1].cmd = SPINAND_CMD_ERASE_128K;
			break;
		default:
			pr_info("Don't support the blksize to erase ! \n");
			break;
	}
	transfer[1].addr = page;
	transfer[1].addr_len = 3;
	transfer[1].addr_dummy_bits = 0;
	cmd[1].dataen = DISABLE;
	transfer[1].data_dummy_bits = 0;
	transfer[1].cmd_info = &cmd[1];
	transfer[1].ops_mode = CPU_OPS;
	transfer[1].sfc_mode = TM_STD_SPI;
	sfc_message_add_tail(&transfer[1], &message);
	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
		return ret;
	}
	udelay(tBERS);

	cmd[2].cmd = SPINAND_CMD_GET_FEATURE;
	transfer[2].addr = SPINAND_ADDR_STATUS;
	transfer[2].addr_len = 1;
	transfer[2].addr_dummy_bits = 0;
	cmd[2].dataen = DISABLE;
	transfer[2].data_dummy_bits = 0;
	cmd[2].sta_exp = (0 << 0);
	cmd[2].sta_msk = SPINAND_IS_BUSY;
	transfer[2].cmd_info = &cmd[2];
	transfer[2].ops_mode = CPU_OPS;
	transfer[2].sfc_mode = TM_STD_SPI;
	sfc_message_add_tail(&transfer[2], &message);
	ret = sfc_sync(flash->sfc, &message);
	if(ret) {
		dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		ret=-EIO;
		return ret;
	}

	status = sfc_get_sta_rt(flash->sfc);
	if(status & E_FALI){
		pr_info("Erase error,get state error ! %s %s %d \n",__FILE__,__func__,__LINE__);
		return -EIO;
	}
	return 0;
}

static int jz_sfc_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int ret;
	uint32_t addr, end;
	int check_addr;
	struct sfc_flash *flash;
	flash=to_jz_spi_nand(mtd);
	check_addr = ((unsigned int)instr->addr) % (mtd->erasesize);
	if (check_addr) {
		pr_info("%s line %d eraseaddr no align\n", __func__,__LINE__);
		return -EINVAL;
	}
	addr = (uint32_t)instr->addr;
	end = addr + (uint32_t)instr->len;
	instr->state = MTD_ERASING;;
	mutex_lock(&flash->lock);
	while (addr < end) {
		ret = jz_sfc_nand_erase_blk(flash, addr);
		if (ret) {
			pr_info("spi nand erase error blk id  %d !\n",addr / mtd->erasesize);
			instr->state = MTD_ERASE_FAILED;
			goto erase_exit;
		}
		addr += mtd->erasesize;
	}

	instr->state = MTD_ERASE_DONE;
erase_exit:
	mtd_erase_callback(instr);
	mutex_unlock(&flash->lock);
	return ret;
}

static int jz_sfcnand_block_isbab(struct mtd_info *mtd,loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	if(!chip->bbt)
		return chip->block_bad(mtd, ofs, 1);
	return nand_isbad_bbt(mtd, ofs, 0);
}

static int jz_sfcnand_block_markbad(struct mtd_info *mtd,loff_t ofs)
{
	struct nand_chip *chip = mtd->priv;
	int ret;
	ret = jz_sfcnand_block_isbab(mtd, ofs);
	if (ret) {
		/* If it was bad already, return success and do nothing */
		if (ret > 0)
			return 0;
		return ret;
	}
	return chip->block_markbad(mtd, ofs);
}

static int jz_sfcnand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int addr,ret;
	addr = instr->addr;

	ret = jz_sfc_nand_erase(mtd,instr);
	if(ret){
		pr_info("WARNING: block %d erase fail !\n",addr / mtd->erasesize);
		ret = jz_sfcnand_block_markbad(mtd,addr);
		if(ret){
			pr_info("mark bad block error, there will occur error,so exit !\n");
			return -1;
		}
	}
	instr->state = MTD_ERASE_DONE;
	return 0;
}


static int jz_sfcnand_read(struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
	struct sfc_flash *flash;
	unsigned int page;
	int column;
	unsigned char *buffer = buf;
	int rlen;
	int page_size = mtd->writesize;
	unsigned int ops_addr = (unsigned int)from;
	size_t ops_len = len;
	int ret = 0;
	u8 status = -1;

	flash=to_jz_spi_nand(mtd);
	mutex_lock(&flash->lock);
	while(1)
	{
		column = ops_addr%page_size;
		page = ops_addr/page_size;
		rlen = min_t(unsigned int,ops_len,page_size-column);
		ret = jz_sfc_nand_read(flash, page, column, buffer, rlen, &status);
		if(ret < 0 || status < 0){
			break;
		}
		ops_len = ops_len-rlen;
		ops_addr = ops_addr+rlen;
		buffer = buffer+rlen;
		*retlen += rlen;                  //check use ret_rlen
		if(ops_len == 0)
			break;
		else if(ops_len<0)
		{
			printk("text read error!\n");
			break;
		}
	}
	mutex_unlock(&flash->lock);
	if(status < 0 || status == ecc_max)
		return status;
	else
		return ret;
}

static int jz_sfcnand_write(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{
	struct sfc_flash *flash;
	const u_char *buffer = buf;
	int page=0;
	int column;
	int wlen = 0;
	int page_size = mtd->writesize;
	int ops_addr = (unsigned int)to;
	int ops_len = len;
	int ret;
	u8 status;

	if(to & (mtd->writesize - 1)){
		pr_info("wirte add don't align ,error ! %s %s %d \n",__FILE__,__func__,__LINE__);
		return -EINVAL;
	}
	*retlen=0;
	flash = to_jz_spi_nand(mtd);
	mutex_lock(&flash->lock);
	while(1){
		page=ops_addr/page_size;
		column=ops_addr%page_size;
		wlen=min_t(int,page_size-column,ops_len);
		ret = jz_sfc_nand_write(flash, buffer, page, column, wlen, &status);
		if(status & P_FAIL){
			pr_info("spi nand write fail %s %s %d\n",__FILE__,__func__,__LINE__);
			ret= -EIO;
		}
		*retlen += wlen;
		ops_len -= wlen;
		ops_addr+=wlen;
		buffer += wlen;
		if(ret<0)	//write fail
		{
			pr_info("spi nand write error %s %s %d \n",__FILE__,__func__,__LINE__);
			break;
		}
		if(ops_len<=0){
			ret=0;
			break;
		}
	}
	mutex_unlock(&flash->lock);
	return ret;
}

static int __init jz_sfc_probe(struct platform_device *pdev)
{
	const char *jz_probe_types[] = {"cmdlinepart",NULL};
	struct mtd_partition *mtd_sfcnand_partition;
	struct jz_spi_nand_platform_data *nand_param;
	struct jz_spi_support *spi_flash;
	struct sfc_flash *flash;
	struct nand_chip *chip;
	int num_partitions;
	int nand_magic= 0;
	u8 feacture;
	int err = 0;
	int ret, i;
	int chip_id;
	int try_times = 0;
	int id_dummy_nbits;
	int id_nbytes;

	chip = kzalloc(sizeof(struct nand_chip),GFP_KERNEL);
	if(!chip)
		return -ENOMEM;
	flash = kzalloc(sizeof(struct sfc_flash), GFP_KERNEL);
	if (!flash) {
		kfree(chip);
		return -ENOMEM;
	}
	flash->dev = &pdev->dev;
	flash->pdata = pdev->dev.platform_data;
	flash->flash_info = flash->pdata->board_info;
	flash->flash_info_num = 1;
	flash->sfc = sfc_res_init(pdev);
	set_flash_timing(flash->sfc, tCHSH, tSLCH, tSHSL_RD, tSHSL_WR);
	platform_set_drvdata(pdev, flash);
	spin_lock_init(&flash->lock_status);
	mutex_init(&flash->lock);

	/* clear write protect */
	jz_sfc_nand_get_feature(flash, 0xa0, &feacture);
	feacture = 0;
	jz_sfc_nand_set_feature(flash, 0xa0, feacture);

	/* ecc enable */
	jz_sfc_nand_get_feature(flash, 0xb0, &feacture);
	feacture |= (1<<4);
	jz_sfc_nand_set_feature(flash, 0xb0, feacture);

	/* quad enable */
#ifdef CONFIG_SPI_QUAD
	jz_sfc_nand_get_feature(flash, 0xb0, &feacture);
	feacture |= (1<<0); /* set FEATURE_REG QE bit */
	err = jz_sfc_nand_set_feature(flash, 0xb0, feacture);
	if(err)
	{
		printk("failed to set quad mode\n");
		goto err_failed_quad;
	}
#endif


	do {
		if (try_times == 0) {
			id_dummy_nbits = 0;
			id_nbytes = 3;
		} else if (try_times ==1) {
			id_dummy_nbits = 8;
			id_nbytes = 3;
		}  else {
			id_dummy_nbits = 8;
			id_nbytes = 2;
		}
		chip_id = sfc_nand_read_id(flash, id_dummy_nbits, id_nbytes);
		for (i = 0; i < 2; i++) {
			if (chip_id == id_table[i]) {
				if (chip_id == id_table[0]) {
					manufacturer_id = MANU_GD;
				} else {
					manufacturer_id = MANU_MX;
				}
				break;
			}
		}
		if (manufacturer_id != 0) {
			break;
		}
	} while(++try_times < 3);

	if (manufacturer_id == 0) {
		printk("unsupport nand id\n");
		return -1;
	}
	flash->chip_id = chip_id;

	jz_sfc_nand_read_param(flash, &nand_param, &nand_magic);
	if(nand_magic == 0x6e616e64){
		flash->flash_info = (void*)nand_param;
	}else{
		nand_param = (struct jz_spi_nand_platform_data *)(flash->flash_info);
	}
	mtd_sfcnand_partition = nand_param->mtd_partition;
	num_partitions = nand_param->num_partitions;

#if 0
	printk("-----------------test param-------------------------------------\n");
	printk("param number=%d\n",nand_param->num_spi_flash);
	printk("partition number=%d\n",nand_param->num_partitions);
	printk("chip id=%x\n",nand_param->jz_spi_support->id_manufactory);
	printk("chip name=%s\n",nand_param->jz_spi_support->name);
	printk("partition0 name=%s\n",nand_param->mtd_partition[0].name);
	printk("partition1 name=%s\n",nand_param->mtd_partition[1].name);
	printk("partition2 name=%s\n",nand_param->mtd_partition[2].name);
	printk("partition3 name=%s\n",nand_param->mtd_partition[3].name);
	printk("-------------------end------------------------------------------\n");
#endif
	spi_flash = jz_sfc_nand_probe_id(flash);
	flash->mtd.name = "sfc_nand";
	flash->mtd.owner = THIS_MODULE;
	flash->mtd.type = MTD_NANDFLASH;
	flash->mtd.flags |= MTD_CAP_NANDFLASH;
	flash->mtd.erasesize = spi_flash->block_size;
	flash->mtd.writesize = spi_flash->page_size;
	flash->mtd.size = spi_flash->size;
	flash->mtd.oobsize = spi_flash->oobsize;
	flash->mtd.writebufsize = flash->mtd.writesize;
	flash->mtd.bitflip_threshold = flash->mtd.ecc_strength = ecc_max;

	column_cmdaddr_bits = spi_flash->column_cmdaddr_bits;
	tRD = spi_flash->tRD_maxbusy;
	tPROG = spi_flash->tPROG_maxbusy;
	tBERS = spi_flash->tBERS_maxbusy;

	chip->select_chip = NULL;
	chip->badblockbits = 8;
	chip->scan_bbt = nand_default_bbt;
	chip->block_bad = jz_sfcnand_block_bad_check;
	chip->block_markbad = jz_sfcnand_chip_block_markbad;
	//chip->ecc.layout= &gd5f_ecc_layout_128; // for erase ops
	chip->bbt_erase_shift = chip->phys_erase_shift = ffs(flash->mtd.erasesize) - 1;
	if (!(chip->options & NAND_OWN_BUFFERS))
		chip->buffers = kmalloc(sizeof(*chip->buffers), GFP_KERNEL);

	/* Set the bad block position */
	if (flash->mtd.writesize > 512 || (chip->options & NAND_BUSWIDTH_16))
		chip->badblockpos = NAND_LARGE_BADBLOCK_POS;
	else
		chip->badblockpos = NAND_SMALL_BADBLOCK_POS;

	flash->mtd.priv = chip;
	flash->mtd._erase = jz_sfcnand_erase;
	flash->mtd._read = jz_sfcnand_read;
	flash->mtd._write = jz_sfcnand_write;
	flash->mtd._read_oob = jz_sfcnand_read_oob;
	flash->mtd._write_oob = jz_sfcnand_write_oob;
	flash->mtd._block_isbad = jz_sfcnand_block_isbab;
	flash->mtd._block_markbad = jz_sfcnand_block_markbad;

	chip->scan_bbt(&flash->mtd);
	ret = mtd_device_parse_register(&flash->mtd,jz_probe_types,NULL, mtd_sfcnand_partition, num_partitions);
	if (ret) {
		kfree(chip);
		kfree(flash);
		return -ENODEV;
	}
	/*	unsigned char user_buf[2]={};
		int rrrr=0,llen;
		struct erase_info ssa={
		.addr=0x900000,
		.len=flash->mtd.erasesize,
		.mtd=&(flash->mtd),
		.callback=0,
		};
		jz_sfcnand_erase(&(flash->mtd), &ssa);
		unsigned char  *bufff=(unsigned char *)kmalloc(2048*3,GFP_KERNEL);
		for(rrrr=0;rrrr<2049;rrrr++)
		bufff[rrrr]=rrrr;
		ret=jz_sfcnand_write(&(flash->mtd),0x900002, 2049,&llen, bufff);
		memset(bufff,0,3*2048);
		jz_sfcnand_read(&(flash->mtd),0x900000,2080, &rrrr, bufff);
		printk("-----------readbuff-------------\n");
		for(rrrr=0;rrrr<2080;)
		{
		printk("%x,",bufff[rrrr]);
		rrrr++;
		if(rrrr%16==0)printk("\n");
		}
		*/

	return 0;
#ifdef CONFIG_SPI_QUAD
err_failed_quad:
#endif
	kfree(flash);
	kfree(chip);
	return err;
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
	.shutdown	= jz_sfc_shutdown,
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
