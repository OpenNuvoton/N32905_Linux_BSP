/* jpegpriv.h
 *
 * Copyright (c) 2008 Nuvoton technology corporation
 * All rights reserved.
 * <clyu2@nuvoton.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef __ASM_ARM_W55FA93_JPEG_PRIV_H
#define __ASM_ARM_W55FA93_JPEG_PRIV_H

#include <linux/semaphore.h>

typedef struct jpeg_priv {
	struct video_device jdev;	/* Must be the first field! */
	struct semaphore           lock;
	struct file *file;
	jpeg_state_t state;
	__u32	vaddr;//if vaddr_src/vaddr_dst == 0, will using vaddr address
	__u32	paddr;//if paddr_src/paddr_dst == 0, will using paddr address
	__u32	vaddr_src;
	__u32	vaddr_dst;
	__u32	paddr_src;
	__u32	paddr_dst;
	__u32	decopw_vaddr;
	__u32	decopw_en;
	__u32	decopw_tnum;
	__u32	decopw_tsize;
	__u32	decopw_tcount;
	__u32	decopw_mcux;
	__u32	decopw_mcuy;
	__u32	decopw_yaddr;
	__u32	decopw_bitaddr;
	__u32	decopw_tmcuynum;
	__u32	decopw_page_index;
	__u32	decopw_page_offset;
	__u32	decopw_TargetBuffersize;
	__u32	windec_mcux_start;
	__u32	windec_mcux_end;
	__u32	windec_mcuy_start;
	__u32	windec_mcuy_end;
	__u32	windec_stride;
	__u32	windec_en;
	struct 	page **pages; 
	
	// TESTTEST
	__u32 nr_pages;
	
	__u32	decopw_end;
	__u32	decInWait_buffer_empty;
	__u32	decInWait_buffer_size;
	__u32	decInWait_counter;
	__u32	src_bufsize;/*source buffer size*/
	__u32	dst_bufsize;/*dst buffer size*/
	__u32	mmap_bufsize;/*mmap buffer size, user can set buffer address or using mmap*/
	__u32	src_datasize;//source image data size for encode/decode
	__u32	yuvformat;	/*for decode*/
	__u32	width;		/*for decode*/
    __u32	height;		/*for decode*/
  __u32	decode_output_format;
	__u32	scale;	//1 enable, 0 disable
	__u32	scaled_width;
	__u32	scaled_height;
    __u32	dec_stride;
    __u32	*image_size; /*image size after encoded*/
    __u32 	byte2pixel;//is 2 times of actual byte
    __u32	encode_width;/*the width that will be encoded image raw data*/
    __u32	encode_height;/*the height that will be encoded image raw data*/
	__u32	encode_image_format;
	__u32	encode_source_format;
    __u8	qadjust;//the larger the better quality[2-16](0.25Q, 0.5Q, 0.75Q, Q, 1.25Q, 1.5Q, 1.75Q, 2Q, 2.25Q, 2.5Q, 2.75Q, 3Q, 3.25Q, 3.5Q, 3.75Q) 
	__u8	qscaling;//the smaller the better quality[1-16]
	__u32	encode;//1 decode, 0 encode	
	__u32	buffersize;/*each encode buffer size*/
    __u32	buffercount;/*total encode buffer count*/
	__u32	bufferend;//jpeg encode buffer end
    __u32	bufferend_bak;//jpeg encode buffer end backup
    __u32   convert;
}jpeg_priv_t;

#endif//__ASM_ARM_W55FA93_JPEG_PRIV_H

