/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "ia_css_dpc2.host.h"

/* TODO: See if some functionality can be embedded in encode
 * See also for range and precision of parameters etc.. */
void
ia_css_dpc2_encode(
	struct ia_css_isp_dpc2_params *to,
	const struct ia_css_dpc2_config *from,
	size_t size)
{
	(void)size;
	/* TODO: Implement following encode functions
	* one_plus_metric1;
	* one_minus_metric1;
	* one_plus_metric3;
	* wb_gain_gr;
	* wb_gain_r;
	* wb_gain_b;
	* wb_gain_gb;
	* wb_gain_gr_scaled_by_metric2;
	* wb_gain_r_scaled_by_metric2;
	* wb_gain_b_scaled_by_metric2;
	* wb_gain_gb_scaled_by_metric2;
	*/
}

