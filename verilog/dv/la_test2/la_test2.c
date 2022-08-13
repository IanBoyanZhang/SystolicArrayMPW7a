/*
 * SPDX-FileCopyrightText: 2020 Efabless Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * SPDX-License-Identifier: Apache-2.0
 */

// This include is relative to $CARAVEL_PATH (see Makefile)
#include <defs.h>
#include <stub.c>

/*
	MPRJ LA Test:
		- Sets counter clk through LA[64]
		- Sets counter rst through LA[65] 
		- Observes count value for five clk cycle through LA[31:0]
*/

int clk = 0;
int i;

/*
uint32_t mat_A[9] = {
  //1.126105
  0x00003c81,
  //0.407351
  0x00003685,
  //2.315680
  0x000040a2,
  //0.930338
  0x00003b71,
  //2.542255
  0x00004116,
  //1.070112
  0x00003c48,
  //1.107074
  0x00003c6e,
  //1.020977
  0x00003c15,
  //2.659628
  0x00004152
};

uint32_t mat_B[9] = {
  //1.435914
  0x00003dbe,
  //1.319322
  0x00003d47,
  //0.348074
  0x00003592,
  //1.898164
  0x00003f98,
  //1.588423
  0x00003e5b,
  //0.815995
  0x00003a87,
  //2.724823
  0x00004173,
  //0.130791
  0x0000302f,
  //2.339815
  0x000040ae
};
*/

uint32_t mat_A[9] = {
  // 1
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
};

uint32_t mat_B[9] = {
  // 1
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
  0x00003c00,
};


void main()
{
        /* Set up the housekeeping SPI to be connected internally so	*/
	/* that external pin changes don't affect it.			*/

	// reg_spimaster_config = 0xa002;	// Enable, prescaler = 2,
        reg_spi_enable = 1;
                                        // connect to housekeeping SPI

	// Connect the housekeeping SPI to the SPI master
	// so that the CSB line is not left floating.  This allows
	// all of the GPIO pins to be used for user functions.


	// All GPIO pins are configured to be output
	// Used to flad the start/end of a test 

        reg_mprj_io_31 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_30 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_29 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_28 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_27 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_26 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_25 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_24 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_23 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_22 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_21 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_20 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_19 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_18 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_17 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_16 = GPIO_MODE_MGMT_STD_OUTPUT;

        reg_mprj_io_15 = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_14 = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_13 = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_12 = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_11 = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_10 = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_9  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_8  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_7  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_5  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_4  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_3  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_2  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_1  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_0  = GPIO_MODE_USER_STD_OUTPUT;


        /* Apply configuration */
        reg_mprj_xfer = 1;
        while (reg_mprj_xfer == 1);

	// Configure All LA probes as inputs to the cpu 
	reg_la0_oenb = reg_la0_iena = 0x00000000;    // [31:0]
	reg_la1_oenb = reg_la1_iena = 0x00000000;    // [63:32]
	reg_la2_oenb = reg_la2_iena = 0x00000000;    // [95:64]
	reg_la3_oenb = reg_la3_iena = 0x00000000;    // [127:96]

	// Flag start of the test
	reg_mprj_datal = 0xAB600000;

	// Configure LA[64] LA[65] LA[66] as outputs from the cpu
	// Configure LA[67] LA[68] LA[69] LA[70] as outputs from the CPU as readout address select
	// Configure LA[71] as input to the CPU for o_done signal
	// Configure LA[72] LA[73] LA[74] LA[75] LA[76] as outputs from the CPU as input matrix address select
	// Configure LA[77] as output from the CPU as 'control module sync' or write enable. (sync/wr)
	// Configure LA[78] as input to the CPU as mem_set_done_oc
	reg_la2_oenb = reg_la2_iena = 0x000003F7F;

	// clk, reset, cs
	//reg_la2_oenb = reg_la2_iena = 0x00000007; 

	// Set clk & reset to one
	reg_la2_data = 0x00000003;

	// Set sync & wr to one
	reg_la2_data = reg_la2_data | 0x00001000;

	// Configure LA[63:32] output from the cpu
	reg_la1_oenb = reg_la1_iena = 0xFFFFFFFF;
	reg_la1_data = 0x00000000;

        // DELAY
        for (i=0; i<5; i=i+1) {}

	// Toggle clk & de-assert reset
	for (i=0; i<5; i=i+1) {
	    clk = !clk;
	    reg_la2_data = 0x00000000 | clk;
	}

	// cs/data_input is ready/valid
	reg_la2_data = reg_la2_data | 0x00000004;

	uint32_t i_mat = 0;
	uint32_t mat_addr = 0x00000000; 

	// Toggle clk & send mat_A data
	for (i=0; i<17; i=i+1) {
	    clk = !clk;
	    reg_la2_data = 0x00000000 | clk;
	    reg_la2_data = reg_la2_data | 0x00000004;
	    reg_la2_data = reg_la2_data | mat_addr;
	    if (clk == 0) {
	        reg_la1_data = mat_A[i_mat];
		i_mat += 1;
		mat_addr += 1;
	    } 
	}

	i_mat = 0;
	// Toggle clk & send mat_B data
	for (i=0; i<17; i=i+1) {
	    clk = !clk;
	    reg_la2_data = 0x00000000 | clk;
	    reg_la2_data = reg_la2_data | 0x00000004;
	    reg_la2_data = reg_la2_data | mat_addr;
	    if (clk == 0) {
	        reg_la1_data = mat_B[i_mat];
		i_mat += 1;
		mat_addr += 1;
	    } 
	}

	for (i=0; i<1; i=i+1) {
	    clk = !clk;
	    reg_la2_data = 0x00000000 | clk;
	    reg_la2_data = reg_la2_data | 0x00000004;
	}

        while (1){
		clk = !clk;
		reg_la2_data = 0x00000000 | clk;
	    	reg_la2_data = reg_la2_data | 0x00000004;

                if ((reg_la0_data_in & 0x0000FFFF) >= 0x00004200 &&
		    (reg_la2_data_in & 0x0000FFFF) == 0x00000080) {
			// de-assert cs
			reg_la2_data = reg_la2_data & ~(0x00001000);
                        reg_mprj_datal = 0xAB610000;
                        break;
                }
        }

}

