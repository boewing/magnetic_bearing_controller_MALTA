/*
 * recorder.h
 *
 *  Created on: 23 Apr 2018
 *      Author: smiric
 */


#ifndef SRC_RECORDER_H_
#define SRC_RECORDER_H_

#include "hw_cfg.h"
#include <stdint.h>
#include "xil_cache.h"
#include "xil_printf.h"


/*
 * This defines the max. number of samples that can be recorded.
 * A sample is 4 bytes long (int32_t)
 * 33'554'432 samples is 134'217'728 Bytes
 * 134'217'728 samples is 536'870'912 Bytes
 * 30'000'000 samples is 120'000'000 Bytes
 */
#define MAXNUMSAMP	30000

/*
 * The number of samples that are thrown away at the beginning of each recording, as they are, mysteriously, sometimes/randomly zero.
 * The core/axi seems to not write the first few accesses... So, we throw away the first few samples. No idea why this happens.
 */
#define SPARESAMPLES 1000

/*
 * The default-value that is written to memory before a new recording is started:
 * By doing this, we can check later when evaluating the data, if some samples have not been written.
 */
#define MEMDEFAULT 2147483646


uint32_t rec_write_baseaddr(void);
uint32_t rec_set_start(void);
void rec_set_stop(void);
void rec_clear_stop(void);
uint32_t rec_set_channel(uint32_t ch);
uint32_t rec_check_status(void);
uint32_t rec_write_numsamp(uint32_t numsamp);
uint32_t rec_clear_start(void);
uint32_t rec_read_to_uart(void);
uint32_t rec_get_numsamp(void);
void rec_init_mem(void);

extern uint32_t g_RecActive;





#endif /* SRC_RECORDER_H_ */

