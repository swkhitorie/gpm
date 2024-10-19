#include "bl.h"

#define _FLASH_KBYTES                   (*(uint32_t *)0x1FF1E880)
#define BOOTLOADER_RESERVATION_SIZE	    (128 * 1024)
#define APP_RESERVATION_SIZE            0
#define BOARD_FLASH_SIZE                (_FLASH_KBYTES * 1024)
#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE))

// bootloader flash update protocol.
//
// Command format:
//
//      <opcode>[<command_data>]<EOC>
//
// Reply format:
//
//      [<reply_data>]<INSYNC><status>
//
// The <opcode> and <status> values come from the PROTO_ defines below,
// the <*_data> fields is described only for opcodes that transfer data;
// in all other cases the field is omitted.
//
// Expected workflow (protocol 3) is:
//
// GET_SYNC   verify that the board is present
// GET_DEVICE   determine which board (select firmware to upload)
// CHIP_ERASE   erase the program area and reset address counter
// loop:
//      PROG_MULTI      program bytes
// GET_CRC    verify CRC of entire flashable area
// RESET    finalise flash programming, reset chip and starts application
//

#define BL_PROTOCOL_VERSION     5   // The revision of the bootloader protocol
//* Next revision needs to update

// protocol bytes
#define PROTO_INSYNC        0x12    // 'in sync' byte sent before status
#define PROTO_EOC         0x20    // end of command

// Reply bytes
#define PROTO_OK          0x10    // INSYNC/OK      - 'ok' response
#define PROTO_FAILED        0x11    // INSYNC/FAILED  - 'fail' response
#define PROTO_INVALID       0x13  // INSYNC/INVALID - 'invalid' response for bad commands
#define PROTO_BAD_SILICON_REV     0x14  // On the F4 series there is an issue with < Rev 3 silicon
#define PROTO_RESERVED_0X15     0x15  // Reserved

// see https://pixhawk.org/help/errata
// Command bytes
#define PROTO_GET_SYNC        0x21    // NOP for re-establishing sync
#define PROTO_GET_DEVICE      0x22    // get device ID bytes
#define PROTO_CHIP_ERASE      0x23    // erase program area and reset program address
#define PROTO_PROG_MULTI      0x27    // write bytes at program address and increment
#define PROTO_GET_CRC       0x29  // compute & return a CRC
#define PROTO_GET_OTP       0x2a  // read a byte from OTP at the given address
#define PROTO_GET_SN        0x2b    // read a word from UDID area ( Serial)  at the given address
#define PROTO_GET_CHIP        0x2c    // read chip version (MCU IDCODE)
#define PROTO_SET_DELAY       0x2d    // set minimum boot delay
#define PROTO_GET_CHIP_DES      0x2e    // read chip version In ASCII
#define PROTO_BOOT          0x30    // boot the application
#define PROTO_DEBUG         0x31    // emit debug information - format not defined
#define PROTO_SET_BAUD        0x33    // set baud rate on uart

#define PROTO_RESERVED_0X36     0x36  // Reserved
#define PROTO_RESERVED_0X37     0x37  // Reserved
#define PROTO_RESERVED_0X38     0x38  // Reserved
#define PROTO_RESERVED_0X39     0x39  // Reserved

#define PROTO_PROG_MULTI_MAX    64  // maximum PROG_MULTI size
#define PROTO_READ_MULTI_MAX    255 // size of the size field

/* argument values for PROTO_GET_DEVICE */
#define PROTO_DEVICE_BL_REV 1 // bootloader revision
#define PROTO_DEVICE_BOARD_ID 2 // board ID
#define PROTO_DEVICE_BOARD_REV  3 // board revision
#define PROTO_DEVICE_FW_SIZE  4 // size of flashable area
#define PROTO_DEVICE_VEC_AREA 5 // contents of reserved vectors 7-10

#define STATE_PROTO_OK                  0x10    // INSYNC/OK      - 'ok' response
#define STATE_PROTO_FAILED              0x11    // INSYNC/FAILED  - 'fail' response
#define STATE_PROTO_INVALID             0x13  // INSYNC/INVALID - 'invalid' response for bad commands
#define STATE_PROTO_BAD_SILICON_REV     0x14  // On the F4 series there is an issue with < Rev 3 silicon
#define STATE_PROTO_RESERVED_0X15       0x15  // Reserved

// State
#define STATE_PROTO_GET_SYNC      0x1     // Have Seen NOP for re-establishing sync
#define STATE_PROTO_GET_DEVICE    0x2     // Have Seen get device ID bytes
#define STATE_PROTO_CHIP_ERASE    0x4     // Have Seen erase program area and reset program address
#define STATE_PROTO_PROG_MULTI    0x8     // Have Seen write bytes at program address and increment
#define STATE_PROTO_GET_CRC       0x10    // Have Seen compute & return a CRC
#define STATE_PROTO_GET_OTP       0x20    // Have Seen read a byte from OTP at the given address
#define STATE_PROTO_GET_SN        0x40    // Have Seen read a word from UDID area ( Serial)  at the given address
#define STATE_PROTO_GET_CHIP      0x80    // Have Seen read chip version (MCU IDCODE)
#define STATE_PROTO_GET_CHIP_DES  0x100   // Have Seen read chip version In ASCII
#define STATE_PROTO_BOOT          0x200   // Have Seen boot the application

#define STATE_ALLOWS_ERASE        (STATE_PROTO_GET_SYNC|STATE_PROTO_GET_DEVICE)
#define STATE_ALLOWS_REBOOT       (STATE_ALLOWS_ERASE|STATE_PROTO_PROG_MULTI|STATE_PROTO_GET_CRC)
#define SET_BL_STATE(s) bl_state |= (s)

#define	APP_FLASH_ADDR         0x08020000
#define APP_FLASH_SECTOR       0x20000
static const uint32_t bl_proto_rev = BL_PROTOCOL_VERSION; 
static volatile uint32_t bl_state = 0;

struct boardinfo board_info = {
	.board_type	= 9,
	.board_rev	= 0,
	.fw_size	= 0,
	.systick_mhz	= 480,
};

static uint32_t crc32(const uint8_t *src, unsigned len, unsigned state)
{
	static uint32_t crctab[256];

	/* check whether we have generated the CRC table yet */
	/* this is much smaller than a static table */
	if (crctab[1] == 0) {
		for (unsigned i = 0; i < 256; i++) {
			uint32_t c = i;

			for (unsigned j = 0; j < 8; j++) {
				if (c & 1) {
					c = 0xedb88320U ^ (c >> 1);

				} else {
					c = c >> 1;
				}
			}

			crctab[i] = c;
		}
	}

	for (unsigned i = 0; i < len; i++) {
		state = crctab[(state ^ src[i]) & 0xff] ^ (state >> 8);
	}

	return state;
}

static void cout_word(uint32_t val)
{
    extern void usb_cout(uint8_t busid, uint8_t *p, uint32_t len);
	usb_cout(0, (uint8_t *)&val, 4);
}

static void sync_response(void)
{
    uint8_t data[] = {
        PROTO_INSYNC, // "in sync"
        PROTO_OK  // "OK"
    };

    USB_LOG_RAW("SYNC \r\n");
    extern void usb_cout(uint8_t busid, uint8_t *p, uint32_t len);
    usb_cout(0, &data[0], 2);
}

static void bad_silicon_response(void)
{
	uint8_t data[] = {
		PROTO_INSYNC,     // "in sync"
		PROTO_BAD_SILICON_REV // "issue with < Rev 3 silicon"
	};

    USB_LOG_RAW("SYNC BAD 1\r\n");
    extern void usb_cout(uint8_t busid, uint8_t *p, uint32_t len);
    usb_cout(0, &data[0], 2);
}

static void invalid_response(void)
{
	uint8_t data[] = {
		PROTO_INSYNC, // "in sync"
		PROTO_INVALID // "invalid command"
	};

    USB_LOG_RAW("SYNC FAILED 2\r\n");
    extern void usb_cout(uint8_t busid, uint8_t *p, uint32_t len);
    usb_cout(0, &data[0], 2);
}

static void failure_response(void)
{
	uint8_t data[] = {
		PROTO_INSYNC, // "in sync"
		PROTO_FAILED  // "command failed"
	};

    USB_LOG_RAW("SYNC FAILED 1\r\n");
    extern void usb_cout(uint8_t busid, uint8_t *p, uint32_t len);
    usb_cout(0, &data[0], 2);
}

static bool wait_for_eoc(uint32_t times)
{
    extern int usb_cin_wait(uint32_t timeout);
	return (usb_cin_wait(times) == PROTO_EOC);
}

static void flash_erase()
{
    erase_sector(get_flash_sector(APP_FLASH_ADDR + 0));
	erase_sector(get_flash_sector(APP_FLASH_ADDR + APP_FLASH_SECTOR * 1));
//    erase_sector(get_flash_sector(0x08060000));
//	erase_sector(get_flash_sector(0x08080000));
//    erase_sector(get_flash_sector(0x080A0000));
//	erase_sector(get_flash_sector(0x080C0000));
//    erase_sector(get_flash_sector(0x080E0000));
}

void jump_to_app()
{
    uint32_t i = 0;
	void (*AppJump)(void);
    const uint32_t *app_base = (const uint32_t *)APP_FLASH_ADDR;
    
    /* the first word of the app is stack top pointer address */
	if (app_base[0] == 0xffffffff) {
        USB_LOG_RAW("jump errcode 0x01: the first word of the app(sp) err \r\n");
		return;
	}
    
    /* the second word of the app is the entrypoint(ResetHandler), it must point within the flash area */
    if (app_base[1] < APP_FLASH_ADDR) {
        USB_LOG_RAW("jump errcode 0x02: the second word of the app(resethandler) err -> out of range \r\n");
        return;
    }
    
    if (app_base[1] >= (APP_FLASH_ADDR + board_info.fw_size)) {
        USB_LOG_RAW("jump errcode 0x03: the second word of the app(resethandler) err -> out of range \r\n");
        return;
    }
    
    DISABLE_INT();
    
	/* just for paranoia's sake */
	arch_flash_lock();

    HAL_RCC_DeInit();
    
	SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

	for (i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

    board_usart_deinit();
    
    ENABLE_INT();
    
	AppJump = (void (*)(void)) (*((uint32_t *) (APP_FLASH_ADDR + 4)));

	__set_MSP(*(uint32_t *)APP_FLASH_ADDR);

	__set_CONTROL(0);

	AppJump(); 

	while (1) {

	}
}

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint32_t blbuf[8];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint32_t ckbuf[8];
static uint32_t address = APP_FLASH_ADDR;
static uint32_t address_start = APP_FLASH_ADDR;
static uint32_t fw_cal_size = 0;
static uint8_t word_n = 0;
static uint32_t ck_sum = 0;

void bt_init()
{
    board_info.fw_size = APP_SIZE_MAX;
}

uint32_t bl_get_state()
{
    return bl_state;
}

void bootloader()
{
    extern int usb_cin();
    extern int usb_cin_wait(uint32_t timeout);
    extern void usb_cout(uint8_t busid, uint8_t *p, uint32_t len);
    
    int arg;
    static union {
        uint8_t   c[256];
        uint32_t  w[64];
    } flash_buffer;
    uint32_t first_word = 0xffffffff;
    int c = usb_cin_wait(20);
    
    if (c < 0) {
        return;
    }
    
    switch (c) {

    // sync
    //
    // command:   GET_SYNC/EOC
    // reply:   INSYNC/OK
    //
    case PROTO_GET_SYNC:
        USB_LOG_RAW("PROTO_GET_SYNC \r\n");
        if (!wait_for_eoc(1000)) {
            goto cmd_bad;
        }
        
        SET_BL_STATE(STATE_PROTO_GET_SYNC);
        break;

    // get device info
    //
    // command:   GET_DEVICE/<arg:1>/EOC
    // BL_REV reply:  <revision:4>/INSYNC/EOC
    // BOARD_ID reply:  <board type:4>/INSYNC/EOC
    // BOARD_REV reply: <board rev:4>/INSYNC/EOC
    // FW_SIZE reply: <firmware size:4>/INSYNC/EOC
    // VEC_AREA reply <vectors 7-10:16>/INSYNC/EOC
    // bad arg reply: INSYNC/INVALID
    //
    case PROTO_GET_DEVICE:
        USB_LOG_RAW("PROTO_GET_DEVICE \r\n");
        /* expect arg then EOC */
        arg = usb_cin_wait(1000);

        if (arg < 0) {
            goto cmd_bad;
        }

        if (!wait_for_eoc(2)) {
            goto cmd_bad;
        }

        switch (arg) {
        case PROTO_DEVICE_BL_REV:
            usb_cout(0, (uint8_t *)&bl_proto_rev, sizeof(bl_proto_rev));
            break;

        case PROTO_DEVICE_BOARD_ID:
            usb_cout(0, (uint8_t *)&board_info.board_type, sizeof(board_info.board_type));
            break;

        case PROTO_DEVICE_BOARD_REV:
            usb_cout(0, (uint8_t *)&board_info.board_rev, sizeof(board_info.board_rev));
            break;

        case PROTO_DEVICE_FW_SIZE:
            usb_cout(0, (uint8_t *)&board_info.fw_size, sizeof(board_info.fw_size));
            break;

        case PROTO_DEVICE_VEC_AREA:
            for (unsigned p = 7; p <= 10; p++) {
                uint32_t bytes = 0; //flash_func_read_word(p * 4);

                usb_cout(0, (uint8_t *)&bytes, sizeof(bytes));
            }

            break;

        default:
            goto cmd_bad;
        }

        SET_BL_STATE(STATE_PROTO_GET_DEVICE);
        break;
        
    // erase and prepare for programming
    //
    // command:   ERASE/EOC
    // success reply: INSYNC/OK
    // erase failure: INSYNC/FAILURE
    //
    case PROTO_CHIP_ERASE:
        USB_LOG_RAW("PROTO_CHIP_ERASE \r\n");
        
        /* expect EOC */
        if (!wait_for_eoc(200)) {
            goto cmd_bad;
        }
        
        if ((bl_state & STATE_ALLOWS_ERASE) != STATE_ALLOWS_ERASE) {
            goto cmd_bad;
        }
        
        flash_erase();
        
        for (address = APP_FLASH_ADDR; address < APP_FLASH_ADDR + APP_FLASH_SECTOR * 2; address++) {
            if (read_word(address) != 0xffffffff) {
                goto cmd_bad; // if goto cmd_fail, program crash
            }
        }
        
        address = APP_FLASH_ADDR;
        
        HAL_Delay(1000);
        SET_BL_STATE(STATE_PROTO_CHIP_ERASE);
        break;
    
    // program bytes at current address
    //
    // command:   PROG_MULTI/<len:1>/<data:len>/EOC
    // success reply: INSYNC/OK
    // invalid reply: INSYNC/INVALID
    // readback failure:  INSYNC/FAILURE
    //
    case PROTO_PROG_MULTI:    // program bytes
        USB_LOG_RAW("PROTO_PROG_MULTI ");
        // expect count
        arg = usb_cin_wait(50);

        if (arg < 0) {
            USB_LOG_RAW("arg < 0");
            goto cmd_bad;
        }

        // sanity-check arguments
        if (arg % 4) {
            USB_LOG_RAW("arg not 4:%x ", arg);
            goto cmd_bad;
        }

        if ((address + arg) > APP_FLASH_ADDR + APP_FLASH_SECTOR * 2) {
            USB_LOG_RAW("arg out range");
            goto cmd_bad;
        }

        if ((unsigned int)arg > sizeof(flash_buffer.c)) {
            USB_LOG_RAW("arg too bit");
            goto cmd_bad;
        }

        for (int i = 0; i < arg; i++) {
            c = usb_cin_wait(1000);

            if (c < 0) {
                goto cmd_bad;
            }

            flash_buffer.c[i] = c;
        }

        if (!wait_for_eoc(200)) {
            USB_LOG_RAW("wait eoc");
            goto cmd_bad;
        }

        if (address == APP_FLASH_ADDR) {
            // save the first word and don't program it until everything else is done
            first_word = flash_buffer.w[0];
            // replace first word with bits we can overwrite later
            // flash_buffer.w[0] = 0xffffffff;
        }

        // number of words : arg / 4, minmum unit program 8
        arg /= 4;
        fw_cal_size += arg * 4;
        
        for (int i = 0; i < arg; i++) {
            blbuf[word_n] = flash_buffer.w[i];
            word_n++;
            ck_sum = crc32((uint8_t *)&flash_buffer.w[i], 4, ck_sum);
            if (word_n >= 8) {
                
                // program the word (8)
                DISABLE_INT();
                write(address, &blbuf[0], 8);
                ENABLE_INT();
                // do immediate read-back verify
                read(address, &ckbuf[0], 8);
                
                for (int j = 0; j < 8; j++) {
                    if (ckbuf[j] != blbuf[j]) {
                        USB_LOG_RAW("%x %d %d %x %x \r\n", address, i, j, ckbuf[j], blbuf[j]);
                        goto cmd_bad;
                    }
                }
                USB_LOG_RAW("Prog success %x ", address);
                address += sizeof(uint32_t) * 8;
                word_n = 0;
                for (int k = 0; k < 8; k++) {
                    ckbuf[k] = 0;
                    blbuf[k] = 0;
                }
            }
        }
        USB_LOG_RAW("pack arg: %d %d ", arg, word_n);
        SET_BL_STATE(STATE_PROTO_PROG_MULTI);
        break;
        
    case PROTO_GET_CRC:
        USB_LOG_RAW("PROTO_GET_CRC ");
        // expect EOC
        if (!wait_for_eoc(2)) {
            goto cmd_bad;
        }

        if (word_n != 0) {
            USB_LOG_RAW("Last program: | ");
            // program the last part
            DISABLE_INT();
            write(address, &blbuf[0], word_n);
            ENABLE_INT();
            // do immediate read-back verify
            read(address, &ckbuf[0], word_n);
            
            for (int j = 0; j < word_n; j++) {
                if (ckbuf[j] != blbuf[j]) {
                    goto cmd_bad;
                }
            }
            USB_LOG_RAW("Last prog success %x ", address);
            address += sizeof(uint32_t) * word_n;
            word_n = 0;
            for (int k = 0; k < 8; k++) {
                ckbuf[k] = 0;
                blbuf[k] = 0;
            }
        }
        
        USB_LOG_RAW("file size: %d bytes crc %x ", fw_cal_size, ck_sum);
        
        
        // compute CRC of the programmed area
        uint32_t sum = 0;

        for (uint32_t p = address_start; p < address_start + board_info.fw_size; p += 4) {
            uint32_t bytes;

            if ((p == address_start) && (first_word != 0xffffffff)) {
                bytes = first_word;
            } else {
                bytes = read_word(p);
            }

            if (p >= address_start + fw_cal_size) {
                bytes = 0xffffffff;
            }
            
            sum = crc32((uint8_t *)&bytes, sizeof(bytes), sum);
        }
        USB_LOG_RAW("read cal crc %x ", sum);
        cout_word(sum);
        SET_BL_STATE(STATE_PROTO_GET_CRC);
        break;

    case PROTO_BOOT:
        USB_LOG_RAW("PROTO_BOOT \r\n");
        // expect EOC
        if (!wait_for_eoc(1000)) {
            goto cmd_bad;
        }
        
        if ((bl_state & STATE_ALLOWS_REBOOT) != STATE_ALLOWS_REBOOT) {
            goto cmd_bad;
        }
            
        // send a sync and wait for it to be collected
        sync_response();
        HAL_Delay(100);
        return;
        
    default: 
        USB_LOG_RAW("UNKOWN STATE : %x \r\n", c);
        break;
    }

    // send the sync response for this command
    sync_response();
    return;
    
cmd_bad:
    // send an 'invalid' response but don't kill the timeout - could be garbage
    invalid_response();
    bl_state = 0;
    return;
    
cmd_fail:
    // send a 'command failed' response but don't kill the timeout - could be garbage
    failure_response();
    return;
    
bad_silicon:
    // send the bad silicon response but don't kill the timeout - could be garbage
    bad_silicon_response();
    return;
}

