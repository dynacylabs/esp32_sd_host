// Raw, protocol-level SD commands - these talk directly to sdmmc_send_cmd() against the session
// in sd_session.h, bypassing any filesystem concept entirely. This is the layer that actually
// exercises the SD Physical Layer spec command-by-command against the Tang Nano FPGA, matching
// how this whole project has been built: one real command proven at a time.
//
// Commands implemented (UART line syntax, space-separated, hex arguments unless noted):
//
//   Manual step-by-step init (mirrors the real negotiation sequence; each one just sends that
//   one command and reports the raw response - nothing is done automatically):
//     CMD0                          GO_IDLE_STATE (no response expected)
//     CMD8 [arg_hex]                SEND_IF_COND / R7 (default arg 0x1AA if omitted)
//     CMD55                         APP_CMD / R1 (uses the session's current RCA, 0 until CMD3)
//     ACMD41 [ocr_hex]              SD_SEND_OP_COND / R3 (sends CMD55 first automatically - this
//                                   one's defined as always preceded by APP_CMD, not a separate
//                                   manual step)
//     CMD2                          ALL_SEND_CID / R2
//     CMD3                          SEND_RELATIVE_ADDR / R6 - on success, stores the returned RCA
//                                   into the session automatically so later per-card commands
//                                   use it without the caller re-entering it each time
//     CMD9                          SEND_CSD / R2
//     CMD7                          SELECT_CARD / R1B (uses the session's current RCA)
//     CMD13                         SEND_STATUS / R1 (same as CARDSTATUS below)
//
//   Block I/O (via sdmmc_read_sectors/write_sectors/erase_sectors - handles CMD17/18/24/25/
//   32/33/38 and multi-vs-single-block selection internally):
//     RDBLOCK <lba>                 -> 512 raw bytes
//     WRBLOCK <lba>                 + 512 raw bytes -> OK/ERR
//     RDMULTI <lba> <count>         -> count*512 raw bytes
//     WRMULTI <lba> <count>         + count*512 raw bytes -> OK/ERR
//     ERASE <start_lba> <end_lba>   -> OK/ERR
//
//   Card registers / status:
//     CARDINFO                      dumps OCR/RCA/CID/CSD/SCR fields as text (from a card struct
//                                   populated by a successful MOUNT or by the manual CMD2/3/9
//                                   sequence above)
//     CARDSTATUS                    CMD13, dumps the raw 32-bit card status word
//
//   Less common / rarely-exercised commands - the actual point of this module:
//     SWITCHFUNC <mode> <group> <value>   CMD6, dumps the 64-byte switch-status response
//     LOCK <pwd_hex>                CMD42, set password and lock the card
//     UNLOCK <pwd_hex>              CMD42, unlock with the given password
//     CLRPWD <old_pwd_hex>          CMD42, clear password protection
//     FORCEERASE                    CMD42, forced erase (recovery from a forgotten password)
//     SETWRPROT <lba>               CMD28, set write-protect on one group
//     CLRWRPROT <lba>               CMD29, clear write-protect on one group
//     SENDWRPROT <lba>              CMD30, query write-protect status of one group
//     VOLTAGESWITCH                 CMD11 (UHS-I signal voltage switch)
//     TUNING                        CMD19 (UHS-I sampling-point tuning block - SD's real tuning
//                                   command; CMD20 is NOT a tuning command for SD at all, it's
//                                   the legacy WRITE_DAT_UNTIL_STOP, so there's no "or 20" here)
//     GENCMD <arg_hex> <read|write> CMD56, generic vendor command with a one-block data phase
//     PREERASE <count>              ACMD23, pre-erase block count hint before a multi-block write
//     CARDDETECT <0|1>              ACMD42, enable/disable the card's own DAT3 pull-up
//
//   Fully generic escape hatch, for anything not named above:
//     RAWCMD <index> <arg_hex> <resp_type> [datalen] [read|write]
//       resp_type one of: R0 R1 R1B R2 R3 R4 R5 R5B R6 R7
//       If datalen>0 and direction is "write", reads datalen raw bytes from UART first and sends
//       them as the command's data phase. If "read", sends the command then dumps datalen raw
//       bytes read back. Response words are always dumped as hex after OK.
#pragma once

#include <Arduino.h>

bool cmd_raw_dispatch(const String &cmd, String *parts, int nparts);
