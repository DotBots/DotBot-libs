#ifndef __FRAME_H
#define __FRAME_H

/**
 * @defgroup    drv_frame    DotBot app frame header
 * @ingroup     drv
 * @brief       Mari-compatible wire header for bare-radio DotBot apps
 *
 * Bare-radio DotBot applications (apps that don't run over the Mari TSCH
 * link layer) prepend this header to each radio payload. The byte layout
 * is a deliberate copy of Mari's `mr_packet_header_t` (see
 * `mari/firmware/mari/models.h`) so that a bare-radio gateway can wrap
 * received frames as `MARI_EDGE_DATA` UART events and PyDotBot's existing
 * MarilibEdge adapter — which dispatches by `next_proto` — handles them
 * with no special case.
 *
 * Bare apps always set `type = DATA` and `next_proto = DOTBOT_APP`. The
 * radio uses a BLE access address distinct from Mari's so the two stacks
 * ignore each other at the peripheral filter.
 *
 * @{
 * @file
 * @author Geovane Fedrecheski <geovane.fedrecheski@inria.fr>
 *
 * @copyright Inria, 2026
 * @}
 */

#include <stdint.h>
#include <string.h>
#include "device.h"

//=========================== defines ==========================================

/// Mari-compatible packet type for DATA frames (mirrors `MARI_PACKET_DATA`).
#define DB_FRAME_TYPE_DATA (16)

/// Mari-compatible next_proto value for the DotBot application protocol
/// (mirrors `MARI_NEXT_PROTO_DOTBOT_APP`). The host's MarilibEdge adapter
/// dispatches frames carrying this value to the DotBot pipeline.
#define DB_FRAME_NEXT_PROTO (0x11)

/// Header version. Matches Mari's current `MARI_PROTOCOL_VERSION` so the
/// host parses a bare-radio frame identically to a Mari frame.
#define DB_FRAME_VERSION (3)

/// Network ID for bare-radio DotBot apps. Reserved at the Crystalfree
/// Mari network-ID registry — distinct from any Mari deployment's
/// `net_id` to keep bare-radio traffic in its own namespace.
#define DB_FRAME_NETWORK_ID (0xD0B0)

/// BLE access address for bare radio. Distinct from Mari's `0x12345678`
/// so a Mari node and a bare-radio node ignore each other at the radio
/// peripheral filter. Also distinct from the BLE advertising access
/// address (`0x8E89BED6`).
#define DB_FRAME_ACCESS_ADDR (0xDB12DB12UL)

/// Default radio frequency offset for bare-radio DotBot apps. The radio
/// driver computes `2400 + freq` MHz from this, so 8 → 2408 MHz — same
/// value bare DotBot apps used pre-TDMA.
#define DB_FRAME_DEFAULT_FREQ (8U)

/// Broadcast destination value.
#define DB_FRAME_DST_BROADCAST (0xFFFFFFFFFFFFFFFFULL)

/// Wire size of the header in bytes — must match Mari's `mr_packet_header_t`.
#define DB_FRAME_HEADER_SIZE (21)

//=========================== types ============================================

/**
 * @brief Wire header for bare-radio DotBot frames.
 *
 * Byte-for-byte mirror of Mari's `mr_packet_header_t`. Mari stores
 * `type` and `next_proto` as enums; here they're `uint8_t` so the
 * layout doesn't depend on the compiler's enum width. The numeric
 * values are pinned by Mari's enum.
 */
typedef struct __attribute__((packed)) {
    uint8_t  version;     ///< Protocol version
    uint8_t  type;        ///< Packet type — always DATA for bare radio
    uint16_t network_id;  ///< Network identifier
    uint64_t dst;         ///< Destination device id (`DB_FRAME_DST_BROADCAST` for broadcast)
    uint64_t src;         ///< Source device id
    uint8_t  next_proto;  ///< Upper-layer protocol — always DOTBOT_APP for bare radio
} db_frame_header_t;

/* Hard guarantee: layout matches Mari's mr_packet_header_t exactly. */
_Static_assert(sizeof(db_frame_header_t) == DB_FRAME_HEADER_SIZE,
               "db_frame_header_t must be 21 bytes (Mari mr_packet_header_t layout)");

//=========================== public ===========================================

/**
 * @brief Write a DotBot app frame header to a buffer.
 *
 * Fills the canonical DATA + DOTBOT_APP header. The sender's device id
 * is read from `db_device_id()`.
 *
 * @param[out] buffer  Destination buffer (must hold at least 21 bytes)
 * @param[in]  dst     Destination device id (`DB_FRAME_DST_BROADCAST` for broadcast)
 *
 * @return Number of bytes written (always `DB_FRAME_HEADER_SIZE`).
 */
static inline size_t db_frame_header_to_buffer(uint8_t *buffer, uint64_t dst) {
    db_frame_header_t header = {
        .version    = DB_FRAME_VERSION,
        .type       = DB_FRAME_TYPE_DATA,
        .network_id = DB_FRAME_NETWORK_ID,
        .dst        = dst,
        .src        = db_device_id(),
        .next_proto = DB_FRAME_NEXT_PROTO,
    };
    memcpy(buffer, &header, sizeof(header));
    return sizeof(header);
}

#endif  // __FRAME_H
