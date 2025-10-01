#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER tracker

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./tracker_tp.h"

#if !defined(_TRACKER_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TRACKER_TP_H

#include <lttng/tracepoint.h>
#include <rmw/types.h>

TRACEPOINT_EVENT(
    tracker,
    pub_init,
    TP_ARGS(const rmw_gid_t *, pub_id, const char *, node, const char *, topic),
    TP_FIELDS(
        ctf_array(uint8_t, pub_id, pub_id->data, RMW_GID_STORAGE_SIZE)
        ctf_string(node, node)
        ctf_string(topic, topic)
    )
)

TRACEPOINT_EVENT(
    tracker,
    sub_init,
    TP_ARGS(uint64_t, sub_id, const char *, node, const char *, topic),
    TP_FIELDS(
        ctf_integer(uint64_t, sub_id, sub_id)
        ctf_string(node, node)
        ctf_string(topic, topic)
    )
)

TRACEPOINT_EVENT(
    tracker,
    publish,
    TP_ARGS(const rmw_gid_t *, pub_id, uint64_t, stamp, __pid_t, cb_tid),
    TP_FIELDS(
        ctf_array(uint8_t, pub_id, pub_id->data, RMW_GID_STORAGE_SIZE)
        ctf_integer(uint64_t, stamp, stamp)
        ctf_integer(__pid_t, cb_tid, cb_tid)
    )
)

TRACEPOINT_EVENT(
    tracker,
    recieve,
    TP_ARGS(const rmw_gid_t *, pub_id, uint64_t, sub_id, uint64_t, stamp, __pid_t, cb_tid),
    TP_FIELDS(
        ctf_array(uint8_t, pub_id, pub_id->data, RMW_GID_STORAGE_SIZE)
        ctf_integer(uint64_t, sub_id, sub_id)
        ctf_integer(uint64_t, stamp, stamp)
        ctf_integer(__pid_t, cb_tid, cb_tid)
    )
)

TRACEPOINT_EVENT(
    tracker,
    no_recieve,
    TP_ARGS(__pid_t, cb_tid),
    TP_FIELDS(
        ctf_integer(__pid_t, cb_tid, cb_tid)
    )
)

TRACEPOINT_EVENT(
    tracker,
    indirect_link,
    TP_ARGS(const rmw_gid_t *, prev_pub_id, uint64_t, prev_stamp, const rmw_gid_t *, next_pub_id, uint64_t, next_stamp, __pid_t, cb_tid),
    TP_FIELDS(
        ctf_array(uint8_t, prev_pub_id, prev_pub_id->data, RMW_GID_STORAGE_SIZE)
        ctf_integer(uint64_t, prev_stamp, prev_stamp)
        ctf_array(uint8_t, next_pub_id, next_pub_id->data, RMW_GID_STORAGE_SIZE)
        ctf_integer(uint64_t, next_stamp, next_stamp)
        ctf_integer(__pid_t, cb_tid, cb_tid)
    )
)

#endif

#include <lttng/tracepoint-event.h>

