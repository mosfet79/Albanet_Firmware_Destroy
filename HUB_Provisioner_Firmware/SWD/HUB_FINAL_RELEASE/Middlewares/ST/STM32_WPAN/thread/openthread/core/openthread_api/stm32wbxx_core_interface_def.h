/**
  ******************************************************************************
  * @file    stm32wbxx_core_interface_def.h
  * @author  MCD Application Team
  * @brief   This file contains all the defines and structures used for the
  *          communication between the two core M0 and M4.
  *          This file is shared between the code running on M4 and the code
  *          running on M0.
  *
  ******************************************************************************
  * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32WBxx_CORE_INTERFACE_DEF_H
#define STM32WBxx_CORE_INTERFACE_DEF_H

#include "stm32_wpan_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Structure of the messages exchanged between M0 and M4 */
#define OT_CMD_BUFFER_SIZE 20U
typedef PACKED_STRUCT
{
  uint32_t  ID;
  uint32_t  Size;
  uint32_t  Data[OT_CMD_BUFFER_SIZE];
}Thread_OT_Cmd_Request_t;


/* List of messages sent by the M4 to the M0 */
typedef enum
{
  /* LINK */
  MSG_M4TOM0_OT_LINK_ACTIVE_SCAN,
  MSG_M4TOM0_OT_LINK_IS_ACTIVE_SCAN_IN_PROGRESS,
  MSG_M4TOM0_OT_LINK_ENERGY_SCAN,
  MSG_M4TOM0_OT_LINK_IS_ENERGY_SCAN_IN_PROGRESS,
  MSG_M4TOM0_OT_LINK_SEND_DATA_REQUEST,
  MSG_M4TOM0_OT_LINK_IS_IN_TRANSMIT_STATE,
  MSG_M4TOM0_OT_LINK_OUT_OF_BAND_TRANSMIT_REQUEST,
  MSG_M4TOM0_OT_LINK_GET_CHANNEL,
  MSG_M4TOM0_OT_LINK_SET_CHANNEL,
  MSG_M4TOM0_OT_LINK_GET_SUPPORTED_CHANNEL_MASK,
  MSG_M4TOM0_OT_LINK_SET_SUPPORTED_CHANNEL_MASK,
  MSG_M4TOM0_OT_LINK_GET_EXTENDED_ADDRESS,
  MSG_M4TOM0_OT_LINK_SET_EXTENDED_ADDRESS,
  MSG_M4TOM0_OT_LINK_GET_FACTORY_ASSIGNED_EUI64,
  MSG_M4TOM0_OT_LINK_GET_PANID,
  MSG_M4TOM0_OT_LINK_SET_PANID,
  MSG_M4TOM0_OT_LINK_GET_POLL_PERIOD,
  MSG_M4TOM0_OT_LINK_SET_POLL_PERIOD,
  MSG_M4TOM0_OT_LINK_GET_SHORT_ADDRESS,
  MSG_M4TOM0_OT_LINK_SET_SHORT_ADDRESS,
  MSG_M4TOM0_OT_LINK_FILTER_GET_ADRESS_MODE,
  MSG_M4TOM0_OT_LINK_FILTER_SET_ADDRESS_MODE,
  MSG_M4TOM0_OT_LINK_FILTER_ADD_ADDRESS,
  MSG_M4TOM0_OT_LINK_FILTER_REMOVE_ADDRESS,
  MSG_M4TOM0_OT_LINK_FILTER_CLEAR_ADDRESSES,
  MSG_M4TOM0_OT_LINK_FILTER_GET_NEXT_ADDRESS,
  MSG_M4TOM0_OT_LINK_FILTER_ADD_RSS_IN,
  MSG_M4TOM0_OT_LINK_FILTER_REMOVE_RSS_IN,
  MSG_M4TOM0_OT_LINK_FILTER_CLEAR_RSS_IN,
  MSG_M4TOM0_OT_LINK_GET_NEXT_RSS_IN,
  MSG_M4TOM0_OT_LINK_CONVERT_RSS_TO_LINK_QUALITY,
  MSG_M4TOM0_OT_LINK_CONVERT_LINK_QUALITY_TO_RSS,
  MSG_M4TOM0_OT_LINK_GET_COUNTERS,
  MSG_M4TOM0_OT_LINK_SET_PCAP_CALLBACK,
  MSG_M4TOM0_OT_LINK_IS_PROMISCUOUS,
  MSG_M4TOM0_OT_LINK_SET_PROMISCUOUS,
  MSG_M4TOM0_OT_LINK_GET_CCA_FAILURE_RATE,
  MSG_M4TOM0_OT_LINK_SET_ENABLED,
  MSG_M4TOM0_OT_LINK_IS_ENABLED,
  /* IP6 */
  MSG_M4TOM0_OT_IP6_SET_ENABLED,
  MSG_M4TOM0_OT_IP6_IS_ENABLED,
  MSG_M4TOM0_OT_IP6_ADD_UNICAST_ADDRESS,
  MSG_M4TOM0_OT_IP6_REMOVE_UNICAST_ADDRESS,
  MSG_M4TOM0_OT_IP6_GET_UNICAST_ADDRESSES,
  MSG_M4TOM0_OT_IP6_SUBSCRIBE_MULTICAST_ADDRESS,
  MSG_M4TOM0_OT_IP6_UNSUBSCRIBE_MULTICAST_ADDRESS,
  MSG_M4TOM0_OT_IP6_GET_MULTICAST_ADDRESSES,
  MSG_M4TOM0_OT_IP6_IS_MULTICAST_PROMISCUOUS_ENABLED,
  MSG_M4TOM0_OT_IP6_SET_MULTICAST_PROMISCUOUS_ENABLED,
  MSG_M4TOM0_OT_IP6_SLAAC_UPDATE,
  MSG_M4TOM0_OT_IP6_CREATE_RANDOM_IID,
  MSG_M4TOM0_OT_IP6_CREATE_MAC_IID,
  MSG_M4TOM0_OT_IP6_CREATE_SEMANTICALLY_OPAQUE_IID,
  MSG_M4TOM0_OT_IP6_NEW_MESSAGE,
  MSG_M4TOM0_OT_IP6_SET_RECEIVE_CALLBACK,
  MSG_M4TOM0_OT_IP6_IS_RECEIVE_FILTER_ENABLED,
  MSG_M4TOM0_OT_IP6_SET_RECEIVE_FILTER_ENABLED,
  MSG_M4TOM0_OT_IP6_SEND,
  MSG_M4TOM0_OT_IP6_ADD_UNSECURE_PORT,
  MSG_M4TOM0_OT_IP6_REMOVE_UNSECURE_PORT,
  MSG_M4TOM0_OT_IP6_REMOVE_ALL_UNSECURE_PORTS,
  MSG_M4TOM0_OT_IP6_GET_UNSECURE_PORTS,
  MSG_M4TOM0_OT_IP6_IS_ADDRESS_EQUAL,
  MSG_M4TOM0_OT_IP6_ADDRESS_FROM_STRING,
  MSG_M4TOM0_OT_IP6_PREFIX_MATCH,
  MSG_M4TOM0_OT_IP6_IS_ADDRESS_UNSPECIFIED,
  /* THREAD */
  MSG_M4TOM0_OT_THREAD_SET_ENABLED,
  MSG_M4TOM0_OT_THREAD_GET_AUTO_START,
  MSG_M4TOM0_OT_THREAD_SET_AUTO_START,
  MSG_M4TOM0_OT_THREAD_IS_SINGLETON,
  MSG_M4TOM0_OT_THREAD_DISCOVER,
  MSG_M4TOM0_OT_THREAD_IS_DISCOVER_IN_PROGRESS,
  MSG_M4TOM0_OT_THREAD_GET_CHILD_TIMEOUT,
  MSG_M4TOM0_OT_THREAD_SET_CHILD_TIMEOUT,
  MSG_M4TOM0_OT_THREAD_GET_EXTPANID,
  MSG_M4TOM0_OT_THREAD_SET_EXTPANID,
  MSG_M4TOM0_OT_THREAD_GET_LEADER_RLOC,
  MSG_M4TOM0_OT_THREAD_GET_LINK_MODE,
  MSG_M4TOM0_OT_THREAD_SET_LINK_MODE,
  MSG_M4TOM0_OT_THREAD_GET_MASTER_KEY,
  MSG_M4TOM0_OT_THREAD_SET_MASTER_KEY,
  MSG_M4TOM0_OT_THREAD_GET_MESH_LOCAL_EID,
  MSG_M4TOM0_OT_THREAD_GET_MESH_LOCAL_PREFIX,
  MSG_M4TOM0_OT_THREAD_SET_MESH_LOCAL_PREFIX,
  MSG_M4TOM0_OT_THREAD_GET_LINK_LOCAL_IP6_ADDRESS,
  MSG_M4TOM0_OT_THREAD_GET_NETWORK_NAME,
  MSG_M4TOM0_OT_THREAD_SET_NETWORK_NAME,
  MSG_M4TOM0_OT_THREAD_GET_KEY_SEQUENCE_COUNTER,
  MSG_M4TOM0_OT_THREAD_SET_KEY_SEQUENCE_COUNTER,
  MSG_M4TOM0_OT_THREAD_GET_KEY_SWITCH_GUARD_TIME,
  MSG_M4TOM0_OT_THREAD_SET_KEY_SWITCH_GUARD_TIME,
  MSG_M4TOM0_OT_THREAD_BECOME_DETACHED,
  MSG_M4TOM0_OT_THREAD_BECOME_CHILD,
  MSG_M4TOM0_OT_THREAD_GET_NEXT_NEIGHBOR_INFO,
  MSG_M4TOM0_OT_THREAD_GET_DEVICE_ROLE,
  MSG_M4TOM0_OT_THREAD_GET_LEADER_DATA,
  MSG_M4TOM0_OT_THREAD_GET_LEADER_ROUTER_ID,
  MSG_M4TOM0_OT_THREAD_GET_LEADER_WEIGHT,
  MSG_M4TOM0_OT_THREAD_GET_PARTITION_ID,
  MSG_M4TOM0_OT_THREAD_GET_RLOC_16,
  MSG_M4TOM0_OT_THREAD_GET_PARENT_INFO,
  MSG_M4TOM0_OT_THREAD_GET_PARENT_AVERAGE_RSSI,
  MSG_M4TOM0_OT_THREAD_GET_PARENT_LAST_RSSI,
  MSG_M4TOM0_OT_THREAD_SET_RECEIVE_DIAGNOSTIC_GET_CALLBACK,
  MSG_M4TOM0_OT_THREAD_SEND_DIAGNOSTIC_GET,
  MSG_M4TOM0_OT_THREAD_SEND_DIAGNOSTIC_RESET,
  MSG_M4TOM0_OT_THREAD_GET_IP6_COUNTERS,
  MSG_M4TOM0_OT_THREAD_GET_MLE_COUNTERS,
  MSG_M4TOM0_OT_THREAD_RESET_MLE_COUNTERS,
  /* THREAD FTD */
  MSG_M4TOM0_OT_THREAD_FTD_GET_MAX_ALLOWED_CHILDREN,
  MSG_M4TOM0_OT_THREAD_FTD_SET_MAX_ALLOWED_CHILDREN,
  MSG_M4TOM0_OT_THREAD_FTD_IS_ROUTER_ROLE_ENABLED,
  MSG_M4TOM0_OT_THREAD_FTD_SET_ROUTER_ROLE_ENABLED,
  MSG_M4TOM0_OT_THREAD_FTD_SET_PREFERRED_ROUTER_ID,
  MSG_M4TOM0_OT_THREAD_FTD_GET_LOCAL_LEADER_WEIGHT,
  MSG_M4TOM0_OT_THREAD_FTD_SET_LOCAL_LEADER_WEIGHT,
  MSG_M4TOM0_OT_THREAD_FTD_GET_LOCAL_LEADER_PARTITION_ID,
  MSG_M4TOM0_OT_THREAD_FTD_SET_LOCAL_LEADER_PARTITION_ID,
  MSG_M4TOM0_OT_THREAD_FTD_GET_JOINER_UDP_PORT,
  MSG_M4TOM0_OT_THREAD_FTD_SET_JOINER_UDP_PORT,
  MSG_M4TOM0_OT_THREAD_FTD_SET_STEERING_DATA,
  MSG_M4TOM0_OT_THREAD_FTD_GET_CONTEXT_ID_REUSE_DELAY,
  MSG_M4TOM0_OT_THREAD_FTD_SET_CONTEXT_ID_REUSE_DELAY,
  MSG_M4TOM0_OT_THREAD_FTD_GET_NETWORK_ID_TIMEOUT,
  MSG_M4TOM0_OT_THREAD_FTD_SET_NETWORK_ID_TIMEOUT,
  MSG_M4TOM0_OT_THREAD_FTD_GET_ROUTER_UPGRADE_THRESHOLD,
  MSG_M4TOM0_OT_THREAD_FTD_SET_ROUTER_UPGRADE_THRESHOLD,
  MSG_M4TOM0_OT_THREAD_FTD_RELEASE_ROUTER_ID,
  MSG_M4TOM0_OT_THREAD_FTD_BECOME_ROUTER,
  MSG_M4TOM0_OT_THREAD_FTD_BECOME_LEADER,
  MSG_M4TOM0_OT_THREAD_FTD_GET_ROUTER_DOWNGRADE_THRESHOLD,
  MSG_M4TOM0_OT_THREAD_FTD_SET_ROUTER_DOWNGRADE_THRESHOLD,
  MSG_M4TOM0_OT_THREAD_FTD_GET_ROUTER_SELECTION_JITTER,
  MSG_M4TOM0_OT_THREAD_FTD_SET_ROUTER_SELECTION_JITTER,
  MSG_M4TOM0_OT_THREAD_FTD_GET_CHILD_INFO_BY_ID,
  MSG_M4TOM0_OT_THREAD_FTD_GET_CHILD_INFO_BY_INDEX,
  MSG_M4TOM0_OT_THREAD_FTD_GET_CHILD_NEXT_IP6_ADDRESS,
  MSG_M4TOM0_OT_THREAD_FTD_GET_ROUTER_ID_SEQUENCE,
  MSG_M4TOM0_OT_THREAD_FTD_GET_MAX_ROUTER_ID,
  MSG_M4TOM0_OT_THREAD_FTD_GET_ROUTER_INFO,
  MSG_M4TOM0_OT_THREAD_FTD_GET_EID_CACHE_ENTRY,
  MSG_M4TOM0_OT_THREAD_FTD_GET_PSKC,
  MSG_M4TOM0_OT_THREAD_FTD_SET_PSKC,
  MSG_M4TOM0_OT_THREAD_FTD_GET_PARENT_PRIORITY,
  MSG_M4TOM0_OT_THREAD_FTD_SET_PARENT_PRIORITY,
  MSG_M4TOM0_OT_THREAD_FTD_GET_CHILD_TABLE_CALLBACK,
  MSG_M4TOM0_OT_THREAD_FTD_SET_CHILD_TABLE_CALLBACK,
  /* INSTANCE */
  MSG_M4TOM0_OT_INSTANCE_INIT,
  MSG_M4TOM0_OT_INSTANCE_INIT_SINGLE,
  MSG_M4TOM0_OT_INSTANCE_IS_INITIALIZED,
  MSG_M4TOM0_OT_INSTANCE_FINALIZE,
  MSG_M4TOM0_OT_SET_STATE_CHANGED_CALLBACK,
  MSG_M4TOM0_OT_REMOVE_STATE_CHANGED_CALLBACK,
  MSG_M4TOM0_OT_INSTANCE_RESET,
  MSG_M4TOM0_OT_INSTANCE_FACTORY_RESET,
  MSG_M4TOM0_OT_INSTANCE_ERASE_PERSISTENT_INFO,
  MSG_M4TOM0_OT_GET_DYNAMIC_LOG_LEVEL,
  MSG_M4TOM0_OT_SET_DYNAMIC_LOG_LEVEL,
  /* COAP */
  MSG_M4TOM0_OT_COAP_HEADER_GET_TYPE,
  MSG_M4TOM0_OT_COAP_HEADER_INIT,
  MSG_M4TOM0_OT_COAP_HEADER_SET_TOKEN,
  MSG_M4TOM0_OT_COAP_HEADER_GENERATE_TOKEN,
  MSG_M4TOM0_OT_COAP_HEADER_APPEND_CONTENT_FORMAT_OPTION,
  MSG_M4TOM0_OT_COAP_HEADER_APPEND_OPTION,
  MSG_M4TOM0_OT_COAP_HEADER_APPEND_UINT_OPTION,
  MSG_M4TOM0_OT_COAP_HEADER_APPEND_OBSERVE_OPTION,
  MSG_M4TOM0_OT_COAP_NEW_MESSAGE,
  MSG_M4TOM0_OT_COAP_SEND_REQUEST,
  MSG_M4TOM0_OT_COAP_SEND_RESPONSE,
  MSG_M4TOM0_OT_COAP_HEADER_SET_MESSAGE_ID,
  MSG_M4TOM0_OT_COAP_HEADER_GET_MESSAGE_ID,
  MSG_M4TOM0_OT_COAP_HEADER_GET_TOKEN_LENGTH,
  MSG_M4TOM0_OT_COAP_HEADER_GET_TOKEN,
  MSG_M4TOM0_OT_COAP_HEADER_GET_FIRST_OPTION,
  MSG_M4TOM0_OT_COAP_HEADER_GET_NEXT_OPTION,
  MSG_M4TOM0_OT_COAP_HEADER_APPEND_URI_PATH_OPTIONS,
  MSG_M4TOM0_OT_COAP_HEADER_APPEND_MAX_AGE_OPTION,
  MSG_M4TOM0_OT_COAP_HEADER_APPEND_URI_QUERY_OPTION,
  MSG_M4TOM0_OT_COAP_HEADER_SET_PAYLOAD_MARKER,
  MSG_M4TOM0_OT_COAP_HEADER_GET_CODE,
  MSG_M4TOM0_OT_COAP_START,
  MSG_M4TOM0_OT_COAP_STOP,
  MSG_M4TOM0_OT_COAP_ADD_RESSOURCE,
  MSG_M4TOM0_OT_COAP_REMOVE_RESSOURCE,
  MSG_M4TOM0_OT_COAP_SET_DEFAULT_HANDLER,
  /* MESSAGE */
  MSG_M4TOM0_OT_MESSAGE_FREE,
  MSG_M4TOM0_OT_MESSAGE_GET_LENGTH,
  MSG_M4TOM0_OT_MESSAGE_SET_LENGTH,
  MSG_M4TOM0_OT_MESSAGE_GET_OFFSET,
  MSG_M4TOM0_OT_MESSAGE_SET_OFFSET,
  MSG_M4TOM0_OT_MESSAGE_IS_LINK_SECURITY_ENABLED,
  MSG_M4TOM0_OT_MESSAGE_SET_DIRECT_TRANSMISSION,
  MSG_M4TOM0_OT_MESSAGE_GET_RSS,
  MSG_M4TOM0_OT_MESSAGE_APPEND,
  MSG_M4TOM0_OT_MESSAGE_READ,
  MSG_M4TOM0_OT_MESSAGE_WRITE,
  MSG_M4TOM0_OT_MESSAGE_QUEUE_INIT,
  MSG_M4TOM0_OT_MESSAGE_QUEUE_ENQUEUE,
  MSG_M4TOM0_OT_MESSAGE_QUEUE_ATHEAD,
  MSG_M4TOM0_OT_MESSAGE_QUEUE_DEQUEUE,
  MSG_M4TOM0_OT_MESSAGE_QUEUE_GET_HEAD,
  MSG_M4TOM0_OT_MESSAGE_QUEUE_GET_NEXT,
  MSG_M4TOM0_OT_MESSAGE_BUFFER_INFO,
  /* COMMISSIONER */
  MSG_M4TOM0_OT_COMMISSIONER_START,
  MSG_M4TOM0_OT_COMMISSIONER_STOP,
  MSG_M4TOM0_OT_COMMISSIONER_ADD_JOINER,
  MSG_M4TOM0_OT_COMMISSIONER_REMOVE_JOINER,
  MSG_M4TOM0_OT_COMMISSIONER_SET_PROVISIONING_URL,
  MSG_M4TOM0_OT_COMMISSIONER_GET_PROVISIONING_URL,
  MSG_M4TOM0_OT_COMMISSIONER_ANNOUNCE_BEGIN,
  MSG_M4TOM0_OT_COMMISSIONER_ENERGY_SCAN,
  MSG_M4TOM0_OT_COMMISSIONER_PANID_QUERY,
  MSG_M4TOM0_OT_COMMISSIONER_SEND_MGMT_GET,
  MSG_M4TOM0_OT_COMMISSIONER_SEND_MGMT_SET,
  MSG_M4TOM0_OT_COMMISSIONER_GET_SESSION_ID,
  MSG_M4TOM0_OT_COMMISSIONER_GET_STATE,
  MSG_M4TOM0_OT_COMMISSIONER_GENERATE_PSKC,
  /* DATASET_FTD */
  MSG_M4TOM0_OT_DATASET_SET_ACTIVE,
  MSG_M4TOM0_OT_DATASET_SET_PENDING,
  MSG_M4TOM0_OT_DATASET_SEND_MGMT_ACTIVE_GET,
  MSG_M4TOM0_OT_DATASET_SEND_MGMT_ACTIVE_SET,
  MSG_M4TOM0_OT_DATASET_SEND_MGMT_PENDING_GET,
  MSG_M4TOM0_OT_DATASET_SEND_MGMT_PENDING_SET,
  MSG_M4TOM0_OT_DATASET_GET_DELAY_TIMER_MINIMAL,
  MSG_M4TOM0_OT_DATASET_SET_DELAY_TIMER_MINIMAL,
  /* DATASET */
  MSG_M4TOM0_OT_DATASET_IS_COMMISSIONED,
  MSG_M4TOM0_OT_DATASET_GET_ACTIVE,
  MSG_M4TOM0_OT_DATASET_GET_PENDING,
  /*DHCP6_CLIENT*/
  MSG_M4TOM0_OT_DHCP6_CLIENT_UPDATE,
  /* DHCP6_SERVER */
  MSG_M4TOM0_OT_DHCP6_SERVER_UPDATE,
  /* DIAG */
  MSG_M4TOM0_OT_DIAG_INIT,
  MSG_M4TOM0_OT_DIAG_PROCESS_CMD,
  MSG_M4TOM0_OT_DIAG_PROCESS_CMD_LINE,
  MSG_M4TOM0_OT_DIAG_IS_ENABLED,
  /* DNS */
  MSG_M4TOM0_OT_DNS_CLIENT_QUERY,
  /* ICMP6 */
  MSG_M4TOM0_OT_ICMP6_GET_ECHO_MODE,
  MSG_M4TOM0_OT_ICMP6_SET_ECHO_MODE,
  MSG_M4TOM0_OT_ICMP6_REGISTER_HANDLER,
  MSG_M4TOM0_OT_ICMP6_SEND_ECHO_REQUEST,
  /* JOINER */
  MSG_M4TOM0_OT_JOINER_START,
  MSG_M4TOM0_OT_JOINER_STOP,
  MSG_M4TOM0_OT_JOINER_GET_STATE,
  MSG_M4TOM0_OT_JOINER_GET_ID,
  /* LINK RAW */
  MSG_M4TOM0_OT_LINK_RAW_SET_ENABLE,
  MSG_M4TOM0_OT_LINK_RAW_IS_ENABLED,
  MSG_M4TOM0_OT_LINK_RAW_SET_PANID,
  MSG_M4TOM0_OT_LINK_RAW_SET_EXTENDED_ADDRESS,
  MSG_M4TOM0_OT_LINK_RAW_SET_SHORT_ADDRESS,
  MSG_M4TOM0_OT_LINK_RAW_GET_PROMISCUOUS,
  MSG_M4TOM0_OT_LINK_RAW_SET_PROMISCUOUS,
  MSG_M4TOM0_OT_LINK_RAW_SLEEP,
  MSG_M4TOM0_OT_LINK_RAW_RECEIVE,
  MSG_M4TOM0_OT_LINK_RAW_GET_TRANSMIT_BUFFER,
  MSG_M4TOM0_OT_LINK_RAW_TRANSMIT,
  MSG_M4TOM0_OT_LINK_RAW_GET_RSSI,
  MSG_M4TOM0_OT_LINK_RAW_GET_CAPS,
  MSG_M4TOM0_OT_LINK_RAW_ENERGY_SCAN,
  MSG_M4TOM0_OT_LINK_RAW_SRC_MATCH_ENABLE,
  MSG_M4TOM0_OT_LINK_RAW_SRC_MATCH_ADD_SHORT_ENTRY,
  MSG_M4TOM0_OT_LINK_RAW_SRC_MATCH_ADD_EXT_ENTRY,
  MSG_M4TOM0_OT_LINK_RAW_SRC_MATCH_CLEAR_SHORT_ENTRY,
  MSG_M4TOM0_OT_LINK_RAW_SRC_MATCH_CLEAR_EXT_ENTRY,
  MSG_M4TOM0_OT_LINK_RAW_SRC_MATCH_CLEAR_SHORT_ENTRIES,
  MSG_M4TOM0_OT_LINK_RAW_SRC_MATCH_CLEAR_EXT_ENTRIES,
  /* NET_DATA */
  MSG_M4TOM0_OT_NET_DATA_GET,
  MSG_M4TOM0_OT_NET_DATA_GET_NEXT_ON_MESH_PREFIX,
  MSG_M4TOM0_OT_NET_DATA_GET_NEXT_ROUTE,
  MSG_M4TOM0_OT_NET_DATA_GET_VERSION,
  MSG_M4TOM0_OT_NET_DATA_GET_STABLE_VERSION,
  /* OPENTHREAD */
  MSG_M4TOM0_OT_OPENTHREAD_GET_VERSION,
  MSG_M4TOM0_OT_OPENTHREAD_ERROR_TO_STRING,
  /* TASKLET */
  MSG_M4TOM0_OT_TASKLETS_PROCESS,
  MSG_M4TOM0_OT_TASKLETS_ARE_PENDING,
  /* TMF_PROXY */
  MSG_M4TOM0_OT_TMF_PROXY_START,
  MSG_M4TOM0_OT_TMF_PROXY_STOP,
  MSG_M4TOM0_OT_TMF_PROXY_SEND,
  MSG_M4TOM0_OT_TMF_PROXY_IS_ENABLED,
  /* UDP */
  MSG_M4TOM0_OT_UDP_NEW_MESSAGE,
  MSG_M4TOM0_OT_UDP_OPEN,
  MSG_M4TOM0_OT_UDP_CLOSE,
  MSG_M4TOM0_OT_UDP_BIND,
  MSG_M4TOM0_OT_UDP_CONNECT,
  MSG_M4TOM0_OT_UDP_SEND,
  MSG_M4TOM0_OT_UDP_PROXY_SET_SENDER,
  MSG_M4TOM0_OT_UDP_PROXY_RECEIVE,
  /* CHANNEL_MANAGER */
  MSG_M4TOM0_OT_CHANNEL_MANAGER_REQUEST_CHANNEL_CHANGE,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_GET_REQUESTED_CHANNEL,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_GET_DELAY,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_SET_DELAY,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_REQUEST_CHANNEL_SELECT,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_SET_AUTO_CHANNEL_SELECTION_ENABLED,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_GET_AUTO_CHANNEL_SELECTION_ENABLED,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_SET_AUTO_CHANNEL_SELECTION_INTERVAL,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_GET_AUTO_CHANNEL_SELECTION_INTERVAL,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_GET_SUPPORTED_CHANNELS,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_SET_SUPPORTED_CHANNELS,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_GET_FAVORED_CHANNELS,
  MSG_M4TOM0_OT_CHANNEL_MANAGER_SET_FAVORED_CHANNELS,
  /* CHANNEL_MONITOR */
  MSG_M4TOM0_OT_CHANNEL_MONITOR_SET_ENABLED,
  MSG_M4TOM0_OT_CHANNEL_MONITOR_IS_ENABLED,
  MSG_M4TOM0_OT_CHANNEL_MONITOR_GET_SAMPLE_INTERVAL,
  MSG_M4TOM0_OT_CHANNEL_MONITOR_GET_RSSI_THRESHOLD,
  MSG_M4TOM0_OT_CHANNEL_MONITOR_GET_SAMPLE_WINDOW,
  MSG_M4TOM0_OT_CHANNEL_MONITOR_GET_SAMPLE_COUNT,
  MSG_M4TOM0_OT_CHANNEL_MONITOR_GET_CHANNEL_OCCUPANCY,
  /* CHILD_SUPERVISION */
  MSG_M4TOM0_OT_CHILD_SUPERVISION_GET_INTERVAL,
  MSG_M4TOM0_OT_CHILD_SUPERVISION_SET_INTERVAL,
  MSG_M4TOM0_OT_CHILD_SUPERVISION_GET_CHECK_TIMEOUT,
  MSG_M4TOM0_OT_CHILD_SUPERVISION_SET_CHECK_TIMEOUT,
  /* JAM_DETECTION */
  MSG_M4TOM0_OT_JAM_DETECTION_SET_RSSI_THRESHOLD,
  MSG_M4TOM0_OT_JAM_DETECTION_GET_RSSI_THRESHOLD,
  MSG_M4TOM0_OT_JAM_DETECTION_SET_WINDOW,
  MSG_M4TOM0_OT_JAM_DETECTION_GET_WINDOW,
  MSG_M4TOM0_OT_JAM_DETECTION_SET_BUSY_PERIOD,
  MSG_M4TOM0_OT_JAM_DETECTION_GET_BUSY_PERIOD,
  MSG_M4TOM0_OT_JAM_DETECTION_START,
  MSG_M4TOM0_OT_JAM_DETECTION_STOP,
  MSG_M4TOM0_OT_JAM_DETECTION_IS_ENABLED,
  MSG_M4TOM0_OT_JAM_DETECTION_GET_STATE,
  MSG_M4TOM0_OT_JAM_DETECTION_GET_HISTORY_BITMAP,
  /* SERVER */
  MSG_M4TOM0_OT_SERVER_GET_NET_DATA_LOCAL,
  MSG_M4TOM0_OT_SERVER_ADD_SERVICE,
  MSG_M4TOM0_OT_SERVER_REMOVE_SERVICE,
  MSG_M4TOM0_OT_SERVER_GET_NEXT_SERVICE,
  MSG_M4TOM0_OT_SERVER_GET_NEXT_LEADER_SERVICE,
  MSG_M4TOM0_OT_SERVER_REGISTER,
  /* NETWORK */
  MSG_M4TOM0_OT_NETWORK_TIME_GET,
  MSG_M4TOM0_OT_NETWORK_TIME_SET_SYNC_PERIOD,
  MSG_M4TOM0_OT_NETWORK_TIME_GET_SYNC_PERIOD,
  MSG_M4TOM0_OT_NETWORK_TIME_SET_XTAL_THRESHOLD,
  MSG_M4TOM0_OT_NETWORK_TIME_GET_XTAL_THRESHOLD,
  /* Crypto */
  MSG_M4TOM0_OT_CRYPTO_HMAC_SHA256,
  MSG_M4TOM0_OT_CRYPTO_AES_CCM,
  /* Radio platform */
  MSG_M4TOM0_OT_RADIO_SET_TRANSMIT_POWER,
  MSG_M4TOM0_OT_RADIO_GET_TRANSMIT_POWER
} MsgId_M4toM0_Enum_t;

/* List of messages sent by the M0 to the M4 */
typedef enum
{
  MSG_M0TOM4_SYNCHRO_INIT,
  MSG_M0TOM4_NOTIFY_STATE_CHANGE,
  MSG_M0TOM4_COAP_REQUEST_HANDLER,
  MSG_M0TOM4_COAP_RESPONSE_HANDLER,
  MSG_M0TOM4_NOTIFY_STACK_RESET,
  MSG_M0TOM4_IP6_RECEIVE,
  MSG_M0TOM4_IP6_SLAAC_IID_CREATE,
  MSG_M0TOM4_HANDLE_ACTIVE_SCAN_RESULT,
  MSG_M0TOM4_HANDLE_ENERGY_SCAN_RESULT,
  MSG_M0TOM4_HANDLE_LINK_PCAP,
  MSG_M0TOM4_RECEIVE_DIAGNOSTIC_GET_CALLBACK,
  MSG_M0TOM4_THREAD_FTD_CHILD_TABLE_CALLBACK,
  MSG_M0TOM4_COMMISSIONER_ENERGY_REPORT_CALLBACK,
  MSG_M0TOM4_COMMISSIONER_PANID_CONFLICT_CALLBACK,
  MSG_M0TOM4_DNS_RESPONSE_HANDLER,
  MSG_M0TOM4_ICMP6_RECEIVE_CALLBACK,
  MSG_M0TOM4_JOINER_CALLBACK,
  MSG_M0TOM4_LINK_RAW_RECEIVE_DONE,
  MSG_M0TOM4_LINK_RAW_TRANSMIT_DONE,
  MSG_M0TOM4_LINK_RAW_ENERGY_SCAN_DONE,
  MSG_M0TOM4_TMF_PROXY_STREAM_HANDLER,
  MSG_M0TOM4_UDP_RECEIVE,
  MSG_M0TOM4_JAM_DETECTION_CALLBACK,
  MSG_M0TOM4_TRACE_SEND,
  MSG_M0TOM4_DEFAULT_COAP_REQUEST_HANDLER
} MsgId_M0toM4_Enum_t;

/* List of modes available for UART configuration */
typedef enum
{
  SYS_LPUART1_CLI,
  SYS_USART1_CLI,
} Sys_ConfigUart_Enum_t;

/* List of modes available on the overall device  */
typedef enum
{
  SYS_PROTOCOL_BLE,
  SYS_PROTOCOL_THREAD,
} Sys_ConfigProtocol_Enum_t;

/* List of errors returned by the interface  */
typedef enum
{
  ERR_INTERFACE_IPCC_INIT_FAIL = 100U,
  ERR_INTERFACE_IPCC_REMOTE_FAIL = 101U,
  ERR_INTERFACE_IPCC_SEND_ACK_TO_M0 = 102U,
} Error_Interface_Id_Enum_t;

/* Gravity error level */
typedef enum
{
  ERR_INTERFACE_FATAL= 1U,
  ERR_INTERFACE_WARNING = 2U
} Error_Interface_Level_Enum_t;


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* STM32WBxx_CORE_INTERFACE_DEF_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
