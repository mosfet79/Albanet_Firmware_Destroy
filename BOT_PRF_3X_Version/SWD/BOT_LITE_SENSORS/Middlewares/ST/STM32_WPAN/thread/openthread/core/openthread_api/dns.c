/**
  ******************************************************************************
  * @file    dns.c
  * @author  MCD Application Team
  * @brief   This file contains the DNS interface shared between M0 and
  *          M4.
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


/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"

#include "stm32wbxx_core_interface_def.h"
#include "tl_thread_hci.h"

/* Include definition of compilation flags requested for OpenThread configuration */
#include OPENTHREAD_CONFIG_FILE

#include "dns.h"


#if OPENTHREAD_CONFIG_DNS_CLIENT_ENABLE

extern otDnsResponseHandler otDnsResponseHandlerCb;

otError otDnsClientQuery(otInstance *         aInstance,
                         const otDnsQuery *   aQuery,
                         otDnsResponseHandler aHandler,
                         void *               aContext)
{
    Pre_OtCmdProcessing();
    otDnsResponseHandlerCb = aHandler;
    /* prepare buffer */
    Thread_OT_Cmd_Request_t* p_ot_req = THREAD_Get_OTCmdPayloadBuffer();

    p_ot_req->ID = MSG_M4TOM0_OT_DNS_CLIENT_QUERY;

    p_ot_req->Size=2;
    p_ot_req->Data[0] = (uint32_t) aQuery;
    p_ot_req->Data[1] = (uint32_t) aContext;

    Ot_Cmd_Transfer();

    p_ot_req = THREAD_Get_OTCmdRspPayloadBuffer();
    return (otError)p_ot_req->Data[0];
}

#endif /* OPENTHREAD_CONFIG_DNS_CLIENT_ENABLE */
