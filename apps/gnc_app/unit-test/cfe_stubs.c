/*
** Minimal cFE stand-ins so gnc_app.c can run inside the Monte Carlo harness
** (dock_mc.c) without the cFE executive. Only what gnc_app.c calls is here:
**   - CFE_TBL_GetAddress hands out the table the harness points it at
**   - CFE_SB_TransmitMsg hands every outgoing message to the harness
**   - CFE_MSG_GetSize / GetFcnCode report what the harness set for the
**     command it is about to inject
**   - CFE_EVS_SendEvent formats and forwards events (verbose mode prints them)
** Everything else succeeds and does nothing.
*/
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "cfe.h"
#include "cfe_stubs.h"

void              *Stub_TblPtr      = NULL;
CFE_MSG_Size_t     Stub_MsgSize     = 0;
CFE_MSG_FcnCode_t  Stub_FcnCode     = 0;
Stub_TransmitHook_t Stub_OnTransmit = NULL;
Stub_EventHook_t   Stub_OnEvent     = NULL;

CFE_Status_t CFE_EVS_Register(const void *Filters, uint16 NumEventFilters, uint16 FilterScheme)
{
    (void)Filters;
    (void)NumEventFilters;
    (void)FilterScheme;
    return CFE_SUCCESS;
}

CFE_Status_t CFE_EVS_SendEvent(uint16 EventID, uint16 EventType, const char *Spec, ...)
{
    char    text[256];
    va_list ap;

    va_start(ap, Spec);
    vsnprintf(text, sizeof(text), Spec, ap);
    va_end(ap);
    if (Stub_OnEvent != NULL)
        Stub_OnEvent(EventID, EventType, text);
    return CFE_SUCCESS;
}

CFE_Status_t CFE_ES_WriteToSysLog(const char *SpecStringPtr, ...)
{
    (void)SpecStringPtr;
    return CFE_SUCCESS;
}

bool CFE_ES_RunLoop(uint32 *RunStatus)
{
    (void)RunStatus;
    return false;
}

void CFE_ES_ExitApp(uint32 ExitStatus)
{
    (void)ExitStatus;
}

CFE_Status_t CFE_MSG_Init(CFE_MSG_Message_t *MsgPtr, CFE_SB_MsgId_t MsgId, CFE_MSG_Size_t Size)
{
    (void)MsgPtr;
    (void)MsgId;
    (void)Size;
    return CFE_SUCCESS;
}

CFE_Status_t CFE_MSG_GetSize(const CFE_MSG_Message_t *MsgPtr, CFE_MSG_Size_t *Size)
{
    (void)MsgPtr;
    *Size = Stub_MsgSize;
    return CFE_SUCCESS;
}

CFE_Status_t CFE_MSG_GetMsgId(const CFE_MSG_Message_t *MsgPtr, CFE_SB_MsgId_t *MsgId)
{
    (void)MsgPtr;
    *MsgId = CFE_SB_INVALID_MSG_ID;
    return CFE_SUCCESS;
}

CFE_Status_t CFE_MSG_GetFcnCode(const CFE_MSG_Message_t *MsgPtr, CFE_MSG_FcnCode_t *FcnCode)
{
    (void)MsgPtr;
    *FcnCode = Stub_FcnCode;
    return CFE_SUCCESS;
}

CFE_Status_t CFE_SB_CreatePipe(CFE_SB_PipeId_t *PipeIdPtr, uint16 Depth, const char *PipeName)
{
    (void)Depth;
    (void)PipeName;
    memset(PipeIdPtr, 0, sizeof(*PipeIdPtr));
    return CFE_SUCCESS;
}

CFE_Status_t CFE_SB_Subscribe(CFE_SB_MsgId_t MsgId, CFE_SB_PipeId_t PipeId)
{
    (void)MsgId;
    (void)PipeId;
    return CFE_SUCCESS;
}

CFE_Status_t CFE_SB_ReceiveBuffer(CFE_SB_Buffer_t **BufPtr, CFE_SB_PipeId_t PipeId, int32 TimeOut)
{
    (void)BufPtr;
    (void)PipeId;
    (void)TimeOut;
    return CFE_SB_NO_MESSAGE;
}

void CFE_SB_TimeStampMsg(CFE_MSG_Message_t *MsgPtr)
{
    (void)MsgPtr;
}

CFE_Status_t CFE_SB_TransmitMsg(const CFE_MSG_Message_t *MsgPtr, bool IsOrigination)
{
    (void)IsOrigination;
    if (Stub_OnTransmit != NULL)
        Stub_OnTransmit(MsgPtr);
    return CFE_SUCCESS;
}

CFE_Status_t CFE_TBL_Register(CFE_TBL_Handle_t *TblHandlePtr, const char *Name, size_t Size, uint16 TblOptionFlags,
                              CFE_TBL_CallbackFuncPtr_t TblValidationFuncPtr)
{
    (void)Name;
    (void)Size;
    (void)TblOptionFlags;
    (void)TblValidationFuncPtr;
    *TblHandlePtr = 0;
    return CFE_SUCCESS;
}

CFE_Status_t CFE_TBL_Load(CFE_TBL_Handle_t TblHandle, CFE_TBL_SrcEnum_t SrcType, const void *SrcDataPtr)
{
    (void)TblHandle;
    (void)SrcType;
    (void)SrcDataPtr;
    return CFE_SUCCESS;
}

CFE_Status_t CFE_TBL_GetAddress(void **TblPtr, CFE_TBL_Handle_t TblHandle)
{
    (void)TblHandle;
    *TblPtr = Stub_TblPtr;
    return CFE_SUCCESS;
}

CFE_Status_t CFE_TBL_Manage(CFE_TBL_Handle_t TblHandle)
{
    (void)TblHandle;
    return CFE_SUCCESS;
}
