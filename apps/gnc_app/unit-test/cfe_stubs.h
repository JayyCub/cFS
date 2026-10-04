#ifndef CFE_STUBS_H
#define CFE_STUBS_H

#include "cfe.h"

typedef void (*Stub_TransmitHook_t)(const CFE_MSG_Message_t *MsgPtr);
typedef void (*Stub_EventHook_t)(uint16 EventID, uint16 EventType, const char *Text);

extern void               *Stub_TblPtr;   /* what CFE_TBL_GetAddress returns          */
extern CFE_MSG_Size_t      Stub_MsgSize;  /* what CFE_MSG_GetSize returns             */
extern CFE_MSG_FcnCode_t   Stub_FcnCode;  /* what CFE_MSG_GetFcnCode returns          */
extern Stub_TransmitHook_t Stub_OnTransmit;
extern Stub_EventHook_t    Stub_OnEvent;

#endif /* CFE_STUBS_H */
