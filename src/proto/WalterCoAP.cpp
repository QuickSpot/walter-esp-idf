#include <WalterDefines.h>

bool WalterModem::coapDidRing(
    uint8_t profileId,
    uint8_t *targetBuf,
    uint16_t targetBufSize,
    WalterModemRsp *rsp)
{
    /* this is by definition a blocking call without callback.
     * it is only used when the arduino user is not taking advantage of
     * the (TBI) ring notification events which give access to the raw
     * buffer (a targetBuf is not needed).
     */
    walterModemCb cb = NULL;
    void *args = NULL;

    if (profileId == 0)
    {
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    if (profileId >= WALTER_MODEM_MAX_COAP_PROFILES)
    {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    uint8_t ringIdx;
    for (ringIdx = 0; ringIdx < WALTER_MODEM_COAP_MAX_PENDING_RINGS; ringIdx++)
    {
        if (_coapContextSet[profileId].rings[ringIdx].messageId)
        {
            break;
        }
    }

    if (ringIdx == WALTER_MODEM_COAP_MAX_PENDING_RINGS)
    {
        _returnState(WALTER_MODEM_STATE_NO_DATA);
    }

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
    stringsBuffer->size +=
        sprintf((char *)stringsBuffer->data, "AT+SQNCOAPRCV=%d,%u,%u", profileId,
                _coapContextSet[profileId].rings[ringIdx].messageId,
                _coapContextSet[profileId].rings[ringIdx].length);

    _runCmd(arr(
                (const char *)stringsBuffer->data),
            "+SQNCOAPRCV: ", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, targetBuf,
            targetBufSize, stringsBuffer);

    _returnAfterReply();
}