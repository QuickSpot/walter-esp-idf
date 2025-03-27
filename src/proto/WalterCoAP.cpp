#include <WalterDefines.h>
#pragma region PUBLIC_METHODS
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

bool WalterModem::coapCreateContext(
    uint8_t profileId,
    const char *serverName,
    int port,
    uint8_t tlsProfileId,
    int localPort,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if(_coapContextSet[profileId].connected) {
        _returnState(WALTER_MODEM_STATE_OK);
    }

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data, "AT+SQNCOAPCREATE=%d,\"%s\",%d,", profileId,
        serverName, port);

    if(localPort > -1) {
        stringsBuffer->size +=
            sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%d", localPort);
    }

    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",%d,60", tlsProfileId != 0);

    if(tlsProfileId) {
        stringsBuffer->size +=
            sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",,%d", tlsProfileId);
    }

    _runCmd(arr(
        (const char*) stringsBuffer->data),
        "+SQNCOAPCONNECTED: ", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0,
        stringsBuffer);

    _returnAfterReply();
}

bool WalterModem::coapClose(uint8_t profileId, WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    if(profileId == 0) {
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    _runCmd(arr(
        "AT+SQNCOAPCLOSE=",
        _atNum(profileId)),
        "+SQNCOAPCLOSED: ", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::coapGetContextStatus(uint8_t profileId)
{
    if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
        return false;
    }

    return _coapContextSet[profileId].connected;
}

bool WalterModem::coapSetHeader(
    uint8_t profileId,
    int messageId,
    const char *token,
    WalterModemRsp *rsp ,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr(
        "AT+SQNCOAPHDR=",
        _atNum(profileId), ",",
        _atNum(messageId),
        ",\"", token, "\""),
        "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::coapSetOptions(
    uint8_t profileId,
    WalterModemCoapOptAction action,
    WalterModemCoapOptCode code,
    const char *const values,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(profileId == 0) {
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    if(action == WALTER_MODEM_COAP_OPT_READ) {
         /* not yet supported - add together with incoming socket con/coap response */
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();

    if(action == WALTER_MODEM_COAP_OPT_SET || action == WALTER_MODEM_COAP_OPT_EXTEND) {
         if(values && *values) {
            stringsBuffer->size +=
                sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d,\"%s\"", profileId,
                action, code, values);
         } else {
            stringsBuffer->size +=
                sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d", profileId, action,
                code);
         }
     } else if(action == WALTER_MODEM_COAP_OPT_DELETE) {
        stringsBuffer->size +=
            sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d", profileId, action, code);
     } else {
         /* make sure something sane is in the buffer if wrong action */
         stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT");
     }

    _runCmd(arr(
        (const char*) stringsBuffer->data),
        "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);
     _returnAfterReply();
}

bool WalterModem::coapSendData(
    uint8_t profileId,
    WalterModemCoapSendType type,
    WalterModemCoapSendMethodRsp methodRsp,
    int length,
    uint8_t *payload,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr(
        "AT+SQNCOAPSEND=",
        _atNum(profileId), ",",
        _atNum(type), ",",
        _atNum(methodRsp), ",",
        _atNum(length)),
        "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, payload, length);
    _returnAfterReply();
}
#pragma endregion
