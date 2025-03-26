#include <WalterDefines.h>
#if CONFIG_WALTER_MODEM_ENABLE_HTTP
bool WalterModem::httpConfigProfile(
    uint8_t profileId,
    const char *serverName,
    uint16_t port,
    uint8_t tlsProfileId,
    bool useBasicAuth,
    const char *authUser,
    const char *authPass,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if (profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if (tlsProfileId && port == 80) {
        port = 443;
    }

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
    stringsBuffer->size +=
        sprintf((char *)stringsBuffer->data, "AT+SQNHTTPCFG=%d,\"%s\",%d,%d,\"%s\",\"%s\"",
                profileId, serverName, port, useBasicAuth, authUser, authPass);

    if (tlsProfileId) {
        stringsBuffer->size +=
            sprintf((char *)stringsBuffer->data + stringsBuffer->size, ",1,,,%u", tlsProfileId);
    }

    _runCmd(arr(
                (const char *)stringsBuffer->data),
            "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);

    _returnAfterReply();
}

bool WalterModem::httpConnect(
    uint8_t profileId,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if (profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if (_httpContextSet[profileId].connected) {
        _returnState(WALTER_MODEM_STATE_OK);
    }

    _runCmd(arr("AT+SQNHTTPCONNECT=", _atNum(profileId)), "+SQNHTTPCONNECT: ", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::httpClose(
    uint8_t profileId,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if (profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    _runCmd(arr("AT+SQNHTTPDISCONNECT=", _atNum(profileId)), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::httpGetContextStatus(uint8_t profileId)
{
    if (profileId >= WALTER_MODEM_MAX_HTTP_PROFILES)
    {
        return false;
    }

    /* note: in my observation the SQNHTTPCONNECT command is to be avoided.
     * if the connection is closed by the server, you will not even
     * receive a +SQNHTTPSH disconnected message (you will on the next
     * connect attempt). reconnect will be impossible even if you try
     * to manually disconnect.
     * and a SQNHTTPQRY will still work and create its own implicit connection.
     *
     * (too bad: according to the docs SQNHTTPCONNECT is mandatory for
     * TLS connections)
     */
    return _httpContextSet[profileId].connected;
}

bool WalterModem::httpQuery(
    uint8_t profileId,
    const char *uri,
    WalterModemHttpQueryCmd httpQueryCmd,
    char *contentTypeBuf,
    uint16_t contentTypeBufSize,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if (profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if (_httpContextSet[profileId].state != WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE) {
        _returnState(WALTER_MODEM_STATE_BUSY);
    }

    _httpContextSet[profileId].contentType = contentTypeBuf;
    _httpContextSet[profileId].contentTypeSize = contentTypeBufSize;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemHttpContext *ctx = (WalterModemHttpContext *)cmd->completeHandlerArg;

        if (result == WALTER_MODEM_STATE_OK) {
            ctx->state = WALTER_MODEM_HTTP_CONTEXT_STATE_EXPECT_RING;
        }
    };

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
    stringsBuffer->size += sprintf((char *)stringsBuffer->data, "AT+SQNHTTPQRY=%d,%d,\"%s\"",
        profileId, httpQueryCmd, uri);

    _runCmd(arr(
                (const char *)stringsBuffer->data),
            "OK", rsp, cb, args, completeHandler, (void *)(_httpContextSet + profileId),
            WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);

    _returnAfterReply();
}

bool WalterModem::httpSend(
    uint8_t profileId,
    const char *uri,
    uint8_t *data,
    uint16_t dataSize,
    WalterModemHttpSendCmd httpSendCmd,
    WalterModemHttpPostParam httpPostParam,
    char *contentTypeBuf,
    uint16_t contentTypeBufSize,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if (profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if (_httpContextSet[profileId].state != WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE) {
        _returnState(WALTER_MODEM_STATE_BUSY);
    }

    _httpContextSet[profileId].contentType = contentTypeBuf;
    _httpContextSet[profileId].contentTypeSize = contentTypeBufSize;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemHttpContext *ctx = (WalterModemHttpContext *)cmd->completeHandlerArg;

        if (result == WALTER_MODEM_STATE_OK) {
            ctx->state = WALTER_MODEM_HTTP_CONTEXT_STATE_EXPECT_RING;
        }
    };

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
    if (httpPostParam == WALTER_MODEM_HTTP_POST_PARAM_UNSPECIFIED)
    {
        stringsBuffer->size += sprintf((char *)stringsBuffer->data,
        "AT+SQNHTTPSND=%d,%d,\"%s\",%d", profileId, httpSendCmd, uri, dataSize);
    }
    else
    {
        stringsBuffer->size += sprintf((char *)stringsBuffer->data,
        "AT+SQNHTTPSND=%d,%d,\"%s\",%d,\"%d\"", profileId, httpSendCmd, uri, dataSize,
        httpPostParam);
    }

    _runCmd(arr(
                (const char *)stringsBuffer->data),
            "OK", rsp, cb, args, completeHandler, (void *)(_httpContextSet + profileId),
            WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, data, dataSize, stringsBuffer);

    _returnAfterReply();
}
bool WalterModem::httpDidRing(
    uint8_t profileId,
    uint8_t *targetBuf,
    uint16_t targetBufSize,
    WalterModemRsp *rsp)
{
    /* this is by definition a blocking call without callback.
     * it is only used when the arduino user is not taking advantage of
     * the (TBI) ring notification events.
     */
    walterModemCb cb = NULL;
    void *args = NULL;

    if(_httpCurrentProfile != 0xff) {
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if(_httpContextSet[profileId].state == WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE) {
        _returnState(WALTER_MODEM_STATE_NOT_EXPECTING_RING);
    }

    if(_httpContextSet[profileId].state == WALTER_MODEM_HTTP_CONTEXT_STATE_EXPECT_RING) {
        _returnState(WALTER_MODEM_STATE_AWAITING_RING);
    }

    if(_httpContextSet[profileId].state != WALTER_MODEM_HTTP_CONTEXT_STATE_GOT_RING) {
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    /* ok, got ring. http context fields have been filled.
     * http status 0 means: timeout (or also disconnected apparently) */
    if(_httpContextSet[profileId].httpStatus == 0) {
        _httpContextSet[profileId].state = WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE;
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    if(_httpContextSet[profileId].contentLength == 0) {
        _httpContextSet[profileId].state = WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE;
        rsp->type = WALTER_MODEM_RSP_DATA_TYPE_HTTP_RESPONSE;
        rsp->data.httpResponse.httpStatus = _httpContextSet[profileId].httpStatus;
        rsp->data.httpResponse.contentLength = 0;
        _returnState(WALTER_MODEM_STATE_NO_DATA);
    }

    _httpCurrentProfile = profileId;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) {
        _httpContextSet[_httpCurrentProfile].state = WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE;
        _httpCurrentProfile = 0xff;
    };

    _runCmd(arr(
        "AT+SQNHTTPRCV=",
        _atNum(profileId)), "<<<",
        rsp, cb, args, completeHandler, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, targetBuf,
        targetBufSize);
    _returnAfterReply();
}
#endif