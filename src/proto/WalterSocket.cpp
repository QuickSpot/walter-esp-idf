#include <WalterDefines.h>

WalterModemSocket *WalterModem::_socketReserve()
{
    WalterModemSocket *sock = NULL;

    for (int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i)
    {
        if (_socketSet[i].state == WALTER_MODEM_SOCKET_STATE_FREE)
        {
            sock = _socketSet + i;
            sock->state = WALTER_MODEM_SOCKET_STATE_RESERVED;
            sock->id = i + 1;
            break;
        }
    }

    if (sock != NULL)
    {
        _socket = sock;
    }

    return sock;
}

WalterModemSocket *WalterModem::_socketGet(int id)
{
    if (id < 0)
    {
        return _socket;
    }

    for (int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i)
    {
        if (_socketSet[i].state != WALTER_MODEM_SOCKET_STATE_FREE && _socketSet[i].id == id)
        {
            _socket = _socketSet + i;
            return _socketSet + i;
        }
    }

    return NULL;
}
void WalterModem::_socketRelease(WalterModemSocket *sock)
{
    if (sock == NULL)
    {
        return;
    }

    sock->state = WALTER_MODEM_SOCKET_STATE_FREE;
}

bool WalterModem::createSocket(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    int pdpCtxId,
    uint16_t mtu,
    uint16_t exchangeTimeout,
    uint16_t connTimeout,
    uint16_t sendDelayMs)
{
    WalterModemPDPContext *ctx = _pdpContextGet(pdpCtxId);
    if (ctx == NULL)
    {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PDP_CONTEXT);
    }

    WalterModemSocket *sock = _socketReserve();
    if (sock == NULL)
    {
        _returnState(WALTER_MODEM_STATE_NO_FREE_SOCKET);
    }

    sock->pdpContextId = ctx->id;
    sock->mtu = mtu;
    sock->exchangeTimeout = exchangeTimeout;
    sock->connTimeout = connTimeout;
    sock->sendDelayMs = sendDelayMs;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemSocket *sock = (WalterModemSocket *)cmd->completeHandlerArg;

        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_SOCKET_ID;
        cmd->rsp->data.socketId = sock->id;

        if (result == WALTER_MODEM_STATE_OK)
        {
            sock->state = WALTER_MODEM_SOCKET_STATE_CREATED;
        }
    };

    _runCmd(arr(
                "AT+SQNSCFG=",
                _digitStr(sock->id), ",",
                _digitStr(sock->pdpContextId), ",",
                _atNum(sock->mtu), ",",
                _atNum(sock->exchangeTimeout), ",",
                _atNum(sock->connTimeout * 10), ",",
                _atNum(sock->sendDelayMs / 100)),
            "OK", rsp, cb, args, completeHandler, sock);
    _returnAfterReply();
}

bool WalterModem::configSocket(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    int socketId)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL)
    {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemSocket *sock = (WalterModemSocket *)cmd->completeHandlerArg;

        if (result == WALTER_MODEM_STATE_OK)
        {
            sock->state = WALTER_MODEM_SOCKET_STATE_CONFIGURED;
        }
    };

    _runCmd(arr(
                "AT+SQNSCFGEXT=",
                _digitStr(sock->id), ",2,0,0,0,0,0"),
            "OK", rsp, cb, args, completeHandler, sock);
    _returnAfterReply();
}

bool WalterModem::connectSocket(
    const char *remoteHost,
    uint16_t remotePort,
    uint16_t localPort,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    WalterModemSocketProto protocol,
    WalterModemSocketAcceptAnyRemote acceptAnyRemote,
    int socketId)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL)
    {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    sock->protocol = protocol;
    sock->acceptAnyRemote = acceptAnyRemote;
    _strncpy_s(sock->remoteHost, remoteHost, WALTER_MODEM_HOSTNAME_MAX_SIZE);
    sock->remotePort = remotePort;
    sock->localPort = localPort;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemSocket *sock = (WalterModemSocket *)cmd->completeHandlerArg;

        if (result == WALTER_MODEM_STATE_OK)
        {
            sock->state = WALTER_MODEM_SOCKET_STATE_OPENED;
        }
    };

    _runCmd(arr(
                "AT+SQNSD=",
                _digitStr(sock->id), ",",
                _digitStr(sock->protocol), ",",
                _atNum(sock->remotePort), ",",
                _atStr(sock->remoteHost), ",0,",
                _atNum(sock->localPort), ",1,",
                _digitStr(sock->acceptAnyRemote), ",0"),
            "OK", rsp, cb, args, completeHandler, sock);
    _returnAfterReply();
}

bool WalterModem::closeSocket(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    int socketId)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL)
    {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemSocket *sock = (WalterModemSocket *)cmd->completeHandlerArg;

        if (result == WALTER_MODEM_STATE_OK)
        {
            _socketRelease(sock);
        }
    };

    _runCmd(arr(
                "AT+SQNSH=",
                _digitStr(sock->id)),
            "OK", rsp, cb, args, completeHandler, sock);
    _returnAfterReply();
}

bool WalterModem::socketSend(
    uint8_t *data,
    uint16_t dataSize,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    WalterModemRAI rai,
    int socketId)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL)
    {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    _runCmd(arr(
                "AT+SQNSSENDEXT=",
                _digitStr(sock->id), ",",
                _atNum(dataSize), ",",
                _digitStr(rai)),
            "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, data, dataSize);
    _returnAfterReply();
}

bool WalterModem::socketSend(
    char *str,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    WalterModemRAI rai,
    int socketId)
{
    return socketSend((uint8_t *)str, strlen(str), rsp, cb, args, rai, socketId);
}