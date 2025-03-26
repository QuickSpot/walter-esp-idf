#include <WalterModem.h>
#include <WalterDefines.h>

#if CONFIG_WALTER_MODEM_ENABLE_GNSS
bool WalterModem::configGNSS(
    WalterModemGNSSSensMode sensMode,
    WalterModemGNSSAcqMode acqMode,
    WalterModemGNSSLocMode locMode,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr(
                "AT+LPGNSSCFG=",
                _digitStr(locMode), ",",
                _digitStr(sensMode),
                ",2,,1,",
                _digitStr(acqMode)),
            "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::getGNSSAssistanceStatus(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr("AT+LPGNSSASSISTANCE?"), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::updateGNSSAssistance(
    WalterModemGNSSAssistanceType type,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr("AT+LPGNSSASSISTANCE=", _digitStr(type)), "+LPGNSSASSISTANCE:", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::performGNSSAction(
    WalterModemGNSSAction action,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    auto gnssActionStr = [](WalterModemGNSSAction action)
    {
        switch (action)
        {
        case WALTER_MODEM_GNSS_ACTION_GET_SINGLE_FIX:
            return "single";

        case WALTER_MODEM_GNSS_ACTION_CANCEL:
            return "stop";
        }
        return "";
    };

    _runCmd(arr("AT+LPGNSSFIXPROG=\"", gnssActionStr(action), "\""), "OK", rsp, cb, args);
    _returnAfterReply();
}

void WalterModem::_dispatchEvent(const WalterModemGNSSFix *fix)
{
    WalterModemEventHandler *handler = _eventHandlers + WALTER_MODEM_EVENT_TYPE_GNSS;
    if (handler->gnssHandler == nullptr)
    {
        return;
    }

    auto start = std::chrono::steady_clock::now();
    handler->gnssHandler(fix, handler->args);
    _checkEventDuration(start);
}
#endif