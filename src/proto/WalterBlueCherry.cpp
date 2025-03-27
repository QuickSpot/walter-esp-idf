#include <WalterDefines.h>

#if CONFIG_WALTER_MODEM_ENABLE_COAP
#pragma region PRIVATE_METHODS
bool WalterModem::_processBlueCherryEvent(uint8_t *data, uint8_t len)
{
    switch (data[0])
    {
    case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_INITIALIZE:
        return _processOtaInitializeEvent(data + 1, len - 1);

    case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_CHUNK:
        return _processOtaChunkEvent(data + 1, len - 1);

    case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_FINISH:
        return _processOtaFinishEvent();

    case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_INITIALIZE:
        return _processMotaInitializeEvent(data + 1, len - 1);

    case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_CHUNK:
        return _processMotaChunkEvent(data + 1, len - 1);

    case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_FINISH:
        return _processMotaFinishEvent();

    default:
        ESP_LOGD("WalterModem", "Error: invalid BlueCherry event type 0x%x from cloud server",
                 data[0]);
        return true;
    }

    return true;
}
#pragma endregion


#endif