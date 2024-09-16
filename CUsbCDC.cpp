/*!
    \file
    \brief Класс обертка для tinyUSB СDС.
    \authors Близнец Р.А. (r.bliznets@gmail.com)
    \version 0.1.0.0
    \date 16.04.2024

    Один объект на приложение.
*/

#include "sdkconfig.h"
#ifdef CONFIG_TINYUSB_CDC_ENABLED

#include "CUsbCDC.h"
#include "esp_log.h"
#include "CTrace.h"
#include <cstring>

#include "esp_sleep.h"

int8_t CUsbCDC::mWakeUpPin = -1;

void CUsbCDC::cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    CUsbCDC::Instance()->rx((tinyusb_cdcacm_itf_t)itf);
}

void CUsbCDC::rx(tinyusb_cdcacm_itf_t itf)
{
    size_t rx_size = 0;
    esp_err_t ret;

    for (;;)
    {
        ret = tinyusb_cdcacm_read(itf, mRxBuf0, USB_MAX_DATA, &rx_size);
        if (ret != ESP_OK)
        {
            TRACE_E("CUsbCDC tinyusb_cdcacm_read failed", ret, false);
            return;
        }
        if (rx_size != 0)
        {
            if (onCmd != nullptr)
                onCmd(itf, mRxBuf0, rx_size);
            else
                TRACEDATA("cdc rx", mRxBuf0, rx_size);
        }
        if (rx_size < USB_MAX_DATA)
            return;
    }
}

void CUsbCDC::cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    if (CUsbCDC::Instance()->onConnect != nullptr)
        CUsbCDC::Instance()->onConnect(itf, (dtr == 1));
}

void CUsbCDC::start(onCDCDataRx *func, onCDCConect *connect)
{
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = nullptr,
        .string_descriptor = nullptr,
        .string_descriptor_count = 0,
        .external_phy = false,
        .configuration_descriptor = nullptr,
        .self_powered = true,
        .vbus_monitor_io = mWakeUpPin};

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = USB_MAX_DATA,
        .callback_rx = &cdc_rx_callback,
        .callback_rx_wanted_char = nullptr,
        .callback_line_state_changed = &cdc_line_state_changed_callback,
        .callback_line_coding_changed = nullptr};

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
#endif

    onCmd = func;
    onConnect = connect;
}

void CUsbCDC::stop()
{
    TRACE_W("CUsbCDC off until reboot", -100, false);
#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    ESP_ERROR_CHECK(tusb_cdc_acm_deinit(TINYUSB_CDC_ACM_1));
#endif
    ESP_ERROR_CHECK(tusb_cdc_acm_deinit(TINYUSB_CDC_ACM_0));
    ESP_ERROR_CHECK(tinyusb_driver_uninstall());
}

bool CUsbCDC::send(int itf, uint8_t *data, size_t size)
{
    size_t sz = tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)itf, data, size);
    if (sz == 0)
        return false;
    while (sz != size)
    {
        sz += tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)itf, &data[sz], size - sz);
    }
    ESP_ERROR_CHECK(tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)itf, 100));
    return true;
}

#endif // CONFIG_TINYUSB_CDC_ENABLED
