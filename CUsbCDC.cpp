/*!
    \file
    \brief Класс-обертка для tinyUSB CDC (Communication Device Class)
    \authors Близнец Р.А. (r.bliznets@gmail.com)
    \version 0.1.0.0
    \date 16.04.2024

    Реализует Singleton-паттерн. Позволяет работать с USB CDC интерфейсом
    как с последовательным портом. Обрабатывает события подключения и приема данных.
    Один объект на приложение.
*/

#include "sdkconfig.h"
#ifdef CONFIG_TINYUSB_CDC_ENABLED

#include "CUsbCDC.h"
#include "esp_log.h"
#include "CTrace.h"
#include <cstring>
#include "esp_sleep.h"

// Статические члены класса
int8_t CUsbCDC::mWakeUpPin = -1;               ///< Пин для пробуждения от сна
CUsbCDC *CUsbCDC::theSingleInstance = nullptr; ///< Единственный экземпляр класса

// Обработчик приема данных через CDC
void CUsbCDC::cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    // Делегируем обработку в метод экземпляра класса
    CUsbCDC::Instance()->rx((tinyusb_cdcacm_itf_t)itf);
}

// Обработчик изменения состояния линии связи (DTR сигнал)
void CUsbCDC::cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    // Уведомляем о подключении/отключении
    if (CUsbCDC::Instance()->onConnect != nullptr)
        CUsbCDC::Instance()->onConnect(itf, (dtr == 1));
}

// Обработка полученных данных
void CUsbCDC::rx(tinyusb_cdcacm_itf_t itf)
{
    size_t rx_size = 0;
    esp_err_t ret;

    // Читаем данные порциями до полного опустошения буфера
    for (;;)
    {
        // Чтение данных в промежуточный буфер
        ret = tinyusb_cdcacm_read(itf, mRxBuf0, USB_MAX_DATA, &rx_size);
        if (ret != ESP_OK)
        {
            TRACE_E("CUsbCDC tinyusb_cdcacm_read failed", ret, false);
            return;
        }

        // Если данные получены - передаем их обработчику
        if (rx_size != 0)
        {
            if (onCmd != nullptr)
                onCmd(itf, mRxBuf0, rx_size); // Пользовательский callback
            else
                TRACEDATA("cdc rx", mRxBuf0, rx_size); // Логирование по умолчанию
        }

        // Выход при неполном буфере (больше данных нет)
        if (rx_size < USB_MAX_DATA)
            return;
    }
}

// Инициализация CDC интерфейса
void CUsbCDC::start(onCDCDataRx *func, onCDCConect *connect)
{
#if CONFIG_PM_ENABLE
    // Фиксируем максимальную частоту CPU для стабильной работы USB
    esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "usb", &mPMLock);
    ESP_ERROR_CHECK(esp_pm_lock_acquire(mPMLock));
#endif

    // Базовая конфигурация USB-устройства
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = nullptr, // Использовать дефолтный дескриптор
        .string_descriptor = nullptr, // Без строковых дескрипторов
        .external_phy = false,        // Внутренний PHY
        .configuration_descriptor = nullptr,
        .self_powered = true,         // Питание от самодостаточного источника
        .vbus_monitor_io = mWakeUpPin // Пин для мониторинга питания
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    // Конфигурация ACM (Abstract Control Model) интерфейса
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,                                    // Использовать первый USB-контроллер
        .cdc_port = TINYUSB_CDC_ACM_0,                                  // Первый CDC порт
        // .rx_unread_buf_sz = USB_MAX_DATA,                               // Размер буфера приема
        .callback_rx = &cdc_rx_callback,                                // Callback на прием данных
        .callback_line_state_changed = &cdc_line_state_changed_callback // Callback на изменение DTR
    };

    // Инициализация ACM интерфейсов
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    // Инициализация второго интерфейса при необходимости
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
#endif

    // Сохранение пользовательских обработчиков
    onCmd = func;
    onConnect = connect;
}

// Деинициализация CDC интерфейса
void CUsbCDC::stop()
{
    TRACE_W("CUsbCDC off until reboot", -100, false);
#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    ESP_ERROR_CHECK(tusb_cdc_acm_deinit(TINYUSB_CDC_ACM_1));
#endif
    ESP_ERROR_CHECK(tusb_cdc_acm_deinit(TINYUSB_CDC_ACM_0));
    ESP_ERROR_CHECK(tinyusb_driver_uninstall());

#if CONFIG_PM_ENABLE
    // Снятие ограничений по частоте CPU
    esp_pm_lock_release(mPMLock);
    esp_pm_lock_delete(mPMLock);
#endif
}

// Отправка данных через CDC интерфейс
bool CUsbCDC::send(int itf, uint8_t *data, size_t size)
{
    // Постановка данных в очередь отправки
    size_t sz = tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)itf, data, size);
    if (sz == 0)
        return false; // Ошибка постановки в очередь

    // Дозапись оставшихся данных (если буфер заполнялся частями)
    while (sz != size)
    {
        size_t s = tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)itf, &data[sz], size - sz);
        if (s != 0)
            sz += s;
        else
            return false;
    }

    // Синхронная отправка данных с таймаутом 100 мс
    if (tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)itf, 100) != ESP_OK)
        return false;

    return true;
}

#endif // CONFIG_TINYUSB_CDC_ENABLED