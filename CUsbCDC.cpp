/*!
    \file
    \brief Wrapper class for tinyUSB CDC (Communication Device Class)
    \authors Bliznets R.A. (r.bliznets@gmail.com)
    \version 1.0.0.0
    \date 16.04.2024

    Implements Singleton pattern. Allows working with USB CDC interface
    as with a serial port. Handles connection and data reception events.
    One object per application.
*/

#include "sdkconfig.h"
#ifdef CONFIG_TINYUSB_CDC_ENABLED

#include "CUsbCDC.h"
#include "esp_log.h"
#include <cstring>
#include "esp_sleep.h"
#include "tinyusb_default_config.h"

// Static class members initialization
// Initialize pin for wake-up from sleep (default: -1 = disabled)
int8_t CUsbCDC::mWakeUpPin = -1;               
// USB Device Task priority (default: 5 - medium priority)
uint8_t CUsbCDC::mPriority = 5;                
// USB Device Task core affinity (default: core 1)
int CUsbCDC::mCoreID = 1;                      
// Singleton instance pointer - points to the single instance of this class
CUsbCDC *CUsbCDC::theSingleInstance = nullptr; 

#if CONFIG_LOG_DEFAULT_LEVEL > 0
static const char* TAG = "CUsbCDC";
#endif

// Static callback function for receiving data through CDC
// This function is called by the tinyUSB library when data is received
// It delegates the processing to the instance method rx()
void CUsbCDC::cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    // Delegate processing to instance method
    CUsbCDC::Instance()->rx((tinyusb_cdcacm_itf_t)itf);
}

// Static device event handler for USB attachment/detachment events
// Handles TINYUSB_EVENT_ATTACHED and TINYUSB_EVENT_DETACHED events
void CUsbCDC::device_event_handler(tinyusb_event_t *event, void *arg)
{
    // ESP_LOGI(TAG, "event %d", event->id); // Debug logging macro (commented out)
    switch (event->id)
    {
    case TINYUSB_EVENT_ATTACHED:
        // Call user-defined connection callback if available
        if (CUsbCDC::Instance()->onConnect != nullptr)
            CUsbCDC::Instance()->onConnect(-1, TINYUSB_ATTACHED);
        break;
    case TINYUSB_EVENT_DETACHED:
        // Call user-defined disconnection callback if available
        if (CUsbCDC::Instance()->onConnect != nullptr)
            CUsbCDC::Instance()->onConnect(-1, TINYUSB_DETACHED);
        break;
    }
}

// Static callback for line state changes (DTR/RTS signals)
// Handles changes in DTR (Data Terminal Ready) and RTS (Request To Send) signals
void CUsbCDC::cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    // ESP_LOGI(TAG, "rts %d", event->line_state_changed_data.rts); // Debug logging (commented out)
    // ESP_LOGI(TAG, "dtr %d", event->line_state_changed_data.dtr); // Debug logging (commented out)
    
    if (CUsbCDC::Instance()->onConnect != nullptr)
    {
        uint32_t x = 0;
        // Set bit flags based on line state
        if (event->line_state_changed_data.rts)
            x |= TINYUSB_CDC_RTS;  // Set RTS flag if RTS is active
        if (event->line_state_changed_data.dtr)
            x |= TINYUSB_CDC_DTR;  // Set DTR flag if DTR is active
        CUsbCDC::Instance()->onConnect(itf, x);
    }
}

// Optional callback for line coding changes (baud rate, data bits, etc.)
// Currently commented out - not in use
// void CUsbCDC::cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event)
// {
//     ESP_LOGI(TAG, "bit_rate %d",event->line_coding_changed_data.p_line_coding->bit_rate);
//     ESP_LOGI(TAG, "data_bits %d",event->line_coding_changed_data.p_line_coding->data_bits);
//     ESP_LOGI(TAG, "stop_bits %d",event->line_coding_changed_data.p_line_coding->stop_bits);
//     ESP_LOGI(TAG, "parity %d",event->line_coding_changed_data.p_line_coding->parity);
// }

// Process received data from USB CDC interface
// Reads data in chunks until the buffer is empty
void CUsbCDC::rx(tinyusb_cdcacm_itf_t itf)
{
    size_t rx_size = 0;
    esp_err_t ret;

    // Read data in chunks until buffer is completely empty
    for (;;)
    {
        // Read data into temporary buffer
        ret = tinyusb_cdcacm_read(itf, mRxBuf0, USB_MAX_DATA, &rx_size);
        if (ret != ESP_OK)
        {
            // Log error if read operation fails
            ESP_LOGE(TAG,"CUsbCDC tinyusb_cdcacm_read failed %d", ret);
            return;
        }

        // Process received data if available
        if (rx_size != 0)
        {
            if (onCmd != nullptr)
                // Call user-defined callback function with received data
                onCmd(itf, mRxBuf0, rx_size); 
            else
                // Default logging if no callback is defined
                ESP_LOG_BUFFER_HEX(TAG, mRxBuf0, rx_size); 
        }

        // Exit when buffer is not full (no more data available)
        if (rx_size < USB_MAX_DATA)
            return;
    }
}

// Initialize CDC interface with callback functions
// Sets up USB device, ACM interface, and power management
void CUsbCDC::start(onCDCDataRx *func, onCDCConect *connect)
{
    // Create power management lock to prevent light sleep when USB is active
#if CONFIG_PM_ENABLE
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "usb", &mPMLock);
    ESP_ERROR_CHECK(esp_pm_lock_acquire(mPMLock));
#endif

    // Basic USB device configuration with device event handler
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(device_event_handler);
    
    // Configure wake-up pin if specified
    if (mWakeUpPin >= 0)
    {
        tusb_cfg.phy.self_powered = true;      // Device is self-powered
        tusb_cfg.phy.vbus_monitor_io = mWakeUpPin; // Monitor VBUS on specified pin
    }
    
    // Set task priority and core affinity
    tusb_cfg.task.priority = mPriority;
    tusb_cfg.task.xCoreID = mCoreID;

    // Install tinyUSB driver with configured parameters
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    // Configure ACM (Abstract Control Model) interface
    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,   // First CDC port
        .callback_rx = &cdc_rx_callback, // Callback for data reception
        .callback_rx_wanted_char = nullptr, // No character matching callback
        .callback_line_state_changed = &cdc_line_state_changed_callback, // DTR change callback
        .callback_line_coding_changed = nullptr}; // No line coding change callback

    // Initialize first ACM interface
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));

#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    // Initialize second interface if configured
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
#endif

    // Store user-defined callback functions
    onCmd = func;      // Data reception callback
    onConnect = connect; // Connection state callback
}

// Deinitialize CDC interface and clean up resources
// Uninitializes ACM interfaces and USB driver, releases power management lock
void CUsbCDC::stop()
{
#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    // Uninitialize second interface if it was configured
    ESP_ERROR_CHECK(tusb_cdc_acm_deinit(TINYUSB_CDC_ACM_1));
#endif
    // Uninitialize first interface and USB driver
    ESP_ERROR_CHECK(tinyusb_cdcacm_deinit(TINYUSB_CDC_ACM_0));
    ESP_ERROR_CHECK(tinyusb_driver_uninstall());

    // Release power management restrictions
#if CONFIG_PM_ENABLE
    esp_pm_lock_release(mPMLock);      // Release power management lock
    esp_pm_lock_delete(mPMLock);       // Delete the lock
#endif
}

// Send data through CDC interface
// Queues data for transmission and performs synchronous flush with timeout
bool CUsbCDC::send(int itf, uint8_t *data, size_t size)
{
    // Queue data for transmission
    size_t sz = tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)itf, data, size);
    if (sz == 0)
        return false; // Error queuing data

    // Write remaining data if buffer was filled in parts
    while (sz != size)
    {
        // Queue remaining data
        size_t s = tinyusb_cdcacm_write_queue((tinyusb_cdcacm_itf_t)itf, &data[sz], size - sz);
        if (s != 0)
            sz += s;  // Increment total sent size
        else
        {
            // Flush buffer and return false on failure
            tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)itf, 0);
            return false;
        }
    }

    // Synchronously flush data with 0ms timeout (blocks until sent)
    if (tinyusb_cdcacm_write_flush((tinyusb_cdcacm_itf_t)itf, 0) != ESP_OK)
        return false;

    return true; // Success
}

#endif // CONFIG_TINYUSB_CDC_ENABLED