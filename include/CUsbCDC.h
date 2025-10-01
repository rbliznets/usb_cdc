/*!
	\file
	\brief Wrapper class for tinyUSB CDC (Communication Device Class).
	\authors Bliznets R.A. (r.bliznets@gmail.com)
	\version 1.0.0.0
	\date 16.04.2024

	Implements the Singleton pattern. Allows working with USB CDC interface
	as with a serial port. Handles connection and data reception events.
	One object per application.
*/

#pragma once

#include "sdkconfig.h"
#ifdef CONFIG_TINYUSB_CDC_ENABLED

#include "esp_pm.h"
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"

// Maximum data size for reception (128 bytes)
#define USB_MAX_DATA (128)

/// Function pointer type for data reception event callback.
/*!
 * \param[in] itf CDC interface number (0 or 1 for multi-interface support).
 * \param[in] data pointer to received data buffer.
 * \param[in] size size of received data in bytes.
 */
typedef void onCDCDataRx(int itf, uint8_t *data, size_t size);

// Bit flags for connection state notifications
#define TINYUSB_CDC_DTR 0x01  ///< Data Terminal Ready signal flag
#define TINYUSB_CDC_RTS 0x02  ///< Request To Send signal flag
#define TINYUSB_ATTACHED 0x04 ///< USB device attached flag
#define TINYUSB_DETACHED 0x08 ///< USB device detached flag

/// Function pointer type for connection event callback.
/*!
	\param[in] itf CDC interface number (-1 TINYUSB_ATTACHED/TINYUSB_DETACHED).
	\param[in] connect bit flags indicating connection state and control signals:
				   - TINYUSB_ATTACHED/TINYUSB_DETACHED for device connection
				   - TINYUSB_CDC_DTR/TINYUSB_CDC_RTS for control signal states
	\warning only DTR events, TINYUSB_DETACHED if mWakeUpPin enabled
*/
typedef void onCDCConect(int itf, uint32_t connect);

/// Wrapper class for tinyUSB CDC (Communication Device Class).
/// Provides a serial port-like interface for USB communication.
class CUsbCDC
{
private:
	// Singleton instance pointer - points to the single instance of this class
	static CUsbCDC *theSingleInstance;

protected:
#if CONFIG_PM_ENABLE
	// Power management lock handle to prevent CPU frequency reduction during USB operations
	esp_pm_lock_handle_t mPMLock;
#endif

	/// Static callback function for handling received data from CDC interface.
	/// Called by tinyUSB library when data is available on the interface.
	/*!
	  \param[in] itf CDC interface number.
	  \param[in] event callback function parameters containing received data info.
	*/
	static void cdc_rx_callback(int itf, cdcacm_event_t *event);

	/// Static callback function for handling line state changes (DTR/RTS signals).
	/// Called when DTR or RTS control signals change state.
	/*!
	  \param[in] itf CDC interface number.
	  \param[in] event callback function parameters containing line state info.
	*/
	static void cdc_line_state_changed_callback(int itf, cdcacm_event_t *event);

	// Optional callback for line coding changes (baud rate, parity, etc.) - currently unused
	// static void cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event);

	/// Static callback function for handling USB device attachment/detachment events.
	/// Called when USB device is connected or disconnected from host.
	/*!
	  \param[in] event USB device event information.
	  \param[in] arg additional event arguments (unused).
	*/
	static void device_event_handler(tinyusb_event_t *event, void *arg);

	/// Instance method for processing received data from CDC interface.
	/// Reads data in chunks and calls user callback or logs data.
	/*!
	  \param[in] itf CDC interface number.
	*/
	void rx(tinyusb_cdcacm_itf_t itf);

	// Temporary buffer for receiving data (size: USB_MAX_DATA bytes)
	uint8_t mRxBuf0[USB_MAX_DATA];

	// User-defined callback for handling received data (can be nullptr)
	onCDCDataRx *onCmd = nullptr;

	// User-defined callback for handling connection/disconnection events (can be nullptr)
	onCDCConect *onConnect = nullptr;

	// Private destructor - called automatically when singleton instance is deleted
	~CUsbCDC() { stop(); };

public:
	// Wakeup pin for managing sleep state (default: -1 = disabled)
	static int8_t mWakeUpPin;

	// USB Device Task priority (default: 5 - medium priority)
	static uint8_t mPriority;

	// USB Device Task core affinity (default: 1 - run on core 1)
	static int mCoreID;

	/// Get the single instance of the CUsbCDC class.
	/// Creates the instance if it doesn't exist yet.
	/*!
	  \return Pointer to CUsbCDC singleton instance
	*/
	static CUsbCDC *Instance()
	{
		if (theSingleInstance == nullptr)
			theSingleInstance = new CUsbCDC();
		return theSingleInstance;
	};

	/// Free the singleton instance and release all resources.
	/// Should be called when USB functionality is no longer needed.
	static void free()
	{
		if (theSingleInstance != nullptr)
		{
			delete theSingleInstance;
			theSingleInstance = nullptr;
		}
	};

	/// Check if the USB CDC driver is running.
	/*
	 * \return true if driver is initialized and running, false otherwise.
	 */
	static inline bool isRun() { return (theSingleInstance != nullptr); };

	/// Initialize and start the USB CDC driver.
	/// Sets up USB device, ACM interface, and registers callback functions.
	/*!
	  \param[in] func Callback function for handling received data (can be nullptr).
	  \param[in] connect Callback function for handling connection events (can be nullptr).
	*/
	void start(onCDCDataRx *func, onCDCConect *connect = nullptr);

	/// Stop and deinitialize the USB CDC driver.
	/// Cleans up all resources and stops USB communication.
	void stop();

	/// Send data through the specified CDC interface.
	/// Queues data for transmission and performs synchronous flush.
	/*!
	  \param[in] itf CDC interface number (0 or 1).
	  \param[in] data pointer to data buffer to send.
	  \param[in] size size of data to send in bytes.
	  \return true on successful transmission, false on error.
	*/
	bool send(int itf, uint8_t *data, size_t size);
};

#endif // CONFIG_TINYUSB_CDC_ENABLED