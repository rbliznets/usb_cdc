/*!
	\file
	\brief Класс обертка для tinyUSB СDС.
	\authors Близнец Р.А. (r.bliznets@gmail.com)
	\version 0.1.0.0
	\date 16.04.2024

	Один объект на приложение.
*/

#pragma once

#include "sdkconfig.h"
#ifdef CONFIG_TINYUSB_CDC_ENABLED

#include "esp_pm.h"
#include "tinyusb.h"
#include "tusb_console.h"
#include "tusb_cdc_acm.h"

#define USB_MAX_DATA (128) // Максимальный размер данных для приема

/// Функция события приема данных.
/*!
 * \param[in] itf номер CDC интерфейса.
 * \param[in] data данные, полученные от устройства.
 * \param[in] size размер данных.
 */
typedef void onCDCDataRx(int itf, uint8_t *data, size_t size);


// Bit flags for connection state notifications
#define TINYUSB_CDC_DTR 0x01  ///< Data Terminal Ready signal flag
#define TINYUSB_CDC_RTS 0x02  ///< Request To Send signal flag
#define TINYUSB_ATTACHED 0x04 ///< USB device attached flag
#define TINYUSB_DETACHED 0x08 ///< USB device detached flag

/// Функция события на установку соединения.
/*!
	\param[in] itf номер CDC интерфейса.
	\param[in] con true - подключение, false - отключение.
*/
typedef void onCDCConect(int itf, uint32_t con);

/// Класс обертка для tinyUSB СDС.
class CUsbCDC
{
private:
	static CUsbCDC *theSingleInstance; ///< Указатель на единственный экземпляр класса

protected:
#if CONFIG_PM_ENABLE
	esp_pm_lock_handle_t mPMLock; ///< Флаг запрета на понижение частоты CPU
#endif

	/// Функция обработки данных из CDC.
	/*!
	  \param[in] itf номер CDC интерфейса.
	  \param[in] event параметры callback функции.
	*/
	static void cdc_rx_callback(int itf, cdcacm_event_t *event);

	/// Функция обработки изменения состояния CDC.
	/*!
	  \param[in] itf номер CDC интерфейса.
	  \param[in] event параметры callback функции.
	*/
	static void cdc_line_state_changed_callback(int itf, cdcacm_event_t *event);

	/// Функция обработки данных из CDC.
	/*!
	  \param[in] itf номер CDC интерфейса.
	*/
	void rx(tinyusb_cdcacm_itf_t itf);

	uint8_t mRxBuf0[USB_MAX_DATA]; ///< Буфер для приема данных.

	onCDCDataRx *onCmd = nullptr;	  ///< Обработка события приема данных.
	onCDCConect *onConnect = nullptr; ///< Обработка события подключения/отключения.

	~CUsbCDC() { stop(); };

public:
	static int8_t mWakeUpPin; ///< Wakeup пин для управления唤醒 состоянием.

	/// Единственный экземпляр класса.
	/*!
	  \return Указатель на CUsbCDC
	*/
	static CUsbCDC *Instance()
	{
		if (theSingleInstance == nullptr)
			theSingleInstance = new CUsbCDC();
		return theSingleInstance;
	};

	/// Освобождение ресурсов.
	static void free()
	{
		if (theSingleInstance != nullptr)
		{
			delete theSingleInstance;
			theSingleInstance = nullptr;
		}
	};

	/// Подключение к USB.
	/*
	 * \return true - если работает, false - если нет.
	 */
	static inline bool isRun() { return (theSingleInstance != nullptr); };

	/// Запуск драйвера.
	/*!
	  \param[in] func Обработчик json команды.
	  \param[in] connect Обработчик подключения.
	*/
	void start(onCDCDataRx *func, onCDCConect *connect = nullptr);

	/// Остановка драйвера.
	void stop();

	/// Отослать данные.
	/*!
	  \param[in] itf номер CDC интерфейса.
	  \param[in] data данные для отправки.
	  \param[in] size размер данных.
	  \return true при успехе
	*/
	bool send(int itf, uint8_t *data, size_t size);
};

#endif // CONFIG_TINYUSB_CDC_ENABLED
