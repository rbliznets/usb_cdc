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

#include "tinyusb.h"
#include "tusb_console.h"
#include "tusb_cdc_acm.h"

#define USB_MAX_DATA (64)

/// Функция события приема данных.
/*!
 * \param[in] itf номер CDC.
 * \param[in] data данные.
 * \param[in] size размер данных.
 */
typedef void onCDCDataRx(int itf, uint8_t *data, size_t size);
/// Функция события на установку соединения.
/*!
	\param[in] itf номер CDC.
	\param[in] con true - подключение, false - отключение.
*/
typedef void onCDCConect(int itf, bool con);

/// Класс обертка для tinyUSB СDС.
class CUsbCDC
{
protected:
	/// функция обработки данных из CDC.
	/*!
	  \param[in] itf номер CDC.
	  \param[in] event параметры callback функции.
	*/
	static void cdc_rx_callback(int itf, cdcacm_event_t *event);
	/// функция обработки изменения состояния CDC.
	/*!
	  \param[in] itf номер CDC.
	  \param[in] event параметры callback функции.
	*/
	static void cdc_line_state_changed_callback(int itf, cdcacm_event_t *event);

	/// функция обработки данных из CDC.
	/*!
	  \param[in] itf номер CDC.
	*/
	void rx(tinyusb_cdcacm_itf_t itf);

	uint8_t mRxBuf0[USB_MAX_DATA]; ///< Буфер для приема данных.

	
	onCDCDataRx *onCmd = nullptr;	  ///< Обработка события приема.
	onCDCConect *onConnect = nullptr; ///< Обработка события подключения

public:
	static int8_t mWakeUpPin; ///< Wakeup.
	/// Единственный экземпляр класса.
	/*!
	  \return Указатель на CUsbCDC
	*/
	static CUsbCDC *Instance()
	{
		static CUsbCDC theSingleInstance;
		return &theSingleInstance;
	}

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
	  \param[in] itf номер CDC.
	  \param[in] data данные.
	  \param[in] size размер.
	  \return true при успехе
	*/
	bool send(int itf, uint8_t *data, size_t size);
};

#endif // CONFIG_TINYUSB_CDC_ENABLED
