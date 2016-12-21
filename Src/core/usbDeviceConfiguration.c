#include "usbDeviceConfiguration.h"
#include "usbd_desc.h"
/*
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"
*/

/*
Думается мне, что не плохо было бы отсюда конфигурировать ЮСБ.
Например, в случае если это будет ЭрЗед Приемо-передатчик.
Указателем пробросим в драйвер.
Нужно только подумать: то ли хранить разные дескрипторы(в таком случае их можно будет даже через ЮСБ сбрасывать;) )
То ли хранить один, и изменять в нем исходные данные. Правда, с учетом того, что в данной реализации дескриптор это масив -
нужно будет морочиться с нужными байтами. Структура была бы круче.
*/


__ALIGN_BEGIN uint8_t usbDeviceConfiguratonDescriptor_Device[ USB_LEN_DEV_DESC ] __ALIGN_END =
	{
	USB_LEN_DEV_DESC,						/*bLength */
	USB_DESC_TYPE_DEVICE,				/*bDescriptorType*/
	0x00,												/*bcdUSB */
	0x02,
	0x02,												/*bDeviceClass*/
	0x00,												/*bDeviceSubClass*/
	0x00,												/*bDeviceProtocol*/
	USB_MAX_EP0_SIZE,						/*bMaxPacketSize*/
	LOBYTE(USBD_VID),						/*idVendor*/
	HIBYTE(USBD_VID),						/*idVendor*/
	LOBYTE(USBD_PID_FS),				/*idVendor*/
	HIBYTE(USBD_PID_FS),				/*idVendor*/
	0x10,												/*bcdDevice rel. 2.00*/
	0x01,
	0,													/*Index of manufacturer  string*/
	0,													/*Index of product string*/
	0,													/*Index of serial number string*/
	USBD_MAX_NUM_CONFIGURATION	/*bNumConfigurations*/
	};

__ALIGN_BEGIN uint8_t usbDeviceConfiguratonDescriptor_Configuration[ USB_CDC_CONFIG_DESC_SIZ ] __ALIGN_END =
	{
	0x09,   										/* bLength: Configuration Descriptor size */
	USB_DESC_TYPE_CONFIGURATION	/* bDescriptorType: Configuration */
	USB_CDC_CONFIG_DESC_SIZ,		/* wTotalLength:no of returned bytes */
	0x00,
	0x01,												/* bNumInterfaces: 2 interface */
	0x01,												/* bConfigurationValue: Configuration value */
	0x00,												/* iConfiguration: Index of string descriptor describing the configuration */
	0xC0,												/* bmAttributes: self powered */
	0x00,												/* MaxPower 0 mA */

	/*---------------------------------------------------------------------------*/

	/*Interface Descriptor */
	0x09,												/* bLength: Interface Descriptor size */
	USB_DESC_TYPE_INTERFACE,  	/* bDescriptorType: Interface */
	/* Interface descriptor type */
	0x00,												/* bInterfaceNumber: Number of Interface */
	0x00,												/* bAlternateSetting: Alternate setting */
	0x02,												/* bNumEndpoints: One endpoints used */
	0x02,												/* bInterfaceClass: Communication Interface Class */
	0x02,												/* bInterfaceSubClass: Abstract Control Model */
	0x00,												/* bInterfaceProtocol: Common AT commands */
	0x00,												/* iInterface: */

	/*Header Functional Descriptor*/
	0x05,												/* bLength: Endpoint Descriptor size */
	0x24,												/* bDescriptorType: CS_INTERFACE */
	0x00,												/* bDescriptorSubtype: Header Func Desc */
	0x10,												/* bcdCDC: spec release number */
	0x01,

	/*ACM Functional Descriptor*/
	0x04,												/* bFunctionLength */
	0x24,												/* bDescriptorType: CS_INTERFACE */
	0x02,												/* bDescriptorSubtype: Abstract Control Management desc */
	0x00,												/* bmCapabilities */

	/*Union Functional Descriptor*/
	0x05,												/* bFunctionLength */
	0x24,												/* bDescriptorType: CS_INTERFACE */
	0x06,												/* bDescriptorSubtype: Union func desc */
	0x00,												/* bMasterInterface: Communication class interface */
	0x01,												/* bSlaveInterface0: Data Class Interface */

	/*Call Management Functional Descriptor*/
	0x05,												/* bFunctionLength */
	0x24,												/* bDescriptorType: CS_INTERFACE */
	0x01,												/* bDescriptorSubtype: Call Management Func Desc */
	0x00,												/* bmCapabilities: D0+D1 */
	0x01,												/* bDataInterface: 1 */

	// Endpoint 1 descriptor //
	0x07,												// bLength
	USB_DESC_TYPE_ENDPOINT,			// bDescriptorType
	CDC_OUT_EP,									// bEndpointAddress, Endpoint 01 - OUT
	USBD_EP_TYPE_BULK,					// bmAttributes      BULK
	LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),	/* wMaxPacketSize: */
	HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
	0x05,												// bInterval

	// Endpoint 3 descriptor //
	0x07,												// bLength
	USB_DESC_TYPE_ENDPOINT,			// bDescriptorType
	CDC_IN_EP,									// bEndpointAddress, Endpoint 03 - IN
	USBD_EP_TYPE_BULK,					// bmAttributes      BULK
	LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),	/* wMaxPacketSize: */
	HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
	0x05,												// bInterval
	};



tdDescriptorPtr deviceDescriptors[	] =
	{
	usbDeviceConfiguratonDescriptor_Device, sizeof(usbDeviceConfiguratonDescriptor_Device)
	};
unsigned char deviceDescriptorsSize = sizeof(deviceDescriptors) / sizeof(deviceDescriptors[ 0 ]);

tdDescriptorPtr *pCurrentDescriptor = deviceDescriptors[ 0 ];

tdUsbDeviceStatus usbDevice_changeDescriptor(unsigned char _descriptor)
	{
	if( !(_descriptor < deviceDescriptorsSize) )
		return USB_DEVICE_CONFIGURATION_ERROR;

	pCurrentDescriptor = &deviceDescriptors[ _descriptor ];

	return USB_DEVICE_STATUS_OK;
	};

tdUsbDeviceStatus usbDevice_setDefaultDescriptor(void)
	{
	return usbDevice_changeDescriptor( USB_DEVICE_CONFIGURATION_DEFAULT );
	};

