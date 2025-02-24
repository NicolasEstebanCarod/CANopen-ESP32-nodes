#ifndef CO_CONFIG_FLAGS_H
#define CO_CONFIG_FLAGS_H

#ifdef __cplusplus
extern "C" {
#endif

//####  CAN CONFIG  ####

#define CAN_TX_IO (17)  /** CAN TX pin */
#define CAN_RX_IO (16) /** CAN RX pin */
#define CAN_TX_QUEUE_LENGTH (5) /** CAN TX buffer size */
#define CAN_RX_QUEUE_LENGTH (5) /** CAN RX buffer size */

#define CO_CAN_PSEUDO_INTERRUPT_INTERVAL (1000) /** CAN "Interrupt" Interval in μs*/
#define CO_MAIN_TASK_INTERVAL (1000)   /* Interval of tmrTask thread in microseconds */

#define CAN_TICKS_TO_WAIT (10000) /*CAN TX/RX Timeout value*/

//----------------------------------


/**
 * @defgroup CO_STACK_CONFIG Stack configuration
 * @ingroup CO_driver
 *
 * Stack configuration macros specify, which parts of the stack will be enabled.
 *
 * Default values for stack configuration macros are set in CO_driver.h file.
 * They can be overridden by CO_driver_target.h file. If specified so, they can
 * further be overridden by CO_driver_custom.h file.
 *
 * Stack configuration macro is specified as bits, where each bit
 * enables/disables some part of the configurable CANopenNode object. Flags are
 * used for enabling or checking specific bit. Multiple flags can be ORed
 * together.
 *
 * Configuration macros may be in relation with @ref CO_NO_OBJ macros from
 * CANopen.h file. Former enables/disables stack functionalities, latter
 * enables/disables objects in object dictionary. Note that some objects in
 * object dictionary may need some configuration macros to be enabled.
 * @{
 */


/**
 * Enable custom callback after CAN receive
 *
 * Flag enables optional callback functions, which are part of some CANopenNode
 * objects. Callbacks can optionally be registered by application, which
 * configures threads in operating system. Callbacks are called after something
 * has been preprocessed by higher priority thread and must be further
 * processed by lower priority thread. For example when CAN message is received
 * and preprocessed, callback should wake up mainline processing function.
 * See also @ref CO_process() function.
 *
 * If callback functions are used, they must be initialized separately, after
 * the object initialization.
 *
 * This flag is common to multiple configuration macros.
 */
#define CO_CONFIG_FLAG_CALLBACK_PRE 0x1000


/**
 * Enable calculation of timerNext_us variable.
 *
 * Calculation of the timerNext_us variable is useful for smooth operation on
 * operating system. See also @ref CO_process() function.
 *
 * This flag is common to multiple configuration macros.
 */
#define CO_CONFIG_FLAG_TIMERNEXT 0x2000


/**
 * Configuration of NMT_Heartbeat object
 *
 * Possible flags, can be ORed:
 * - #CO_CONFIG_FLAG_CALLBACK_PRE - Enable custom callback after preprocessing
 *   received NMT CAN message.
 *   Callback is configured by CO_NMT_initCallbackPre().
 * - #CO_CONFIG_FLAG_TIMERNEXT - Enable calculation of timerNext_us variable
 *   inside CO_NMT_process().
 * - CO_CONFIG_NMT_CALLBACK_CHANGE - Enable custom callback after NMT
 *   state changes. Callback is configured by
 *   CO_NMT_initCallbackChanged().
 * - CO_CONFIG_NMT_MASTER - Enable simple NMT master
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_NMT (CO_CONFIG_FLAG_CALLBACK_PRE | CO_CONFIG_FLAG_TIMERNEXT | CO_CONFIG_NMT_CALLBACK_CHANGE | CO_CONFIG_NMT_MASTER0)
#endif
#define CO_CONFIG_NMT_CALLBACK_CHANGE 0x01
#define CO_CONFIG_NMT_MASTER 0x02


/**
 * Configuration of SDO server object
 *
 * Possible flags, can be ORed:
 * - #CO_CONFIG_FLAG_CALLBACK_PRE - Enable custom callback after preprocessing
 *   received SDO CAN message.
 *   Callback is configured by CO_SDO_initCallbackPre().
 * - #CO_CONFIG_FLAG_TIMERNEXT - Enable calculation of timerNext_us variable
 *   inside CO_SDO_process().
 * - CO_CONFIG_SDO_SEGMENTED - Enable SDO server segmented transfer.
 * - CO_CONFIG_SDO_BLOCK - Enable SDO server block transfer. If set, then
 *   CO_CONFIG_SDO_SEGMENTED must also be set.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_SDO (CO_CONFIG_FLAG_CALLBACK_PRE | CO_CONFIG_FLAG_TIMERNEXT | CO_CONFIG_SDO_SEGMENTED | CO_CONFIG_SDO_BLOCK)
#endif
/* TODO with new OD */
#define CO_CONFIG_SDO_SEGMENTED 0x01
#define CO_CONFIG_SDO_BLOCK 0x02


/**
 * Size of the internal data buffer for the SDO server.
 *
 * Size must be at least equal to size of largest variable in
 * @ref CO_SDO_objectDictionary. If data type is domain, data length is not
 * limited to SDO buffer size. If block transfer is implemented, value should be
 * set to 889.
 *
 * Value can be in range from 7 to 889 bytes.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_SDO_BUFFER_SIZE 32
#endif


/**
 * Configuration of Emergency object
 *
 * Possible flags, can be ORed:
 * - #CO_CONFIG_FLAG_CALLBACK_PRE - Enable custom callback after preprocessing
 *   emergency condition by CO_errorReport() or CO_errorReset() call.
 *   Callback is configured by CO_EM_initCallbackPre().
 * - #CO_CONFIG_FLAG_TIMERNEXT - Enable calculation of timerNext_us variable
 *   inside CO_EM_process().
 * - CO_CONFIG_EM_CONSUMER - Enable emergency consumer.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_EM (CO_CONFIG_FLAG_CALLBACK_PRE | CO_CONFIG_FLAG_TIMERNEXT | CO_CONFIG_EM_CONSUMER)
#endif
#define CO_CONFIG_EM_CONSUMER 0x01


/**
 * Configuration of Heartbeat Consumer
 *
 * Possible flags, can be ORed:
 * - #CO_CONFIG_FLAG_CALLBACK_PRE - Enable custom callback after preprocessing
 *   received heartbeat CAN message.
 *   Callback is configured by CO_HBconsumer_initCallbackPre().
 * - #CO_CONFIG_FLAG_TIMERNEXT - Enable calculation of timerNext_us variable
 *   inside CO_HBconsumer_process().
 * - CO_CONFIG_HB_CONS_CALLBACK_CHANGE - Enable custom callback after NMT
 *   state of the monitored node changes. Callback is configured by
 *   CO_HBconsumer_initCallbackNmtChanged().
 * - CO_CONFIG_HB_CONS_CALLBACK_MULTI - Enable multiple custom callbacks, which
 *   can be configured for each monitored node. Callback are configured by
 *   CO_HBconsumer_initCallbackHeartbeatStarted(),
 *   CO_HBconsumer_initCallbackTimeout() and
 *   CO_HBconsumer_initCallbackRemoteReset() functions.
 * - CO_CONFIG_HB_CONS_QUERY_FUNCT - Enable functions for query HB state or
 *   NMT state of the specific monitored node.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_HB_CONS (CO_CONFIG_FLAG_CALLBACK_PRE | CO_CONFIG_FLAG_TIMERNEXT | CO_CONFIG_HB_CONS_CALLBACK_CHANGE | CO_CONFIG_HB_CONS_CALLBACK_MULTI | CO_CONFIG_HB_CONS_QUERY_FUNCT)
#endif
#define CO_CONFIG_HB_CONS_CALLBACK_CHANGE 0x01
#define CO_CONFIG_HB_CONS_CALLBACK_MULTI 0x02
#define CO_CONFIG_HB_CONS_QUERY_FUNCT 0x04


/**
 * Configuration of PDO
 *
 * Possible flags, can be ORed:
 * - #CO_CONFIG_FLAG_CALLBACK_PRE - Enable custom callback after preprocessing
 *   received RPDO CAN message.
 *   Callback is configured by CO_RPDO_initCallbackPre().
 * - #CO_CONFIG_FLAG_TIMERNEXT - Enable calculation of timerNext_us variable
 *   inside CO_TPDO_process().
 * - CO_CONFIG_PDO_SYNC_ENABLE - Enable SYNC object inside PDO objects.
 * - CO_CONFIG_RPDO_CALLS_EXTENSION - Enable calling configured extension
 *   callbacks when received RPDO CAN message modifies OD entries.
 * - CO_CONFIG_TPDO_CALLS_EXTENSION - Enable calling configured extension
 *   callbacks before TPDO CAN message is sent.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_PDO (CO_CONFIG_FLAG_CALLBACK_PRE | CO_CONFIG_FLAG_TIMERNEXT | CO_CONFIG_PDO_SYNC_ENABLE | CO_CONFIG_RPDO_CALLS_EXTENSION | CO_CONFIG_TPDO_CALLS_EXTENSION)
#endif
#define CO_CONFIG_PDO_SYNC_ENABLE 0x01
#define CO_CONFIG_RPDO_CALLS_EXTENSION 0x02
#define CO_CONFIG_TPDO_CALLS_EXTENSION 0x04


/**
 * Configuration of SYNC
 *
 * Possible flags, can be ORed:
 * - #CO_CONFIG_FLAG_CALLBACK_PRE - Enable custom callback after preprocessing
 *   received SYNC CAN message.
 *   Callback is configured by CO_SYNC_initCallbackPre().
 * - #CO_CONFIG_FLAG_TIMERNEXT - Enable calculation of timerNext_us variable
 *   inside CO_SYNC_process().
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_SYNC (CO_CONFIG_FLAG_CALLBACK_PRE | CO_CONFIG_FLAG_TIMERNEXT)
#endif


/**
 * Configuration of SDO client object
 *
 * Possible flags, can be ORed:
 * - #CO_CONFIG_FLAG_CALLBACK_PRE - Enable custom callback after preprocessing
 *   received SDO CAN message.
 *   Callback is configured by CO_SDOclient_initCallbackPre().
 * - #CO_CONFIG_FLAG_TIMERNEXT - Enable calculation of timerNext_us variable
 *   inside CO_SDOclientDownloadInitiate(), CO_SDOclientDownload(),
 *   CO_SDOclientUploadInitiate(), CO_SDOclientUpload().
 * - CO_CONFIG_SDO_CLI_SEGMENTED - Enable SDO client segmented transfer.
 * - CO_CONFIG_SDO_CLI_BLOCK - Enable SDO client block transfer. If set, then
 *   CO_CONFIG_SDO_CLI_SEGMENTED must also be set.
 * - CO_CONFIG_SDO_CLI_LOCAL - Enable local transfer, if Node-ID of the SDO
 *   server is the same as node-ID of the SDO client. (SDO client is the same
 *   device as SDO server.) Transfer data directly without communication on CAN.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_SDO_CLI (CO_CONFIG_FLAG_CALLBACK_PRE | CO_CONFIG_FLAG_TIMERNEXT | CO_CONFIG_SDO_CLI_SEGMENTED | CO_CONFIG_SDO_CLI_BLOCK | CO_CONFIG_SDO_CLI_LOCAL)
#endif
#define CO_CONFIG_SDO_CLI_SEGMENTED 0x01
#define CO_CONFIG_SDO_CLI_BLOCK 0x02
#define CO_CONFIG_SDO_CLI_LOCAL 0x04


/**
 * Size of the internal data buffer for the SDO client.
 *
 * Circular buffer is used for SDO communication. it can be read or written
 * between successive SDO calls. So size of the buffer can be lower than size of
 * the actual size of data transferred. If only segmented transfer is used, then
 * buffer size can be as low as 7 bytes, if data are read/written each cycle. If
 * block transfer is used, buffer size should be set to at least 889 bytes, so
 * maximum blksize can be used. In that case data should be read/written proper
 * time between cycles.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_SDO_CLI_BUFFER_SIZE 32
#endif


/**
 * Configuration of TIME
 *
 * Possible flags, can be ORed:
 * - #CO_CONFIG_FLAG_CALLBACK_PRE - Enable custom callback after preprocessing
 *   received TIME CAN message.
 *   Callback is configured by CO_TIME_initCallbackPre().
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_TIME (CO_CONFIG_FLAG_CALLBACK_PRE)
#endif


/**
 * Configuration of LEDs object
 *
 * Possible flags, can be ORed:
 * - #CO_CONFIG_FLAG_TIMERNEXT - Enable calculation of timerNext_us variable
 *   inside CO_NMT_process().
 * - CO_CONFIG_LEDS_ENABLE - Enable calculation of the CANopen LED indicators.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_LEDS (CO_CONFIG_FLAG_TIMERNEXT | CO_CONFIG_LEDS_ENABLE)
#endif
#define CO_CONFIG_LEDS_ENABLE 0x01


/**
 * Configuration of LSS objects
 *
 * Possible flags, can be ORed:
 * - #CO_CONFIG_FLAG_CALLBACK_PRE - Enable custom callback after preprocessing
 *   received CAN message.
 *   Callback is configured by CO_LSSmaster_initCallbackPre().
 * - CO_CONFIG_LSS_SLAVE - Enable LSS slave
 * - CO_CONFIG_LSS_SLAVE_FASTSCAN_DIRECT_RESPOND - Send LSS fastscan respond
 *   directly from CO_LSSslave_receive() function.
 * - CO_CONFIG_LSS_MASTER - Enable LSS master
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_LSS (CO_CONFIG_FLAG_CALLBACK_PRE | CO_CONFIG_LSS_SLAVE | CO_CONFIG_LSS_SLAVE_FASTSCAN_DIRECT_RESPOND | CO_CONFIG_LSS_MASTER )
#endif
#define CO_CONFIG_LSS_SLAVE 0x01
#define CO_CONFIG_LSS_SLAVE_FASTSCAN_DIRECT_RESPOND 0x02
#define CO_CONFIG_LSS_MASTER 0x10


/**
 * Configuration of gateway object usage.
 *
 * Gateway object is covered by standard CiA 309 - CANopen access from other
 * networks. It enables usage of the NMT master, SDO client and LSS master as a
 * gateway device.
 *
 * Possible flags, can be ORed:
 * - CO_CONFIG_GTW_MULTI_NET - Enable multiple network interfaces in gateway
 *   device. This functionality is currently not implemented.
 * - CO_CONFIG_GTW_ASCII - Enable gateway device with ASCII mapping (CiA 309-3)
 * - CO_CONFIG_GTW_ASCII_SDO - Enable SDO client
 * - CO_CONFIG_GTW_ASCII_NMT - Enable NMT master
 * - CO_CONFIG_GTW_ASCII_LSS - Enable LSS master
 * - CO_CONFIG_GTW_ASCII_LOG - Enable non-standard message log read
 * - CO_CONFIG_GTW_ASCII_ERROR_DESC - Print error description as additional
 *   comments in gateway-ascii device for SDO and gateway errors.
 * - CO_CONFIG_GTW_ASCII_PRINT_HELP - use non-standard command "help" to print
 *   help usage.
 * - CO_CONFIG_GTW_ASCII_PRINT_LEDS - Display "red" and "green" CANopen status
 *   LED diodes on terminal.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_GTW (CO_CONFIG_GTW_MULTI_NET | CO_CONFIG_GTW_ASCII | CO_CONFIG_GTW_ASCII_SDO | CO_CONFIG_GTW_ASCII_NMT | CO_CONFIG_GTW_ASCII_LSS | CO_CONFIG_GTW_ASCII_LOG | CO_CONFIG_GTW_ASCII_ERROR_DESC | CO_CONFIG_GTW_ASCII_PRINT_HELP | CO_CONFIG_GTW_ASCII_PRINT_LEDS)
#endif
#define CO_CONFIG_GTW_MULTI_NET 0x01
#define CO_CONFIG_GTW_ASCII 0x02
#define CO_CONFIG_GTW_ASCII_SDO 0x04
#define CO_CONFIG_GTW_ASCII_NMT 0x08
#define CO_CONFIG_GTW_ASCII_LSS 0x10
#define CO_CONFIG_GTW_ASCII_LOG 0x20
#define CO_CONFIG_GTW_ASCII_ERROR_DESC 0x40
#define CO_CONFIG_GTW_ASCII_PRINT_HELP 0x80
#define CO_CONFIG_GTW_ASCII_PRINT_LEDS 0x100


/**
 * Number of loops of #CO_SDOclientDownload() in case of block download
 *
 * If SDO clint has block download in progress and OS has buffer for CAN tx
 * messages, then #CO_SDOclientDownload() functionion can be called multiple
 * times within own loop (up to 127). This can speed-up SDO block transfer.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_GTW_BLOCK_DL_LOOP 1
#endif


/**
 * Size of command buffer in ASCII gateway object.
 *
 * If large amount of data is transferred (block transfer), then this should be
 * increased to 1000 or more. Buffer may be refilled between the block transfer.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_GTWA_COMM_BUF_SIZE 200
#endif


/**
 * Size of message log buffer in ASCII gateway object.
 */
#ifdef CO_DOXYGEN
#define CO_CONFIG_GTWA_LOG_BUF_SIZE 2000
#endif


/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_CONFIG_FLAGS_H */
