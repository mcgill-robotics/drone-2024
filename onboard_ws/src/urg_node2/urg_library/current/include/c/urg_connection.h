#ifndef URG_CONNECTION_H
#define URG_CONNECTION_H

/*!
  \file
  \~japanese
  \brief �ʐM�̏���
  \~english
  \brief Process communications
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_serial.h"
#include "urg_tcpclient.h"


/*!
  \~japanese
  \brief �萔��`
  \~english
  \brief Defines constants
*/
enum {
    URG_CONNECTION_TIMEOUT = -1, //!< \~japanese �^�C���A�E�g�����������Ƃ��̖߂�l  \~english Return value in case of timeout
};


/*!
  \~japanese
  \brief �ʐM�^�C�v
  \~english
  \brief Connection type
*/
typedef enum {
    URG_SERIAL,                 //!< \~japanese �V���A��, USB �ڑ�  \~english Serial/USB connection
    URG_ETHERNET,               //!< \~japanese �C�[�T�[�l�b�g�ڑ�  \~english Ethernet connection
} urg_connection_type_t;


/*!
  \~japanese
  \brief �ʐM���\�[�X
  \~english
  \brief Connection resources
*/
typedef struct
{
    urg_connection_type_t type; //!< \~japanese �ڑ��^�C�v  \~english Type of connection
    urg_serial_t serial;        //!< \~japanese �V���A���ڑ� \~english Serial connection
    urg_tcpclient_t tcpclient;  //!< \~japanese �C�[�T�[�l�b�g�ڑ� \~english Ethernet connection
} urg_connection_t;


/*!
  \~japanese
  \brief �ڑ�

  �w�肳�ꂽ�f�o�C�X�ɐڑ�����B

  \param[in,out] connection �ʐM���\�[�X
  \param[in] connection_type �ڑ��^�C�v
  \param[in] device �ڑ���
  \param[in] baudrate_or_port �{�[���[�g / �|�[�g�ԍ�

  \retval 0 ����
  \retval <0 �G���[

  connection_type �ɂ�

  - URG_SERIAL ... �V���A���ʐM
  - URG_ETHERNET .. �C�[�T�[�l�b�g�ʐM

  ���w�肷��B

  device, baudrate_or_port �̎w��� connection_type �ɂ��w��ł���l���قȂ�B
  �Ⴆ�΁A�V���A���ʐM�̏ꍇ�͈ȉ��̂悤�ɂȂ�B

  \~english
  \brief Connection

  Connects to the specified device

  \param[in,out] connection Connection resource
  \param[in] connection_type Connection type
  \param[in] device Device name
  \param[in] baudrate_or_port Baudrate or port number

  \retval 0 Success
  \retval <0 Error

  The connection_type is either of:

  - URG_SERIAL ... Serial connection
  - URG_ETHERNET .. Ethernet connection

  device and baudrate_or_port arguments are defined according to connection_type
  For example, in case of serial connection:

  \~
  Example
  \code
  connection_t connection;
  if (! connection_open(&connection, URG_SERIAL, "COM1", 115200)) {
      return 1;
  } \endcode

  And, in case of ethernet connection:

  \~
  Example
  \code
  connection_t connection;
  if (! connection_open(&connection, URG_ETHERNET, "192.168.0.10", 10940)) {
      return 1;
  } \endcode

  \~
  \see connection_close()
*/
extern int connection_open(urg_connection_t *connection,
                           urg_connection_type_t connection_type,
                           const char *device, long baudrate_or_port);


/*!
  \~japanese
  \brief �ؒf

  �f�o�C�X�Ƃ̐ڑ���ؒf����B

  \param[in,out] connection �ʐM���\�[�X
  \~english
  \brief Disconnection

  Closes the connection with the device

  \param[in,out] connection Connection resource
  \~
  \code
  connection_close(&connection); \endcode
  \~
  \see connection_open()
*/
extern void connection_close(urg_connection_t *connection);


/*!
  \~japanese
  \brief �{�[���[�g��ݒ肷��
  \~english
  \brief Configures the baudrate
*/
extern int connection_set_baudrate(urg_connection_t *connection, long baudrate);


/*!
  \~japanese
  \brief ���M

  �f�[�^�𑗐M����B

  \param[in,out] connection �ʐM���\�[�X
  \param[in] data ���M�f�[�^
  \param[in] size ���M�o�C�g��

  \retval >=0 ���M�f�[�^��
  \retval <0 �G���[

  \~english
  \brief Send

  Writes data over the communication channel

  \param[in,out] connection Connection resource
  \param[in] data Data to send
  \param[in] size Number of bytes to send

  \retval >=0 Number of bytes sent
  \retval <0 Error
  \~
  Example
  \code
  n = connection_write(&connection, "QT\n", 3); \endcode

  \~
  \see connection_read(), connection_readline()
*/
extern int connection_write(urg_connection_t *connection,
                            const char *data, int size);


/*!
  \~japanese
  \brief ��M

  �f�[�^����M����B

  \param[in,out] connection �ʐM���\�[�X
  \param[in] data ��M�f�[�^���i�[����o�b�t�@
  \param[in] max_size ��M�f�[�^���i�[�ł���o�C�g��
  \param[in] timeout �^�C���A�E�g���� [msec]

  \retval >=0 ��M�f�[�^��
  \retval <0 �G���[

  timeout �ɕ��̒l���w�肵���ꍇ�A�^�C���A�E�g�͔������Ȃ��B

  1 ��������M���Ȃ������Ƃ��� #URG_CONNECTION_TIMEOUT ��Ԃ��B

  \~english
  \brief Receive

  Reads data from the communication channel

  \param[in,out] connection Connection resource
  \param[in] data Buffer to store received data
  \param[in] max_size Maximum size of the buffer
  \param[in] timeout Timeout [msec]

  \retval >=0 Number of bytes received
  \retval <0 Error

  If timeout argument is negative then the function waits until some data is received

  In case no data is received #URG_CONNECTION_TIMEOUT is returned.
  \~
  Example
  \code
enum {
    BUFFER_SIZE = 256,
    TIMEOUT_MSEC = 1000,
};
char buffer[BUFFER_SIZE];
n = connection_read(&connection, buffer, BUFFER_SIZE, TIMEOUT_MSEC); \endcode

  \~
  \see connection_write(), connection_readline()
*/
extern int connection_read(urg_connection_t *connection,
                           char *data, int max_size, int timeout);


/*!
  \~japanese
  \brief ���s�����܂ł̎�M

  ���s�����܂ł̃f�[�^����M����B

  \param[in,out] connection �ʐM���\�[�X
  \param[in] data ��M�f�[�^���i�[����o�b�t�@
  \param[in] max_size ��M�f�[�^���i�[�ł���o�C�g��
  \param[in] timeout �^�C���A�E�g���� [msec]

  \retval >=0 ��M�f�[�^��
  \retval <0 �G���[

  data �ɂ́A'\\0' �I�[���ꂽ������ max_size ���z���Ȃ��o�C�g�������i�[�����B �܂�A��M�ł��镶���̃o�C�g���́A�ő�� max_size - 1 �ƂȂ�B

  ���s������ '\\r' �܂��� '\\n' �Ƃ���B

  ��M�����ŏ��̕��������s�̏ꍇ�́A0 ��Ԃ��A1 ��������M���Ȃ������Ƃ��� #URG_CONNECTION_TIMEOUT ��Ԃ��B

  \~english
  \brief Receive until end-of-line

  Reads data until the end-of-line character is detected.

  \param[in,out] connection Connection resource
  \param[in] data Buffer to store received data
  \param[in] max_size Maximum size of the buffer
  \param[in] timeout Timeout [msec]

  \retval >=0 Number of bytes received
  \retval <0 Error

  If timeout argument is negative then the function waits until some data is received

  The null terminator character '\\0' is used at the end of data so that the number of bytes does not exceed max_size.
  This is, the maximum number of received characters is max_size - 1.

  The end-of-line character is either '\\r' or '\\n'

  In case no end-of-line is received then returns 0, if no data is received #URG_CONNECTION_TIMEOUT is returned.
  \~
  \see connection_write(), connection_read()
*/
extern int connection_readline(urg_connection_t *connection,
                               char *data, int max_size, int timeout);

#ifdef __cplusplus
}
#endif

#endif /* !URG_CONNECTION_H */
