#ifndef URG_SERIAL_H
#define URG_SERIAL_H

/*!
  \file
  \~japanese
  \brief �V���A���ʐM
  \~english
  \brief Serial communications
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_detect_os.h"

#if defined(URG_WINDOWS_OS)
#include <windows.h>
#else
#include <termios.h>
#endif
#include "urg_ring_buffer.h"


enum {
    RING_BUFFER_SIZE_SHIFT = 7,
    RING_BUFFER_SIZE = 1 << RING_BUFFER_SIZE_SHIFT,

    ERROR_MESSAGE_SIZE = 256,
};


//! \~japanese �V���A���ʐM�p  \~english Control information for serial connection
typedef struct
{
#if defined(URG_WINDOWS_OS)
    HANDLE hCom;                //!< \~japanese �ڑ����\�[�X  \~english Connection resource
    int current_timeout;        //!< \~japanese �^�C���A�E�g�̐ݒ莞�� [msec]  \~english Timeout configuration value
#else
    int fd;                     //!< \~japanese �t�@�C���f�B�X�N���v�^  \~english File descriptor
    struct termios sio;         //!< \~japanese �ʐM�ݒ�  \~english Connection configuration
#endif

    ring_buffer_t ring;         //!< \~japanese �����O�o�b�t�@  \~english Ring buffer structure
    char buffer[RING_BUFFER_SIZE]; //!< \~japanese �o�b�t�@�̈�  \~english Data buffer
    char has_last_ch;          //!< \~japanese �����߂������������邩�̃t���O  \~english Whether the last character was received or not
    char last_ch;              //!< \~japanese �����߂����P����  \~english Last character received
} urg_serial_t;


//! \~japanese �ڑ����J��  \~english Opens the connection
extern int serial_open(urg_serial_t *serial, const char *device, long baudrate);


//! \~japanese �ڑ������  \~english Closes the connection
extern void serial_close(urg_serial_t *serial);


//! \~japanese �{�[���[�g��ݒ肷��  \~english Configures the baudrate
extern int serial_set_baudrate(urg_serial_t *serial, long baudrate);


//! \~japanese �f�[�^�𑗐M����  \~english Sends data over serial connection
extern int serial_write(urg_serial_t *serial, const char *data, int size);


//! \~japanese �f�[�^����M����  \~english Gets data from serial connection
extern int serial_read(urg_serial_t *serial,
                       char *data, int max_size, int timeout);


//! \~japanese ���s�܂ł̃f�[�^����M����  \~english Gets data from serial connection until end-of-line
extern int serial_readline(urg_serial_t *serial,
                           char *data, int max_size, int timeout);


//! \~japanese �G���[��������i�[���ĕԂ�  \~english Stores the serial error message
extern int serial_error(urg_serial_t *serial,
                        char *error_message, int max_size);

#ifdef __cplusplus
}
#endif

#endif /* !URG_SERIAL_H */
