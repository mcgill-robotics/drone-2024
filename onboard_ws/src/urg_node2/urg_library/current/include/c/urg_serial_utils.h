#ifndef URG_SERIAL_UTILS_H
#define URG_SERIAL_UTILS_H

/*!
  \file
  \~japanese
  \brief �V���A���p�̕⏕�֐�
  \~english
  \brief Auxiliary functions for serial communications
  \~
  \author Satofumi KAMIMURA

  $Id$
*/


//! \~japanese �V���A���|�[�g����������  \~english Finds the serial port
extern int urg_serial_find_port(void);


//! \~japanese ���������V���A���|�[�g����Ԃ�  \~english Returns the name of the serial port found
extern const char *urg_serial_port_name(int index);


/*!
  \~japanese
  \brief �|�[�g�� URG ���ǂ���

  \retval 1 URG �̃|�[�g
  \retval 0 �s��
  \retval <0 �G���[

  \~english
  \brief Checks whether the serial port corresponds to a URG or not

  \retval 1 It is a URG
  \retval 0 Unknown
  \retval <0 Error
*/
extern int urg_serial_is_urg_port(int index);

#endif /* !URG_SERIAL_UTILS_H */
