#ifndef URG_RING_BUFFER_H
#define URG_RING_BUFFER_H

/*!
  \file
  \~japanese
  \brief �����O�o�b�t�@
  \~english
  \brief Ring buffer functions
  \~
  \author Satofumi KAMIMURA

  $Id$
*/


//! \~japanese �����O�o�b�t�@�̊Ǘ����  \~english Control structure of the ring buffer
typedef struct
{
    char *buffer;                 //!< \~japanese �o�b�t�@�ւ̃|�C���^  \~english Pointer to the data buffer
    int buffer_size;              //!< \~japanese �o�b�t�@�T�C�Y  \~english Buffer size
    int first;                    //!< \~japanese �o�b�t�@�̐擪�ʒu  \~english Index of the first entry of the buffer
    int last;                     //!< \~japanese �o�b�t�@�̍ŏI�ʒu  \~english Index of the last entry of the buffer
} ring_buffer_t;


/*!
  \~japanese
  \brief ������

  \param[in] ring �����O�o�b�t�@�̍\����
  \param[in] buffer ���蓖�Ă�o�b�t�@
  \param[in] shift_length �o�b�t�@�T�C�Y�� 2 �̏搔

  \~english
  \brief Initialization

  \param[in] ring Pointer to the ring buffer data structure
  \param[in] buffer Actual buffer to use
  \param[in] shift_length Buffer size as multiple of 2
*/
extern void ring_initialize(ring_buffer_t *ring,
                            char *buffer, const int shift_length);


/*!
  \~japanese
  \brief �����O�o�b�t�@�̃N���A

  \param[in] ring �����O�o�b�t�@�̍\����

  \~english
  \brief Clears the ring  buffer

  \param[in] ring Pointer to the ring buffer data structure
*/
extern void ring_clear(ring_buffer_t *ring);


/*!
  \~japanese
  \brief �i�[�f�[�^����Ԃ�

  \param[in] ring �����O�o�b�t�@�̍\����

  \~english
  \brief Returns the number of elements on the buffer

  \param[in] ring Pointer to the ring buffer data structure
*/
extern int ring_size(const ring_buffer_t *ring);


/*!
  \~japanese
  \brief �ő�̊i�[�f�[�^����Ԃ�

  \param[in] ring �����O�o�b�t�@�̍\����

  \~english
  \brief Returns the maximum number of elements of the ring buffer

  \param[in] ring Pointer to the ring buffer data structure
*/
extern int ring_capacity(const ring_buffer_t *ring);


/*!
  \~japanese
  \brief �f�[�^�̊i�[

  \param[in] ring �����O�o�b�t�@�̍\����
  \param[in] data �f�[�^
  \param[in] size �f�[�^�T�C�Y

  \return �i�[�����f�[�^��

  \~english
  \brief Stores data on the ring buffer

  \param[in] ring Pointer to the ring buffer data structure
  \param[in] data Data to store
  \param[in] size Number of elements to store

  \return The number of elements written to the ring buffer
*/
extern int ring_write(ring_buffer_t *ring, const char *data, int size);


/*!
  \~japanese
  \brief �f�[�^�̎��o��

  \param[in] ring �����O�o�b�t�@�̍\����
  \param[out] buffer �f�[�^
  \param[in] size �ő�̃f�[�^�T�C�Y

  \return ���o�����f�[�^��

  \~english
  \brief Extracts data from the ring buffer

  \param[in] ring Pointer to the ring buffer data structure
  \param[out] buffer Buffer to hold the extracted data
  \param[in] size Maximum size of the buffer

  \return The number of elements read from the ring buffer
*/
extern int ring_read(ring_buffer_t *ring, char *buffer, int size);

#endif /* ! RING_BUFFER_H */
