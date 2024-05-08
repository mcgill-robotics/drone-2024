#ifndef URG_SENSOR_H
#define URG_SENSOR_H

/*!
  \file
  \~japanese
  \brief URG �Z���T����

  URG �p�̊�{�I�Ȋ֐���񋟂��܂��B

  \~english
  \brief URG sensor control

  Provides the basic functions for URG

  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_connection.h"

    /*!
      \~japanese
      \brief �v���^�C�v
      \~english
      \brief Measurement types
    */
    typedef enum {
        URG_DISTANCE,              //!< \~japanese ����  \~english Distance (range)
        URG_DISTANCE_INTENSITY,    //!< \~japanese ���� + ���x  \~english Distance (range) and intensity (strength)
        URG_DISTANCE_IO,           //!< \~japanese ���� + IO���  \~english Distance (range) and IO(input/output)
        URG_DISTANCE_INTENSITY_IO, //!< \~japanese ���� + ���x + IO���  \~english Distance (range), intensity and IO(input/output)
        URG_MULTIECHO,             //!< \~japanese �}���`�G�R�[�̋���  \~english Multiecho distance
        URG_MULTIECHO_INTENSITY,   //!< \~japanese �}���`�G�R�[��(���� + ���x)  \~english Multiecho distance and intensity
        URG_STOP,                  //!< \~japanese �v���̒�~  \~english Stop measurement
        URG_UNKNOWN,               //!< \~japanese �s��  \~english Unknown measurement type
    } urg_measurement_type_t;

    /*!
      \~japanese
      \brief �������� byte �ŕ\�����邩�̎w��
      \~english
      \brief Distance data encoding types (number of bytes)
    */
    typedef enum {
        URG_COMMUNICATION_3_BYTE, //!< \~japanese ������ 3 byte �ŕ\������  \~english Use 3-bytes encoding for distance
        URG_COMMUNICATION_2_BYTE, //!< \~japanese ������ 2 byte �ŕ\������  \~english Use 2-bytes encoding for distance
    } urg_range_data_byte_t;


    enum {
        URG_SCAN_INFINITY = 0,  //!< \~japanese ������̃f�[�^�擾  \~english Continuous data scanning
        URG_MAX_ECHO = 3,       //!< \~japanese �}���`�G�R�[�̍ő�G�R�[��  \~english Maximum number of echoes
        URG_MAX_IO = 2,         //!< \~japanese IO���̍ő�f�[�^��  \~english Maximum number of IO(input/output)
    };


    /*!
       \~japanese
       \brief �G���[�n���h��
       \~english
       \brief Error handler
    */
    typedef urg_measurement_type_t
    (*urg_error_handler)(const char *status, void *urg);


    /*!
      \~japanese
      \brief URG �Z���T�Ǘ�

      \~english
      \brief URG sensor control structure
    */
    typedef struct
    {
        int is_active;
        int last_errno;
        urg_connection_t connection;

        int first_data_index;
        int last_data_index;
        int front_data_index;
        int area_resolution;
        long scan_usec;
        int min_distance;
        int max_distance;
        int scanning_first_step;
        int scanning_last_step;
        int scanning_skip_step;
        int scanning_skip_scan;
        urg_range_data_byte_t range_data_byte;

        int timeout;
        int specified_scan_times;
        int scanning_remain_times;
        int is_laser_on;

        int received_first_index;
        int received_last_index;
        int received_skip_step;
        urg_range_data_byte_t received_range_data_byte;
        int is_sending;

        urg_error_handler error_handler;

        int ignore_checkSumError;

        char return_buffer[80];
    } urg_t;

    /*!
      \~japanese
      \brief urg_t�\���̂̏�����

      URG �Z���T�Ǘ��\����(urg_t)�����������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�

      \attention ���̊֐���urg_open()�̏��߂Ɏ��s����܂��B�C�ӂɃZ���T�Ǘ��\����(urg_t)�����������������͂��̊֐����Ăяo���Ă��������B
      \see urg_open()

      \~english
      \brief URG control structure (urg_t) initialization

      Initialize URG control structure(urg_t)

      \param[in,out] urg URG control structure

      \attention
      This function is executed at the start of urg_open ().
      Call this function if you want to initialize the URG control structure (urg_t) arbitrarily.

      \see urg_open()
      \~
    */
    void urg_t_initialize(urg_t *urg);

    /*!
      \~japanese
      \brief �ڑ�

      �w�肵���f�o�C�X�ɐڑ����A�������v���ł���悤�ɂ���B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[in] connection_type �ʐM�^�C�v
      \param[in] device_or_address �ڑ��f�o�C�X��
      \param[in] baudrate_or_port �ڑ��{�[���[�g [bps] / TCP/IP �|�[�g

      \retval 0 ����
      \retval <0 �G���[

      connection_type �ɂ́A�ȉ��̍��ڂ��w��ł��܂��B

      - #URG_SERIAL
      - �V���A���AUSB �ڑ�

      - #URG_ETHERNET
      - �C�[�T�[�l�b�g�ڑ�

      \~english
      \brief Connect

      Connects to the given device and enables measurement

      \param[in,out] urg URG control structure
      \param[in] connection_type Type of the connection
      \param[in] device_or_address Name of the device
      \param[in] baudrate_or_port Connection baudrate [bps] or TCP/IP port number

      \retval 0 Successful
      \retval <0 Error

      The following values can be used in connection_type:

      - #URG_SERIAL
      - Serial, USB connection

      - #URG_ETHERNET
      - Ethernet connection
      \~
      Example
      \code
      urg_t urg;

      if (urg_open(&urg, URG_SERIAL, "/dev/ttyACM0", 115200) < 0) {
      return 1;
      }

      ...

      urg_close(&urg); \endcode

      \~japanese
      \attention URG C ���C�u�����̑��̊֐����Ăяo���O�ɁA���̊֐����Ăяo���K�v������܂��B
      \~english
      \attention Call this function before using any other function on the URG library.

      \~
      \see urg_close()
    */
    extern int urg_open(urg_t *urg, urg_connection_type_t connection_type,
                        const char *device_or_address,
                        long baudrate_or_port);


    /*!
      \~japanese
      \brief �ؒf

      ���[�U���������AURG �Ƃ̐ڑ���ؒf���܂��B

      \param[in,out] urg URG �Z���T�Ǘ�

      \~english
      \brief Disconnection

      Turns off the laser and closes the connection with the URG sensor.

      \param[in,out] urg URG control structure
      \~
      \see urg_open()
    */
    extern void urg_close(urg_t *urg);


    /*!
      \~japanese
      \brief �^�C���A�E�g���Ԃ̐ݒ�

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[in] msec �^�C���A�E�g���鎞�� [msec]

      \attention urg_open() ���Ăяo���� timeout �̐ݒ�l�̓f�t�H���g�l�ɏ���������邽�߁A���̊֐��� urg_open() ��ɌĂяo�����ƁB
      \~english
      \brief Defines the timeout value to use during communication

      \param[in,out] urg URG control structure
      \param[in] msec Timeout value [msec]

      \attention The urg_open() function always sets the timeout value to its default, if necessary call this function after urg_open().
    */
    extern void urg_set_timeout_msec(urg_t *urg, int msec);


    /*!
       \~japanese
       \brief �^�C���X�^���v���[�h�̊J�n
       \~english
       \brief Starts the timestamp mode (time adjustment state)
    */
    extern int urg_start_time_stamp_mode(urg_t *urg);


    /*!
      \~japanese
      \brief �^�C���X�^���v�̎擾

      \param[in,out] urg URG �Z���T�Ǘ�

      \retval >=0 �^�C���X�^���v [msec]
      \retval <0 �G���[

      \~english
      \brief Read timestamp data

      \param[in,out] urg URG control structure

      \retval >=0 Timestamp value [msec]
      \retval <0 Error

      \~
      Example
      \code
      urg_start_time_stamp_mode(&urg);

      before_ticks = get_pc_msec_function();
      time_stamp = urg_time_stamp(&urg);
      after_ticks = get_pc_msec_function();

      \~japanese
      // �^�C���X�^���v�ɂ��Ă̌v�Z
      \~english
      // Processing of timestamp data
      \~
      ...

      urg_stop_time_stamp_mode(&urg); \endcode

      \~japanese
      �ڂ����� \ref sync_time_stamp.c ���Q�Ƃ��ĉ������B
      \~english
      For a detailed use consult the \ref sync_time_stamp.c example
    */
    extern long urg_time_stamp(urg_t *urg);


    /*!
       \~japanese
       \brief �^�C���X�^���v���[�h�̏I��
       \~english
       \brief Stops the timestamp mode (returning to idle state)
    */
    extern int urg_stop_time_stamp_mode(urg_t *urg);


    /*!
      \~japanese
      \brief �����f�[�^�̎擾���J�n

      �����f�[�^�̎擾���J�n���܂��B���ۂ̃f�[�^�� urg_get_distance(), urg_get_distance_intensity(), urg_get_multiecho(), urg_get_multiecho_intensity() �Ŏ擾�ł��܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[in] type �f�[�^�E�^�C�v
      \param[in] scan_times �f�[�^�̎擾��
      \param[in] skip_scan �f�[�^�̎擾�Ԋu
      \param[in] ignore_checkSumError 0�ȊO:�`�F�b�N�T���G���[�𖳎����A�v���p�� 0:�`�F�b�N�T���G���[�Ōv����~

      \retval 0 ����
      \retval <0 �G���[

      type �ɂ͎擾����f�[�^�̎�ނ��w�肵�܂��B

      - #URG_DISTANCE ... �����f�[�^
      - #URG_DISTANCE_INTENSITY ... �����f�[�^�Ƌ��x�f�[�^
      - #URG_MULTIECHO ... �}���`�G�R�[�ł̋����f�[�^
      - #URG_MULTIECHO_INTENSITY ... �}���`�G�R�[�ł�(�����f�[�^�Ƌ��x�f�[�^)

      scan_times �͉���̃f�[�^���擾���邩�� 0 �ȏ�̐��Ŏw�肵�܂��B�������A0 �܂��� #URG_SCAN_INFINITY ���w�肵���ꍇ�́A������̃f�[�^���擾���܂��B\n
      �J�n�����v���𒆒f����ɂ� urg_stop_measurement() ���g���܂��B

      skip_scan �̓~���[�̉�]���̂����A�P��̃X�L������ɉ���X�L�������Ȃ������w�肵�܂��Bskip_scan �Ɏw��ł���͈͂� [0, 9] �ł��B

      \image html skip_scan_image.png ����ɂP�񂾂��v�����邩

      ���Ƃ��΁A�~���[�̂P��]�� 100 [msec] �̃Z���T�� skip_scan �� 1 ���w�肵���ꍇ�A�f�[�^�̎擾�Ԋu�� 200 [msec] �ɂȂ�܂��B

      \~english
      \brief Start getting distance measurement data

      Starts measurement data acquisition. The actual data can be retrieved using urg_get_distance(), urg_get_distance_intensity(), urg_get_multiecho(), urg_get_multiecho_intensity().

      \param[in,out] urg URG control structure
      \param[in] type Measurement type
      \param[in] scan_times Number of scans to request
      \param[in] skip_scan Interval between scans
      \param[in] ignore_checkSumError non-0:continue measurement 0: stop measurement

      \retval 0 Successful
      \retval <0 Error

      The following values are possible for the type argument

      - #URG_DISTANCE ... Distance (range) data
      - #URG_DISTANCE_INTENSITY ... Distance (range) and intensity (strength) data
      - #URG_MULTIECHO ... Multiecho distance data
      - #URG_MULTIECHO_INTENSITY ... Multiecho distance and intensity data

      scan_times defines how many scans to capture from the sensor: >0 means a fixed number of scans, 0 or #URG_SCAN_INFINITY means continuous (infinite) scanning.
      To interrupt measurement at any time use urg_stop_measurement().

      skip_scan means, after obtaining one scan, skip measurement for the following X mirror (motor) revolutions.
      The values for skip_scan are from the range [0, 9].

      \image html skip_scan_image.png shows scan skip

      For example, for a sensor with one mirror (motor) revolution is 100 [msec] and skip_scan is set to 1, measurement data will be obtained with an interval of 200 [msec].

      \~
      Example
      \code
      enum { CAPTURE_TIMES = 10 };
      urg_start_measurement(&urg, URG_DISTANCE, CAPTURE_TIMES, 0);

      for (i = 0; i < CAPTURE_TIMES; ++i) {
      int n = urg_get_distance(&urg, data, &time_stamp);

      \~japanese
      // ��M�����f�[�^�̗��p
      \~english
      // Processing of obtained data
      \~
      ...
      } \endcode

      \~
      \see urg_get_distance(), urg_get_distance_intensity(), urg_get_multiecho(), urg_get_multiecho_intensity(), urg_stop_measurement()
    */
    extern int urg_start_measurement(urg_t *urg, urg_measurement_type_t type,
                                     int scan_times, int skip_scan, int ignore_checkSumError);


    /*!
      \~japanese
      \brief �����f�[�^�̎擾

      �Z���T���狗���f�[�^���擾���܂��B���O�� urg_start_measurement() �� #URG_DISTANCE �w��ŌĂяo���Ă����K�v������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[out] data �����f�[�^ [mm]
      \param[out] time_stamp �^�C���X�^���v [msec]

      \retval >=0 ��M�����f�[�^��
      \retval <0 �G���[

      data �ɂ́A�Z���T����擾���������f�[�^���i�[����܂��Bdata �̓f�[�^���i�[����̃T�C�Y���m�ۂ��Ă����K�v������܂��Bdata �Ɋi�[�����f�[�^���� urg_max_data_size() �Ŏ擾�ł��܂��B

      time_stamp �ɂ́A�Z���T�����̃^�C���X�^���v���i�[����܂��Btime_stamp ���擾�������Ȃ��ꍇ NULL ���w�肵�ĉ������B

      \~english
      \brief Gets distance data

      Receives distance data from the sensor. The urg_start_measurement() function was called beforehand with #URG_DISTANCE as type argument.

      \param[in,out] urg URG control structure
      \param[out] data Distance data array [mm]
      \param[out] time_stamp Timestamp [msec]

      \retval >=0 Number of data points received
      \retval <0 Error

      Distance data received from the sensor are stored in data array. data array should be previously allocated to hold all the data points requested from the sensor. To know how many data points are received, use the urg_max_data_size() function.

      time_stamp will hold the timestamp value stored on the sensor. When not necessary just pass NULL as argument.

      \~
      Example
      \code
      long *data = (long*)malloc(urg_max_data_size(&urg) * sizeof(data[0]));

      ...

      \~japanese
      // �f�[�^�̂ݎ擾����
      \~english
      // Gets only measurement data
      urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
      int n = urg_get_distance(&urg, data, NULL);

      ...

      \~japanese
      // �f�[�^�ƃ^�C���X�^���v���擾����
      \~english
      // Gets measurement data and timestamp
      long time_stamp;
      urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
      n = urg_get_distance(&urg, data, &time_stamp); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_distance(urg_t *urg, long data[], long *time_stamp);

    /*!
      \~japanese
      \brief �����f�[�^��IO���̎擾

      �Z���T���狗���f�[�^��IO�����擾���܂��B���O�� urg_start_measurement() �� #URG_DISTANCE_IO �w��ŌĂяo���Ă����K�v������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[out] data �����f�[�^ [mm]
      \param[out] io IO���
      \param[out] time_stamp �^�C���X�^���v [msec]

      \retval >=0 ��M�����f�[�^��
      \retval <0 �G���[

      io �ɂ́A�Z���T����擾����IO��񂪊i�[����܂��B
	  io[0]�ɓ��͒l�Aio[1]�ɏo�͒l���i�[����邽�߁A2�v�f���̃T�C�Y���m�ۂ��Ă����K�v������܂��B
	  
	  data, time_stamp �ɂ��Ă� urg_get_distance() �Ɠ����ł��B

	  �Ȃ��AIO���擾�ɑΉ����Ă��Ȃ��@��ł́A�G���[�ɂȂ�܂��B

      \~english
      \brief Gets distance and IO(input/output) data

      Receives distance and IO(input/output) data from the sensor. The urg_start_measurement() function was called beforehand with #URG_DISTANCE as type argument.

      \param[in,out] urg URG control structure
      \param[out] data Distance data array [mm]
      \param[out] io IO data array
      \param[out] time_stamp Timestamp [msec]

      \retval >=0 Number of data points received
      \retval <0 Error

      IO data received from the sensor are stored in data array. 
      The input value is stored in io[0] and the output value in io[1].
      The size of the two elements should be reserved in advance.

      Regarding data and time_stamp arguments, refer to urg_get_distance().

      Note that an error will occur for models that do not support IO information acquisition.

      \~
      Example
      \code
      long *data = (long*)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
	  long *io = malloc(2 * sizeof(long));
      long time_stamp;

	  ...

	  urg_start_measurement(&urg, URG_DISTANCE_IO, 1, 0);
	  int n = urg_get_distance_io(&urg, data, io, &time_stamp); \endcode

	  \see urg_start_measurement()
    */
    extern int urg_get_distance_io(urg_t* urg, long data[], long io[], long* time_stamp);

    /*!
      \~japanese
      \brief �����Ƌ��x�f�[�^�̎擾

      urg_get_distance() �ɉ����A���x�f�[�^�̎擾���ł���֐��ł��B���O�� urg_start_measurement() �� #URG_DISTANCE_INTENSITY �w��ŌĂяo���Ă����K�v������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[out] data �����f�[�^ [mm]
      \param[out] intensity ���x�f�[�^
      \param[out] time_stamp �^�C���X�^���v [msec]

      \retval >=0 ��M�����f�[�^��
      \retval <0 �G���[

      ���x�f�[�^�Ƃ́A�����v�Z�Ɏg�����g�`�̔��ˋ��x�ł���A�Z���T�̃V���[�Y���ɓ������قȂ�܂��B ���x�f�[�^���g�����ƂŁA���̂̔��˗�����̑�܂��ȔZ�W�𐄑��ł��܂��B

      data, time_stamp �ɂ��Ă� urg_get_distance() �Ɠ����ł��B

      intensity �ɂ́A�Z���T����擾�������x�f�[�^���i�[����܂��Bintensity �̓f�[�^���i�[����̃T�C�Y���m�ۂ��Ă����K�v������܂��Bintensity �Ɋi�[�����f�[�^���� urg_max_data_size() �Ŏ擾�ł��܂��B

      \~english
      \brief Gets distance and intensity data

      This is an extension to urg_get_distance() which allows to obtain also the intensity (strength) data. The urg_start_measurement() function was called beforehand with #URG_DISTANCE_INTENSITY as type argument.

      \param[in,out] urg URG control structure
      \param[out] data Distance data array [mm]
      \param[out] intensity Intensity data array
      \param[out] time_stamp Timestamp [msec]

      \retval >=0 Number of data points received
      \retval <0 Error

      Intensity data corresponds to the laser strength received during distance calculation. The characteristics of this value changes with sensor series. With some limitations, this property can be used to guess the reflectivity and color shade of an object.

      Regarding data and time_stamp arguments, refer to urg_get_distance().

      Intensity data received from the sensor are stored in intensity array. intensity array should be previously allocated to hold all the data points requested from the sensor. To know how many data points are received, use the urg_max_data_size() function.

      \~
      Example
      \code
      int data_size = urg_max_data_size(&urg);
      long *data = malloc(data_size * sizeof(long));
      long *intensity = malloc(data_size * sizeof(unsigned short));

      ...

      urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, 1, 0);
      int n = urg_get_distance_intensity(&urg, data, intesnity, NULL); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_distance_intensity(urg_t *urg, long data[],
                                          unsigned short intensity[],
                                          long *time_stamp);

    /*!
      \~japanese
      \brief �����E���x�f�[�^��IO���̎擾

      urg_get_distance_io() �ɉ����A���x�f�[�^�̎擾���ł���֐��ł��B���O�� urg_start_measurement() �� #URG_DISTANCE_INTENSITY_IO �w��ŌĂяo���Ă����K�v������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[out] data �����f�[�^ [mm]
      \param[out] intensity ���x�f�[�^
      \param[out] io IO���
      \param[out] time_stamp �^�C���X�^���v [msec]

      \retval >=0 ��M�����f�[�^��
      \retval <0 �G���[

      data, time_stamp �ɂ��Ă� urg_get_distance() 
	  intensity �ɂ��Ă� urg_get_distance_intensity() 
	  io �ɂ��Ă� urg_get_distance_io() ���Q�Ƃ��Ă��������B

	  �Ȃ��AIO���擾�ɑΉ����Ă��Ȃ��@��ł́A�G���[�ɂȂ�܂��B

      \~english
      \brief Gets distance, intensity and IO(input/output) data

      This is an extension to urg_get_distance_io() which allows to obtain also the intensity (strength) data. The urg_start_measurement() function was called beforehand with #URG_DISTANCE_INTENSITY_IO as type argument.

      \param[in,out] urg URG control structure
      \param[out] data Distance data array [mm]
      \param[out] intensity Intensity data array
      \param[out] io IO data array
      \param[out] time_stamp Timestamp [msec]

      \retval >=0 Number of data points received
      \retval <0 Error

      Regarding data and time_stamp arguments, refer to urg_get_distance().
      Regarding io arguments, refer to urg_get_distance_io().

      Note that an error will occur for models that do not support IO information acquisition.

      \~
      Example
      \code
      int data_size = urg_max_data_size(&urg);
	  long *data = malloc(data_size * sizeof(long));
	  long *intensity = malloc(data_size * sizeof(unsigned short));
	  long *io = malloc(2 * sizeof(long));

	  ...

	  urg_start_measurement(&urg, URG_DISTANCE_INTENSITY_IO, 1, 0);
	  int n = urg_get_distance_intensity(&urg, data, intesnity, NULL); \endcode

	  \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_distance_intensity_io(urg_t* urg, long data[],
                                             unsigned short intensity[],
                                             long io[], long* time_stamp);

    /*!
      \~japanese
      \brief �����f�[�^�̎擾 (�}���`�G�R�[��)

      �}���`�G�R�[�ł̋����f�[�^�擾�֐��ł��B���O�� urg_start_measurement() �� #URG_MULTIECHO �w��ŌĂяo���Ă����K�v������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[out] data_multi �����f�[�^ [mm]
      \param[out] time_stamp �^�C���X�^���v [msec]

      \retval >=0 ��M�����f�[�^��
      \retval <0 �G���[

      �}���`�G�R�[�Ƃ͕����̋����f�[�^�ł��B �}���`�G�R�[�́A�P�̃��[�U�����ɂ����ĕ����̋����f�[�^������ꂽ�Ƃ��ɓ����܂��B

      \image html multiecho_image.png �}���`�G�R�[�̃C���[�W�}

      time_stamp �ɂ��Ă� urg_get_distance() �Ɠ����ł��B

      data_multi �ɂ́A�Z���T����擾���������f�[�^���P�� step ������ő�� #URG_MAX_ECHO (3 ��)�i�[����܂��B�}���`�G�R�[�����݂��Ȃ����ڂ̃f�[�^�l�� -1 ���i�[����Ă��܂��B

      \verbatim
      data_multi[0] ... step n �̋����f�[�^ (1 ��)
      data_multi[1] ... step n �̋����f�[�^ (2 ��)
      data_multi[2] ... step n �̋����f�[�^ (3 ��)
      data_multi[3] ... step (n + 1) �� �����f�[�^ (1 ��)
      data_multi[4] ... step (n + 1) �� �����f�[�^ (2 ��)
      data_multi[5] ... step (n + 1) �� �����f�[�^ (3 ��)
      ... \endverbatim

      �i�[���́A�e step �ɂ����� urg_get_distance() �̂Ƃ��Ɠ��������̃f�[�^�� (3n + 0) �̈ʒu�Ɋi�[����A����ȊO�̃f�[�^�� (3n + 1), (3n + 2) �̈ʒu�ɍ~���Ɋi�[����܂��B\n
      �܂� data_multi[3n + 1] >= data_multi[3n + 2] �ɂȂ邱�Ƃ͕ۏ؂���܂��� data_multi[3n + 0] �� data_multi[3n + 1] �̊֌W�͖���`�ł��B(data_multi[3n + 1] == data_multi[3n + 2] �����藧�̂̓f�[�^�l�� -1 �̂Ƃ��B)

      \~english
      \brief Gets distance data (multiecho mode)

      Receives multiecho distance data from the sensor. The urg_start_measurement() function was called beforehand with #URG_MULTIECHO as type argument.

      \param[in,out] urg URG control structure
      \param[out] data_multi Distance data array [mm]
      \param[out] time_stamp Timestamp [msec]

      \retval >=0 Number of data points received
      \retval <0 Error

      Multiecho means multiple range responses (echoes). For a single laser beam, multiple laser returns reflected from different targets may be received, and thus multiple range values are calculated.

      \image html multiecho_image.png shows multiecho measurement

      time_stamp will hold the timestamp value stored on the sensor, same as with urg_get_distance().

      The array data_multi will hold the multiecho range data, up to a maximum of #URG_MAX_ECHO (3) echoes per step. In case the echo does not exists then -1 will be stored on the array.

      \verbatim
      data_multi[0] ... step n range data (1st echo)
      data_multi[1] ... step n range data (2nd echo)
      data_multi[2] ... step n range data (3rd echo)
      data_multi[3] ... step (n + 1) range data (1st echo)
      data_multi[4] ... step (n + 1) range data (2nd echo)
      data_multi[5] ... step (n + 1) range data (3rd echo)
      ... \endverbatim

      In the array, the cells numbered (3n + 0) will hold the range data for first echo (same data as for the urg_get_distance() function), for the other cells (3n + 1) and (3n + 2) data is stored in descending order. \n
      This is, the order data_multi[3n + 1] >= data_multi[3n + 2] is assured, however the relation between data_multi[3n + 0] and data_multi[3n + 1] is not defined. (When data_multi[3n + 1] == data_multi[3n + 2] it means the echo does not exists and the stored value is -1.)

      \~
      Example
      \code
      long *data_multi = malloc(3 * urg_max_data_size(&urg) * sizeof(long));

      ...

      urg_start_measurement(&urg, URG_MULTIECHO, 1, 0);
      int n = urg_get_distance_intensity(&urg, data_multi, NULL); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_multiecho(urg_t *urg, long data_multi[], long *time_stamp);


    /*!
      \~japanese
      \brief �����Ƌ��x�f�[�^�̎擾 (�}���`�G�R�[��)

      urg_get_multiecho() �ɉ����A���x�f�[�^�̎擾�ł���֐��ł��B���O�� urg_start_measurement() �� #URG_MULTIECHO_INTENSITY �w��ŌĂяo���Ă����K�v������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[out] data_multi �����f�[�^ [mm]
      \param[out] intensity_multi ���x�f�[�^
      \param[out] time_stamp �^�C���X�^���v [msec]

      \retval >=0 ��M�����f�[�^��
      \retval <0 �G���[

      data_multi, time_stamp �ɂ��Ă� urg_get_multiecho() �Ɠ����ł��B

      intensity_multi �̃f�[�^�̕��т� data_multi �ƑΉ��������̂ɂȂ�܂��Bintensity_multi �Ɋi�[�����f�[�^���� urg_max_data_size() �Ŏ擾�ł��܂��B

      \~english
      \brief Gets distance and intensity data (multiecho mode)

      This is an extension to urg_get_multiecho() which allows to obtain also the intensity (strength) data for multiple echoes. The urg_start_measurement() function was called beforehand with #URG_MULTIECHO_INTENSITY as type argument.

      \param[in,out] urg URG control structure
      \param[out] data_multi Distance data array [mm]
      \param[out] intensity_multi Intensity data array
      \param[out] time_stamp Timestamp [msec]

      \retval >=0 Number of data points received
      \retval <0 Error

      data_multi and time_stamp are as described in urg_get_multiecho() function.

      The order of data in the array intensity_multi is defined by how the data_multi array was sorted. The size of the intensity_multi can be obtained using urg_max_data_size().
      \~
      Example
      \code
      int data_size = urg_max_data_size(&urg);
      long *data_multi = malloc(3 * data_size * sizeof(long));
      long *intensity_multi = malloc(3 * data_size * sizeof(unsigned short));

      ...

      urg_start_measurement(&urg, URG_DISTANCE_INTENSITY, 1, 0);
      int n = urg_get_multiecho_intensity(&urg, data_multi,
      intesnity_multi, NULL); \endcode

      \~
      \see urg_start_measurement(), urg_max_data_size()
    */
    extern int urg_get_multiecho_intensity(urg_t *urg, long data_multi[],
                                           unsigned short intensity_multi[],
                                           long *time_stamp);


    /*!
      \~japanese
      \brief �v���𒆒f���A���[�U�����������܂�

      \ref urg_start_measurement() �̌v���𒆒f���܂��B

      \param[in,out] urg URG �Z���T�Ǘ�

      \retval 0 ����
      \retval <0 �G���[

      \~english
      \brief Stops measurement process and turns off the laser.

      It stops the measurement started with \ref urg_start_measurement() function.

      \param[in,out] urg URG control structure

      \retval 0 Successful
      \retval <0 Error

      \~
      Example
      \code
      urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, 0);
      for (int i = 0; i < 10; ++i) {
      urg_get_distance(&urg, data, NULL);
      }
      urg_stop_measurement(&urg); \endcode

      \~
      \see urg_start_measurement()
    */
    extern int urg_stop_measurement(urg_t *urg);


    /*!
      \~japanese
      \brief �v���͈͂�ݒ肵�܂�

      �Z���T���v������͈͂� step �l�Ŏw�肵�܂��Burg_get_distance() �Ȃǂ̋����f�[�^�擾�̊֐��ŕԂ����f�[�^���́A�����Ŏw�肵���͈͂Ő�������܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[in] first_step �v���̊J�n step
      \param[in] last_step �v���̏I�� step
      \param[in] skip_step �v���f�[�^���O���[�s���O�����

      \retval 0 ����
      \retval <0 �G���[

      �Z���T�� step �́A�Z���T���ʂ� 0 �Ƃ��A�Z���T�㕔���猩�Ĕ����v�܂��̌��������̒l�ƂȂ鏇�Ɋ���U���܂��B

      \image html sensor_angle_image.png �Z���T�� step �̊֌W

      step �̊Ԋu�ƁA�ő�l�A�ŏ��l�̓Z���T�ˑ��ł��Bstep �l�̍ő�l�A�ŏ��l�� urg_step_min_max() �Ŏ擾�ł��܂��B\n

      first_step, last_step �Ńf�[�^�̌v���͈͂��w�肵�܂��B�v���͈͂� [first_step, last_step] �ƂȂ�܂��B

      skip_step �́A�v���f�[�^���O���[�s���O��������w�肵�܂��B�w��ł���l�� [0, 99] �ł��B\n
      skip_step �́A�w�肳�ꂽ���̃f�[�^�� 1 �ɂ܂Ƃ߂邱�ƂŁA�Z���T�����M����f�[�^�ʂ����炵�A�����擾���s���֐��̉����������߂�Ƃ��Ɏg���܂��B�������A�f�[�^���܂Ƃ߂邽�߁A������f�[�^�̕���\�͌���܂��B

      �Ⴆ�Έȉ��̂悤�ȋ����f�[�^��������ꍇ��
      \verbatim
      100, 101, 102, 103, 104, 105, 106, 107, 108, 109
      \endverbatim

      skip_step �� 2 ���w�肷��ƁA������f�[�^��
      \verbatim
      100, 102, 104, 106, 108
      \endverbatim

      �f�[�^�́A�܂Ƃ߂�f�[�^�̂����A��ԏ����Ȓl�̃f�[�^���p�����܂��B

      \~english
      \brief Configure measurement parameters

      This function allows definining the scope (start and end steps) for measurement. The number of measurement data (steps) returned by urg_get_distance() and similar is defined here.

      \param[in,out] urg URG control structure
      \param[in] first_step start step number
      \param[in] last_step end step number
      \param[in] skip_step step grouping factor

      \retval 0 Successful
      \retval <0 Error

      Observing the sensor from the top, the step 0 corresponds to the very front of the sensor, steps at the left side (counter clockwise) of step 0 are positive numbers and those to the right side (clockwise) are negative numbers.

      \image html sensor_angle_image.png shows the relation between sensor and steps

      The spacing between steps, the minimum and maximum step numbers depend on the sensor. Use urg_step_min_max() to get the minimum and maximum step values.\n

      first_step and last_step define the data measurement scope ([first_step, last_step])�B

      skip_step allows setting a step grouping factor, where valid values are [0, 99].\n
      With the skip_step parameter, several adjacent steps are grouped and combined into 1 single step, thus the amount of data transmitted from the sensor is reduced and so the response time of measurement data adquisition functions. Of course, grouping several steps into one means the measurement resolution is reduced.

      For example, for the following range data obtained in the sensor:
      \verbatim
      100, 101, 102, 103, 104, 105, 106, 107, 108, 109
      \endverbatim

      And setting skip_step to 2, the range data returned is:
      \verbatim
      100, 102, 104, 106, 108
      \endverbatim

      for each group, the smallest range value is returned.
      \~
      Example
      \code
      urg_set_scanning_parameter(&urg, urg_deg2step(&urg, -45),
      urg_deg2step(&urg, +45), 1);
      urg_start_measurement(&urg, URG_DISTANCE, 0);
      int n = urg_get_distance(&urg, data, NULL);
      for (int i = 0; i < n; ++i) {
      printf("%d [mm], %d [deg]\n", data[i], urg_index2deg(&urg, i));
      } \endcode

      \~
      \see urg_step_min_max(), urg_rad2step(), urg_deg2step()
    */
    extern int urg_set_scanning_parameter(urg_t *urg, int first_step,
                                          int last_step, int skip_step);

    /*!
      \~japanese
      \brief �ʐM�f�[�^�̃T�C�Y�ύX

      �����f�[�^���Z���T�����M�̍ۂ̃f�[�^�T�C�Y��ύX���܂��B

      \param[in,out] urg URG �Z���T�Ǘ�
      \param[in] data_byte �����l��\������f�[�^�̃o�C�g��

      \retval 0 ����
      \retval <0 �G���[

      data_byte �ɂ�

      - URG_COMMUNICATION_3_BYTE ... ������ 3 byte �ŕ\������
      - URG_COMMUNICATION_2_BYTE ... ������ 2 byte �ŕ\������

      ���w��ł��܂��B\n
      ������Ԃł͋����� 3 byte �ŕ\������悤�ɂȂ��Ă��܂��B���̐ݒ�� 2 byte �ɐݒ肷�邱�ƂŁA�Z���T�����M����f�[�^���� 2/3 �ɂȂ�܂��B�������A�擾�ł��鋗���̍ő�l�� 4095 �ɂȂ邽�߁A�ϑ��������Ώۂ� 4 [m] �ȓ��͈̔͂ɑ��݂���ꍇ�̂ݗ��p���ĉ������B

      \~english
      \brief Change the size (number of bytes) of measurement data used during communications.

      When receiving data from the sensor, changes the number of bytes used to represent measurement data.

      \param[in,out] urg URG control structure
      \param[in] data_byte Number of bytes used to represent measurement data

      \retval 0 Successful
      \retval <0 Error

      data_byte can be:

      - URG_COMMUNICATION_3_BYTE ... to represent data in 3 bytes
      - URG_COMMUNICATION_2_BYTE ... to represent data in 2 bytes

       \n
      The initial (default) data size is 3 bytes. If the number of bytes is changed to 2, the actual received message length becomes around 2/3 of the original length. However, using 2 bytes means the maximum measurement range is 4095, therefore use it only when measurement targets are 4 [m] from the sensor.
      \~
    */
    extern int urg_set_measurement_data_size(urg_t *urg,
                                               urg_range_data_byte_t data_byte);


    /*!
       \~japanese
       \brief ���[�U�𔭌�������
       \~english
       \brief Turns on the laser
    */
    extern int urg_laser_on(urg_t *urg);


    /*!
       \~japanese
       \brief ���[�U����������
       \~english
       \brief Turns off the laser
    */
    extern int urg_laser_off(urg_t *urg);


    /*!
       \~japanese
       \brief �Z���T���ċN������
       \~english
       \brief Reboots the sensor
    */
    extern int urg_reboot(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T������d�͂̏�ԂɑJ�ڂ�����

      �����d�͂̃��[�h�ł́A�X�L���i�̉�]����~���v�������f����܂��B

      - �����d�͂̃��[�h
        - ���[�U���������Čv�������f�����B
        - �X�L���i�̉�]����~����B

      �����d�͂̃��[�h���甲���邽�߂ɂ� \ref urg_wakeup() �֐����Ăяo���ĉ������B

      \~english
      \brief Sets the sensor into low power mode (sleep state)

      During low power mode, the scanner motor stops and so measurement is interrupted.

      - Low power mode
        - Laser is turned off and so measurement is stopped
        - The scanner motor is stopped.

      To recover from low power mode call the function \ref urg_wakeup()
      \~
      \see urg_wakeup()
    */
    extern void urg_sleep(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T������d�͂̃��[�h����ʏ�̏�ԂɑJ�ڂ�����
      \~english
      \brief Returns from the low power mode (sleep state) to the normal mode (idle state)
      \~
      \see urg_sleep()
    */
    extern void urg_wakeup(urg_t *urg);

    /*!
      \~japanese
      \brief �Z���T���v���ł����Ԃ���Ԃ�

      \retval 1 �Z���T���v���ł����Ԃɂ���
      \retval 0 �Z���T���v���ł����ԂɂȂ�

      �N������ŃX�L���i�̉�]�����肵�Ă��Ȃ��ꍇ��A���炩�̃G���[�Ōv���ł��Ȃ��ꍇ�A���̊֐��� 0 ��Ԃ��܂��B

      \~english
      \brief Returns whether the sensor is stable to perform measurement or not

      \retval 1 The sensor can do measurement
      \retval 0 The sensor cannot do measurement

      Right after power on the motor rotation is not yet stable for measurement, or if any failure condition was detected,
      0 is returned.
    */
    extern int urg_is_stable(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T�^���𕶎���ŕԂ�

      �Z���T�̌^���𕶎���ŕԂ��B�Ԃ���镶����̓Z���T�ˑ��ƂȂ�B

      \param[in] urg URG �Z���T�Ǘ�
      \return �Z���T�^���̕�����

      \~english
      \brief Returns the sensor model string

      Returns the string message corresponding to the sensor model. This message is sensor dependent.

      \param[in] urg URG control structure
      \return sensor model string
    */
    extern const char *urg_sensor_product_type(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T�̃V���A�� ID �������Ԃ�

      �Z���T�̃V���A�� ID �������Ԃ��B�Ԃ���镶����̓Z���T�ˑ��ƂȂ�B

      \param[in] urg URG �Z���T�Ǘ�
      \return �V���A�� ID ������

      \~english
      \brief Returns the sensor serial number string

      Returns the string message corresponding to the sensor serial number. This message is sensor dependent.

      \param[in] urg URG control structure
      \return serial number string
    */
    extern const char *urg_sensor_serial_id(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T�̃o�[�W�����������Ԃ�

      �Z���T�̃\�t�g�E�F�A�E�o�[�W�����������Ԃ��B�Ԃ���镶����̓Z���T�ˑ��ƂȂ�B

      \param[in] urg URG �Z���T�Ǘ�
      \return �o�[�W����������

      \~english
      \brief Returns the current sensor firmware version string

      Returns the string message corresponding to the current sensor firmware version. This message is sensor dependent.

      \param[in] urg URG control structure
      \return firmware version string
    */
    extern const char *urg_sensor_firmware_version(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T�̃X�e�[�^�X�������Ԃ�

      �Z���T�̃X�e�[�^�X�������Ԃ��B�Ԃ���镶����̓Z���T�ˑ��ƂȂ�B

      \param[in] urg URG �Z���T�Ǘ�
      \return �X�e�[�^�X������

      \~english
      \brief Returns the current sensor status string

      Returns the string message corresponding to the current sensor status. This message is sensor dependent.

      \param[in] urg URG control structure
      \return current sensor status string
    */
    extern const char *urg_sensor_status(urg_t *urg);


    /*!
      \~japanese
      \brief �Z���T�̏�Ԃ�Ԃ�

      �Z���T�̃X�e�[�^�X�������Ԃ��B�Ԃ���镶����̓Z���T�ˑ��ƂȂ�B

      \param[in] urg URG �Z���T�Ǘ�
      \return ��Ԃ�����������

      \attention ��Ԃɂ��Ă� SCIP �̒ʐM�d�l�����Q�Ƃ̂��ƁB

      \~english
      \brief Returns the current sensor state string

      Returns the string message corresponding to the current sensor state. This message is sensor dependent.

      \param[in] urg URG control structure
      \return current sensor state string

      \attention For details, please refer to the SCIP communication protocol specification.
    */
    extern const char *urg_sensor_state(urg_t *urg);


    /*!
      \~japanese
      \brief �v���p�̃G���[�n���h����o�^����

      �G���[�n���h���� Gx, Mx �n�̃R�}���h�̉����� "00" �� "99" �ȊO�̂Ƃ��ɌĂяo�����B

      \~english
      \brief Registers an error handler for measurement functions

      The error handler will be called for the Gx, Mx commands when the response code is not "00" or "99".
    */
    extern void urg_set_error_handler(urg_t *urg, urg_error_handler handler);


    /*!
      \~japanese
      \brief SCIP ������̃f�R�[�h���s��

      \param[in] data SCIP ������
      \param[in] size data �� byte �T�C�Y

      \retval �f�R�[�h��̐��l

      \~english
      \brief Decodes a SCIP message

      \param[in] data the SCIP message to decode
      \param[in] size the data encoding types (number of bytes for encoding)

      \retval Value after decoding
    */
    extern long urg_scip_decode(const char data[], int size);


#ifdef __cplusplus
}
#endif

#endif /* !URG_SENSOR_H */
