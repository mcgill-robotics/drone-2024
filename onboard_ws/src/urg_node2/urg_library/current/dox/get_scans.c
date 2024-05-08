#include "urg_sensor.h"
#include <stddef.h>

int main(void)
{
    urg_t urg;
    int ret;
    long *length_data = NULL;
// \~japanese scan_times ��̃X�L�����f�[�^���擾
// \~english Obtains measurement data for scan_times scans

// \~japanese urg_start_measurement() �֐��ŃX�L�����񐔂��w�肵
// \~english Uses urg_start_measurement() function to define the number of scans
// \~japanese urg_get_distance() �֐��Ŏw�肵���񐔂����f�[�^����M����B
// \~english Uses urg_get_distance() function to receive the measurement data

const int scan_times = 123;
int length_data_size;
int i;

// \~japanese �Z���T���狗���f�[�^���擾����B
// \~english Starts range data measurement
ret = urg_start_measurement(&urg, URG_DISTANCE, scan_times, 0);
// \todo check error code

for (i = 0; i < scan_times; ++i) {
    length_data_size = urg_get_distance(&urg, length_data, NULL);
    // \todo process length_data array
}
return 0;
}
