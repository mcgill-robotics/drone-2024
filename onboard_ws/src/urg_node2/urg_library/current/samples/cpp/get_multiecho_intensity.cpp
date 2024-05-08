/*!
  \~japanese
  \example get_multiecho_intensity.cpp �����E���x�f�[�^(�}���`�G�R�[)���擾����
  \~english
  \example get_multiecho_intensity.cpp Obtains multiecho distance and intensity data
  \~
  \author Satofumi KAMIMURA

  $Id$
*/

#include "Urg_driver.h"
#include "Connection_information.h"
#include <iostream>

using namespace qrk;
using namespace std;


// \~japanese �����A���x�̃f�[�^��\������
// \~english Prints distance and intensity data
namespace
{
    void print_echo_data(const vector<long>& data,
                         const vector<unsigned short> intensity,
                         int index, int max_echo_size)
    {
        // [mm]
        for (int i = 0; i < max_echo_size; ++i) {
            cout << data[(max_echo_size * index) + i] << ", ";
        }

        // [1]
        for (int i = 0; i < max_echo_size; ++i) {
            cout << intensity[(max_echo_size * index) + i] << ", ";
        }
    }


    void print_data(const Urg_driver& urg,
                    const vector<long>& data,
                    const vector<unsigned short>& intensity,
                    long time_stamp)
    {
#if 1
        // \~japanese �O���̃f�[�^�݂̂�\��
        // \~english Shows only the front step
        int front_index = urg.step2index(0);
        print_echo_data(data, intensity, front_index, urg.max_echo_size());
        cout << time_stamp << endl;

#else
        static_cast<void>(urg);

        // \~japanese �S�Ẵf�[�^��\��
        // \~english Prints the multiecho range/intensity values for all the measurement points
        size_t data_n = data.size();
        cout << "# n = " << data_n << ", timestamp = " << time_stamp << endl;

        int max_echo_size = urg.max_echo_size();
        for (size_t i = 0; i < data_n; ++i) {
            print_echo_data(data, intensity, i, max_echo_size);
            cout << endl;
        }
        cout << endl;
#endif
    }
}


int main(int argc, char *argv[])
{
    Connection_information information(argc, argv);

    // \~japanese �ڑ�
    // \~english Connects to the sensor
    Urg_driver urg;
    if (!urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type())) {
        cout << "Urg_driver::open(): "
             << information.device_or_ip_name() << ": " << urg.what() << endl;
        return 1;
    }

    // \~japanese �f�[�^�擾
    // \~english Gets measurement data
    enum { Capture_times = 10 };
    urg.start_measurement(Urg_driver::Multiecho_intensity, Urg_driver::Infinity_times, 0);
    for (int i = 0; i < Capture_times; ++i) {
        vector<long> data;
        vector<unsigned short> intensity;
        long time_stamp = 0;

        if (!urg.get_multiecho_intensity(data, intensity, &time_stamp)) {
            cout << "Urg_driver::get_distance(): " << urg.what() << endl;
            return 1;
        }
        print_data(urg, data, intensity, time_stamp);
    }

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}