#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#include <unistd.h>
static inline void delay(sl_word_size_t ms) {
    while (ms >= 1000) {
        usleep(1000 * 1000);
        ms -= 1000;
    };
    if (ms != 0)
        usleep(ms * 1000);
}

using namespace sl;

void print_usage(int argc, const char *argv[]) {
    printf("Simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s \n"
           "Usage:\n"
           " For serial channel %s --channel --serial <com port> [baudrate]\n"
           "The baudrate is 115200(for A2) or 256000(for A3).\n"
           " For udp channel %s --channel --udp <ipaddr> [port NO.]\n"
           "The LPX default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n",
           "SL_LIDAR_SDK_VERSION", argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth(ILidarDriver *drv) {
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool process_stopped;
void signal_handler(int) {
    process_stopped = true;
}

int main(int argc, const char *argv[]) {
    const char *opt_is_channel = NULL;
    const char *opt_channel = NULL;
    const char *opt_channel_param_first = NULL;
    sl_u32 opt_channel_param_second = 0;
    sl_u32 baudrateArray[2] = {115200, 256000};
    sl_result op_result;
    int opt_channel_type = CHANNEL_TYPE_SERIALPORT;

    bool useArgcBaudrate = false;

    IChannel *_channel;

    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s\n",
           "SL_LIDAR_SDK_VERSION");

    if (argc > 1) {
        opt_is_channel = argv[1];
    } else {
        print_usage(argc, argv);
        return -1;
    }

    if (strcmp(opt_is_channel, "--channel") == 0) {
        opt_channel = argv[2];
        if (strcmp(opt_channel, "-s") == 0 || strcmp(opt_channel, "--serial") == 0) {
            // read serial port from the command line...
            opt_channel_param_first = argv[3]; // or set to a fixed value: e.g. "com3"
            // read baud rate from the command line if specified...
            if (argc > 4)
                opt_channel_param_second = strtoul(argv[4], NULL, 10);
            useArgcBaudrate = true;
        } else if (strcmp(opt_channel, "-u") == 0 || strcmp(opt_channel, "--udp") == 0) {
            // read ip addr from the command line...
            opt_channel_param_first = argv[3]; // or set to a fixed value: e.g. "192.168.11.2"
            if (argc > 4)
                opt_channel_param_second = strtoul(argv[4], NULL, 10); // e.g. "8089"
            opt_channel_type = CHANNEL_TYPE_UDP;
        } else {
            print_usage(argc, argv);
            return -1;
        }
    } else {
        print_usage(argc, argv);
        return -1;
    }

    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
        if (!opt_channel_param_first) {
            opt_channel_param_first = "/dev/ttyUSB0";
        }
    }

    // create the driver instance
    ILidarDriver *drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT) {
        if (useArgcBaudrate) {
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK((drv)->connect(_channel))) {
                op_result = drv->getDeviceInfo(devinfo);

                if (SL_IS_OK(op_result)) {
                    connectSuccess = true;
                } else {
                    delete drv;
                    drv = NULL;
                }
            }
        } else {
            size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
            for (size_t i = 0; i < baudRateArraySize; ++i) {
                _channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK((drv)->connect(_channel))) {
                    op_result = drv->getDeviceInfo(devinfo);

                    if (SL_IS_OK(op_result)) {
                        connectSuccess = true;
                        break;
                    } else {
                        delete drv;
                        drv = NULL;
                    }
                }
            }
        }
    } else if (opt_channel_type == CHANNEL_TYPE_UDP) {
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK((drv)->connect(_channel))) {
            op_result = drv->getDeviceInfo(devinfo);

            if (SL_IS_OK(op_result)) {
                connectSuccess = true;
            } else {
                delete drv;
                drv = NULL;
            }
        }
    }

    if (!connectSuccess) {
        if (opt_channel_type == CHANNEL_TYPE_SERIALPORT)
            fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_channel_param_first);
        else
            fprintf(stderr, "Error, cannot connect to the specified ip addr %s.\n", opt_channel_param_first);

        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n",
           devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF, (int)devinfo.hardware_version);

    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed();
    // start scan...
    drv->startScan(0, 1);

    while (true) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count; ++pos) {
                printf("%3d %5.1f %5d\n",
                       nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT,
                       (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                       nodes[pos].dist_mm_q2 / 4);
                if (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) {
                    printf("---\n");
                }
            }
        }

        if (process_stopped) {
            break;
        }
    }

    drv->stop();
    delay(200);
    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed(0);
    // done!
on_finished:
    if (drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}
