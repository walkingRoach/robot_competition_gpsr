#include "CYdLidar.h"
#include "common.h"
#include <map>
#include <angles.h>

using namespace std;
using namespace ydlidar;
using namespace impl;
using namespace angles;


/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CYdLidar::CYdLidar(): lidarPtr(nullptr)
{
    m_SerialPort        = "";
    m_SerialBaudrate    = 230400;
    m_FixedResolution   = true;
    m_Reversion         = true;
    m_AutoReconnect     = true;
    m_MaxAngle          = 180.f;
    m_MinAngle          = -180.f;
    m_MaxRange          = 16.0;
    m_MinRange          = 0.08;
    m_SampleRate        = 9;
    m_ScanFrequency     = 10;
    isScanning          = false;
    node_counts         = 1440;
    each_angle          = 0.25;
    frequencyOffset     = 0.4;
    m_AbnormalCheckCount  = 4;
    Major               = 0;
    Minjor              = 0;
    m_IgnoreArray.clear();
}

/*-------------------------------------------------------------
                    ~CYdLidar
-------------------------------------------------------------*/
CYdLidar::~CYdLidar()
{
    disconnecting();
}

void CYdLidar::disconnecting()
{
    if (lidarPtr)
    {
        lidarPtr->disconnect();
        delete lidarPtr;
        lidarPtr = nullptr;
    }

    isScanning = false;
}

std::map<std::string, std::string>  CYdLidar::lidarPortList()
{
    return ydlidar::YDlidarDriver::lidarPortList();
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
bool  CYdLidar::doProcessSimple(LaserScan &outscan, bool &hardwareError)
{
    hardwareError			= false;

    // Bound?
    if (!checkHardware())
    {
        hardwareError = true;
        delay(1000 / m_ScanFrequency);
        return false;
    }

    node_info nodes[2048];
    size_t   count = _countof(nodes);
    size_t all_nodes_counts = node_counts;

    //wait Scan data:
    uint64_t tim_scan_start = getTime();
    result_t op_result =  lidarPtr->grabScanData(nodes, count);
    uint64_t tim_scan_end = getTime();

    // Fill in scan data:
    if (IS_OK(op_result))
    {
        op_result = lidarPtr->ascendScanData(nodes, count);
        tim_scan_start = nodes[0].stamp;
        tim_scan_end   = nodes[count - 1].stamp;;

        if (IS_OK(op_result) && count > 1)
        {
            if (!m_FixedResolution)
            {
                all_nodes_counts = count;
            }
            else
            {
                all_nodes_counts = node_counts;
            }

            each_angle = 360.0 / all_nodes_counts;
            node_info *angle_compensate_nodes = new node_info[all_nodes_counts];
            memset(angle_compensate_nodes, 0, all_nodes_counts * sizeof(node_info));
            unsigned int i = 0;
            for( ; i < count; i++)
            {
                if (nodes[i].distance_q2 != 0)
                {
                    float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);
                    if(m_Reversion)
                    {
                        angle = angle + 180;
                        if(angle >= 360)
                        {
                            angle = angle - 360;
                        }
                        nodes[i].angle_q6_checkbit = ((uint16_t)(angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
                    }
                    int inter = (int)( angle / each_angle );
                    float angle_pre = angle - inter * each_angle;
                    float angle_next = (inter + 1) * each_angle - angle;
                    if (angle_pre < angle_next)
                    {
                        if(inter < all_nodes_counts)
                        {
                            angle_compensate_nodes[inter] = nodes[i];
                        }
                    }
                    else
                    {
                        if (inter < all_nodes_counts - 1)
                        {
                            angle_compensate_nodes[inter + 1] = nodes[i];
                        }
                    }
                }
                if(nodes[i].stamp < tim_scan_start)
                {
                    tim_scan_start = nodes[i].stamp;
                }
                if(nodes[i].stamp > tim_scan_end)
                {
                    tim_scan_end = nodes[i].stamp;
                }
            }


            LaserScan scan_msg;
            if (m_MaxAngle < m_MinAngle)
            {
                float temp = m_MinAngle;
                m_MinAngle = m_MaxAngle;
                m_MaxAngle = temp;
            }


            int counts = all_nodes_counts * ((m_MaxAngle - m_MinAngle) / 360.0f);
            int angle_start = 180 + m_MinAngle;
            int node_start = all_nodes_counts * (angle_start / 360.0f);

            scan_msg.ranges.resize(counts, 0.0);
            scan_msg.intensities.resize(counts, 0.0);
            float range = 0.0;
            float intensity = 0.0;
            int index = 0;
            for (size_t i = 0; i < all_nodes_counts; i++)
            {
                range = (float)angle_compensate_nodes[i].distance_q2 / 4.f / 1000;
                intensity = (float)(angle_compensate_nodes[i].sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

                if (i < all_nodes_counts / 2)
                {
                    index = all_nodes_counts / 2 - 1 - i;
                }
                else
                {
                    index = all_nodes_counts - 1 - (i - all_nodes_counts / 2);
                }

                if (m_IgnoreArray.size() != 0)
                {
                    float angle = (float)((angle_compensate_nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f);
                    if (angle > 180)
                    {
                        angle = 360 - angle;
                    }
                    else
                    {
                        angle = -angle;
                    }

                    for (uint16_t j = 0; j < m_IgnoreArray.size(); j = j + 2)
                    {
                        if ((m_IgnoreArray[j] < angle) && (angle <= m_IgnoreArray[j + 1]))
                        {
                            range = 0.0;
                            break;
                        }
                    }
                }

                if (range > m_MaxRange || range < m_MinRange)
                {
                    range = 0.0;
                }

                int pos = index - node_start ;
                if (0 <= pos && pos < counts)
                {
                    scan_msg.ranges[pos] =  range;
                    scan_msg.intensities[pos] = intensity;
                }
            }




            double scan_time = (tim_scan_end - tim_scan_start);
            scan_time /= 1e9;
            scan_msg.system_time_stamp = tim_scan_start;
            scan_msg.self_time_stamp = tim_scan_start;
            scan_msg.config.min_angle = from_degrees(m_MinAngle);
            scan_msg.config.max_angle = from_degrees(m_MaxAngle);

            if (scan_msg.config.max_angle - scan_msg.config.min_angle == 2 * M_PI)
            {
                scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) /
                                                (double)counts;
                scan_msg.config.time_increment = scan_time / (double)counts;
            }
            else
            {
                scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) /
                                                (double)(counts - 1);
                scan_msg.config.time_increment = scan_time / (double)(counts - 1);
            }

            scan_msg.config.scan_time = scan_time;
            scan_msg.config.min_range = m_MinRange;
            scan_msg.config.max_range = m_MaxRange;
            outscan = scan_msg;
            delete[] angle_compensate_nodes;
            return true;


        }

    }
    else
    {
        if (IS_FAIL(op_result))
        {
            // Error? Retry connection
        }
    }

    return false;

}


/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CYdLidar::turnOn()
{
    if (isScanning && lidarPtr->isscanning())
    {
        return true;
    }
    // start scan...
    result_t op_result = lidarPtr->startScan();
    if (!IS_OK(op_result))
    {
        op_result = lidarPtr->startScan();
        if (!IS_OK(op_result))
        {
            fprintf(stderr, "[CYdLidar] Failed to start scan mode: %x\n", op_result);
            isScanning = false;
            return false;
        }
    }
    if (checkLidarAbnormal())
    {
        lidarPtr->stop();
        fprintf(stderr, "[CYdLidar] Failed to turn on the Lidar, because the lidar is blocked or the lidar hardware is faulty.\n");
        isScanning = false;
        return false;
    }
    isScanning = true;
    lidarPtr->setAutoReconnect(m_AutoReconnect);
    printf("[YDLIDAR INFO] Now YDLIDAR is scanning ......\n");
    fflush(stdout);
    return true;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CYdLidar::turnOff()
{
    if (lidarPtr)
    {
        lidarPtr->stop();
    }
    if (isScanning)
    {
        printf("[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......\n");
    }
    isScanning = false;
    return true;
}

bool CYdLidar::checkLidarAbnormal()
{
    node_info nodes[2048];
    size_t   count = _countof(nodes);
    int check_abnormal_count = 0;
    if (m_AbnormalCheckCount < 2)
    {
        m_AbnormalCheckCount = 2;
    }
    result_t op_result = RESULT_FAIL;
    while (check_abnormal_count < m_AbnormalCheckCount)
    {
        //Ensure that the voltage is insufficient or the motor resistance is high, causing an abnormality.
        if (check_abnormal_count > 0)
        {
            delay(check_abnormal_count * 1000);
        }
        op_result =  lidarPtr->grabScanData(nodes, count);
        if (IS_OK(op_result))
        {
            return false;
        }
        check_abnormal_count++;
    }
    return !IS_OK(op_result);
}

/** Returns true if the device is connected & operative */
bool CYdLidar::getDeviceHealth()
{
    if (!lidarPtr)
    {
        return false;
    }
    lidarPtr->stop();
    result_t op_result;
    device_health healthinfo;
    printf("[YDLIDAR]:SDK Version: %s\n", YDlidarDriver::getSDKVersion().c_str());
    op_result = lidarPtr->getHealth(healthinfo);

    if (IS_OK(op_result))
    {
        printf("[YDLIDAR]:Lidar running correctly ! The health status: %s\n",
               (int)healthinfo.status == 0 ? "good" : "bad");

        if (healthinfo.status == 2)
        {
            fprintf(stderr, "Error, Yd Lidar internal error detected. Please reboot the device to retry.\n");
            return false;
        }
        else
        {
            return true;
        }

    }
    else
    {
        fprintf(stderr, "Error, cannot retrieve Yd Lidar health code: %x\n", op_result);
        return false;
    }

}

bool CYdLidar::getDeviceInfo()
{
    if (!lidarPtr)
    {
        return false;
    }
    device_info devinfo;
    result_t op_result = lidarPtr->getDeviceInfo(devinfo);
    if (!IS_OK(op_result))
    {
        fprintf(stderr, "get Device Information Error\n");
        return false;
    }
    if ( devinfo.model != YDlidarDriver::YDLIDAR_G4)
    {
        printf("[YDLIDAR INFO] Current SDK does not support current lidar models[%d]\n", devinfo.model);
        return false;
    }
    std::string model = "G4";
    switch (devinfo.model)
    {
    case YDlidarDriver::YDLIDAR_G4:
        model = "G4";
        break;
    default:
        break;
    }

    Major = (uint8_t)(devinfo.firmware_version >> 8);
    Minjor = (uint8_t)(devinfo.firmware_version & 0xff);
    printf("[YDLIDAR] Connection established in [%s][%d]:\n"
           "Firmware version: %u.%u\n"
           "Hardware version: %u\n"
           "Model: %s\n"
           "Serial: ",
           m_SerialPort.c_str(),
           m_SerialBaudrate,
           Major,
           Minjor,
           (unsigned int)devinfo.hardware_version,
           model.c_str());

    for (int i = 0; i < 16; i++)
    {
        printf("%01X", devinfo.serialnum[i] & 0xff);
    }

    printf("\n");
    checkSampleRate();
    printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n", m_SampleRate);
    checkScanFrequency();
    return true;
}


void CYdLidar::checkSampleRate()
{
    sampling_rate _rate;
    int _samp_rate = 9;
    int try_count;
    node_counts = 1440;
    each_angle = 0.25;
    result_t ans = lidarPtr->getSamplingRate(_rate);
    if(IS_OK(ans))
    {
        switch (m_SampleRate)
        {
        case 4:
            _samp_rate = YDlidarDriver::YDLIDAR_RATE_4K;
            break;
        case 8:
            _samp_rate = YDlidarDriver::YDLIDAR_RATE_8K;
            break;
        case 9:
            _samp_rate = YDlidarDriver::YDLIDAR_RATE_9K;
            break;
        default:
            _samp_rate = _rate.rate;
            break;
        }
        while (_samp_rate != _rate.rate)
        {
            ans = lidarPtr->setSamplingRate(_rate);
            if (!IS_OK(ans))
            {
                try_count++;
                if (try_count > 3)
                {
                    break;
                }
            }
        }
        switch (_rate.rate)
        {
        case YDlidarDriver::YDLIDAR_RATE_4K:
            _samp_rate = 4;
            node_counts = 720;
            each_angle = 0.5;
            break;
        case YDlidarDriver::YDLIDAR_RATE_8K:
            node_counts = 1440;
            each_angle = 0.25;
            _samp_rate = 8;
            break;
        case YDlidarDriver::YDLIDAR_RATE_9K:
            node_counts = 1440;
            each_angle = 0.25;
            _samp_rate = 9;
            break;
        default:
            break;
        }
    }
    m_SampleRate = _samp_rate;

}

/*-------------------------------------------------------------
                        checkScanFrequency
-------------------------------------------------------------*/
bool CYdLidar::checkScanFrequency()
{
    float frequency = 7.4f;
    scan_frequency _scan_frequency;
    float hz = 0;
    result_t ans = RESULT_FAIL;
    m_ScanFrequency += frequencyOffset;
    if (5.0 - frequencyOffset <= m_ScanFrequency && m_ScanFrequency <= 12 + frequencyOffset)
    {
        ans = lidarPtr->getScanFrequency(_scan_frequency) ;
        if (IS_OK(ans))
        {
            frequency = _scan_frequency.frequency / 100.f;
            hz = m_ScanFrequency - frequency;
            if (hz > 0)
            {
                while (hz > 0.95)
                {
                    lidarPtr->setScanFrequencyAdd(_scan_frequency);
                    hz = hz - 1.0;
                }
                while (hz > 0.09)
                {
                    lidarPtr->setScanFrequencyAddMic(_scan_frequency);
                    hz = hz - 0.1;
                }
                frequency = _scan_frequency.frequency / 100.0f;
            }
            else
            {
                while (hz < -0.95)
                {
                    lidarPtr->setScanFrequencyDis(_scan_frequency);
                    hz = hz + 1.0;
                }
                while (hz < -0.09)
                {
                    lidarPtr->setScanFrequencyDisMic(_scan_frequency);
                    hz = hz + 0.1;
                }
                frequency = _scan_frequency.frequency / 100.0f;
            }
        }
    }
    else
    {
        fprintf(stderr, "current scan frequency[%f] is out of range.",
                m_ScanFrequency - frequencyOffset);
    }
    ans = lidarPtr->getScanFrequency(_scan_frequency);
    if (IS_OK(ans))
    {
        frequency = _scan_frequency.frequency / 100.0f;
        m_ScanFrequency = frequency;
    }
    m_ScanFrequency -= frequencyOffset;
    node_counts = m_SampleRate * 1000 / (m_ScanFrequency - 0.1);
    each_angle = 360.0 / node_counts;
    printf("[YDLIDAR INFO] Current Scan Frequency: %fHz\n", m_ScanFrequency);
    return true;
}


/*-------------------------------------------------------------
						checkCOMMs
-------------------------------------------------------------*/
bool  CYdLidar::checkCOMMs()
{
    if (!lidarPtr)
    {
        // create the driver instance
        lidarPtr = new YDlidarDriver();

        if (!lidarPtr)
        {
            fprintf(stderr, "Create Driver fail\n");
            return false;
        }
    }

    if (lidarPtr->isconnected())
    {
        return true;
    }

    // Is it COMX, X>4? ->  "\\.\COMX"
    if (m_SerialPort.size() >= 3)
    {
        if (tolower(m_SerialPort[0]) == 'c' && tolower(m_SerialPort[1]) == 'o' &&
                tolower(m_SerialPort[2]) == 'm')
        {
            // Need to add "\\.\"?
            if (m_SerialPort.size() > 4 || m_SerialPort[3] > '4')
            {
                m_SerialPort = std::string("\\\\.\\") + m_SerialPort;
            }
        }
    }

    // make connection...
    result_t op_result = lidarPtr->connect(m_SerialPort.c_str(), m_SerialBaudrate);

    if (!IS_OK(op_result))
    {
        fprintf(stderr, "[CYdLidar] Error, cannot bind to the specified serial port[%s] and baudrate[%d]\n",
                m_SerialPort.c_str(), m_SerialBaudrate);
        return false;
    }

    return true;
}

/*-------------------------------------------------------------
                        checkStatus
-------------------------------------------------------------*/
bool CYdLidar::checkStatus()
{

    if (!checkCOMMs())
    {
        return false;
    }

    bool ret = getDeviceHealth();

    if (!ret)
    {
        delay(2000);
        ret = getDeviceHealth();
        if(!ret)
        {
            delay(1000);
        }
    }

    if (!getDeviceInfo())
    {
        delay(2000);
        ret = getDeviceInfo();

        if (!ret)
        {
            return false;
        }
    }
    return true;
}

/*-------------------------------------------------------------
                        checkHardware
-------------------------------------------------------------*/
bool CYdLidar::checkHardware()
{
    if (!lidarPtr)
    {
        return false;
    }
    if (isScanning && lidarPtr->isscanning())
    {
        return true;
    }
    return false;
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
bool CYdLidar::initialize()
{
    if (!checkCOMMs())
    {
        fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR check Comms.\n");
        fflush(stderr);
        return false;
    }

    if(!checkStatus())
    {
        fprintf(stderr, "[CYdLidar::initialize] Error initializing YDLIDAR check status.\n");
        fflush(stderr);
        return false;
    }

    return true;
}
