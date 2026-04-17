#include "ros/ros.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "chcnav/bestpos.h"
#include "chcnav/hc_sentence.h"
#include "sensor_msgs/NavSatFix.h"

static ros::Publisher gs_bestpos_pub;
static ros::Publisher gs_fix_pub;

static constexpr uint16_t NOVATEL_BESTPOS_MSG_ID = 42;
static constexpr std::size_t NOVATEL_LONG_HEADER_LENGTH = 28;
static constexpr std::size_t BESTPOS_BODY_LENGTH = 72;
static constexpr std::size_t CRC32_LENGTH = 4;
static constexpr std::size_t BESTPOS_FRAME_LENGTH = NOVATEL_LONG_HEADER_LENGTH + BESTPOS_BODY_LENGTH + CRC32_LENGTH;
static constexpr unsigned int GPS_EPOCH_TO_UNIX = 315964800;

static void hc_sentence_callback(const chcnav::hc_sentence::ConstPtr &msg);
static bool is_fix_status(uint32_t sol_status, uint32_t pos_type);
static uint32_t novatel_crc32(const int8_t *buffer, std::size_t len);
static bool novatel_check_crc32(const std::vector<int8_t> &data);

template <typename T>
static T read_le(const std::vector<int8_t> &data, std::size_t offset)
{
    T value;
    std::memcpy(&value, &data[offset], sizeof(T));
    return value;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bestpos_process_node");

    ros::NodeHandle nh;

    ros::Subscriber hc_sentence_sub = nh.subscribe("hc_sentence", 1000, hc_sentence_callback);
    gs_bestpos_pub = nh.advertise<chcnav::bestpos>("bestpos", 1000);
    gs_fix_pub = nh.advertise<sensor_msgs::NavSatFix>("bestpos/fix", 1000);

    ros::spin();

    return 0;
}

static bool is_fix_status(uint32_t sol_status, uint32_t pos_type)
{
    if (sol_status != 0)
        return false;

    switch (pos_type)
    {
        case 16: // SINGLE
        case 17: // PSRDIFF
        case 18: // WAAS
        case 32: // L1_FLOAT
        case 34: // NARROW_FLOAT
        case 48: // L1_INT
        case 49: // WIDE_INT
        case 50: // NARROW_INT
        case 51: // RTK_DIRECT_INS
        case 52: // INS_SBAS
        case 53: // INS_PSRSP
        case 54: // INS_PSRDIFF
        case 55: // INS_RTKFLOAT
        case 56: // INS_RTKFIXED
        case 68: // PPP_CONVERGING
        case 69: // PPP
        case 73: // INS_PPP_CONVERGING
        case 74: // INS_PPP
        case 77: // PPP_BASIC_CONVERGING
        case 78: // PPP_BASIC
        case 79: // INS_PPP_BASIC_CONVERGING
        case 80: // INS_PPP_BASIC
            return true;
        default:
            return false;
    }
}

static uint32_t novatel_crc32(const int8_t *buffer, std::size_t len)
{
    uint32_t crc = 0;

    for (std::size_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint8_t>(buffer[i]);
        for (int bit = 0; bit < 8; ++bit)
        {
            if ((crc & 1U) != 0U)
                crc = (crc >> 1) ^ 0xEDB88320U;
            else
                crc >>= 1;
        }
    }

    return crc;
}

static bool novatel_check_crc32(const std::vector<int8_t> &data)
{
    if (data.size() < CRC32_LENGTH)
        return false;

    const uint32_t crc_origin = read_le<uint32_t>(data, data.size() - CRC32_LENGTH);
    const uint32_t crc_result = novatel_crc32(data.data(), data.size() - CRC32_LENGTH);
    return crc_origin == crc_result;
}

static void hc_sentence_callback(const chcnav::hc_sentence::ConstPtr &msg)
{
    if (msg->msg_id != NOVATEL_BESTPOS_MSG_ID)
        return;

    if (msg->data.size() != BESTPOS_FRAME_LENGTH)
    {
        ROS_WARN_THROTTLE(1.0, "BESTPOSB frame length mismatch: %zu", msg->data.size());
        return;
    }

    if ((unsigned char)msg->data[0] != 0xAA ||
        (unsigned char)msg->data[1] != 0x44 ||
        (unsigned char)msg->data[2] != 0x12 ||
        (unsigned char)msg->data[3] != 0x1C)
    {
        ROS_WARN_THROTTLE(1.0, "BESTPOSB sync mismatch");
        return;
    }

    if (!novatel_check_crc32(msg->data))
    {
        ROS_WARN_THROTTLE(1.0, "BESTPOSB crc32 check failed");
        return;
    }

    chcnav::bestpos bestpos_msg;
    sensor_msgs::NavSatFix fix_msg;

    bestpos_msg.header = msg->header;
    bestpos_msg.gps_week_number = read_le<uint16_t>(msg->data, 14);
    bestpos_msg.gps_week_milliseconds = read_le<uint32_t>(msg->data, 16);

    const double second = static_cast<double>(bestpos_msg.gps_week_milliseconds) / 1000.0;
    bestpos_msg.header.stamp = ros::Time(bestpos_msg.gps_week_number * 7.0 * 24.0 * 3600.0 + second + GPS_EPOCH_TO_UNIX);

    const std::size_t payload_offset = NOVATEL_LONG_HEADER_LENGTH;
    bestpos_msg.sol_status = read_le<uint32_t>(msg->data, payload_offset + 0);
    bestpos_msg.pos_type = read_le<uint32_t>(msg->data, payload_offset + 4);
    bestpos_msg.lat = read_le<double>(msg->data, payload_offset + 8);
    bestpos_msg.lon = read_le<double>(msg->data, payload_offset + 16);
    bestpos_msg.hgt = read_le<double>(msg->data, payload_offset + 24);
    bestpos_msg.undulation = read_le<float>(msg->data, payload_offset + 32);
    bestpos_msg.datum_id = read_le<uint32_t>(msg->data, payload_offset + 36);
    bestpos_msg.lat_stdev = read_le<float>(msg->data, payload_offset + 40);
    bestpos_msg.lon_stdev = read_le<float>(msg->data, payload_offset + 44);
    bestpos_msg.hgt_stdev = read_le<float>(msg->data, payload_offset + 48);

    char stn_id[5] = {0};
    std::memcpy(stn_id, &msg->data[payload_offset + 52], 4);
    bestpos_msg.stn_id = stn_id;

    bestpos_msg.diff_age = read_le<float>(msg->data, payload_offset + 56);
    bestpos_msg.sol_age = read_le<float>(msg->data, payload_offset + 60);
    bestpos_msg.num_svs = msg->data[payload_offset + 64];
    bestpos_msg.num_sol_svs = msg->data[payload_offset + 65];
    bestpos_msg.num_sol_l1_svs = msg->data[payload_offset + 66];
    bestpos_msg.num_sol_multi_svs = msg->data[payload_offset + 67];
    bestpos_msg.reserved = msg->data[payload_offset + 68];
    bestpos_msg.ext_sol_stat = msg->data[payload_offset + 69];
    bestpos_msg.galileo_beidou_sig_mask = msg->data[payload_offset + 70];
    bestpos_msg.gps_glonass_sig_mask = msg->data[payload_offset + 71];

    gs_bestpos_pub.publish(bestpos_msg);

    fix_msg.header = bestpos_msg.header;
    fix_msg.latitude = bestpos_msg.lat;
    fix_msg.longitude = bestpos_msg.lon;
    fix_msg.altitude = bestpos_msg.hgt;
    fix_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    fix_msg.status.status = is_fix_status(bestpos_msg.sol_status, bestpos_msg.pos_type) ? sensor_msgs::NavSatStatus::STATUS_FIX : sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    fix_msg.position_covariance[0] = std::pow(bestpos_msg.lat_stdev, 2.0);
    fix_msg.position_covariance[4] = std::pow(bestpos_msg.lon_stdev, 2.0);
    fix_msg.position_covariance[8] = std::pow(bestpos_msg.hgt_stdev, 2.0);
    fix_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    gs_fix_pub.publish(fix_msg);
}
