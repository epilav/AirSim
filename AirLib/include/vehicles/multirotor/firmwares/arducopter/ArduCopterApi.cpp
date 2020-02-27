#include "ArducopterApi.hpp"
#include "ArducopterPacketBuffer.hpp"

namespace msr { namespace airlib {

void ArduCopterApi::sendSensors() 
{
    if (sensors_ == nullptr || udpSocket_ == nullptr)
        return;

    using PacketBuffer = ArduCopterPacketBuffer<32768>;

    PacketBuffer buffer;

    // Reserve headroom for the TOC.
    buffer.put<uint16_t>(0, 0, 0, 0, 0);

    uint16_t toc_rc         = 0;
    uint16_t toc_lidar      = 0;
    uint16_t toc_distance   = 0;
    uint16_t toc_imu        = 0;
    uint16_t toc_gps        = 0;

    // RC
    // --
    //
    // {
    //     channel-count:           8-bit value
    //     channel-values:          variable length float array
    //                              (roll, yaw, pitch, and 4 switches in this case)
    // }
    if (is_rc_connected && last_rcData_.is_valid) {
        toc_rc = static_cast<uint16_t>(buffer.getIndex());
        buffer.put<uint8_t>(7); // Here we always have 7 channels
        buffer.put<float>(
            (last_rcData_.roll + 1) * 0.5f, (last_rcData_.yaw + 1) * 0.5f, (last_rcData_.pitch + 1) * 0.5f,
            static_cast<float>(last_rcData_.getSwitch(0)), static_cast<float>(last_rcData_.getSwitch(1)), 
            static_cast<float>(last_rcData_.getSwitch(2)), static_cast<float>(last_rcData_.getSwitch(3)));
    }

    // Lidars
    // ------
    //
    // lidar-count: 8-bit value
    // lidars: variable length array of 
    // {
    //     name-length:             16-bit value
    //     name:                    variable length 8-bit value array
    //     point-cloud-size:        32-bit value
    //     point-cloud:             variable length float array
    // }
    uint lidar_count = sensors_->size(SensorBase::SensorType::Lidar);
    if (lidar_count) {
        toc_lidar = static_cast<uint16_t>(buffer.getIndex());
        buffer.put<uint8_t>(lidar_count);
        for (uint i = 0; i < lidar_count; i++) {
            auto lidar = static_cast<const LidarBase *>(sensors_->getByType(SensorBase::SensorType::Lidar, i));
            auto& lidar_output = lidar->getOutput();
            buffer.putString(lidar->getName());
            buffer.putVector(lidar_output.point_cloud);
        }
    }

    // Distance sensors
    // ----------------
    //
    // distance-conut: 8-bit value
    // distance-sensors: variable length array of 
    // {
    //     name-length:             16-bit value
    //     name:                    variable length 8-bit value array
    //     distance:                float
    //     max-distance:            float
    //     min-distance:            float
    //     roll:                    float
    //     pitch:                   float
    //     yaw:                     float
    // }
    uint distance_count = sensors_->size(SensorBase::SensorType::Distance);
    if (distance_count) {
        toc_distance = static_cast<uint16_t>(buffer.getIndex());
        buffer.put<uint8_t>(distance_count);
        for (uint i = 0; i < distance_count; i++) {
            auto distance = static_cast<const DistanceBase *>(sensors_->getByType(SensorBase::SensorType::Distance, i));
            auto& distance_output = distance->getOutput();
            float yaw, pitch, roll;
            VectorMath::toEulerianAngle(distance_output.relative_pose.orientation, pitch, roll, yaw);
            buffer.putString(distance->getName());
            buffer.put<float>(
                distance_output.distance, distance_output.max_distance, distance_output.min_distance,
                roll, pitch, yaw);
        }
    }

    // IMU
    // ---
    //
    // {
    //     angular-velocity:        3 floats
    //     linear-acceleration:     3 floats
    //     roll:                    float 
    //     pitch:                   float
    //     yaw:                     float
    // }
    const auto& imu_output = getImu()->getOutput();
    float yaw, pitch, roll;
    VectorMath::toEulerianAngle(imu_output.orientation, pitch, roll, yaw);
    toc_imu = static_cast<uint16_t>(buffer.getIndex());
    buffer.put<float>(
        imu_output.angular_velocity[0], imu_output.angular_velocity[1], imu_output.angular_velocity[2],
        imu_output.linear_acceleration[0], imu_output.linear_acceleration[1], imu_output.linear_acceleration[2],
        roll, pitch, yaw);

    // GPS
    // ---
    //
    // {
    //     latitude:                double
    //     longitude:               double
    //     altitude:                double
    //     velocity:                3 floats
    // }
    const auto& gps_output = getGps()->getOutput();
    toc_gps = static_cast<uint16_t>(buffer.getIndex());
    buffer.put<double>(
        gps_output.gnss.geo_point.latitude, gps_output.gnss.geo_point.longitude, gps_output.gnss.geo_point.altitude);
    buffer.put<float>(gps_output.gnss.velocity[0], gps_output.gnss.velocity[1], gps_output.gnss.velocity[2]);

    // Build the TOC
    buffer.setIndex(0);
    buffer.put<uint16_t>(toc_rc, toc_lidar, toc_distance, toc_imu, toc_gps);

    // Finally, dispatch the packet.
    udpSocket_->sendto(buffer.getBytes(), buffer.getSize(), ip, port);
}

}} // namespace