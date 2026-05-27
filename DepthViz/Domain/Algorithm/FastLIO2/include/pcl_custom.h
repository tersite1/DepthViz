#ifdef USE_LEGACY_FAST_LIO2
#ifndef PCL_CUSTOM_H
#define PCL_CUSTOM_H

using MsgFieldMap = std::vector< pcl::detail::FieldMapping >;

namespace pcl_custom{
    void createMapping (const std::vector<pcl::PCLPointField>& msg_fields, MsgFieldMap& field_map)
{
    // Create initial 1-1 mapping between serialized data segments and struct fields
    pcl::detail::FieldMapper<velodyne_ros::Point> mapper(msg_fields, field_map);
    // for (size_t i = 0; i < mapper.map_.size(); i++)
    // {
    //   std::cout << mapper.map_[i] << std::endl;
    // }
    
    pcl::for_each_type< typename pcl::traits::fieldList<velodyne_ros::Point>::type > (mapper);

    // Coalesce adjacent fields into single memcpy's where possible
    if (field_map.size() > 1)
    {
        std::sort(field_map.begin(), field_map.end(), pcl::detail::fieldOrdering);
        MsgFieldMap::iterator i = field_map.begin(), j = i + 1;
        while (j != field_map.end())
        {
            // This check is designed to permit padding between adjacent fields.
            /// @todo One could construct a pathological case where the struct has a
            /// field where the serialized data has padding
            if (j->serialized_offset - i->serialized_offset == j->struct_offset - i->struct_offset)
            {
                i->size += (j->struct_offset + j->size) - (i->struct_offset + i->size);
                j = field_map.erase(j);
            }
            else
            {
                ++i;
                ++j;
            }
        }
    }
}

void fromPCLPointCloud2 (const pcl::PCLPointCloud2& msg, pcl::PointCloud<velodyne_ros::Point>& cloud,
            const MsgFieldMap& field_map)
{
    // Copy info fields
    cloud.header   = msg.header;
    cloud.width    = msg.width;
    cloud.height   = msg.height;
    cloud.is_dense = msg.is_dense == 1;

    // Copy point data
    cloud.resize (msg.width * msg.height);
    std::uint8_t* cloud_data = reinterpret_cast<std::uint8_t*>(&cloud[0]);

    // Check if we can copy adjacent points in a single memcpy.  We can do so if there
    // is exactly one field to copy and it is the same size as the source and destination
    // point types.
    if (field_map.size() == 1 &&
        field_map[0].serialized_offset == 0 &&
        field_map[0].struct_offset == 0 &&
        field_map[0].size == msg.point_step &&
        field_map[0].size == sizeof(velodyne_ros::Point))
    {
        const auto cloud_row_step = (sizeof (velodyne_ros::Point) * cloud.width);
        const std::uint8_t* msg_data = &msg.data[0];
        // Should usually be able to copy all rows at once
        if (msg.row_step == cloud_row_step)
        {
            std::copy(msg.data.cbegin(), msg.data.cend(), cloud_data);
        }
        else
        {
            for (size_t i = 0; i < msg.height; ++i, cloud_data += cloud_row_step, msg_data += msg.row_step)
            memcpy (cloud_data, msg_data, cloud_row_step);
        }

    }
    else
    {
        // If not, memcpy each group of contiguous fields separately
        for (size_t row = 0; row < msg.height; ++row)
        {
            const std::uint8_t* row_data = &msg.data[row * msg.row_step];
            for (size_t col = 0; col < msg.width; ++col)
            {
                const std::uint8_t* msg_data = row_data + col * msg.point_step;
                for (const pcl::detail::FieldMapping& mapping : field_map)
                {
                    std::copy(msg_data + mapping.serialized_offset, msg_data + mapping.serialized_offset + mapping.size,
                                cloud_data + mapping.struct_offset);
                }
                cloud_data += sizeof(velodyne_ros::Point);
            }
        }
    }
}

void fromPCLPointCloud2 (const pcl::PCLPointCloud2& msg, pcl::PointCloud<velodyne_ros::Point>& cloud)
{
    MsgFieldMap field_map;
    createMapping(msg.fields, field_map);
    fromPCLPointCloud2 (msg, cloud, field_map);
}

void toPCL(const custom_messages::PointField &pf, pcl::PCLPointField &pcl_pf)
{
    pcl_pf.name = pf.name;
    pcl_pf.offset = pf.offset;
    pcl_pf.datatype = pf.datatype;
    pcl_pf.count = pf.count;
}

void toPCL(const std::vector<custom_messages::PointField> &pfs, std::vector<pcl::PCLPointField> &pcl_pfs)
{
    pcl_pfs.resize(pfs.size());
    std::vector<custom_messages::PointField>::const_iterator it = pfs.begin();
    int i = 0;
    for(; it != pfs.end(); ++it, ++i) {
        toPCL(*(it), pcl_pfs[i]);
    }
}

void toPCL(const custom_messages::Time &stamp, pcl::uint64_t &pcl_stamp)
{
    pcl_stamp = stamp.toNsec() / 1000ull;  // Convert from ns to us
}

void toPCL(const custom_messages::Header &header, pcl::PCLHeader &pcl_header)
{
    toPCL(header.stamp, pcl_header.stamp);
    pcl_header.seq = header.seq;
    pcl_header.frame_id = header.frame_id;
}

void copyPointCloud2MetaData(const custom_messages::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
{
    toPCL(pc2.header, pcl_pc2.header);
    pcl_pc2.height = pc2.height;
    pcl_pc2.width = pc2.width;
    toPCL(pc2.fields, pcl_pc2.fields);
    pcl_pc2.is_bigendian = pc2.is_bigendian;
    pcl_pc2.point_step = pc2.point_step;
    pcl_pc2.row_step = pc2.row_step;
    pcl_pc2.is_dense = pc2.is_dense;
}

void toPCL(const custom_messages::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
{
    copyPointCloud2MetaData(pc2, pcl_pc2);

    unsigned long int size_ = pc2.data.size();
    pcl_pc2.data.resize(size_);
    for (size_t i = 0; i < size_; i++)
    {
        pcl_pc2.data[i] = pc2.data[i];
    }
}

    void fromROSMsg(const custom_messages::PointCloud2 &cloud, pcl::PointCloud<velodyne_ros::Point> &pcl_cloud)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        toPCL(cloud, pcl_pc2);

        fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    }
}

#endif
#endif
