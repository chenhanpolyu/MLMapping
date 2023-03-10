#include "rviz_vis.h"

rviz_vis::rviz_vis()
{
}

rviz_vis::~rviz_vis()
{
}

void rviz_vis::set_as_awareness_map_publisher(ros::NodeHandle &nh,
                                              string topic_name,
                                              string frame_id,
                                              unsigned int buffer_size,
                                              awareness_map_cylindrical *awareness_map)
{
    this->map_pub = nh.advertise<visualization_msgs::Marker>(topic_name, buffer_size);
    this->frame_id = frame_id;
    this->min_z = awareness_map->z_border_min;
    this->max_z = awareness_map->z_border_min + (awareness_map->map_dZ * awareness_map->map_nZ);
    this->range_z = max_z - min_z;
}

void rviz_vis::set_as_local_map_publisher(ros::NodeHandle &nh,
                                          string topic_name,
                                          string frame_id,
                                          unsigned int buffer_size,
                                          local_map_cartesian *localmap)
{
    this->map_pub = nh.advertise<visualization_msgs::Marker>(topic_name, buffer_size);
    this->frame_id = frame_id;
    this->cube_size_xyz = localmap->vis_paras.cube_size_xyz;
    this->min_z = localmap->vis_paras.map_minz;
    this->max_z = localmap->vis_paras.map_maxz;
    this->range_z = max_z - min_z;
}

void rviz_vis::set_as_global_map_publisher(ros::NodeHandle &nh,
                                           string topic_name,
                                           string frame_id,
                                           unsigned int buffer_size)
{
    this->map_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, buffer_size);
    cout << "frame id: " << endl;
    this->frame_id = frame_id;

    cout << this->frame_id << endl;
}

void rviz_vis::set_as_unknownpts_publisher(ros::NodeHandle &nh,
                                 string topic_name,
                                 string frame_id,
                                 unsigned int buffer_size)
{
    this->map_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, buffer_size);
    cout << "frame id: " << endl;
    this->frame_id = frame_id;
    cout << this->frame_id << endl;
    
}
void rviz_vis::set_as_frontier_publisher(ros::NodeHandle &nh,
                                         string topic_name,
                                         string frame_id,
                                         unsigned int buffer_size)
{
    this->map_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, buffer_size);
    this->frame_id = frame_id;
}

void rviz_vis::set_as_odds_publisher(ros::NodeHandle &nh,
                                     string topic_name,
                                     string frame_id,
                                     unsigned int buffer_size)
{
    this->map_pub = nh.advertise<visualization_msgs::MarkerArray>(topic_name, buffer_size);
    this->frame_id = frame_id;
}
// input: ratio is between 0 to 1
// output: rgb color
Vec3 spherecolor(double ratio)
{
    // we want to normalize ratio so that it fits in to 6 regions
    // where each region is 256 units long
    int normalized = int(ratio * 256 * 3);
    // find the distance to the start of the closest region
    int x = normalized % 256;
    int red = 0, grn = 0, blu = 0;
    switch (normalized / 256)
    {
    case 0:
        red = 255;
        grn = x;
        blu = 0;
        break; // red->yellow
    case 1:
        red = 255 - x;
        grn = 255;
        blu = 0;
        break; // yellow->green
    case 2:
        red = 0;
        grn = 255;
        blu = x;
        break; // green->cyan
        // case 3: red = 0;        grn = 255 - x;  blu = 255;     break;//cyan->blue
        // case 4: red = x;        grn = 0;        blu = 255;     break;//blue
    }

    return Vec3(red / 260.0, grn / 260.0, blu / 260.0);
}

Vec3 cubecolor(double ratio)
{
    // we want to normalize ratio so that it fits in to 6 regions
    // where each region is 256 units long
    int normalized = int(ratio * 256 * 3);
    // find the distance to the start of the closest region
    int x = normalized % 256;
    int red = 0, grn = 0, blu = 0;
    switch (normalized / 256)
    {
    case 0:
        red = 0;
        grn = 255 - x;
        blu = 255;
        break; // cyan->blue
    case 1:
        red = x;
        grn = 0;
        blu = 255;
        break; // blue->magenta
    case 2:
        red = 255;
        grn = 0;
        blu = 255 - x;
        break; // magenta->red
    }

    return Vec3(red / 260.0, grn / 260.0, blu / 260.0);
}

void rviz_vis::pub_awareness_map(awareness_map_cylindrical *localmap, const ros::Time stamp)
{
    visualization_msgs::Marker spheres;
    spheres.header.frame_id = this->frame_id;
    spheres.header.stamp = stamp;
    spheres.ns = "points";
    spheres.type = visualization_msgs::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::Marker::ADD;
    spheres.pose.orientation.w = 1.0;
    spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.4;
    spheres.id = 0;
    for (auto i : localmap->occupied_cell_idx)
    {
        geometry_msgs::Point point;
        Vec3 pt = localmap->map->at(i).center_pt;
        point.x = pt.x();
        point.y = pt.y();
        point.z = pt.z();
        spheres.points.push_back(point);
        std_msgs::ColorRGBA color;
        //        double ratio = (pt.z()-min_z)/range_z;
        //        Vec3 rgb = spherecolor(ratio);
        //        color.r= static_cast<float>(rgb(0));
        //        color.g= static_cast<float>(rgb(1));
        //        color.b= static_cast<float>(rgb(2));
        //        color.a= static_cast<float>(0.9);
        color.r = static_cast<float>(1.0);
        color.g = static_cast<float>(1.0);
        color.b = static_cast<float>(1.0);
        color.a = static_cast<float>(1.0);
        spheres.colors.push_back(color);
    }
    if (spheres.points.size() != 0)
    {
        this->map_pub.publish(spheres);
    }
}

void rviz_vis::pub_local_map(local_map_cartesian *localmap, const ros::Time stamp)
{
    visualization_msgs::Marker cubes;
    cubes.header.frame_id = this->frame_id;
    cubes.header.stamp = stamp;
    cubes.ns = "points";
    cubes.type = visualization_msgs::Marker::CUBE_LIST;
    cubes.action = visualization_msgs::Marker::ADD;
    cubes.pose.orientation.w = 1.0;
    cubes.scale.x = cubes.scale.y = cubes.scale.z = cube_size_xyz;
    cubes.id = 0;
    // cout << "localmap occupied_cell_idx size " << localmap->occupied_cell_idx.size() << endl;
    for (auto i : localmap->occupied_cell_idx)
    {
        geometry_msgs::Point point;
        Vec3 pt = localmap->map->at(i).center_pt;
        point.x = pt(0);
        point.y = pt(1);
        point.z = pt(2);
        cubes.points.push_back(point);
        double ratio = (pt.z() - min_z) / range_z;
        Vec3 rgb = cubecolor(ratio);
        std_msgs::ColorRGBA color;
        color.r = static_cast<float>(rgb(0));
        color.g = static_cast<float>(rgb(1));
        color.b = static_cast<float>(rgb(2));
        color.a = static_cast<float>(0.5);
        cubes.colors.push_back(color);
    }
    if (cubes.points.size() != 0)
    {
        this->map_pub.publish(cubes);
    }
    // publish the range
    visualization_msgs::Marker range;
    range.header.frame_id = this->frame_id;
    range.header.stamp = ros::Time();
    range.ns = "range";
    range.id = 0;
    range.type = visualization_msgs::Marker::CUBE;
    range.action = visualization_msgs::Marker::ADD;
    range.pose.position.x = range.pose.position.y = range.pose.position.z = 0;
    range.pose.orientation.x = 0.0;
    range.pose.orientation.y = 0.0;
    range.pose.orientation.z = 0.0;
    range.pose.orientation.w = 1.0;
    range.scale.x = range.scale.y = localmap->vis_paras.map_size_xy;
    range.scale.z = localmap->vis_paras.map_size_z;
    range.color.a = 0.1; // Don't forget to set the alpha!
    range.color.r = 0.0;
    range.color.g = 1.0;
    range.color.b = 0.0;
    // only if using a MESH_RESOURCE marker type:
    this->map_pub.publish(range);
    visualization_msgs::Marker range2 = range;
    visualization_msgs::Marker range3 = range;
    range2.id = 1;
    range2.scale.x = range2.scale.y = localmap->vis_paras.map_size_xy * 0.2;
    range2.scale.z = localmap->vis_paras.map_size_z * 0.2;
    range2.color.a = 0.2;
    this->map_pub.publish(range2);
    range3.id = 2;
    //    range3.scale.x = range3.scale.y = localmap->vis_paras.map_size_xy*0.6;
    //    range3.scale.z = localmap->vis_paras.map_size_z*0.6;
    //    range3.color.a = 0.2;
    //    this->map_pub.publish(range3);
}

// typedef pcl::PointXYZ             PointP;
// typedef pcl::PointCloud<PointP>   PointCloudP;
// typedef PointCloudP::Ptr          PointCloudP_ptr;

// void rviz_vis::pub_frontier(local_map_cartesian *localmap,
//                             const ros::Time stamp)
// {
//     sensor_msgs::PointCloud2 output;
//     PointCloudP_ptr pc(new PointCloudP);
//     pc->header.frame_id = this->frame_id;
//     pc->height = 1;
//     for (auto iter = localmap->frontier_cell_global_idx_map.begin(); iter != localmap->frontier_cell_global_idx_map.end(); iter++)
//     {

//             pc->points.emplace_back(iter->first[0]*localmap->map_dxyz_obv,
//             iter->first[1]*localmap->map_dxyz_obv,
//             iter->first[2]*localmap->map_dxyz_obv);

//     }
//     pc->width = pc->points.size();
//     pcl::toROSMsg(*pc, output);
//     output.header.stamp = stamp;
//     map_pub.publish(output);
// }

// PointP subbox_id2xyz_glb(local_map_cartesian *localmap, Vec3I origin, int idx)
// {
//  return localmap->subbox_id2xyz_glb(origin,idx);
// }

void rviz_vis::pub_frontier(local_map_cartesian *localmap,
                            const ros::Time stamp)
{
    sensor_msgs::PointCloud2 output;
    PointCloudP_ptr pc(new PointCloudP);
    pc->header.frame_id = this->frame_id;
    pc->height = 1;
    // Vec3I pt;
    // size_t aa = 1;
    // PointP p1 = localmap->subbox_id2xyz_glb(pt,2);
    // localmap->clear_map();
    // pt =  localmap->global_xyzidx(aa);
    // localmap->allocate_memory_for_local_map();
    for (auto iter = localmap->observed_group_map.begin(); iter != localmap->observed_group_map.end(); iter++)
    {
        // int subbox_id = 0;
        for (auto it = iter->second.frontier.begin(); it != iter->second.frontier.end(); it++)
        {
            // PointP p1 = localmap->subbox_id2xyz_glb(pt,2);

            pc->points.emplace_back(localmap->subbox_id2xyz_glb(iter->first, *it));
        }
    }
    pc->width = pc->points.size();
    pcl::toROSMsg(*pc, output);
    output.header.stamp = stamp;
    map_pub.publish(output);
}

void rviz_vis::pub_global_local_map(
    local_map_cartesian *localmap,
    const ros::Time stamp)
{

    sensor_msgs::PointCloud2 output;

    PointCloudP_ptr pc(new PointCloudP);
    pc->header.frame_id = this->frame_id;
    pc->height = 1;
    // int cnt = 0;
    for (auto iter = localmap->observed_group_map.begin(); iter != localmap->observed_group_map.end(); iter++)
    {
        int subbox_id = 0;
        // cout<<"global box id: "<<iter->first.transpose()<<" number: "<<cnt++<<endl;
        for (auto it = iter->second.occupancy.begin(); it != iter->second.occupancy.end(); it++)
        {
            // PointP p1 = localmap->subbox_id2xyz_glb(pt,2);
            if (*it == 'o')
            {
                pc->points.emplace_back(localmap->subbox_id2xyz_glb(iter->first, subbox_id));
                // cout<<"point: "<<pc->points.back()<<"glb id: "<<iter->first<<"subbox id: "<<*it<<endl;
            }
            subbox_id++;
        }
    }

    pc->width = pc->points.size();
    pcl::toROSMsg(*pc, output);
    output.header.stamp = stamp;
    map_pub.publish(output);
}

void rviz_vis::pub_unkown_pts(
    local_map_cartesian *localmap,
    const ros::Time stamp,
    const Vector3d &pos)
{ 
    Vec3I glb_idx;
    size_t subbox_id;
    localmap->get_global_idx(pos, glb_idx, subbox_id);
    sensor_msgs::PointCloud2 output;

    PointCloudP_ptr pc(new PointCloudP);
    pc->header.frame_id = this->frame_id;
    pc->height = 1;
    // int cnt = 0;
    for (auto iter = localmap->observed_group_map.begin(); iter != localmap->observed_group_map.end(); iter++)
    {
        if ((iter->first - glb_idx).norm() > 1)
        continue;
        subbox_id = 0;
        // cout<<"global box id: "<<iter->first.transpose()<<" number: "<<cnt++<<endl;
        for (auto it = iter->second.occupancy.begin(); it != iter->second.occupancy.end(); it++)
        {
            // PointP p1 = localmap->subbox_id2xyz_glb(pt,2);
            if (*it == 'u')
            {
                pc->points.emplace_back(localmap->subbox_id2xyz_glb(iter->first, subbox_id));
                // cout<<"point: "<<pc->points.back()<<"glb id: "<<iter->first<<"subbox id: "<<*it<<endl;
            }
            subbox_id++;
        }
    }

    pc->width = pc->points.size();
    pcl::toROSMsg(*pc, output);
    output.header.stamp = stamp;
    map_pub.publish(output);
}


void rviz_vis::pub_odd_slice(local_map_cartesian *localmap,
                             const ros::Time stamp, Fun_odds fun)
{
    visualization_msgs::MarkerArray mks;
    visualization_msgs::Marker spheres, lines;
    spheres.header.frame_id = this->frame_id;
    spheres.header.stamp = stamp;
    spheres.ns = "points";
    spheres.type = visualization_msgs::Marker::CUBE_LIST;
    spheres.action = visualization_msgs::Marker::ADD;
    spheres.pose.orientation.w = 1.0;
    spheres.scale.x = spheres.scale.y = 0.2;
    spheres.scale.z = 0.01;
    spheres.id = 0;

    lines.header.frame_id = this->frame_id;
    lines.header.stamp = stamp;
    lines.ns = "lines";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.05;
    lines.id = 0;
    lines.color.r = lines.color.g = lines.color.b = 1.0;
    for (auto iter = localmap->observed_group_map.begin(); iter != localmap->observed_group_map.end(); iter++)
    {
        int subbox_id = 0;
        // cout<<"global box id: "<<iter->first.transpose()<<" number: "<<cnt++<<endl;
        for (auto it = iter->second.log_odds.begin(); it != iter->second.log_odds.end(); it++)
        {
            // PointP p1 = localmap->subbox_id2xyz_glb(pt,2);
            Vec3 pt = localmap->subbox_id2xyz_glb_vec(iter->first, subbox_id);
            if (pt[2] < 0.701 && pt[2] > 0.699)
            {
                geometry_msgs::Point point;
                point.x = pt[0];
                point.y = pt[1];
                point.z = pt[2];

                spheres.points.push_back(point);
                std_msgs::ColorRGBA color;
                double ratio = logit_inv(*it);
                Vec3 rgb = spherecolor(ratio);
                color.r = static_cast<float>(rgb(0));
                color.g = static_cast<float>(rgb(1));
                color.b = static_cast<float>(rgb(2));
                color.a = static_cast<float>(0.9);
                spheres.colors.push_back(color);

                if (ratio > 0.8)
                {
                    geometry_msgs::Point point1;
                    Vec3 grad = fun(pt, 5);
                    point1.x = point.x + grad[0];
                    point1.y = point.y + grad[1];
                    point1.z = point.z + grad[2];
                    lines.points.emplace_back(point);
                    lines.points.emplace_back(point1);
                }
                subbox_id++;
                // cout<<"point: "<<pc->points.back()<<"glb id: "<<iter->first<<"subbox id: "<<*it<<endl;
                // subbox_id++;
            }
        }
    }
    mks.markers.push_back(spheres);
    mks.markers.push_back(lines);
    // cout<<"size: "<<spheres.points.size()<<endl;
    if (spheres.points.size() != 0)
    {
        this->map_pub.publish(mks);
    }
}