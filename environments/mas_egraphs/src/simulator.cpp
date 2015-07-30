






void Simulator::simulate(){
  // make request for sensor update

  // publish pointcloud

  visualization_msgs::Marker pos;
    
  ros::Time sim_time = ros::Time::now();  
  sensor_msgs::PointCloud pointcloud;
  pos = plan.markers[t];
  int id = 0;
  
  pointcloud.header.stamp = sim_time;
  pointcloud.header.frame_id = costmap_ros_->getGlobalFrameID();

  /*
    geometry_msgs::Point32 point;
    sensor_msgs::ChannelFloat32 channel;
    marker.id = id; 
    id++;
    point.x = ix;
    point.y = iy;
    point.z = 0;	
    
    double cost = cost_map_.getCost(ix, iy)/255;
    channel.name = "intensity";
    channel.values.push_back(cost);
    }
    pointcloud.push_back(point);
    channels.push_back(channel);
    }
    
    obstacles_pub.publish(pointcloud);
  */
}
