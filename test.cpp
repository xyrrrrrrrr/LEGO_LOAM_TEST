//gps消息数据类型
#include <sensor_msgs/JointState.h>

using namespace gtsam;

//gps相关
//gps订阅器
ros::Subscriber gps_sub;
//gps数据队列互斥锁
std::mutex gps_queue_mutex;
//gps数据队列
std::deque<sensor_msgs::JointStateConstPtr> gps_queue;
//起点
boost::optional<Eigen::Vector3d> zero_utm;

deque<int> key_numbers_changed;
//utm分区信息发布器
ros::Publisher umt_zone_number_pub;
//起点(LiDAR世界系原点)的utm坐标发布器
ros::Publisher zero_utm_coordinate_pub;


    //下面三行放入nh
    //gps订阅
        gps_sub = nh.subscribe("/chattergps", 1024, &mapOptimization::gps_callback, this);
        //发布utm坐标系的latitude band letter(纬度分区字母)和longitude zone number(经度分区数字)，这两个构成utm分区编号
        umt_zone_number_pub = nh.advertise<std_msgs::UInt8MultiArray>("/umt_zone_number",2);
        //发布起点(LiDAR世界系原点)的utm坐标
        zero_utm_coordinate_pub = nh.advertise<geometry_msgs::Vector3>("/zero_utm_coordinate",2);

    //gps订阅
    void gps_callback(const sensor_msgs::JointState::ConstPtr& gps_msg) {
        std::lock_guard<std::mutex> lock(gps_queue_mutex);

        //GPS数据
        gps_queue.push_back(gps_msg);
        //数据说明
        //gps_msg->effort[0] 经度 longitude
        //gps_msg->effort[1] 纬度 latitude
        //gps_msg->effort[2] 高度 altitude
        //gps_msg->effort[3] 水平速度 speed_hor
        //gps_msg->effort[4] 天向速度 speed_ver
        //gps_msg->effort[5] 航迹角 speed_sog
        //gps_msg->effort[6] 标志位 status
        //gps_msg->effort[7] 航向角 heading
    }

    //将GPS约束添加进因子图
    bool flush_gps_queue() {
        std::lock_guard<std::mutex> lock(gps_queue_mutex);


        //如果关键帧队列为空或只有第0个关键帧，或GPS队列为空，则无法添加GPS约束
        if (cloudKeyPoses3D->points.size() < 1 || gps_queue.empty()){
            return false;
        }

        bool updated = false; //是否有关键帧添加GPS约束的标志位
        auto gps_cursor = gps_queue.begin();
        double keyframe_stamp;
        //遍历所有关键帧，添加GPS约束（第0个关键帧不能添加GPS约束，第0个关键帧的位姿是LiDAR系到车体系的转换关系，是需要通过后续测量估计的，而非GPS的直接测量信息）
        for(int i = 0; i < cloudKeyPoses6D->points.size(); ++i){

            //第0关键帧作为GPS坐标系原点
            if(i == 0 && !zero_utm){
                geographic_msgs::GeoPointStamped gps_data;
                gps_data.position.latitude  = (*gps_queue.begin())->effort[1];
                gps_data.position.longitude = (*gps_queue.begin())->effort[0];
                gps_data.position.altitude  = (*gps_queue.begin())->effort[2];

                geodesy::UTMPoint utm;
                geodesy::fromMsg(gps_data.position, utm);
                Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

                //发布起点utm坐标
                //东北天系
                geometry_msgs::Vector3 zero_utm_coordinate;
                zero_utm_coordinate.x = xyz(0);
                zero_utm_coordinate.y = xyz(1);
                zero_utm_coordinate.z = xyz(2);
                zero_utm_coordinate_pub.publish(zero_utm_coordinate);

                zero_utm = xyz;
                xyz -= (*zero_utm);

                //发布utm分区信息
                //(南京所在utm分区为R50)
                std_msgs::UInt8MultiArray umt_zone_number;
                umt_zone_number.data.push_back((uint8_t)utm.band);
                umt_zone_number.data.push_back(utm.zone);
                umt_zone_number_pub.publish(umt_zone_number);
                //gcc在编译时将uint8_t处理成char，如果不将utm.zone转换为int型再输出，就会输出50对应的ASCII字符'2'
                std::cout << "utm_band:" << utm.band << " " << "utm_zone:" << (int)utm.zone << std::endl;

                cloudKeyPoses6D->points[0].utm_x = xyz(0);
                cloudKeyPoses6D->points[0].utm_y = xyz(1);
                cloudKeyPoses6D->points[0].utm_z = xyz(2);
                //加入GPS约束后，标志位设为1
                cloudKeyPoses6D->points[0].gps_status = 1;

                //向因子图中添加GPS约束
                std::lock_guard<std::mutex> lock(mtx);
                gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(0,0,0),Point3(xyz(0), xyz(1), xyz(2))), gpsNoise));

                double error_distance = sqrt((cloudKeyPoses6D->points[i].z - xyz(0)) * (cloudKeyPoses6D->points[i].z - xyz(0))
                                            +(cloudKeyPoses6D->points[i].x - xyz(1)) * (cloudKeyPoses6D->points[i].x - xyz(1))
                                            +(cloudKeyPoses6D->points[i].y - xyz(2)) * (cloudKeyPoses6D->points[i].y - xyz(2)));
                std::cout << "Adding gps constrain success!" << std::endl
                          << "key_number:" << i << std::endl
                          << "KeyPoses_before_update:" << cloudKeyPoses6D->points[i].z << " " << cloudKeyPoses6D->points[i].x << " " << cloudKeyPoses6D->points[i].y << std::endl
                          << "              GPSPoses:" << xyz(0) << " " << xyz(1) << " " << xyz(2) << std::endl;
                std::cout << "        error_distance:" << error_distance << std::endl;

                updated = true;

                continue;
            }
            //下面两个if循环保证关键帧时间戳处在GPS队列的时间范围内，且关键帧还没有对应的GPS约束
            //判断该关键帧时间戳是否大于GPS最新数据的时间，如果大于，代表后续关键帧没有对应GPS数据，就跳出for循环
            keyframe_stamp = cloudKeyPoses6D->points[i].time;
            if(keyframe_stamp > gps_queue.back()->header.stamp.toSec()){
                break;
            }

            //如果该关键帧时间戳小于GPS队列中最早数据的时间，或者该关键帧已有GPS数据约束，则进行下一个关键帧的判断
            if((keyframe_stamp < (*gps_cursor)->header.stamp.toSec()) || (cloudKeyPoses6D->points[i].gps_status > 0)) {
                continue;
            }
           //<<<<<相近时间戳的GPS数据直接作为位姿约束
            /*
            // find the gps data which is closest to the keyframe
            //找到距离该关键帧时间最接近的GPS数据
            //（这里后续可以改成找到前后最近的GPS数据，然后进行插值）
            auto closest_gps = gps_cursor;
            for(auto gps = gps_cursor; gps != gps_queue.end(); gps++) {
                auto dt = (*closest_gps)->header.stamp.toSec() - keyframe_stamp;
                auto dt2 = (*gps)->header.stamp.toSec() - keyframe_stamp;
                if(std::abs(dt) < std::abs(dt2)) {
                    break;
                }

                closest_gps = gps;
            }

            // if the time residual between the gps and keyframe is too large, skip it
            //如果时间相差过大，则不添加为约束
            gps_cursor = closest_gps;
            if(0.5 < std::abs((*closest_gps)->header.stamp.toSec() - keyframe_stamp)) {
                continue;
            }

            // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
            //将经纬高转换为UTM系（局部东北天）
            geographic_msgs::GeoPointStamped gps_data;
            gps_data.position.latitude  = (*closest_gps)->effort[1];
            gps_data.position.longitude = (*closest_gps)->effort[0];
            gps_data.position.altitude  = (*closest_gps)->effort[2];
            */
            //近时间戳的GPS数据直接作为位姿约束>>>>>





            //<<<<<关键帧前后相邻时间的GPS数据线性插值后作为位姿约束
            
            double dt1 = -1;
            double dt2 = -1;
            auto before_gps = gps_cursor;
            auto next_gps   = gps_cursor;
            for(auto gps = gps_cursor; gps != gps_queue.end(); gps++) {
                dt1 = keyframe_stamp - (*gps)->header.stamp.toSec();
                dt2 = (*(gps+1))->header.stamp.toSec() - keyframe_stamp;
                if(dt1 < 0) {
                    break;
                }
                if(dt2 < 0) {
                    continue;
                }

                before_gps = gps;
                next_gps = gps + 1;
                break;
            }

            //如果没有找到合适的GPS数据，或时间相差过大，就进入下一次循环
            if(dt1 < 0 || dt2 < 0 || dt1 + dt2 > 1.5){
                continue;
            }

            //下一关键帧从next_gps开始搜索GPS数据
            gps_cursor = next_gps;

            //如果GPS信号质量不好，则舍弃
            if((*before_gps)->effort[6] < 3.5 || (*next_gps)->effort[6] < 3.5 ){
                continue;
            }

            double ratio1 = dt1 / (dt1 + dt2);
            double ratio2 = dt2 / (dt1 + dt2);

            geographic_msgs::GeoPointStamped gps_data;
            gps_data.position.latitude  = (*before_gps)->effort[1] * ratio2 + (*next_gps)->effort[1] * ratio1;
            gps_data.position.longitude = (*before_gps)->effort[0] * ratio2 + (*next_gps)->effort[0] * ratio1;
            gps_data.position.altitude  = (*before_gps)->effort[2] * ratio2 + (*next_gps)->effort[2] * ratio1;
            
            //关键帧前后相邻时间的GPS数据线性插值后作为位姿约束>>>>>





            geodesy::UTMPoint utm;
            geodesy::fromMsg(gps_data.position, utm);
            Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

            if(!zero_utm) {
                zero_utm = xyz;
            }
            xyz -= (*zero_utm);

            //std::cout << "x:" << xyz(0) << " "
            //          << "y:" << xyz(1) << " "
            //          << "z:" << xyz(2) << std::endl;

            //水平位置相差太大，说明GPS信号差
            //不对高程作判断，因为高程LiDAR SLAM漂移较快，如果短时间不融GPS，就会导致两者偏差过大，从而后续再也无法融进GPS
            double error_distance = sqrt((cloudKeyPoses6D->points[i].z - xyz(0)) * (cloudKeyPoses6D->points[i].z - xyz(0))
                                        +(cloudKeyPoses6D->points[i].x - xyz(1)) * (cloudKeyPoses6D->points[i].x - xyz(1)));
                                        //+(cloudKeyPoses6D->points[i].y - xyz(2)) * (cloudKeyPoses6D->points[i].y - xyz(2)));
            std::cout << "        error_distance(cm):" << (error_distance*100) << std::endl;

            if(error_distance > 0.50){
                //关键帧没有有效的对应GPS数据，标志位设为1
                cloudKeyPoses6D->points[i].gps_status = 1;
                continue;
            }

            cloudKeyPoses6D->points[i].utm_x = xyz(0);
            cloudKeyPoses6D->points[i].utm_y = xyz(1);
            cloudKeyPoses6D->points[i].utm_z = xyz(2);
            //加入GPS约束后，标志位设为1
            cloudKeyPoses6D->points[i].gps_status = 1;

            //向因子图中添加GPS约束
            std::lock_guard<std::mutex> lock(mtx);
            gtSAMgraph.add(PriorFactor<Pose3>(i, Pose3(Rot3::RzRyRx(0,0,0),Point3(xyz(0), xyz(1), xyz(2))), gpsNoise));

            updated = true;

            //key_numbers_changed.push_back(i);

            std::cout << "Adding gps constrain success!" << std::endl
                      << "            key_number:" << i << std::endl
                      << "KeyPoses_before_update:" << cloudKeyPoses6D->points[i].z << " " << cloudKeyPoses6D->points[i].x << " " << cloudKeyPoses6D->points[i].y << std::endl
                      << "              GPSPoses:" << xyz(0) << " " << xyz(1) << " " <<xyz(2) << std::endl;
        }

        //时间戳小于等于当前最新关键帧的GPS数据从GPS数据队列中移除
        //移除后就无法插值了，因此不移除
        //auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), cloudKeyPoses6D->points[cloudKeyPoses6D->points.size() - 1].time,
        //    [=](const double& stamp, const sensor_msgs::JointState::ConstPtr& geopoint) {
        //        return stamp < geopoint->header.stamp.toSec();
        //    }
        //);
        //gps_queue.erase(gps_queue.begin(), remove_loc);
        return updated;
    }

    void loopClosureThread(){

        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(1);
        while (ros::ok()){
            rate.sleep();
            //如果添加了新的约束，就更新因子图
            if(!flush_gps_queue() & !performLoopClosure()){
            }
            else{
                update_graph();
            }

            /*
            if(key_numbers_changed.size() > 0){
                for(int i = 0; i < key_numbers_changed.size(); ++i){
                    std::cout << "key_number:" << key_numbers_changed[i] << std::endl
                              << "KeyPoses_after_update:" << cloudKeyPoses6D->points[key_numbers_changed[i]].z << " "
                                                          << cloudKeyPoses6D->points[key_numbers_changed[i]].x << " "
                                                          << cloudKeyPoses6D->points[key_numbers_changed[i]].y << std::endl;
                }
                key_numbers_changed.clear();
            }
            */

            std::cout <<"StartAngleRPY:" << (cloudKeyPoses6D->points[0].roll  / 3.141593 * 180) << " "
                                    << (cloudKeyPoses6D->points[0].pitch / 3.141593 * 180) << " "
                                    << (cloudKeyPoses6D->points[0].yaw   / 3.141593 * 180) << std::endl
                                                                                           << std::endl;
        }

        if(cloudKeyPoses6D->points.size() > 0){
            for(int i = 0; i < cloudKeyPoses6D->points.size(); ++i){
                std::cout << "key_number:" << i << std::endl;
                std::cout << "             GPSPoses:" << cloudKeyPoses6D->points[i].utm_x << " "
                                                      << cloudKeyPoses6D->points[i].utm_y << " "
                                                      << cloudKeyPoses6D->points[i].utm_z << " " << std::endl
                          << "KeyPoses_after_update:" << cloudKeyPoses6D->points[i].z     << " "
                                                      << cloudKeyPoses6D->points[i].x     << " "
                                                      << cloudKeyPoses6D->points[i].y     << " " << std::endl
                                                                                                 << std::endl;
            }
        }

    }
