/* 
This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored, 
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".

Author: Jiarong Lin   < ziv.lin.ljr@gmail.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored, 
    LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package." 
[2] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Lin, Jiarong, et al. "R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual
     tightly-coupled state Estimator and mapping." 
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry 
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for 
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision 
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < ziv.lin.ljr@gmail.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#include "r3live.hpp"

/**
 * @note IMU topic订阅的回调函数
 *       用于向LIO和VIO子系统的IMU缓冲中插入IMU数据
 */ 
void R3LIVE::imu_cbk( const sensor_msgs::Imu::ConstPtr &msg_in )
{
    sensor_msgs::Imu::Ptr msg( new sensor_msgs::Imu( *msg_in ) );
    double                timestamp = msg->header.stamp.toSec(); // 获取IMU的当前时间
    g_camera_lidar_queue.imu_in( timestamp );   // 设置IMU的当前时刻时间
    mtx_buffer.lock();  // imu上锁
    if ( timestamp < last_timestamp_imu )   // last_timestamp_imu初始为-1
    {
        ROS_ERROR( "imu loop back, clear buffer" );
        imu_buffer_lio.clear();
        imu_buffer_vio.clear();
        flg_reset = true;
    }
    //以IMU的当前时间设置上一时刻时间.
    last_timestamp_imu = timestamp;

    // 是否乘以重力 : m_if_acc_mul_G默认为0,即不乘以重力9.8
    if ( g_camera_lidar_queue.m_if_acc_mul_G )
    {
        msg->linear_acceleration.x *= G_m_s2;
        msg->linear_acceleration.y *= G_m_s2;
        msg->linear_acceleration.z *= G_m_s2;
    }

    //向lio和vio中输入imu数据
    imu_buffer_lio.push_back( msg );
    imu_buffer_vio.push_back( msg );
    // std::cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer_lio.size()<<std::endl;

    mtx_buffer.unlock();    // imu解锁
    sig_buffer.notify_all();
}

void printf_field_name( sensor_msgs::PointCloud2::ConstPtr &msg )
{
    cout << "Input pointcloud field names: [" << msg->fields.size() << "]: ";
    for ( size_t i = 0; i < msg->fields.size(); i++ )
    {
        cout << msg->fields[ i ].name << ", ";
    }
    cout << endl;
}

/**
 * @note 从ros消息msg中获取并构造点云数据pcl_pc
 */ 
bool R3LIVE::get_pointcloud_data_from_ros_message( sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud< pcl::PointXYZINormal > &pcl_pc )
{

    // printf("Frame [%d] %.3f ", g_LiDAR_frame_index,  msg->header.stamp.toSec() - g_camera_lidar_queue.m_first_imu_time);
    pcl::PointCloud< pcl::PointXYZI > res_pc;
    scope_color( ANSI_COLOR_YELLOW_BOLD );
    // printf_field_name(msg);
    if ( msg->fields.size() < 3 )
    {
        cout << "Get pointcloud data from ros messages fail!!!" << endl;
        scope_color( ANSI_COLOR_RED_BOLD );
        printf_field_name( msg );
        return false;
    }
    else
    {
        if ( ( msg->fields.size() == 8 ) && ( msg->fields[ 3 ].name == "intensity" ) &&
             ( msg->fields[ 4 ].name == "normal_x" ) ) // Input message type is pcl::PointXYZINormal
        {
            pcl::fromROSMsg( *msg, pcl_pc );
            return true;
        }
        else if ( ( msg->fields.size() == 4 ) && ( msg->fields[ 3 ].name == "rgb" ) )
        {
            double maximum_range = 5;
            get_ros_parameter< double >( m_ros_node_handle, "iros_range", maximum_range, 5 );
            pcl::PointCloud< pcl::PointXYZRGB > pcl_rgb_pc;
            pcl::fromROSMsg( *msg, pcl_rgb_pc );
            double lidar_point_time = msg->header.stamp.toSec();
            int    pt_count = 0;
            pcl_pc.resize( pcl_rgb_pc.points.size() );
            for ( int i = 0; i < pcl_rgb_pc.size(); i++ )
            {
                pcl::PointXYZINormal temp_pt;
                temp_pt.x = pcl_rgb_pc.points[ i ].x;
                temp_pt.y = pcl_rgb_pc.points[ i ].y;
                temp_pt.z = pcl_rgb_pc.points[ i ].z;
                double frame_dis = sqrt( temp_pt.x * temp_pt.x + temp_pt.y * temp_pt.y + temp_pt.z * temp_pt.z );
                if ( frame_dis > maximum_range )
                {
                    continue;
                }
                temp_pt.intensity = ( pcl_rgb_pc.points[ i ].r + pcl_rgb_pc.points[ i ].g + pcl_rgb_pc.points[ i ].b ) / 3.0;
                temp_pt.curvature = 0;
                pcl_pc.points[ pt_count ] = temp_pt;
                pt_count++;
            }
            pcl_pc.points.resize( pt_count );
            return true;
        }
        else // TODO, can add by yourself
        {
            cout << "Get pointcloud data from ros messages fail!!! ";
            scope_color( ANSI_COLOR_RED_BOLD );
            printf_field_name( msg );
            return false;
        }
    }
}

/**
 * @note 从lidar_buffer和imu_buffer_lio中分别取出雷达和imu数据,存放至meas中
 * @param MeasureGroup &meas : 存放雷达数据(点云,起始/结束时间)和IMU数据
 */ 
bool R3LIVE::sync_packages( MeasureGroup &meas )
{
    //(1)判断lidar和imu的数据buffer是否为空
    if ( lidar_buffer.empty() || imu_buffer_lio.empty() )
    {
        return false;
    }

    /*** push lidar frame 取出lidar数据***/
    if ( !lidar_pushed )    // lidar_pushed初始false
    {
        meas.lidar.reset( new PointCloudXYZINormal() );
        // 从lidar_buffer中获取点云数据(出队列: 取队列队首数据)
        if ( get_pointcloud_data_from_ros_message( lidar_buffer.front(), *( meas.lidar ) ) == false )
        {
            return false;
        }
        // pcl::fromROSMsg(*(lidar_buffer.front()), *(meas.lidar));
        meas.lidar_beg_time = lidar_buffer.front()->header.stamp.toSec();   // 当前lidar帧的开始时间
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double( 1000 );
        meas.lidar_end_time = lidar_end_time;   // 当前lidar帧的结束时间
        // printf("Input LiDAR time = %.3f, %.3f\n", meas.lidar_beg_time, meas.lidar_end_time);
        // printf_line_mem_MB;
        lidar_pushed = true;
    }

    if ( last_timestamp_imu < lidar_end_time )
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer 取出imu数据***/
    double imu_time = imu_buffer_lio.front()->header.stamp.toSec();
    meas.imu.clear();
    while ( ( !imu_buffer_lio.empty() ) && ( imu_time < lidar_end_time ) )
    {
        imu_time = imu_buffer_lio.front()->header.stamp.toSec();
        if ( imu_time > lidar_end_time + 0.02 )
            break;
        meas.imu.push_back( imu_buffer_lio.front() );
        imu_buffer_lio.pop_front();
    }

    lidar_buffer.pop_front();
    lidar_pushed = false;
    // if (meas.imu.empty()) return false;
    // std::cout<<"[IMU Sycned]: "<<imu_time<<" "<<lidar_end_time<<std::endl;
    return true;
}

// project lidar frame to world - 将参数1-pi转换至世界坐标系下,并赋值给参数2-po
void R3LIVE::pointBodyToWorld( PointType const *const pi, PointType *const po )
{
    Eigen::Vector3d p_body( pi->x, pi->y, pi->z );
    Eigen::Vector3d p_global( g_lio_state.rot_end * ( p_body + Lidar_offset_to_IMU ) + g_lio_state.pos_end );

    po->x = p_global( 0 );
    po->y = p_global( 1 );
    po->z = p_global( 2 );
    po->intensity = pi->intensity;
}

void R3LIVE::RGBpointBodyToWorld( PointType const *const pi, pcl::PointXYZI *const po )
{
    Eigen::Vector3d p_body( pi->x, pi->y, pi->z );
    Eigen::Vector3d p_global( g_lio_state.rot_end * ( p_body + Lidar_offset_to_IMU ) + g_lio_state.pos_end );

    po->x = p_global( 0 );
    po->y = p_global( 1 );
    po->z = p_global( 2 );
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor( intensity );

    int reflection_map = intensity * 10000;
}

/**
 * @note 获取立方体中第k个面的第j行第i列的下标
 */ 
int R3LIVE::get_cube_index( const int &i, const int &j, const int &k )
{
    return ( i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k );
}

bool R3LIVE::center_in_FOV( Eigen::Vector3f cube_p )
{
    Eigen::Vector3f dis_vec = g_lio_state.pos_end.cast< float >() - cube_p;
    float           squaredSide1 = dis_vec.transpose() * dis_vec;

    if ( squaredSide1 < 0.4 * cube_len * cube_len )
        return true;

    dis_vec = XAxisPoint_world.cast< float >() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;

    float ang_cos =
        fabs( squaredSide1 <= 3 ) ? 1.0 : ( LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2 ) / ( 2 * LIDAR_SP_LEN * sqrt( squaredSide1 ) );

    return ( ( ang_cos > HALF_FOV_COS ) ? true : false );
}

bool R3LIVE::if_corner_in_FOV( Eigen::Vector3f cube_p )
{
    Eigen::Vector3f dis_vec = g_lio_state.pos_end.cast< float >() - cube_p;
    float           squaredSide1 = dis_vec.transpose() * dis_vec;
    dis_vec = XAxisPoint_world.cast< float >() - cube_p;
    float squaredSide2 = dis_vec.transpose() * dis_vec;
    float ang_cos =
        fabs( squaredSide1 <= 3 ) ? 1.0 : ( LIDAR_SP_LEN * LIDAR_SP_LEN + squaredSide1 - squaredSide2 ) / ( 2 * LIDAR_SP_LEN * sqrt( squaredSide1 ) );
    return ( ( ang_cos > HALF_FOV_COS ) ? true : false );
}

/**
 * @note   ? : 以lidar帧最后一个状态位置构建立方体
 *          应该和FAST-LIO2中第5节-地图管理相关 : 为了防止地图大小不受约束,在ikd树上保留当前lidar位置周围长度为L的局部区域的地图点.
 *         立方体的结果放置在featsArray[48*48*48]中,表明立方体长宽高为48,一共有48^3个体素
 */ 
void R3LIVE::lasermap_fov_segment()
{
    laserCloudValidNum = 0;
    pointBodyToWorld( XAxisPoint_body, XAxisPoint_world );// 将lidar坐标系变换至世界坐标系
    // ? 计算以当前lidar点为中心的立方体长宽高
    // cube_len从参数服务器上获取值,默认为10000000, laserCloudCenWidth=24
    int centerCubeI = int( ( g_lio_state.pos_end( 0 ) + 0.5 * cube_len ) / cube_len ) + laserCloudCenWidth;
    int centerCubeJ = int( ( g_lio_state.pos_end( 1 ) + 0.5 * cube_len ) / cube_len ) + laserCloudCenHeight;
    int centerCubeK = int( ( g_lio_state.pos_end( 2 ) + 0.5 * cube_len ) / cube_len ) + laserCloudCenDepth;
    if ( g_lio_state.pos_end( 0 ) + 0.5 * cube_len < 0 )
        centerCubeI--;
    if ( g_lio_state.pos_end( 1 ) + 0.5 * cube_len < 0 )
        centerCubeJ--;
    if ( g_lio_state.pos_end( 2 ) + 0.5 * cube_len < 0 )
        centerCubeK--;
    bool last_inFOV_flag = 0;
    int  cube_index = 0;
    cub_needrm.clear();
    cub_needad.clear();
    T2[ time_log_counter ] = Measures.lidar_beg_time;
    double t_begin = omp_get_wtime();

    // 计算立方体内每个体素的点信息
    while ( centerCubeI < FOV_RANGE + 1 )
    {
        for ( int j = 0; j < laserCloudHeight; j++ ) // laserCloudHeight=48
        {
            for ( int k = 0; k < laserCloudDepth; k++ ) // laserCloudDepth=48
            {   
                // 下面用到的get_cube_index(i, j, k) : 获取立方体中第k个面的第j行第i列的下标

                int i = laserCloudWidth - 1;    // laserCloudWidth=48

                // 单独获取(47,0,0)处的cube信息 (featsArray[48*48*48])
                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i - 1, j, k ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i - 1, j, k ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }
        centerCubeI++;
        laserCloudCenWidth++;
    }

    // 没用到的while循环
    while ( centerCubeI >= laserCloudWidth - ( FOV_RANGE + 1 ) )
    {
        for ( int j = 0; j < laserCloudHeight; j++ )
        {
            for ( int k = 0; k < laserCloudDepth; k++ )
            {
                int i = 0;

                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )   // 因为上面定义i=0,因此不进入循环
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i + 1, j, k ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i + 1, j, k ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeI--;
        laserCloudCenWidth--;
    }

    while ( centerCubeJ < ( FOV_RANGE + 1 ) )
    {
        for ( int i = 0; i < laserCloudWidth; i++ )
        {
            for ( int k = 0; k < laserCloudDepth; k++ )
            {
                int j = laserCloudHeight - 1; // laserCloudHeight=48

                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i, j - 1, k ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i, j - 1, k ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ++;
        laserCloudCenHeight++;
    }

    while ( centerCubeJ >= laserCloudHeight - ( FOV_RANGE + 1 ) )
    {
        for ( int i = 0; i < laserCloudWidth; i++ )
        {
            for ( int k = 0; k < laserCloudDepth; k++ )
            {
                int                       j = 0;
                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i, j + 1, k ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i, j + 1, k ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeJ--;
        laserCloudCenHeight--;
    }

    while ( centerCubeK < ( FOV_RANGE + 1 ) )
    {
        for ( int i = 0; i < laserCloudWidth; i++ )
        {
            for ( int j = 0; j < laserCloudHeight; j++ )
            {
                int                       k = laserCloudDepth - 1;
                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i, j, k - 1 ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i, j, k - 1 ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }

        centerCubeK++;
        laserCloudCenDepth++;
    }

    while ( centerCubeK >= laserCloudDepth - ( FOV_RANGE + 1 ) )
    {
        for ( int i = 0; i < laserCloudWidth; i++ )
        {
            for ( int j = 0; j < laserCloudHeight; j++ )
            {
                int                       k = 0;
                PointCloudXYZINormal::Ptr laserCloudCubeSurfPointer = featsArray[ get_cube_index( i, j, k ) ];
                last_inFOV_flag = _last_inFOV[ cube_index ];

                for ( ; i >= 1; i-- )
                {
                    featsArray[ get_cube_index( i, j, k ) ] = featsArray[ get_cube_index( i, j, k + 1 ) ];
                    _last_inFOV[ get_cube_index( i, j, k ) ] = _last_inFOV[ get_cube_index( i, j, k + 1 ) ];
                }

                featsArray[ get_cube_index( i, j, k ) ] = laserCloudCubeSurfPointer;
                _last_inFOV[ get_cube_index( i, j, k ) ] = last_inFOV_flag;
                laserCloudCubeSurfPointer->clear();
            }
        }
        centerCubeK--;
        laserCloudCenDepth--;
    }

    cube_points_add->clear();
    featsFromMap->clear();
    memset( now_inFOV, 0, sizeof( now_inFOV ) );
    copy_time = omp_get_wtime() - t_begin;
    double fov_check_begin = omp_get_wtime();

    fov_check_time = omp_get_wtime() - fov_check_begin;

    double readd_begin = omp_get_wtime();
#ifdef USE_ikdtree
    if ( cub_needrm.size() > 0 )
        ikdtree.Delete_Point_Boxes( cub_needrm );
    delete_box_time = omp_get_wtime() - readd_begin;
    // s_plot4.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
    if ( cub_needad.size() > 0 )
        ikdtree.Add_Point_Boxes( cub_needad );
    readd_box_time = omp_get_wtime() - readd_begin - delete_box_time;
    // s_plot5.push_back(omp_get_wtime() - t_begin); t_begin = omp_get_wtime();
    if ( cube_points_add->points.size() > 0 )
        ikdtree.Add_Points( cube_points_add->points, true );
#endif
    readd_time = omp_get_wtime() - readd_begin - delete_box_time - readd_box_time;
    // s_plot6.push_back(omp_get_wtime() - t_begin);
}

/**
 * @note Lidar订阅Lidar Topic(/laser_cloud_flat)的回调函数
 *       主要用于向Lidar_buffer中加入接收到的雷达数据
 */ 
void R3LIVE::feat_points_cbk( const sensor_msgs::PointCloud2::ConstPtr &msg_in )
{
    // 由接收到的点云数据定义新的点云变量
    sensor_msgs::PointCloud2::Ptr msg( new sensor_msgs::PointCloud2( *msg_in ) );
    // 获取当前时刻lidar数据的时间: (m_lidar_imu_time_delay:雷达imu的延迟时间,默认=0)
    msg->header.stamp = ros::Time( msg_in->header.stamp.toSec() - m_lidar_imu_time_delay );
    if ( g_camera_lidar_queue.lidar_in( msg_in->header.stamp.toSec() + 0.1 ) == 0 )
    {
        return;
    }
    mtx_buffer.lock();  // imu上锁
    // std::cout<<"got feature"<<std::endl;
    if ( msg->header.stamp.toSec() < last_timestamp_lidar )
    {   // last_timestamp_lidar初始=-1
        ROS_ERROR( "lidar loop back, clear buffer" );
        lidar_buffer.clear();
    }
    // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
    lidar_buffer.push_back( msg );
    last_timestamp_lidar = msg->header.stamp.toSec();
    mtx_buffer.unlock();        // imu解锁
    sig_buffer.notify_all();    // 唤醒LIO处理线程,进行Lidar数据处理
}

void R3LIVE::wait_render_thread_finish()
{
    if ( m_render_thread != nullptr )
    {
        m_render_thread->get(); // wait render thread to finish.
        // m_render_thread = nullptr;
    }
}

/**
 * @note LIO子系统处理线程
 */ 
int R3LIVE::service_LIO_update()
{
    /**
     * ===================================================================================================
     * @note (1) 定义用到的变量 : 发布的lidar路径,协方差矩阵,kalman增益矩阵和4个重要的点云容器,IMU前向/后向传播处理器等.
     * ===================================================================================================
     */
    nav_msgs::Path path;    // Lidar的路径 : 主要有两个成员变量: header和pose
    path.header.stamp = ros::Time::now();   // header的时间
    path.header.frame_id = "/world";        // header的id
    /*** variables definition ***/
    // DIM_OF_STATES=29. G:, H_T_H, I_STATE:
    // G : 用于传递协方差用的矩阵, H_T_H : 用于计算kalman增益用的, I_STATE : 单位阵
    Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    cv::Mat matA1( 3, 3, CV_32F, cv::Scalar::all( 0 ) );    // 后面没用到
    cv::Mat matD1( 1, 3, CV_32F, cv::Scalar::all( 0 ) );    // 后面没用到
    cv::Mat matV1( 3, 3, CV_32F, cv::Scalar::all( 0 ) );    // 后面没用到
    cv::Mat matP( 6, 6, CV_32F, cv::Scalar::all( 0 ) );    // 后面没用到

    PointCloudXYZINormal::Ptr feats_undistort( new PointCloudXYZINormal() );//去除畸变后的点云
    PointCloudXYZINormal::Ptr feats_down( new PointCloudXYZINormal() );     // 保存下采样后的点云
    // 有M个特征点找到了M个对应的最近平面Si,则coeffSel存储Si的平面方程系数,和点到平面的残差
    PointCloudXYZINormal::Ptr coeffSel( new PointCloudXYZINormal() );// 存放M个最近平面信息的容器: 平面方程,点-面残差
    PointCloudXYZINormal::Ptr laserCloudOri( new PointCloudXYZINormal() );// 存放找到了最近平面的M个点的容器

    /*** variables initialize ***/
    FOV_DEG = fov_deg + 10; // fov_deg=360
    HALF_FOV_COS = std::cos( ( fov_deg + 10.0 ) * 0.5 * PI_M / 180.0 );// cos(185)

    for ( int i = 0; i < laserCloudNum; i++ )   // laserCloudNum = 48x48x48
    {
        // featsArray[48x48x48]
        featsArray[ i ].reset( new PointCloudXYZINormal() );
    }

    std::shared_ptr< ImuProcess > p_imu( new ImuProcess() );    // 定义用于前向/后向传播的IMU处理器
    m_imu_process = p_imu;
    //------------------------------------------------------------------------------------------------------
    ros::Rate rate( 5000 );
    bool      status = ros::ok();
    g_camera_lidar_queue.m_liar_frame_buf = &lidar_buffer;//取出lidar数据
    set_initial_state_cov( g_lio_state );   //初始化g_lio_state的状态协方差矩阵

    /**
     * ===================================================================================================
     * @note (2) LIO线程主循环
     * ===================================================================================================
     */
    while ( ros::ok() ) //运行LIO线程循环
    {
        /**
         * @note (2-1) : 判断退出标识, 判断是否可以处理lidar数据, 锁住当前线程(等待唤醒)
         */ 
        if ( flg_exit ) // 判断退出标识
            break;
        ros::spinOnce();
        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
        while ( g_camera_lidar_queue.if_lidar_can_process() == false )
        {   // 考虑camera和lidar在频率上的时间对齐关系
            // 判断当前时间是否可以处理lidar数据, 不可以则sleep一会
            ros::spinOnce();
            std::this_thread::yield();
            std::this_thread::sleep_for( std::chrono::milliseconds( THREAD_SLEEP_TIM ) );
        }
        // https://www.jianshu.com/p/34d219380d90
        // 结合条件变量创建互斥锁的局部变量(析构时自动解锁) : 进行LIO具体处理时锁住当前线程
        std::unique_lock< std::mutex > lock( m_mutex_lio_process ); 
        if ( 1 )
        {
            // printf_line;
            Common_tools::Timer tim;

            /**
             * @note (2-2) 从lidar_buffer和imu_buffer_lio中分别获取雷达和imu数据,存放至Measures中
             *              MeasureGroup Measures; // 存放雷达数据和IMU数据的结构体全局变量
             */             
            if ( sync_packages( Measures ) == 0 )
            {
                continue;   // 提取数据失败
            }
            int lidar_can_update = 1;   // 提取数据成功后,进行lidar的更新
            // 设置当前lidar帧处理的起始时间 : g_lidar_star_tim和frame_first_pt_time初始为0
            g_lidar_star_tim = frame_first_pt_time;
            if ( flg_reset )    // 判断重置标识
            {
                ROS_WARN( "reset when rosbag play back" );
                p_imu->Reset(); // 重置前向/后向传播用的处理器 : 重置处理时用到的期望和方差等变量
                flg_reset = false;
                continue;
            }
            g_LiDAR_frame_index++;      // lidar帧++
            tim.tic( "Preprocess" );    // time_current : 获取当前时间
            double t0, t1, t2, t3, t4, t5, match_start, match_time, solve_start, solve_time, pca_time, svd_time;
            // 重置处理时用于记录各块处理时间的变量
            match_time = 0;
            kdtree_search_time = 0;
            solve_time = 0;
            pca_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();
            /**
             * @note (2-3) 通过测量数据Measures(IMU数据和Lidar数据)计算LIO的状态和去畸变后的Lidar点云数据
             *             前向传播 + 后向传播
             *      LIO状态结果保存在g_lio_state,去运动畸变后的点保存在feats_undistort中
             */
            p_imu->Process( Measures, g_lio_state, feats_undistort );

            g_camera_lidar_queue.g_noise_cov_acc = p_imu->cov_acc;  // 获取加速度误差状态传递的协方差
            g_camera_lidar_queue.g_noise_cov_gyro = p_imu->cov_gyr; // 获取角速度误差状态传递的协方差
            StatesGroup state_propagate( g_lio_state ); // 状态传播值(先验):通过计算得到的状态实例化一个StatesGroup变量

            // 输出lio上一帧更新的时间 : 上一帧更新记录时间 - lidar开始时间
            // cout << "G_lio_state.last_update_time =  " << std::setprecision(10) << g_lio_state.last_update_time -g_lidar_star_tim  << endl;
            if ( feats_undistort->empty() || ( feats_undistort == NULL ) )  // 没有成功去除点云运动畸变
            {
                // 重置当前处理lidar帧的起始时间 : 比如当前处理t1-t2间的lidar但失败了
                // 因此后面处理时间为t1-t3,所以需要把t1保存进frame_first_pt_time
                frame_first_pt_time = Measures.lidar_beg_time;
                std::cout << "not ready for odometry" << std::endl;
                continue;
            }

            // 当前帧中lidar的开始时间,必须<=记录的帧开始时间
            // 按理应该相等,这里的判断估计是实际使用中发生的问题(从理解的角度-不一定正确:如果lidar中损失了初始点,那么
            // 当前最开始的lidar点的时间就<记录下来的帧开始时间)
            if ( ( Measures.lidar_beg_time - frame_first_pt_time ) < INIT_TIME ) // INIT_TIME=0
            {
                flg_EKF_inited = false;
                std::cout << "||||||||||Initiallizing LiDAR||||||||||" << std::endl;
            }
            else    // 时间满足关系,开始EKF过程
            {
                flg_EKF_inited = true;
            }
            /*** Compute the euler angle 这里的euler_cur就是当前的lidar里程计的旋转信息,后面需要用kalman迭代更新,最后发布到ros中***/
            Eigen::Vector3d euler_cur = RotMtoEuler( g_lio_state.rot_end );// 最后时刻时lidar的旋转向量 : 四元数
#ifdef DEBUG_PRINT  // 默认注释了DEBUG_PRINT的定义
            std::cout << "current lidar time " << Measures.lidar_beg_time << " "
                      << "first lidar time " << frame_first_pt_time << std::endl;
            // 打印预积分后的结果(最后时刻IMU的状态) : 旋转向量(1rad=57.3度), 位置向量, 速度向量, 角速度bias向量, 加速度bias量
            std::cout << "pre-integrated states: " << euler_cur.transpose() * 57.3 << " " << g_lio_state.pos_end.transpose() << " "
                      << g_lio_state.vel_end.transpose() << " " << g_lio_state.bias_g.transpose() << " " << g_lio_state.bias_a.transpose()
                      << std::endl;
#endif
            /**
             * @note (2-4) 以lidar帧最后一个状态位置构建立方体
             *          立方体的结果放置在featsArray[48*48*48]中,表明立方体长宽高为48,一共有48^3个体素
             */ 
            lasermap_fov_segment();//为防地图大小不受约束,ikd树保留lidar位置周围长度为L的局部区域地图点.

            /**
             * @note (2-5)根据给定去畸变后的点云构造三维体素栅格,对体素栅格进行下采样达到滤波的目的
             *      (一个体素用重心点来近似,减少点的数量,保持点云形状),结果保存在feats_down中
             */ 
            downSizeFilterSurf.setInputCloud( feats_undistort );//构建三维体素栅格 
            downSizeFilterSurf.filter( *feats_down );           //下采样滤波
            // cout <<"Preprocess cost time: " << tim.toc("Preprocess") << endl;
            
            /**
             * @note (2-6)initialize the map kdtree 使用下采样后得到的特征点云构造ikd树**
             * @note
             *      判断条件 : if(下采样后特征点数量大于1) && (ikd树根节点为空) : 
             * ***重点*** 判断条件表明这里只进来一次,且第一帧lidar数据用来构造了ikd树后,就进入下一次循环了
             *           所以可知,第一帧Lidar数据直接用来构造ikd树,第>=2帧Lidar数据不再直接添加到树上,
             *    而是找与树上最近点,由最近点构成平面,然后计算点到平面的残差,并运用EKF迭代更新后,才加到树上
             */
            if ( ( feats_down->points.size() > 1 ) && ( ikdtree.Root_Node == nullptr ) )
            {
                // std::vector<PointType> points_init = feats_down->points;
                ikdtree.set_downsample_param( filter_size_map_min ); // filter_size_map_min默认=0.4
                ikdtree.Build( feats_down->points );    // 构造idk树
                flg_map_initialized = true;
                continue;   // 进入下一次循环
            }

            if ( ikdtree.Root_Node == nullptr ) // 构造ikd树失败
            {
                flg_map_initialized = false;
                std::cout << "~~~~~~~ Initialize Map iKD-Tree Failed! ~~~~~~~" << std::endl;
                continue;
            }
            int featsFromMapNum = ikdtree.size();   // ikd树的节点数
            int feats_down_size = feats_down->points.size();    // 下采样过滤后的点数
            
            // ***重要*** : 下面的所有操作都是>=第2帧的情况下才进行的

            /**
             * @note (2-7) ICP and iterated Kalman filter update : ICP迭代 + Kalman更新
             *      在后面操作中需要对原始点云feats_down做很多操作,操作后都需要有一个容器来存储结果
             * @param coeffSel_tmpt : 临时容器 : M个特征点找到了M个对应的最近平面Si,则coeffSel_tmpt
             *                                 存储Si的平面方程系数,和点到平面的残差
             * @param feats_down_updated : 保存更新后并转到世界坐标系下的点,最后这些点放入ikd树
             * @param res_last 存放每个特征点的残差值
             * */
            PointCloudXYZINormal::Ptr coeffSel_tmpt( new PointCloudXYZINormal( *feats_down ) );
            PointCloudXYZINormal::Ptr feats_down_updated( new PointCloudXYZINormal( *feats_down ) );
            std::vector< double >     res_last( feats_down_size, 1000.0 ); // initial : 存放每个特征点的残差值
            if ( featsFromMapNum >= 5 ) // ***重点*** : 正式开始ICP和迭代Kalman : ikd树上至少有5个点才进行操作
            {
                t1 = omp_get_wtime();

                /**
                 * @note (2-7-1) : 在ros上发布特征点云数据 - 默认不发布
                 */ 
                if ( m_if_publish_feature_map ) 
                {
                    PointVector().swap( ikdtree.PCL_Storage );
                    // flatten会将需要删除的点放入Points_deleted或Multithread_Points_deleted中
                    ikdtree.flatten( ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD );
                    featsFromMap->clear();
                    featsFromMap->points = ikdtree.PCL_Storage;

                    sensor_msgs::PointCloud2 laserCloudMap;
                    pcl::toROSMsg( *featsFromMap, laserCloudMap );  // 将点云数据格式转换为发布的消息格式
                    laserCloudMap.header.stamp = ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
                    // laserCloudMap.header.stamp.fromSec(Measures.lidar_end_time); // ros::Time().fromSec(last_timestamp_lidar);
                    laserCloudMap.header.frame_id = "world";
                    pubLaserCloudMap.publish( laserCloudMap );
                }

                /**
                 * @note (2-7-2) : 定义后面点-面计算时需要用到的变量
                 * 变量的理解举例: E为所有特征点,P点属于E,现在为P点找平面S:
                 *      当成功为P点找到平面S,则point_selected_surf[P] = true, 否则=false
                 *      平面S由m个点构成:pointSearchInd_surf[0]到pointSearchInd_surf[m]
                 *      E中离P点按距离排序后的点放在Nearest_Points[]中:
                 *        Nearest_Points是二维数组(PointVector是一维),存储每个点的最近点集合
                 */                 
                std::vector< bool >               point_selected_surf( feats_down_size, true ); // 记录有那些点成功找到了平面
                std::vector< std::vector< int > > pointSearchInd_surf( feats_down_size );       // 构成平面的点的index
                std::vector< PointVector >        Nearest_Points( feats_down_size );    // 二维数组,存点i的最近点排序后的集合

                int  rematch_num = 0;
                bool rematch_en = 0;
                flg_EKF_converged = 0;
                deltaR = 0.0;
                deltaT = 0.0;
                t2 = omp_get_wtime();
                double maximum_pt_range = 0.0;
                // cout <<"Preprocess 2 cost time: " << tim.toc("Preprocess") << endl;

                /**
                 * @note (2-7-3) 进行误差Kalman迭代
                 *      //TODO 
                 */ 
                for ( iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++ ) // NUM_MAX_ITERATIONS默认为4
                {
                    tim.tic( "Iter" );  // 本次迭代起始时间
                    match_start = omp_get_wtime();
                    laserCloudOri->clear(); // 清空存放找到了最近平面的点的容器
                    coeffSel->clear();      // 清空存放最近平面信息的容器

                    /**
                     * @note (2-7-3-1) : closest surface search and residual computation *
                     *      遍历所有(下采样后)特征点,搜索每个点在树上的最近点集(5个点),
                     *      由最近点集通过PCA方法拟合最近平面,再计算点-面残差
                     */
                    for ( int i = 0; i < feats_down_size; i += m_lio_update_point_step ) // m_lio_update_point_step默认为1
                    {
                        double     search_start = omp_get_wtime();
                        PointType &pointOri_tmpt = feats_down->points[ i ]; // 获取当前下标的特征点 - 原始点
                        // 计算特征点与原点的距离
                        double     ori_pt_dis =
                            sqrt( pointOri_tmpt.x * pointOri_tmpt.x + pointOri_tmpt.y * pointOri_tmpt.y + pointOri_tmpt.z * pointOri_tmpt.z );
                        maximum_pt_range = std::max( ori_pt_dis, maximum_pt_range );// 保存离原点最远的点产生的最远距离
                        PointType &pointSel_tmpt = feats_down_updated->points[ i ]; // 获取当前下标的特征点 - 更新后的点

                        /* transform to world frame */
                        pointBodyToWorld( &pointOri_tmpt, &pointSel_tmpt );// 将特征点转到世界坐标下,并保存至可变点pointSel_tmpt中
                        std::vector< float > pointSearchSqDis_surf; // 搜索点-平面时产生的距离序列

                        auto &points_near = Nearest_Points[ i ];    // 下标i的特征点的最近点集合(一维数组)

                        /**
                         * @note (2-7-3-1-1) : 搜索与点最近平面上的5个点
                         * ***注意*** 只有第一次迭代时才进入,后面都用迭代的方法,就不找平面了
                         *  (猜测应该是寻找平面耗时,后面直接用第一次找到的平面迭代优化就好了)
                         */
                        if ( iterCount == 0 || rematch_en ) // 第一次迭代或者重匹配使能时才在ikd树上搜索最近平面
                        {
                            point_selected_surf[ i ] = true;
                            /** Find the closest surfaces in the map 在地图中找到最近的平面**/
                            // NUM_MATCH_POINTS=5
                            ikdtree.Nearest_Search( pointSel_tmpt, NUM_MATCH_POINTS, points_near, pointSearchSqDis_surf );
                            float max_distance = pointSearchSqDis_surf[ NUM_MATCH_POINTS - 1 ];//最近点集的最后一个元素自然最远
                            //  max_distance to add residuals
                            // ANCHOR - Long range pt stragetry
                            if ( max_distance > m_maximum_pt_kdtree_dis )   // 超出限定距离,放弃为这个点寻找平面
                            {
                                point_selected_surf[ i ] = false;   // 当前点寻找平面失败
                            }
                        }

                        kdtree_search_time += omp_get_wtime() - search_start;
                        if ( point_selected_surf[ i ] == false )    // 当前点寻找平面失败,进入下一个点的寻找流程
                            continue;

                        // match_time += omp_get_wtime() - match_start;
                        double pca_start = omp_get_wtime();
                        /**
                         * @note (2-7-3-2) : 使用PCA方法拟合平面 (using minimum square method)
                         * ***注意*** : 具体PCA解释待补充
                         */
                        cv::Mat matA0( NUM_MATCH_POINTS, 3, CV_32F, cv::Scalar::all( 0 ) );
                        cv::Mat matB0( NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all( -1 ) );
                        cv::Mat matX0( NUM_MATCH_POINTS, 1, CV_32F, cv::Scalar::all( 0 ) );

                        for ( int j = 0; j < NUM_MATCH_POINTS; j++ )
                        {
                            matA0.at< float >( j, 0 ) = points_near[ j ].x;
                            matA0.at< float >( j, 1 ) = points_near[ j ].y;
                            matA0.at< float >( j, 2 ) = points_near[ j ].z;
                        }

                        cv::solve( matA0, matB0, matX0, cv::DECOMP_QR ); // TODO

                        float pa = matX0.at< float >( 0, 0 );
                        float pb = matX0.at< float >( 1, 0 );
                        float pc = matX0.at< float >( 2, 0 );
                        float pd = 1;

                        float ps = sqrt( pa * pa + pb * pb + pc * pc );
                        pa /= ps;
                        pb /= ps;
                        pc /= ps;
                        pd /= ps;

                        /**
                         * @note (2-7-3-1-3) : 检测计算平面的有效性 - 将最近点集合带入平面方程中求残差做判断
                         */
                        bool planeValid = true;
                        for ( int j = 0; j < NUM_MATCH_POINTS; j++ )    // NUM_MATCH_POINTS=5
                        {
                            // ANCHOR -  Planar check : 将特征点在平面上的最近5个点带入平面公式中,得到的结果>0.05 (表明当前点不在拟合平面上)
                            if ( fabs( pa * points_near[ j ].x + pb * points_near[ j ].y + pc * points_near[ j ].z + pd ) >
                                 m_planar_check_dis ) // Raw 0.05
                            {
                                // ANCHOR - Far distance pt processing
                                // ori_pt_dis:当前点到原点的距离, maximum_pt_range:特征点云中的最远点离原点的距离
                                // m_long_rang_pt_dis:lidar点最远有效距离(默认500)
                                if ( ori_pt_dis < maximum_pt_range * 0.90 || ( ori_pt_dis < m_long_rang_pt_dis ) )
                                // if(1)
                                {
                                    planeValid = false;
                                    point_selected_surf[ i ] = false;   // 当前特征点寻找平面失败
                                    break;
                                }
                            }
                        }

                        /**
                         * @note (2-7-3-1-4) : 前面计算平面有效时,计算残差res_last[] : 残差定义为点到平面的距离
                         *                     对应FAST-LIO公式(12)
                         */ 
                        if ( planeValid )
                        {
                            // 将特征点(世界坐标系下) 代入平面方程,计算残差 (在平面的点代入=0,不在平面的点代入会产生残差)
                            // 按照点到平面的距离公式,这里省略了除以分母的内容 : 对应FAST-LIO公式(12)
                            float pd2 = pa * pointSel_tmpt.x + pb * pointSel_tmpt.y + pc * pointSel_tmpt.z + pd;
                            // s计算了后面没用 : s考虑了除以分母的步骤.
                            float s = 1 - 0.9 * fabs( pd2 ) /
                                              sqrt( sqrt( pointSel_tmpt.x * pointSel_tmpt.x + pointSel_tmpt.y * pointSel_tmpt.y +
                                                          pointSel_tmpt.z * pointSel_tmpt.z ) );
                            // ANCHOR -  Point to plane distance
                                                    // 点到原点的距离<500 ? 0.3 : 1.0
                            double acc_distance = ( ori_pt_dis < m_long_rang_pt_dis ) ? m_maximum_res_dis : 1.0;
                            if ( pd2 < acc_distance )   // 残差小于0.3或者1.0
                            {
                                {// if(std::abs(pd2) > 5 * res_mean_last)
                                // {
                                //     point_selected_surf[i] = false;
                                //     res_last[i] = 0.0;
                                //     continue;
                                // }
                                }
                                point_selected_surf[ i ] = true;    // 当前点寻找平面成功
                                coeffSel_tmpt->points[ i ].x = pa;  // 记录当前特征点对应最近平面的平面方程
                                coeffSel_tmpt->points[ i ].y = pb;
                                coeffSel_tmpt->points[ i ].z = pc;
                                coeffSel_tmpt->points[ i ].intensity = pd2;
                                res_last[ i ] = std::abs( pd2 );    // 当前特征点代入平面方程产生的残差
                            }
                            else
                            {
                                point_selected_surf[ i ] = false;// 当前点寻找平面失败
                            }
                        }
                        pca_time += omp_get_wtime() - pca_start;
                    }  // 遍历所有特征点寻找平面,计算点-面残差完成

                    tim.tic( "Stack" );
                    double total_residual = 0.0;
                    laserCloudSelNum = 0;

                    /**
                     * @note (2-7-3-2) : 从找到平面的点中筛选满足条件的点,用新的变量保存该点和对应平面(下标对齐)
                     *      后面进行Kalman等计算时,都使用这里得到的"laserCloudOri"和"coeffSel"
                     */ 
                    for ( int i = 0; i < coeffSel_tmpt->points.size(); i++ )    // 遍历找到了对应平面的特征点
                    {
                        // 平面有效 && 点到面的残差小于2    (这里的残差条件按理都满足,因为前面存入的残差值小于1.0)
                        if ( point_selected_surf[ i ] && ( res_last[ i ] <= 2.0 ) )
                        {                                                           // 下面重新放入容器是为了对齐点-面的索引
                            laserCloudOri->push_back( feats_down->points[ i ] );    // 将找到最近平面的点放入laserCloudOri中
                            coeffSel->push_back( coeffSel_tmpt->points[ i ] );      // 将最近平面容器放入coeffSel中
                            total_residual += res_last[ i ];        // 总残差 - 从最小二乘的角度,优化的就是让这个总残差最小
                            laserCloudSelNum++;                     // 找到平面的点的数量
                        }
                    }
                    res_mean_last = total_residual / laserCloudSelNum;  // 均值-期望, 后面没用此变量

                    match_time += omp_get_wtime() - match_start;
                    solve_start = omp_get_wtime();

                    /**
                     * @note (2-7-3-3) Computation of Measuremnt Jacobian matrix H and measurents vector
                     *               计算测量Jacobian矩阵和测量向量. 猜测对应FAST-LIO的公式(14)(15)(16)(17)
                     */  
                    Eigen::MatrixXd Hsub( laserCloudSelNum, 6 );    // Hsub(n x 6)
                    Eigen::VectorXd meas_vec( laserCloudSelNum );   // meas_vec(n x 1)
                    Hsub.setZero();
                    for ( int i = 0; i < laserCloudSelNum; i++ )
                    {
                        const PointType &laser_p = laserCloudOri->points[ i ];// 获取当前点
                        Eigen::Vector3d  point_this( laser_p.x, laser_p.y, laser_p.z );// 点坐标
                        point_this += Lidar_offset_to_IMU;  // Lidar和IMU的偏移
                        Eigen::Matrix3d point_crossmat;
                        point_crossmat << SKEW_SYM_MATRIX( point_this );    // 将点转为反对称矩阵用于叉乘

                        /*** get the normal vector of closest surface/corner 获取最近平面的法向量***/
                        const PointType &norm_p = coeffSel->points[ i ];    // 当前点的最近平面方程系数
                        Eigen::Vector3d  norm_vec( norm_p.x, norm_p.y, norm_p.z );// 平面法向量

                        /*** calculate the Measuremnt Jacobian matrix H ***/
                        // A = 当前点的反对称矩阵 * (lidar帧最后时刻时的旋转)转置 * 最近平面法向量
                        // ?TODO : 这里可能又是一个近似,猜测是因为直接计算H矩阵太耗时
                        Eigen::Vector3d A( point_crossmat * g_lio_state.rot_end.transpose() * norm_vec );
                        Hsub.row( i ) << VEC_FROM_ARRAY( A ), norm_p.x, norm_p.y, norm_p.z;// row(i)=A[0],A[1],A[2],norm_p.x, norm_p.y, norm_p.z

                        /*** Measuremnt: distance to the closest surface/corner ***/
                        meas_vec( i ) = -norm_p.intensity;  
                    }

                    Eigen::Vector3d                           rot_add, t_add, v_add, bg_add, ba_add, g_add; //更新量:旋转,平移,速度,偏置等
                    Eigen::Matrix< double, DIM_OF_STATES, 1 > solution; // 最终解 : 29维
                    Eigen::MatrixXd                           K( DIM_OF_STATES, laserCloudSelNum );// kalman增益

                    /**
                     * @note (2-7-3-4) : Iterative Kalman Filter Update 
                     *       对应(有出入)FAST-LIO中公式(18)(19)(20)
                     */
                    if ( !flg_EKF_inited )  // 未初始化时初始化 - 前面已经初始化了
                    {
                        cout << ANSI_COLOR_RED_BOLD << "Run EKF init" << ANSI_COLOR_RESET << endl;
                        /*** only run in initialization period ***/
                        set_initial_state_cov( g_lio_state );
                    } 
                    else{
                        // cout << ANSI_COLOR_RED_BOLD << "Run EKF uph" << ANSI_COLOR_RESET << endl;
                        //1>:求公式中需要用到的 H 和 H^T*T
                        auto &&Hsub_T = Hsub.transpose();   // H转置 : 6xn = (nx6)^T
                        H_T_H.block< 6, 6 >( 0, 0 ) = Hsub_T * Hsub;//(0,0)处6x6块.H^T*T
                        //2>:求公式(20)中的Kalman增益的前面部分(省略R) : (H^T * R^-1 * H + P^-1)^-1
                        Eigen::Matrix< double, DIM_OF_STATES, DIM_OF_STATES > &&K_1 =
                            ( H_T_H + ( g_lio_state.cov / LASER_POINT_COV ).inverse() ).inverse();
                        //3>:结合2>求的前面部分,求公式(20)Kalman增益(省略R)
                        K = K_1.block< DIM_OF_STATES, 6 >( 0, 0 ) * Hsub_T;// K = (29x6) * (6xn) = (29xn)

                        //4>:求公式(18)中的最右边部分 : x^kk-x^k : Kalman迭代时的传播状态-预估(也是更新)状态
                        auto vec = state_propagate - g_lio_state;//state_propagate初始=g_lio_state
                        //5>:求公式(18)的中间和右边部分(有出入:I什么的都省略了)
                        solution = K * ( meas_vec - Hsub * vec.block< 6, 1 >( 0, 0 ) ); // kalman增益
                        // double speed_delta = solution.block( 0, 6, 3, 1 ).norm();
                        // if(solution.block( 0, 6, 3, 1 ).norm() > 0.05 )
                        // {
                        //     solution.block( 0, 6, 3, 1 ) = solution.block( 0, 6, 3, 1 ) / speed_delta * 0.05;
                        // }
                        //6>:结合5>中结果,求公式18计算结果,得到k+1次kalman的迭代更新值
                        g_lio_state = state_propagate + solution;   // kalman增益后的状态结果
                        print_dash_board();
                        // cout << ANSI_COLOR_RED_BOLD << "Run EKF uph, vec = " << vec.head<9>().transpose() << ANSI_COLOR_RESET << endl;
                        rot_add = solution.block< 3, 1 >( 0, 0 );   // 旋转增量
                        t_add = solution.block< 3, 1 >( 3, 0 );     // 平移增量
                        flg_EKF_converged = false;                  // 收敛标识
                        //7>:判断是否收敛
                        if ( ( ( rot_add.norm() * 57.3 - deltaR ) < 0.01 ) && ( ( t_add.norm() * 100 - deltaT ) < 0.015 ) )
                        {
                            flg_EKF_converged = true;   // 通过旋转和平移增量与上一次迭代的差值,判断是否收敛
                                                        // ? : 收敛了为啥不加break,而是继续进行迭代
                        }
                        //8>:旋转和平移增量转换单位
                        deltaR = rot_add.norm() * 57.3; // 角度单位
                        deltaT = t_add.norm() * 100;    // 厘米单位
                    }

                    // printf_line;
                    g_lio_state.last_update_time = Measures.lidar_end_time;
                    euler_cur = RotMtoEuler( g_lio_state.rot_end ); // 获得当前lidar的里程计信息,最后这个需要发布到ros中去
                    dump_lio_state_to_log( m_lio_state_fp );

                    /*** Rematch Judgement 重匹配判断 ***/
                    rematch_en = false;
                    if ( flg_EKF_converged || ( ( rematch_num == 0 ) && ( iterCount == ( NUM_MAX_ITERATIONS - 2 ) ) ) )
                    {
                        rematch_en = true;
                        rematch_num++;
                    }

                    /**
                     * @note (2-7-3-5) Convergence Judgements and Covariance Update 
                     *       收敛判断和协方差更新 : 对应FAST-LIO公式(19)
                     */
                    // if (rematch_num >= 10 || (iterCount == NUM_MAX_ITERATIONS - 1))
                    if ( rematch_num >= 2 || ( iterCount == NUM_MAX_ITERATIONS - 1 ) ) // Fast lio ori version.
                    {
                        if ( flg_EKF_inited )
                        {
                            /*** Covariance Update ***/
                            G.block< DIM_OF_STATES, 6 >( 0, 0 ) = K * Hsub; // 对应公式(19)中 : K * H
                            g_lio_state.cov = ( I_STATE - G ) * g_lio_state.cov;//公式(19): (单位阵-K*H)*Cur_协方差
                            total_distance += ( g_lio_state.pos_end - position_last ).norm();// 两次state间的距离
                            position_last = g_lio_state.pos_end;
                            // std::cout << "position: " << g_lio_state.pos_end.transpose() << " total distance: " << total_distance << std::endl;
                        }
                        solve_time += omp_get_wtime() - solve_start;
                        break;
                    }
                    solve_time += omp_get_wtime() - solve_start;
                    // cout << "Match cost time: " << match_time * 1000.0
                    //      << ", search cost time: " << kdtree_search_time*1000.0
                    //      << ", PCA cost time: " << pca_time*1000.0
                    //      << ", solver_cost: " << solve_time * 1000.0 << endl;
                    // cout <<"Iter cost time: " << tim.toc("Iter") << endl;
                } // (2-7-3) : Kalman滤波迭代完成

                t3 = omp_get_wtime();

                /**
                 * @note (2-7-4):add new frame points to map ikdtree 
                 *   1> : 更新维持的固定大小的map立方体 (参考FAST-LIO2:V.A地图管理)
                 *   2> : 将Kalman更新后的新lidar帧特征点先转世界坐标系,再加入ikd树
                 */
                PointVector points_history; // 将ikd树中需要移除的点放入points_history中
                ikdtree.acquire_removed_points( points_history );// 从Points_deleted和Multithread_Points_deleted获取点
                memset( cube_updated, 0, sizeof( cube_updated ) );
                //1> : 更新维持的固定大小的map立方体 (参考FAST-LIO2:V.A地图管理)
                for ( int i = 0; i < points_history.size(); i++ )
                {
                    PointType &pointSel = points_history[ i ];

                    int cubeI = int( ( pointSel.x + 0.5 * cube_len ) / cube_len ) + laserCloudCenWidth;
                    int cubeJ = int( ( pointSel.y + 0.5 * cube_len ) / cube_len ) + laserCloudCenHeight;
                    int cubeK = int( ( pointSel.z + 0.5 * cube_len ) / cube_len ) + laserCloudCenDepth;

                    if ( pointSel.x + 0.5 * cube_len < 0 )
                        cubeI--;
                    if ( pointSel.y + 0.5 * cube_len < 0 )
                        cubeJ--;
                    if ( pointSel.z + 0.5 * cube_len < 0 )
                        cubeK--;

                    if ( cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth )
                    {
                        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
                        featsArray[ cubeInd ]->push_back( pointSel );
                    }
                }

                //2-1> : 将Kalman更新后的新lidar帧特征点先转世界坐标系
                for ( int i = 0; i < feats_down_size; i++ )
                {
                    /* transform to world frame */
                    pointBodyToWorld( &( feats_down->points[ i ] ), &( feats_down_updated->points[ i ] ) );
                }
                t4 = omp_get_wtime();
                //2-2> : 将特征点加入世界坐标中
                ikdtree.Add_Points( feats_down_updated->points, true ); // 存入ikd树中
                
                kdtree_incremental_time = omp_get_wtime() - t4 + readd_time + readd_box_time + delete_box_time;
                t5 = omp_get_wtime();
            }   // (2-7) ICP迭代+Kalman更新完成

            /**
             * @note (2-8) : Publish current frame points in world coordinates:  
             *              发布当前帧的点云数据
             */
            laserCloudFullRes2->clear();
            *laserCloudFullRes2 = dense_map_en ? ( *feats_undistort ) : ( *feats_down );//去畸变or下采样点
            int laserCloudFullResNum = laserCloudFullRes2->points.size();// 发布点数量

            pcl::PointXYZI temp_point;
            laserCloudFullResColor->clear();
            {
                 // 将laserCloudFullRes2的点转到世界坐标系下,再存入laserCloudFullResColor
                for ( int i = 0; i < laserCloudFullResNum; i++ )
                {  
                    RGBpointBodyToWorld( &laserCloudFullRes2->points[ i ], &temp_point );
                    laserCloudFullResColor->push_back( temp_point );
                }
                sensor_msgs::PointCloud2 laserCloudFullRes3;// 将laserCloudFullResColor转为发布形式
                pcl::toROSMsg( *laserCloudFullResColor, laserCloudFullRes3 );
                // laserCloudFullRes3.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
                laserCloudFullRes3.header.stamp.fromSec( Measures.lidar_end_time );
                laserCloudFullRes3.header.frame_id = "world"; // world; camera_init
                pubLaserCloudFullRes.publish( laserCloudFullRes3 );
            }

            /**
             * @note (2-9) : append point cloud to global map.
             *               将点云添加至世界地图中 
             */ 
            if ( 1 )
            {
                static std::vector< double > stastic_cost_time;
                Common_tools::Timer          tim;
                // tim.tic();
                // ANCHOR - RGB maps update
                wait_render_thread_finish();
                if ( m_if_record_mvs )
                {
                    std::vector< std::shared_ptr< RGB_pts > > pts_last_hitted;
                    pts_last_hitted.reserve( 1e6 );
                    m_number_of_new_visited_voxel = m_map_rgb_pts.append_points_to_global_map(
                        *laserCloudFullResColor, Measures.lidar_end_time - g_camera_lidar_queue.m_first_imu_time, &pts_last_hitted,
                        m_append_global_map_point_step );
                    m_map_rgb_pts.m_mutex_pts_last_visited->lock();
                    m_map_rgb_pts.m_pts_last_hitted = pts_last_hitted;
                    m_map_rgb_pts.m_mutex_pts_last_visited->unlock();
                }
                else
                {
                    m_number_of_new_visited_voxel = m_map_rgb_pts.append_points_to_global_map(
                        *laserCloudFullResColor, Measures.lidar_end_time - g_camera_lidar_queue.m_first_imu_time, nullptr,
                        m_append_global_map_point_step );
                }
                stastic_cost_time.push_back( tim.toc( " ", 0 ) );
            }
            /**
             * @note (2-10) : 仅发布找到了最近平面的有效点
             */ 
            if(0) // Uncomment this code scope to enable the publish of effective points. 
            {
                /******* Publish effective points *******/
                laserCloudFullResColor->clear();
                pcl::PointXYZI temp_point;
                for ( int i = 0; i < laserCloudSelNum; i++ )
                {
                    RGBpointBodyToWorld( &laserCloudOri->points[ i ], &temp_point );
                    laserCloudFullResColor->push_back( temp_point );
                }
                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg( *laserCloudFullResColor, laserCloudFullRes3 );
                // laserCloudFullRes3.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
                laserCloudFullRes3.header.stamp.fromSec( Measures.lidar_end_time ); //.fromSec(last_timestamp_lidar);
                laserCloudFullRes3.header.frame_id = "world";
                pubLaserCloudEffect.publish( laserCloudFullRes3 );
            }

            /**
             * @note (2-11)***** Publish Maps:  发布地图******
             */
            sensor_msgs::PointCloud2 laserCloudMap;
            pcl::toROSMsg( *featsFromMap, laserCloudMap );
            laserCloudMap.header.stamp.fromSec( Measures.lidar_end_time ); // ros::Time().fromSec(last_timestamp_lidar);
            laserCloudMap.header.frame_id = "world";
            pubLaserCloudMap.publish( laserCloudMap );

            /**
             * @note (2-12)***** Publish Odometry 发布里程计*****
             */
            geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw( euler_cur( 0 ), euler_cur( 1 ), euler_cur( 2 ) );
            odomAftMapped.header.frame_id = "world";
            odomAftMapped.child_frame_id = "/aft_mapped";
            odomAftMapped.header.stamp = ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
            odomAftMapped.pose.pose.orientation.x = geoQuat.x;
            odomAftMapped.pose.pose.orientation.y = geoQuat.y;
            odomAftMapped.pose.pose.orientation.z = geoQuat.z;
            odomAftMapped.pose.pose.orientation.w = geoQuat.w;
            odomAftMapped.pose.pose.position.x = g_lio_state.pos_end( 0 );
            odomAftMapped.pose.pose.position.y = g_lio_state.pos_end( 1 );
            odomAftMapped.pose.pose.position.z = g_lio_state.pos_end( 2 );
            pubOdomAftMapped.publish( odomAftMapped );

            static tf::TransformBroadcaster br;
            tf::Transform                   transform;
            tf::Quaternion                  q;
            transform.setOrigin(
                tf::Vector3( odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y, odomAftMapped.pose.pose.position.z ) );
            q.setW( odomAftMapped.pose.pose.orientation.w );
            q.setX( odomAftMapped.pose.pose.orientation.x );
            q.setY( odomAftMapped.pose.pose.orientation.y );
            q.setZ( odomAftMapped.pose.pose.orientation.z );
            transform.setRotation( q );
            br.sendTransform( tf::StampedTransform( transform, ros::Time().fromSec( Measures.lidar_end_time ), "world", "/aft_mapped" ) );

            msg_body_pose.header.stamp = ros::Time::now();
            msg_body_pose.header.frame_id = "/camera_odom_frame";
            msg_body_pose.pose.position.x = g_lio_state.pos_end( 0 );
            msg_body_pose.pose.position.y = g_lio_state.pos_end( 1 );
            msg_body_pose.pose.position.z = g_lio_state.pos_end( 2 );
            msg_body_pose.pose.orientation.x = geoQuat.x;
            msg_body_pose.pose.orientation.y = geoQuat.y;
            msg_body_pose.pose.orientation.z = geoQuat.z;
            msg_body_pose.pose.orientation.w = geoQuat.w;

            /**
             * @note (2-13)***** Publish Path 发布路径*******
             */
            msg_body_pose.header.frame_id = "world";
            if ( frame_num > 10 )
            {
                path.poses.push_back( msg_body_pose );
            }
            pubPath.publish( path );
            
            /**
             * @note (2-14)* save debug variables 保存debug变量**
             */
            frame_num++;
            aver_time_consu = aver_time_consu * ( frame_num - 1 ) / frame_num + ( t5 - t0 ) / frame_num;
            // aver_time_consu = aver_time_consu * 0.8 + (t5 - t0) * 0.2;
            T1[ time_log_counter ] = Measures.lidar_beg_time;
            s_plot[ time_log_counter ] = aver_time_consu;
            s_plot2[ time_log_counter ] = kdtree_incremental_time;
            s_plot3[ time_log_counter ] = kdtree_search_time;
            s_plot4[ time_log_counter ] = fov_check_time;
            s_plot5[ time_log_counter ] = t5 - t0;
            s_plot6[ time_log_counter ] = readd_box_time;
            time_log_counter++;
            fprintf( m_lio_costtime_fp, "%.5f %.5f\r\n", g_lio_state.last_update_time - g_camera_lidar_queue.m_first_imu_time, t5 - t0 );
            fflush( m_lio_costtime_fp );
        }
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
