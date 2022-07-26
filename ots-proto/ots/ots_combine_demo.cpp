//
//  ots_combine_demo.cpp
//  ots-proto
//
//  Created by Matthew Fonken on 3/12/22.
//

#include "ots_combine_demo.hpp"
#include "unfisheye.hpp"
#include <format>

using namespace cv;

CombineDemo::CombineDemo(Environment * env, Combine* combine)
{
    this->env = env;
    this->combine = combine;
    
    this->env->addTest( &fps, 1.0 );
}

void CombineDemo::Init(combine_utility_rate_t rates[Combine::NUM_UTILITIES])
{
    for(int i = 0; i < Combine::NUM_UTILITIES; i++)
    {
        InitUtility(rates[i].name, rates[i].rate);
    }
}

void CombineDemo::Start()
{
    env->start();
}

void CombineDemo::Pause()
{
    env->pause();
}

pthread_mutex_t* CombineDemo::InitUtility(Combine::combine_utility_e name, int rate)
{
    TestInterface* utility = combine->GetUtility(name);
    if(utility == NULL) return NULL;
    
    env->addTest(utility, rate);
    return &utility->mutex;
}

void CombineDemo::HandleKey(char k)
{
    switch(k)
    {
        case ' ':
            OrienterFunctions.Tare( &combine->kin.orienter);
            break;
        case 'r':
            combine->det.tracker.reorder();
            break;
        default:
            break;
    }
}

void CombineDemo::ShowWebcam()
{
    while(true)
    { LOCK(&combine->wcu.mutex)
        if(!combine->wcu.frame.empty())
            imshow("figure", combine->wcu.frame);
        waitKey(10);
    }
}

void CombineDemo::ShowFrame(bool show_processed)
{
    const double scale = 0.75;
    sleep(1);
    OrienterFunctions.Tare( &combine->kin.orienter );
    
    while(true)
    {
        if((show_processed && combine->new_processed_frame) || (!show_processed && combine->new_frame))
        {
//            LOCK(&combine->frame_mutex)
            Mat m;
            if(!combine->frame.empty())
            {
                fps.Tick();
                resize(combine->frame, m, Size(), scale, scale, INTER_LINEAR);
                putText(m, to_string((int)fps.Get()), Point(3, 13), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 100));
                
//                // Unfisheye image
//                Rect crop(m.cols/2 - m.rows/2, 0, m.rows, m.rows);
//                Mat M(m, crop);
//                Mat U = Mat(M.rows, M.cols, M.type(), Scalar(255, 255, 255));
//                invfisheye(M, U);
                
                // Draw kinetic position
                vec3_t p = combine->kin.GetPosition();
//                p.i = -p.i;
                double* v = (double*)&p;
                int x = 70;
                Scalar rgb[] = { cv::viz::Color::red(), cv::viz::Color::green(), cv::viz::Color::celestial_blue()  };
                for( int i = 0; i < 3; i++ )
                {
                    putText(m, format("%.2f", v[i] * 1000), Point(x * (i + 1), 13), FONT_HERSHEY_DUPLEX, 0.5, rgb[i]);
                }
                
                // Draw crosshair
                int l = 4, t = 1;
                cv::viz::Color c = cv::viz::Color::celestial_blue ();
                double w = m.cols/2, h = m.rows/2;
                line( m, Point(w - l, h), Point(w + l, h), c, t);
                line( m, Point(w, h - l), Point(w, h + l), c, t);
                
                Mat show = m.clone();
                imshow("Demo", show);
            }
        }
        char k = waitKey(50);
        if(k > 0 && k != key_) HandleKey(k);
        key_ = k;
    }
}

/// TESTS
void CombineDemo::TestRho(int rate)
{
    InitUtility(Combine::WEBCAM, rate);
    int n = 1;
    int nframes = 26;
    Mat m;
#ifdef USE_RHO
    combine->rho.Init(FRAME_WIDTH_BASE, FRAME_HEIGHT);//combine->wcu.size.width, combine->wcu.size.height);
    RhoDrawer drawer(&RhoSystem.Variables.Utility);
#endif
    Start();
    while(true)
    {
        // cv::imread("/Users/matthew/Desktop/PersonalResources/TestImages/frames/single/" + to_string(n++) + ".png");
//        if(combine->wcu.frame.cols > 0)
        { //LOCK(mutex)
            resize(cv::imread("/Users/matthew/Desktop/PersonalResources/TestImages/frames/ellipse/" + to_string(n++) + ".png"), m, Size(FRAME_WIDTH_BASE, FRAME_HEIGHT), INTER_NEAREST);
//            Mat m; // cv::imread("/Users/matthew/Desktop/PersonalResources/TestImages/frames/ellipse/" + to_string(n) + ".png");//
//            m.at<Vec3b>(1, 1) = Vec3b(255, 255, 255);
            //
#ifdef USE_RHO
            m = combine->wcu.frame.clone();
//            resize(combine->wcu.frame, m, Size(size, size));
//#else
#ifdef IMAGE_THRESHOLD
            threshold( combine->wcu.frame, m, IMAGE_THRESHOLD, 255, 0 );
#endif
#endif
            if(m.cols > 0)
            {
#ifdef USE_RHO
                combine->rho.Perform( m );
                drawer.DrawDensityGraph( m );
#else
                combine->det.perform(m);
                combine->det.draw(m);
#endif
                imshow("figure", m);
            }
        }
        switch(waitKey(1000 / rate))
        {
            case ' ':
                if(env->status == PAUSED)
                    env->resume();
                else
                    env->pause();
            default:
                break;
        }
        if(n > nframes) n = 1;
    }
}

void CombineDemo::TestWebcam(int rate)
{
    pthread_mutex_t * mutex = InitUtility(Combine::WEBCAM, rate);
    Start();
    printf("demo->%p\n", mutex);
    ShowWebcam();
}

void CombineDemo::TestIMU(int rate)
{
    pthread_mutex_t * mutex = InitUtility(Combine::IMU, rate);
    Start();
    
    imu_t * imu = &combine->imu.imu;
//    int i = 0;
    double s = TIMESTAMP(TIME_SEC);
    while(true)
    {
        { LOCK(mutex)
            printf("%.2fs: p%.2f r%.2f y%.2f\n", TIMESTAMP(TIME_SEC) - s, imu->pitch, imu->roll, imu->yaw);
        }
        waitKey(1000 / rate);
        
//        if(i++ > rate)
//        {
//            Pause();
//            break;
//        }
    }
//    sleep(1);
}

void CombineDemo::TestKinetic(int rate)
{
    InitUtility(Combine::IMU, rate*2);
    InitUtility(Combine::KINETIC, rate);
    Start();
//    vec3_t g = { 0, 0, -1 };
    int i = 0;
    while(true)
    {
        vec3_t n, a;
        { LOCK(&combine->imu.mutex)
            IMUUtility::imu_data_t imu_data = combine->imu.FetchIMUData();
            n = { imu_data.accel[0], imu_data.accel[1], imu_data.accel[2] };
            a = { imu_data.roll, imu_data.pitch, imu_data.yaw };
//            g = { imu_data.gravity[0], imu_data.gravity[1], imu_data.gravity[2] };
        }
        { LOCK(&combine->kin.orienter_data_mutex);
            OrienterFunctions.Update( &combine->kin.orienter, &a );
            ang3_t * r = &combine->kin.orienter.rotation;
            ang3_t * rr = &combine->kin.orienter.rotation_raw;
            LOG_KU(DEBUG_2, "<%.2f, %.2f, %.2f> | <%.2f, %.2f, %.2f>\n", rr->x, rr->y, rr->z, r->x, r->y, r->z);
        }
        waitKey(1000 / rate);
        if( i++ % 5 == 0 )
        {
//            LOG_KU(DEBUG_2, "Tared: <%.2f, %.2f, %.2f>\n", g.i, g.j, g. k);
            OrienterFunctions.Tare( &combine->kin.orienter );//, &g );
        }
    }
}

void CombineDemo::TestTracker(int rate)
{
    vector<Point2f> pts = { Point2f(0, 0) };
    combine->det.tracker.Update(pts);
}

void CombineDemo::Record(int seconds, int webcam_rate, int imu_rate, string path) {
    // Environment
    InitUtility(Combine::WEBCAM, webcam_rate);
    InitUtility(Combine::IMU, imu_rate);
    Start();
    
//    using namespace std::placeholders;
//    combine->wcu.OnFrame = std::bind(&CombineDemo::OnFrame, this, _1, _2);
    
    // Wait for first camera frame
    while(combine->wcu.GetFrame().empty())
        waitKey(100);
    
    // Cycles
    int i = 0;
    double start_ms = TIMESTAMP(TIME_MS);
    double duration_ms = (double)seconds * 1000.0;
    
    // Data
    IMUUtility::imu_data_t imu_data;
    Mat img;
    double img_timestamp_ms = 0.0;
    FileWriter imu_writer;
    string imu_file = string(path + "/imu0.csv");
    imu_writer.init(imu_file.c_str());
    string img_path = string(path + "/cam0");
    
    int cam_interval = imu_rate / webcam_rate;
    sleep(2);
    
    double time = 0;
    while(time < duration_ms)
    {
        waitKey( 1000.0 / imu_rate );
        time = TIMESTAMP(TIME_MS) - start_ms;
        printf("%.1fs\n", time * 1.0e-3);
        i++;
        { LOCK(&combine->imu.mutex)
            imu_data = combine->imu.FetchIMUData();
        }
        if(i % cam_interval == 0)
        { LOCK(&combine->wcu.mutex)
            img = combine->wcu.frame.clone();
            img_timestamp_ms = combine->wcu.frame_time_ms;
            fps.Tick();
        }
        
        if(imu_data.timestamp_ms < 1 || img_timestamp_ms < start_ms ) continue;
        
        // IMU Output
        std::stringstream imu_s;
        long imu_timestamp_ns = (long)( ( imu_data.timestamp_ms - start_ms ) * 1.0e6 );
        imu_s << imu_timestamp_ns;
        for( int i = 0; i < 3; i++ ) imu_s << "," << imu_data.gyro[i];
        for( int i = 0; i < 3; i++ ) imu_s << "," << imu_data.accel[i];
        imu_s << "\n";
        imu_writer.trigger(imu_s.str(), true);
        
        // Img Output
        if(i % cam_interval == 0)
        {
            std::stringstream img_s;
            long img_timestamp_ns = (long)( ( combine->wcu.frame_time_ms - start_ms ) * 1.0e6 );
            img_s << img_path << "/" << img_timestamp_ns << ".png";
            imwrite(img_s.str(), img);
//            putText(img, img_s.str(), Point(2, 20), 1, FONT_HERSHEY_DUPLEX, Scalar(255, 255, 255));
//            imshow("fig", img);
//            printf("img: %s %.2f\n", img_s.str().c_str(), fps.Get());
        }
//        printf("imu: %s", imu_s.str().c_str());
        
    }
    printf("Ran %d cycles\n", i);
}

//void CombineDemo::OnFrame(Mat img, double timestamp_ms)
//{
//    if(img_path.length() == 0 || start_ms == 0) return;
////    img = frame.clone();
//    std::stringstream img_s;
//    long img_timestamp_ns = (long)( ( combine->wcu.frame_time_ms - start_ms ) * 1.0e6 );
//    img_s << img_path << "/" << img_timestamp_ns << ".png";
////    putText(img, img_s.str(), Point(2, 20), 1, FONT_HERSHEY_DUPLEX, Scalar(255, 255, 255));
////            imwrite(img_s.str(), img);
//    printf("img: %s\n", img_s.str().c_str());
//}

void CombineDemo::Visualizer(int rate)
{
    int dim = 350;
    double scale = 5;
    
    // Perspective
    double rv[3] = { 0.0, 0.0, 0.0 }; //45.0 * DEG_TO_RAD }; // X>Y>Z
    Mat R, r(3, 1, CV_64F, (void*)rv);
    cv::Rodrigues(r, R);
    cout << r << endl;
    cout << R << endl;
    
    double tv[3] = { 0, -2, 10 };
    Mat T(3, 1, CV_64F, (void*)tv);
    cout << T << endl;
    
    double f = dim / 2.0;
    double c = (double)dim / 2.0;
    double kv[9] = { f, 0, c, 0, f, c, 0, 0, 1 };
    Mat K(3, 3, CV_64F, (void*)kv);
    
    cout << K << endl;
    
    // Points
    double pv[12] = { 1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 0 };
    Mat p(4, 3, CV_64F, (void*)pv);
    p *= scale;
    cout << p << endl;
    
    Mat P;
    projectPoints(p, R, T, K, Mat::zeros(4, 1, CV_64F), P);
    cout << P << endl;
    
    imu_t * imu = &combine->imu.imu;
    InitUtility(Combine::IMU, rate);
    Start();
    orienter_t ori;
    
    while(1)
    {
        Mat vis(dim, dim, CV_8UC3, Scalar(255, 255, 255));
        
        // Draw Axes
        Point o = (Point)P.at<Vec2d>(3);
        Scalar colors[3] = { Scalar(0, 0, 255), Scalar(0, 255, 0), Scalar(255, 0, 0) };
        for( int i = 0; i < 3; i++ )
        {
            Point pi = (Point)P.at<Vec2d>(i);
            cv::line(vis, o, pi, colors[i]);
        }
        
        double * mrv;
        vec3_t a = { imu->pitch, imu->roll, imu->yaw };
        { LOCK(&combine->kin.orienter_data_mutex);
            OrienterFunctions.Update( &ori, &a );
            mrv = (double*)&ori.rotation;
        }
        
        float c1 = cos(mrv[1] * DEG_TO_RAD);
        float s1 = sin(mrv[1] * DEG_TO_RAD);
        float c2 = cos(mrv[0] * DEG_TO_RAD); // intrinsic rotation
        float s2 = sin(mrv[0] * DEG_TO_RAD);
        float c3 = cos(mrv[2] * DEG_TO_RAD);
        float s3 = sin(mrv[2] * DEG_TO_RAD);
        
        double MRv[9] = { c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3,
                            -s2,          c1*c2,          c2*s1,
                          c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3 };
        Mat mr(1, 3, CV_64F, (void*)mrv), MR(3, 3, CV_64F, MRv);
        cout << mr << endl;
        mr *= DEG_TO_RAD;
//        MR *= DEG_TO_RAD;
//        cv::Rodrigues(mr, MR);
//        MR *= R;
        
        double mpv[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        Mat mp(3, 3, CV_64F, (void*)mpv);
        mp *= scale;
        Mat MP;
        projectPoints(mp, MR, T, K, Mat::zeros(4, 1, CV_64F), MP);
        Scalar colors_[3] = { Scalar(100, 0, 255), Scalar(0, 255, 100), Scalar(255, 100, 0) };
        for(int i = 0; i < 3; i++)
        {
            Point p = (Point)MP.at<Vec2d>(i);
            cv::line(vis, o, p, colors_[i], 2);
        }
        
//        cout << o << endl;
        
        imshow("3d", vis);
        switch(waitKey(1000 / 10))
        {
            case ' ':
                OrienterFunctions.Tare( &ori );
                break;
            default:
                break;
        }
    }
}
