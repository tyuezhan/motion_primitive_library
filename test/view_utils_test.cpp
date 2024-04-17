#include <mpl_planner/common/view_utils.h>

// TODO: more test cases, also comparing the old version with the new version
//Driver code
int main()
{

    int poly_size = 3;
    double poly_points[20][2] = {{0,0}, {0,200},
                              {200,0}};
    
    int clipper_size = 3;
    double clipper_points[20][2] = {{0,0}, {200,200},
                              {200,0}};
    //Calling the clipping function
    double area1 = MPL::suthHodgClip(poly_points, poly_size, clipper_points,
                 clipper_size);
    printf("Area of polygon = %f\n", area1);
    
    //test new version
    std::vector<MPL::Point> vp1_pts, vp2_pts;
    MPL::Point vp1_pt1, vp1_pt2, vp1_pt3, vp2_pt1, vp2_pt2, vp2_pt3;
    vp1_pt1.x_ = 0; 
    vp1_pt1.y_ = 0;
    vp1_pt2.x_ = 0;
    vp1_pt2.y_ = 200;
    vp1_pt3.x_ = 200;
    vp1_pt3.y_ = 0;
    vp1_pts.push_back(vp1_pt3);
    vp1_pts.push_back(vp1_pt1);
    vp1_pts.push_back(vp1_pt2);
    ROS_INFO("vp1 pts x: %f, %f, %f", vp1_pts[0].x_, vp1_pts[1].x_, vp1_pts[2].x_);
    ROS_INFO("vp1 pts y: %f, %f, %f", vp1_pts[0].y_, vp1_pts[1].y_, vp1_pts[2].y_);
    vp2_pt1.x_ = 0;
    vp2_pt1.y_ = 0;
    // vp2_pt2.x_, vp2_pt2.y_ = 200, 200;
    vp2_pt2.x_ = 200;
    vp2_pt2.y_ = 200;
    vp2_pt3.x_ = 200;
    vp2_pt3.y_ = 0;
    vp2_pts.push_back(vp2_pt3);
    vp2_pts.push_back(vp2_pt2);
    vp2_pts.push_back(vp2_pt1);
    ROS_INFO("vp2 pts x: %f, %f, %f", vp2_pts[0].x_, vp2_pts[1].x_, vp2_pts[2].x_);
    ROS_INFO("vp2 pts y: %f, %f, %f", vp2_pts[0].y_, vp2_pts[1].y_, vp2_pts[2].y_);
    // test sort
    sort(vp1_pts.begin(), vp1_pts.end(), MPL::clockwise_less);
    sort(vp2_pts.begin(), vp2_pts.end(), MPL::clockwise_less);
    ROS_INFO("After sort-vp1 pts x: %f, %f, %f", vp1_pts[0].x_, vp1_pts[1].x_, vp1_pts[2].x_);
    ROS_INFO("After sort-vp1 pts y: %f, %f, %f", vp1_pts[0].y_, vp1_pts[1].y_, vp1_pts[2].y_);
    ROS_INFO("After sort-vp2 pts x: %f, %f, %f", vp2_pts[0].x_, vp2_pts[1].x_, vp2_pts[2].x_);
    ROS_INFO("After sort-vp2 pts y: %f, %f, %f", vp2_pts[0].y_, vp2_pts[1].y_, vp2_pts[2].y_);
    double area2 = MPL::suthHodgClip(vp1_pts, vp2_pts);
    printf("Area of polygon = %f\n", area2);


    // Test view correlation
    // case 1: ~0.5
    Eigen::Vector3d pos1, pos2;
    pos1 << 0, 0, 0;
    pos2 << 50*sqrt(2), 0, 0;
    double yaw1 = 0;
    double yaw2 = 3.14;
    double h_fov = 3.14/2;
    double max_ray_len = 100;
    double corr = MPL::getViewCorrelation(pos1, yaw1, pos2, yaw2, h_fov, max_ray_len);
    printf("Correlation = %f\n", corr);

    // case 2: ~1.0
    pos1 << 0, 0, 0;
    pos2 << 5, 5*sqrt(3), 0;
    yaw1 = 1.57;
    yaw2 = -2.617;
    h_fov = 3.14/3;
    max_ray_len = 10;
    corr = MPL::getViewCorrelation(pos1, yaw1, pos2, yaw2, h_fov, max_ray_len);
    printf("Correlation = %f\n", corr);
    return 0;

}

