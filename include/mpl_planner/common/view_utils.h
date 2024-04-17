#ifndef _VIEW_UTILS_H_
#define _VIEW_UTILS_H_

#include <ros/ros.h>
#include <vector>
#include <Eigen/Eigen>


namespace MPL {
/*
 *  Input needs to be sorted following clockwise order to use Sutherland–Hodgman algo
 */
const int MAX_POINTS = 20;

struct Point {
  double x_;
  double y_;
  double center_x_;
  double center_y_;
};


bool clockwise_less(const Point& a, const Point& b) {
    if (a.x_ - a.center_x_ >= 0 && b.x_ - a.center_x_ < 0)
        return true;
    if (a.x_ - a.center_x_ < 0 && b.x_ - a.center_x_ >= 0)
        return false;
    if (a.x_ - a.center_x_ == 0 && b.x_ - a.center_x_ == 0) {
        if (a.y_ - a.center_y_ >= 0 || b.y_ - a.center_y_ >= 0)
            return a.y_ > b.y_;
        return b.y_ > a.y_;
    }

    // compute the cross product of vectors (center -> a) x (center -> b)
    double det = (a.x_ - a.center_x_) * (b.y_ - a.center_y_) - (b.x_ - a.center_x_) * (a.y_ - a.center_y_);
    if (det < 0)
        return true;
    if (det > 0)
        return false;

    // points a and b are on the same line from the center
    // check which point is closer to the center
    double d1 = (a.x_ - a.center_x_) * (a.x_ - a.center_x_) + (a.y_ - a.center_y_) * (a.y_ - a.center_y_);
    double d2 = (b.x_ - a.center_x_) * (b.x_ - a.center_x_) + (b.y_ - a.center_y_) * (b.y_ - a.center_y_);
    return d1 > d2;
}

// (X[i], Y[i]) are coordinates of i'th point.
double polygonArea(double X[], double Y[], int n) {
    // Initialize area
    double area = 0.0;
 
    // Calculate value of shoelace formula
    int j = n - 1;
    for (int i = 0; i < n; i++)
    {
        area += (X[j] + X[i]) * (Y[j] - Y[i]);
        j = i;  // j is previous vertex to i
    }
 
    // Return absolute value
    return abs(area / 2.0);
}
 

double x_intersect(double x1, double y1, double x2, double y2,
                double x3, double y3, double x4, double y4) {
    double num = (x1*y2 - y1*x2) * (x3-x4) -
              (x1-x2) * (x3*y4 - y3*x4);
    double den = (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4);
    return num/den;
}


// Returns y-value of point of intersection of
// two lines
double y_intersect(double x1, double y1, double x2, double y2,
                double x3, double y3, double x4, double y4) {
    double num = (x1*y2 - y1*x2) * (y3-y4) -
              (y1-y2) * (x3*y4 - y3*x4);
    double den = (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4);
    return num/den;
}


// This functions clips all the edges w.r.t one clip
// edge of clipping area
void clip(double poly_points[][2], int &poly_size,
          double x1, double y1, double x2, double y2) {
    double new_points[MAX_POINTS][2];
    int new_poly_size = 0;
  
    // (ix,iy),(kx,ky) are the co-ordinate values of
    // the points
    for (int i = 0; i < poly_size; i++) {
        // i and k form a line in polygon
        int k = (i+1) % poly_size;
        double ix = poly_points[i][0], iy = poly_points[i][1];
        double kx = poly_points[k][0], ky = poly_points[k][1];

        // Calculating position of first point
        // w.r.t. clipper line
        double i_pos = (x2-x1) * (iy-y1) - (y2-y1) * (ix-x1);

        // Calculating position of second point
        // w.r.t. clipper line
        double k_pos = (x2-x1) * (ky-y1) - (y2-y1) * (kx-x1);
        // Case 1 : When both points are inside
        if (i_pos < 0  && k_pos < 0) {
            //Only second point is added
            new_points[new_poly_size][0] = kx;
            new_points[new_poly_size][1] = ky;
            new_poly_size++;
        }
  
        // Case 2: When only first point is outside
        else if (i_pos >= 0  && k_pos < 0) {
            // Point of intersection with edge
            // and the second point is added
            new_points[new_poly_size][0] = x_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_points[new_poly_size][1] = y_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_poly_size++;
  
            new_points[new_poly_size][0] = kx;
            new_points[new_poly_size][1] = ky;
            new_poly_size++;
        }
  
        // Case 3: When only second point is outside
        else if (i_pos < 0  && k_pos >= 0) {
            //Only point of intersection with edge is added
            new_points[new_poly_size][0] = x_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_points[new_poly_size][1] = y_intersect(x1,
                              y1, x2, y2, ix, iy, kx, ky);
            new_poly_size++;
        }
  
        // Case 4: When both points are outside
        else {
            //No points are added
        }
    }
  
    // Copying new points into original array
    // and changing the no. of vertices
    poly_size = new_poly_size;
    for (int i = 0; i < poly_size; i++)
    {
        poly_points[i][0] = new_points[i][0];
        poly_points[i][1] = new_points[i][1];
        // ROS_INFO("[old method]new points: (%f, %f)", poly_points[i][0], poly_points[i][1]);
    }
}



// Implements Sutherland–Hodgman algorithm
double suthHodgClip(double poly_points[][2], int poly_size,
                  double clipper_points[][2], int clipper_size) {
    //i and k are two consecutive indexes
    for (int i=0; i<clipper_size; i++) {
        int k = (i+1) % clipper_size;
  
        // We pass the current array of vertices, it's size
        // and the end points of the selected clipper line
        clip(poly_points, poly_size, clipper_points[i][0],
             clipper_points[i][1], clipper_points[k][0],
             clipper_points[k][1]);
    }
    if (poly_size == 0) {
        return 0;
    } else {
        double X[poly_size];
        double Y[poly_size];
        for (int j = 0; j < poly_size; j++) {
            X[j] = poly_points[j][0];
            Y[j] = poly_points[j][1];
        }
        return polygonArea(X, Y, poly_size);
    }
}


// New version:
// This functions clips all the edges w.r.t one clip
// edge of clipping area
std::vector<Eigen::Vector2d> clip(std::vector<Eigen::Vector2d> poly_points,
          double x1, double y1, double x2, double y2) {
    // double new_points[MAX_POINTS][2];
    int poly_size = poly_points.size();
    std::vector<Eigen::Vector2d> new_points;
    // int new_poly_size = 0;
    // (ix,iy),(kx,ky) are the co-ordinate values of
    // the points
    for (int i = 0; i < poly_size; i++) {
        // i and k form a line in polygon
        int k = (i+1) % poly_size;
        double ix = poly_points[i](0);
        double iy = poly_points[i](1);
        double kx = poly_points[k](0);
        double ky = poly_points[k](1);
        // Calculating position of first point
        // w.r.t. clipper line
        double i_pos = (x2-x1) * (iy-y1) - (y2-y1) * (ix-x1);

        // Calculating position of second point
        // w.r.t. clipper line
        double k_pos = (x2-x1) * (ky-y1) - (y2-y1) * (kx-x1);
        // Case 1 : When both points are inside
        if (i_pos < 0  && k_pos < 0) {
            //Only second point is added
            new_points.push_back(Eigen::Vector2d(kx, ky));
        }
  
        // Case 2: When only first point is outside
        else if (i_pos >= 0  && k_pos < 0) {
            // Point of intersection with edge
            // and the second point is added
            double x_int = x_intersect(x1, y1, x2, y2, ix, iy, kx, ky);
            double y_int = y_intersect(x1, y1, x2, y2, ix, iy, kx, ky);
            new_points.push_back(Eigen::Vector2d(x_int, y_int));
            new_points.push_back(Eigen::Vector2d(kx, ky));
        }
  
        // Case 3: When only second point is outside
        else if (i_pos < 0  && k_pos >= 0) {
            //Only point of intersection with edge is added
            double x_int = x_intersect(x1, y1, x2, y2, ix, iy, kx, ky);
            double y_int = y_intersect(x1, y1, x2, y2, ix, iy, kx, ky);
            new_points.push_back(Eigen::Vector2d(x_int, y_int));
        }
  
        // Case 4: When both points are outside
        else {
            //No points are added
        }
    }
  
    // for (int i = 0; i < new_points.size(); i++) {
    //     ROS_INFO("new points: (%f, %f)", new_points[i](0), new_points[i](1));
    // }
    return new_points;
}

// New version
double suthHodgClip(std::vector<Point> poly_points,
                   std::vector<Point> clipper_points) {
    //i and k are two consecutive indexes
    std::vector<Eigen::Vector2d> poly;
    for (int i=0; i<poly_points.size(); i++) {
        poly.push_back(Eigen::Vector2d(poly_points[i].x_, poly_points[i].y_));
        // ROS_INFO("poly points inited: (%f, %f)", poly_points[i].x_, poly_points[i].y_);
    }
    
    for (int i=0; i<clipper_points.size(); i++) {
        int k = (i+1) % clipper_points.size();
        // We pass the current array of vertices, it's size
        // and the end points of the selected clipper line
        // ROS_INFO("i and k: %d, %d", i, k);
        // ROS_INFO("clipper points1: (%f, %f)", clipper_points[i].x_, clipper_points[i].y_);
        // ROS_INFO("clipper points2: (%f, %f)", clipper_points[k].x_, clipper_points[k].y_);
        poly = clip(poly, 
               clipper_points[i].x_,
               clipper_points[i].y_, 
               clipper_points[k].x_,
               clipper_points[k].y_);
    }
    // After iterations, the end poly should be the clipped polygon
    int poly_size = poly.size();
    // ROS_INFO("poly size: %d", poly_size);
    if (poly_size == 0) {
        return 0;
    } else {
        double X[poly_size];
        double Y[poly_size];
        for (int j = 0; j < poly_size; j++) {
            X[j] = poly[j](0);
            Y[j] = poly[j](1);
        }
        return polygonArea(X, Y, poly_size);
    }
}


double getViewCorrelation(const Eigen::Vector3d pos1, const double yaw1, 
                          const Eigen::Vector3d pos2, const double yaw2,
                          const double h_fov, const double max_ray_len) {
  std::vector<Point> vp1_pts, vp2_pts;
  Point vp1_pt1, vp1_pt2, vp1_pt3, vp2_pt1, vp2_pt2, vp2_pt3;
  double half_h_fov = h_fov/2;
  double center_x, center_y;

  // First view frustum
  vp1_pt1.x_ = pos1(0);
  vp1_pt1.y_ = pos1(1);
  vp1_pt2.x_ = pos1(0) + max_ray_len * cos(yaw1 - half_h_fov);
  vp1_pt2.y_ = pos1(1) + max_ray_len * sin(yaw1- half_h_fov);
  vp1_pt3.x_ = pos1(0) + max_ray_len * cos(yaw1 + half_h_fov);
  vp1_pt3.y_ = pos1(1) + max_ray_len * sin(yaw1 + half_h_fov);

  center_x = (vp1_pt1.x_ + vp1_pt2.x_ + vp1_pt3.x_) / 3;
  center_y = (vp1_pt1.y_ + vp1_pt2.y_ + vp1_pt3.y_) / 3;
  vp1_pt1.center_x_ = center_x;
  vp1_pt2.center_x_ = center_x;
  vp1_pt3.center_x_ = center_x;
  vp1_pt1.center_y_ = center_x;
  vp1_pt2.center_y_ = center_y;
  vp1_pt3.center_y_ = center_y;
  vp1_pts.push_back(vp1_pt1);
  vp1_pts.push_back(vp1_pt2);
  vp1_pts.push_back(vp1_pt3);

  // Second view frustum
  vp2_pt1.x_ = pos2(0);
  vp2_pt1.y_ = pos2(1);
  vp2_pt2.x_ = pos2(0) + max_ray_len * cos(yaw2 - half_h_fov);
  vp2_pt2.y_ = pos2(1) + max_ray_len * sin(yaw2 - half_h_fov);
  vp2_pt3.x_ = pos2(0) + max_ray_len * cos(yaw2 + half_h_fov);
  vp2_pt3.y_ = pos2(1) + max_ray_len * sin(yaw2 + half_h_fov);

  center_x = (vp2_pt1.x_ + vp2_pt2.x_ + vp2_pt3.x_) / 3;
  center_y = (vp2_pt1.y_ + vp2_pt2.y_ + vp2_pt3.y_) / 3;
  vp2_pt1.center_x_ = center_x;
  vp2_pt2.center_x_ = center_x;
  vp2_pt3.center_x_ = center_x;
  vp2_pt1.center_y_ = center_y;
  vp2_pt2.center_y_ = center_y;
  vp2_pt3.center_y_ = center_y;
  vp2_pts.push_back(vp2_pt1);
  vp2_pts.push_back(vp2_pt2);
  vp2_pts.push_back(vp2_pt3);
  // Sort both triangles coordinates in clockwise order
  sort(vp1_pts.begin(), vp1_pts.end(), clockwise_less);
  sort(vp2_pts.begin(), vp2_pts.end(), clockwise_less);  

  //   ROS_INFO("Point x, y after sort: (%f, %f), (%f, %f), (%f, %f)", vp1_pts[0].x_, vp1_pts[0].y_, vp1_pts[1].x_, vp1_pts[1].y_, vp1_pts[2].x_, vp1_pts[2].y_);
  //   ROS_INFO("Point x, y: (%f, %f), (%f, %f), (%f, %f)", vp2_pts[0].x_, vp2_pts[0].y_, vp2_pts[1].x_, vp2_pts[1].y_, vp2_pts[2].x_, vp2_pts[2].y_);

  // Find polygon intersection and polygon area
  double inter_area = suthHodgClip(vp1_pts, vp2_pts);
  if (inter_area != 0) {
    double X[] = {vp1_pt1.x_, vp1_pt2.x_, vp1_pt3.x_};
    double Y[] = {vp1_pt1.y_, vp1_pt2.y_, vp1_pt3.y_};
    double view_area = polygonArea(X, Y, 3);
    return inter_area/view_area;
  }
  return 0.0;
}


} // namespace MPL

#endif

