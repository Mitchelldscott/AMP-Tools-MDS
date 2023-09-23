#include "AMPCore.h"

bool has_point(Eigen::Vector2d point, std::vector<Eigen::Vector2d> vertices, int start=0) {
    for (int i = start; i < vertices.size(); i++) {
    
        if (point == vertices[i]) {
    
            return true;
    
        }
    
    }

    return false;
}

Eigen::Vector2d center_of_mass(std::vector<Eigen::Vector2d> points) {
    int n = points.size();
    float x_avg = 0;
    float y_avg = 0;

    for (int i = 0; i < n; i++) {
        x_avg += points[i][0] / n;
        y_avg += points[i][1] / n;
    }

    return Eigen::Vector2d(x_avg, y_avg);
}

float line_slope(Eigen::Vector2d p1, Eigen::Vector2d p2) {
    return (p2[1] - p1[1]) / (p2[0] - p1[0]);
}

float line_offset(Eigen::Vector2d p, float m) {
    return p[1] - (m * p[0]);
}

float distance(Eigen::Vector2d p1, Eigen::Vector2d p2) {
    return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));
}

struct LineSegment {
    float m;
    float b;
    Eigen::Vector2d point1;
    Eigen::Vector2d point2;
    Eigen::Vector2d x_range;
    Eigen::Vector2d y_range;

    LineSegment(Eigen::Vector2d p1, Eigen::Vector2d p2) {

        point1 = p1;
        point2 = p2; 

        m = line_slope(point1, point2);
        
        if (std::isinf(m) || m == 0) {
        	m = std::abs(m);
        }

        if (std::isinf(b) || b == 0) {
            b = std::abs(b);
        }

        b = line_offset(point1, m);

        // get the bounds on the line segment
        x_range = Eigen::Vector2d(std::min(point1[0], point2[0]), std::max(point1[0], point2[0]));
        y_range = Eigen::Vector2d(std::min(point1[1], point2[1]), std::max(point1[1], point2[1]));
    }
};


void log_line(LineSegment line) {
    LOG("\t\t\t\t <" << line.point1[0] << "," << line.point1[1] << "><" << line.point2[0] << "," << line.point2[1] << ">  " << line.m << "x + " << line.b);
}

std::vector<Eigen::Vector2d> line_intersection(LineSegment line1, LineSegment line2) {
	// LOG("\t\t\tcomparing lines");
	// log_line(line1);
	// log_line(line2);

    std::vector<Eigen::Vector2d> intersections;

	// line segments do not intersect
    if (line1.x_range[1] < line2.x_range[0] || line1.x_range[0] > line2.x_range[1]) {
        return intersections;
    }

    if (line1.y_range[1] < line2.y_range[0] || line1.y_range[0] > line2.y_range[1]) {
        return intersections;
    }

    float x, y;

    // neither slope is infinity so do the normal calculation
    if (!std::isinf(line1.m) && !std::isinf(line2.m)) {
    
        x = (line1.b - line2.b) / (line2.m - line1.m);
        y = (line1.m * x) + line1.b;

    }
    
    // edge slope is infinity so intersect must be at vx[0] == vx[1]
    else if (!std::isinf(line1.m) && std::isinf(line2.m)) {
    
        x = line2.point1[0];
        y = (line1.m * x) + line1.b;
    
    }

    // mline slope is infinity so intersect must be at mx[0] == mx[1]
    else if (std::isinf(line1.m) && !std::isinf(line2.m)) { // (line1.m != line2.m here)
    
        x = line1.point1[0];
        y = (line2.m * x) + line2.b;
    
    }

    else if (std::isinf(line1.m) && std::isinf(line2.m)) { // (line1.m != line2.m here)
    
        x = line1.point1[0];
        y = std::min(line1.y_range[1], line2.y_range[1]);
    
    }

	// LOG("\t\t\t\tFound <" << x << "," << y << ">");
    // LOG("\t\t\t\tmax " << std::min(line1.x_range[1], line2.x_range[1]) << ", " << std::min(line1.y_range[1], line2.y_range[1]));
    // LOG("\t\t\t\tmin " << std::max(line1.x_range[0], line2.x_range[0]) << ", " << std::max(line1.y_range[0], line2.y_range[0]));
    // LOG("\t\t\t\tmax " << (x - std::min(line1.x_range[1], line2.x_range[1])) << ", " << (y - std::min(line1.y_range[1], line2.y_range[1])));
    // LOG("\t\t\t\tmin " << (x - std::max(line1.x_range[0], line2.x_range[0]))  << ", " << (y - std::max(line1.y_range[0], line2.y_range[0])));
    

    // if the intersection point is within the line segment bounds add it to the set
    if (x - std::max(line1.x_range[0], line2.x_range[0]) >= -1E-5 && x - std::min(line1.x_range[1], line2.x_range[1]) <= 1E-5) {

        if (y - std::max(line1.y_range[0], line2.y_range[0]) >= -1E-5 && y - std::min(line1.y_range[1], line2.y_range[1]) <= 1E-5) {

            // LOG("\t\t\t\tFound <" << x << "," << y << ">");
            intersections.push_back(Eigen::Vector2d(x, y));

        }
    
    }
    

    return intersections;

}