#include "AMPCore.h"
#include "LineSegment.h"

struct PolygonCollision {
    int object;
    std::vector<Eigen::Vector2d> hit_points;
};

PolygonCollision polygon_intersections(int hit, Eigen::Vector2d point1, Eigen::Vector2d point2, std::vector<Eigen::Vector2d>& vertices) {

    LineSegment l1(point1, point2);
    // log_line(l1);

    // buffer of intersections with obstacle
    std::vector<Eigen::Vector2d> intersections;

    // use the last vertex as the startpoint for the first line
    Eigen::Vector2d prev_vertex = vertices[vertices.size()-1];


    // LOG("Searching <" << point1[0] << "," << point1[1] << "><" << point2[0] << "," << point2[1] << ">  " << line1.m << "x + " << b1);
    // check if each line in the polygon has an intersection
    for (int i = 0; i < vertices.size(); i++) {

        LineSegment l2(vertices[i], prev_vertex);
        // log_line(l2);

        std::vector<Eigen::Vector2d> new_intersection = line_intersection(l1, l2);


        if (new_intersection.size() > 0) {
        
            
            if (!has_point(prev_vertex, intersections)) {
                intersections.push_back(prev_vertex);
            }
            
            if (!has_point(new_intersection[0], intersections)) {
                intersections.push_back(new_intersection[0]);
            }

            if (!has_point(vertices[i], intersections)) {
                intersections.push_back(vertices[i]);
            }
        
        }
        

        prev_vertex = vertices[i];
    }

    return PolygonCollision {
            hit,
            intersections,
        };
}

bool is_ccw(Eigen::Vector2d point1, Eigen::Vector2d point2, std::vector<Eigen::Vector2d>& vertices) {
    Eigen::Vector2d com = center_of_mass(vertices);
    return ((point2[0] - com[0]) * (point1[1] - com[1])) - ((point2[1] - com[1]) * (point1[0] - com[0])) > 0;
}

bool point_is_inside(Eigen::Vector2d point, std::vector<Eigen::Vector2d>& vertices) {
    for (int i = 0; i < vertices.size(); i++) {
        if (point == vertices[i]) {
            return false;
        }
    }
    
    PolygonCollision collision = polygon_intersections(-1, point, center_of_mass(vertices), vertices);
    // LOG("\t\t\t\tis inside found " << collision.hit_points.size() << " intersections");
    return collision.hit_points.size() == 0;
}

bool line_overlaps_polygon(Eigen::Vector2d point1, Eigen::Vector2d point2, std::vector<Eigen::Vector2d>& vertices) {

    LineSegment l1(point1, point2);

    Eigen::Vector2d com = center_of_mass(vertices);

    for (int i = 0; i < vertices.size(); i++) {
        
        LineSegment l2(com, vertices[i]);

        // line segments do not intersect (exclude)
        if (l2.x_range[1] <= l1.x_range[0] || l2.x_range[0] >= l1.x_range[1]) {
            continue;
        }

        if (l2.y_range[1] <= l1.y_range[0] || l2.y_range[0] >= l1.y_range[1]) {
            continue;
        }

        std::vector<Eigen::Vector2d> new_intersections = line_intersection(l1, l2);

        if (new_intersections.size() > 0) {

            return true;
        
        }

    }

    return false;
}

bool polygons_overlap(std::vector<Eigen::Vector2d>& vertices1, std::vector<Eigen::Vector2d>& vertices2) {

    Eigen::Vector2d com1 = center_of_mass(vertices1);
    Eigen::Vector2d com2 = center_of_mass(vertices2);

    for (int i = 0; i < vertices1.size(); i++) {
        
        if (line_overlaps_polygon(com1, vertices1[i], vertices2)) {
        
            return true;
        
        }

    }

    return false;
}

bool points_share_edge(Eigen::Vector2d point1, Eigen::Vector2d point2, std::vector<Eigen::Vector2d>& vertices) {

    Eigen::Vector2d prev_vertex = vertices[vertices.size()-1];

    LineSegment line1(point1, point2);
    // log_line(line1);
    
    for (int i = 0; i < vertices.size(); i++) {

        LineSegment line2(vertices[i], prev_vertex);
        // log_line(line2);

        if ((point1 == prev_vertex && point2 == vertices[i]) || (point2 == prev_vertex && point1 == vertices[i])) {
            // LOG("\t\t\t\tSame line");
            return true;
        }

        if (line1.m == line2.m && line1.b == line2.b) {
            // LOG("\t\t\t\tSimilar lines");
            if (std::isinf(line1.m) && std::isinf(line2.m)) {
            
                if (line1.x_range[0] == line2.x_range[1]) {
            
                    if (line1.y_range[0] < line2.y_range[1] && line1.y_range[1] > line2.y_range[0]) {
            
                        return true;
            
                    }
            
                }
            
            }

            else if (!std::isinf(line1.m) && !std::isinf(line2.m)) {
            
                if (line1.x_range[0] < line2.x_range[1] && line1.x_range[1] > line2.x_range[0]) {
            
                    if (line1.m == 0) {
                        return true;
                    }

                    if (line1.y_range[0] < line2.y_range[1] && line1.y_range[1] > line2.y_range[0]) {
            
                        return true;
            
                    }
            
                }
            
            }

        }

        prev_vertex = vertices[i];

    }

    return false;

}

Eigen::Vector2d endpoint_from_point_repeats(Eigen::Vector2d pose, std::vector<Eigen::Vector2d> vertices, bool right_turning) {

    // use the last vertex as the startpoint for the first line
    Eigen::Vector2d prev_vertex = vertices[vertices.size()-1];

    // LOG("\tpose <" << pose[0] << "," << pose[1] << ">");

    if (pose == prev_vertex) {
        return prev_vertex;
    }

    // check if each line in the polygon has an intersection
    for (int i = 0; i < vertices.size(); i++) {

        // get the slope and y intercept of the poygons edge
        float m = line_slope(vertices[i], prev_vertex);
        float b = line_offset(vertices[i], m);
        // LOG("\tedge " << i << " <" << m << "," << b << ">");
        // LOG("\ttarget <" << target[0] << "," << target[1] << ">");
        // LOG("\tline <" << prev_vertex[0] << "," << prev_vertex[1] << "><" << vertices[i][0] << "," << vertices[i][1] << ">");

        if (pose == vertices[i]) {
            return vertices[i];
        }

        // if the slope of the edge is not infinite the pose is used to verify the intersection
        if (!std::isinf(m)) {

            if ((m * pose[0]) + b == pose[1] ) {

                // LOG("\t\thit non-inf");

                if (right_turning) {

                    return vertices[i];
                
                }
                
                else {
                
                    return prev_vertex;
                
                }
            
            }

        }

        // if the slope of the edge is infinite and point is on the line the point's x and edge's x coordinates should equal
        else {

            if ((pose[0] == prev_vertex[0] && pose[0] == vertices[i][0])) {

                // LOG("\t\thit inf");

                if (right_turning) {

                    return vertices[i];
                
                }
                
                else {
                
                    return prev_vertex;
                
                }
        
            }

        }

        prev_vertex = vertices[i];
    }

    // LOG("\ttarget <" << target[0] << "," << target[1] << ">");

    return pose;
}


Eigen::Vector2d endpoint_from_point(Eigen::Vector2d pose, std::vector<Eigen::Vector2d> vertices, bool right_turning) {

    // use the last vertex as the startpoint for the first line
    Eigen::Vector2d prev_vertex = vertices[vertices.size()-1];

    if (!right_turning && prev_vertex == pose) {
        return vertices[vertices.size()-2];
    }

    // LOG("\tpose <" << pose[0] << "," << pose[1] << ">");

    // check if each line in the polygon has an intersection
    for (int i = 0; i < vertices.size(); i++) {

        // get the slope and y intercept of the poygons edge
        float m = line_slope(vertices[i], prev_vertex);
        float b = line_offset(vertices[i], m);
        // LOG("\tedge " << i << " <" << m << "," << b << ">");
        // LOG("\ttarget <" << target[0] << "," << target[1] << ">");
        // LOG("\tline <" << prev_vertex[0] << "," << prev_vertex[1] << "><" << vertices[i][0] << "," << vertices[i][1] << ">");

        if (pose == vertices[i]) {

            // LOG("\t\thit coincidence");

            if (right_turning) {

                return vertices[(i+1) % vertices.size()];
            
            }
            
            else {
            
                return prev_vertex;
            
            }

        }

        // if the slope of the edge is not infinite the pose is used to verify the intersection
        if (!std::isinf(m)) {

            if ((m * pose[0]) + b == pose[1] ) {

                // LOG("\t\thit non-inf");

                if (right_turning) {

                    return vertices[i];
                
                }
                
                else {
                
                    return prev_vertex;
                
                }
            
            }

        }

        // if the slope of the edge is infinite and point is on the line the point's x and edge's x coordinates should equal
        else {

            if ((pose[0] == prev_vertex[0] && pose[0] == vertices[i][0])) {

                // LOG("\t\thit inf");

                if (right_turning) {

                    return vertices[i];
                
                }
                
                else {
                
                    return prev_vertex;
                
                }
        
            }

        }

        prev_vertex = vertices[i];
    }

    // LOG("\ttarget <" << target[0] << "," << target[1] << ">");

    return pose;
}