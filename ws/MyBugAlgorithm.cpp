#include "MyBugAlgorithm.h"

float line_slope(Eigen::Vector2d p1, Eigen::Vector2d p2) {
    return (p2[1] - p1[1]) / (p2[0] - p1[0]);
}

float line_offset(Eigen::Vector2d p, float m) {
    return p[1] - (m * p[0]);
}

float distance(Eigen::Vector2d p1, Eigen::Vector2d p2) {
    return pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2);
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

std::vector<Eigen::Vector2d> get_intersections(Eigen::Vector2d pose, Eigen::Vector2d goal, std::vector<Eigen::Vector2d>& vertices) {

    // get the slope and y intercept of the mline
    float m1 = line_slope(pose, goal);
    float b1 = line_offset(pose, m1);

    // get the bounds on the line segment
    Eigen::Vector2d x_range = Eigen::Vector2d(std::min(pose[0], goal[0]), std::max(pose[0], goal[0]));
    Eigen::Vector2d y_range = Eigen::Vector2d(std::min(pose[1], goal[1]), std::max(pose[1], goal[1]));

    // buffer of intersections with obstacle
    std::vector<Eigen::Vector2d> intersections;

    // use the last vertex as the startpoint for the first line
    Eigen::Vector2d prev_vertex = vertices[vertices.size()-1];

    // check if each line in the polygon has an intersection
    for (int i = 0; i < vertices.size(); i++) {

        // get the slope and y intercept of the poygons edge
        float m2 = line_slope(vertices[i], prev_vertex);
        float b2 = line_offset(vertices[i], m2);

        // get the bounds on the line segment
        Eigen::Vector2d v_x_range = Eigen::Vector2d(std::min(vertices[i][0], prev_vertex[0]), std::max(vertices[i][0], prev_vertex[0]));
        Eigen::Vector2d v_y_range = Eigen::Vector2d(std::min(vertices[i][1], prev_vertex[1]), std::max(vertices[i][1], prev_vertex[1]));


        // if slopes are equal either lines never touch or both vertices are on the line and other lines will also intersect
        if (m1 != m2) {

            float x, y;
            
            // neither slope is infinity so do the normal calculation
            if(!std::isinf(m1) && !std::isinf(m2)) {
            
                x = (b1 - b2) / (m2 - m1);
                y =  (m1 * x) + b1;
            
            }
            
            // edge slope is infinity so intersect must be at vx[0] == vx[1]
            else if (!std::isinf(m1) && std::isinf(m2)) {
            
                x = vertices[i][0];
                y = (m1 * x) + b1;
            
            }

            // mline slope is infinity so intersect must be at mx[0] == mx[1]
            else { // (m1 != m2 here)
            
                x = x_range[0];
                y = (m2 * x) + b2;
            
            }

            // if the intersection point is within the line segment bounds add it to the set
            if (x >= v_x_range[0] && x >= x_range[0] && x <= v_x_range[1] && x <= x_range[1]) {

                if (y >= v_y_range[0] && y >= y_range[0] && y <= v_y_range[1] && y <= y_range[1]) {

                    bool skip = false;

                    // make sure the intersection hasn't been recorded already
                    for (int j = 0; j < intersections.size(); j++) { // intersections are at the vertices, not convenient

                        if (abs(intersections[j][0] - x) < 1E-3 && abs(intersections[j][1] - y) < 1E-3) {
                    
                            skip = true;
                    
                        }
                    
                    }

                    if (!skip) {
                    
                        intersections.push_back(Eigen::Vector2d(x, y));
                    
                    }
                
                }
            
            }
        
        }

        prev_vertex = vertices[i];
    }

    return intersections;
}

// struct PolygonCollision {
//     Eigen::Vector2d p1;
//     Eigen::Vector2d p2;
//     std::vector<Eigen::Vector2d> hit_points;
// };

// std::vector<PolygonCollision> line_collisions(Eigen::Vector2d pose, Eigen::Vector2d goal, std::vector<amp::Polygon> obstacles) {

//     std::vector<PolygonCollision> collisions;
    
//     for (int i = 0; i < obstacles.size(); i++) { 

//         // Collision object for trajectory and obstacle i
//         collisions.push_back(PolygonCollision {
//             p1 = pose, 
//             p2 = goal, 
//             hit_points = get_intersections(pose, goal, obstacles[i].verticesCCW())
//         });
//     }

//     return collisions;
// }

// std::vector<PolygonCollision> safe_line_collisions(Eigen::Vector2d pose, Eigen::Vector2d goal, std::vector<amp::Polygon> obstacles) {

//     std::vector<PolygonCollision> collisions;
    
//     for (int i = 0; i < obstacles.size(); i++) { 

//         std::vector<Eigen::Vector2d> intersections = get_intersections(pose, goal, obstacles[i].verticesCCW());

//         for (int j = 0; j < intersections.size(); j++) {

//             if ()

//         }
//         // Collision object for trajectory and obstacle i
//         collisions.push_back(PolygonCollision {
//             p1 = pose, 
//             p2 = goal, 
//             hit_points = get_intersections(pose, goal, obstacles[i].verticesCCW())
//         });
//     }

//     return collisions;
// }


Eigen::Vector2d left_endpoint_from_point(Eigen::Vector2d pose, std::vector<Eigen::Vector2d> vertices) {

    // use the last vertex as the startpoint for the first line
    Eigen::Vector2d prev_vertex = vertices[vertices.size()-1];

    Eigen::Vector2d target = prev_vertex;

    LOG("\tpose <" << pose[0] << "," << pose[1] << ">");

    // check if each line in the polygon has an intersection
    for (int i = 0; i < vertices.size(); i++) {

        // get the slope and y intercept of the poygons edge
        float m = line_slope(vertices[i], prev_vertex);
        float b = line_offset(vertices[i], m);
        LOG("\tedge " << i << " <" << m << "," << b << ">");
        LOG("\ttarget <" << target[0] << "," << target[1] << ">");
        LOG("\tline <" << prev_vertex[0] << "," << prev_vertex[1] << "><" << vertices[i][0] << "," << vertices[i][1] << ">");

        if (pose == vertices[i]) {
            target = prev_vertex;
            LOG("\t\thit coincidence");
            break;
        }

        // if the slope of the edge is not infinite the pose is used to verify the intersection
        if (!std::isinf(m)) {

            if ((m * pose[0]) + b == pose[1] ) {

                target = prev_vertex;
                LOG("\t\thit non-inf");
            
            }

        }

        // if the slope of the edge is infinite and point is on the line the point's x and edge's x coordinates should equal
        else {
            if ((pose[0] == prev_vertex[0] && pose[0] == vertices[i][0])) {

                target = prev_vertex;
                LOG("\t\thit inf");
        
            }

        }

        prev_vertex = vertices[i];
    }

    LOG("\ttarget <" << target[0] << "," << target[1] << ">");

    return target;
}

Eigen::Vector2d right_endpoint_from_point(Eigen::Vector2d pose, std::vector<Eigen::Vector2d> vertices) {

    // use the last vertex as the startpoint for the first line
    Eigen::Vector2d prev_vertex = vertices[vertices.size()-1];

    Eigen::Vector2d target = prev_vertex;

    // check if each line in the polygon has an intersection
    for (int i = 0; i < vertices.size(); i++) {

        // get the slope and y intercept of the poygons edge
        float m = line_slope(vertices[i], prev_vertex);
        float b = line_offset(vertices[i], m);

        // if the point is a vertex get the next vertex
        if (pose == vertices[i]) {
            target = vertices[i+1];
            break;
        }

        // if the slope of the edge is not infinite the pose is used to verify the intersection
        if (!std::isinf(m)) {

            if ((m * pose[0]) + b == pose[1] && distance(pose, vertices[i]) < distance(pose, target)) {

                target = vertices[i];
            
            }

        }

        else {

            if (pose[0] == vertices[i][0] && distance(pose, vertices[i]) < distance(pose, target)) {

                target = vertices[i];
            
            }

        }

        prev_vertex = vertices[i];
    }

    return target;
}

int move_along_mline(int obstacle_hit, Eigen::Vector2d& curr_pose, Eigen::Vector2d goal, std::vector<amp::Polygon> obstacles) {
    
    // Default to moving to the goal
    obstacle_hit = -1;
    Eigen::Vector2d next_pose = goal; 
    
    // Serach obstacles for nearest collision
    for (int i = 0; i < obstacles.size(); i++) { 

        if (i == obstacle_hit) {
            continue;
        }
        
        // Get all collisions with each obstacle
        std::vector<Eigen::Vector2d> intersections = get_intersections(curr_pose, goal, obstacles[i].verticesCCW());

        // If there is an even number of collisions greater than 0, it is a valid candidate
        if (intersections.size() > 0 && (intersections.size() % 2 == 0 || intersections.size() == 1)) {

            // Check all intersections with obstacle
            for (int j = 0; j < intersections.size(); j++) {

                // If the intersection is closer than the current next pose, set it as the next pose 
                if (distance(curr_pose, next_pose) > distance(curr_pose, intersections[j]) && curr_pose != intersections[j]) {
                    
                    next_pose = intersections[j];
                    obstacle_hit = i;
                
                }

            }

        }

        // invalid obstacle case, should we quit here? (maybe started inside obstacle or maybe goal is inside obstacle)
        else if (intersections.size() % 2 != 0) {

            obstacles[i].print();
            ERROR("Dyse-Nav found an odd number of intersections with an obstacle");
        
        }

    }

    curr_pose = next_pose;
    return obstacle_hit;
}

int follow_obstacle(int obstacle_hit, Eigen::Vector2d& curr_pose, std::vector<amp::Polygon> obstacles) {

    int new_hit = obstacle_hit;
    std::vector<Eigen::Vector2d> vertices = obstacles[obstacle_hit].verticesCCW();
    Eigen::Vector2d next = left_endpoint_from_point(curr_pose, obstacles[obstacle_hit].verticesCCW());

    // Serach obstacles for collision
    for (int i = 0; i < obstacles.size(); i++) { 

        if (i == obstacle_hit || i == new_hit) {
            continue;
        }

        // get the slope and y intercept of the mline
        float m = line_slope(curr_pose, next);
        float b = line_offset(curr_pose, m);

        // get the bounds on the line segment
        Eigen::Vector2d x_range = Eigen::Vector2d(std::min(curr_pose[0], next[0]), std::max(curr_pose[0], next[0]));
        Eigen::Vector2d y_range = Eigen::Vector2d(std::min(curr_pose[1], next[1]), std::max(curr_pose[1], next[1]));

        // Get all collisions with each obstacle
        std::vector<Eigen::Vector2d> intersections = get_intersections(curr_pose, next, obstacles[i].verticesCCW());

        // If there is an even number of collisions greater than 0, it is a valid candidate
        if (intersections.size() > 0 && (intersections.size() % 2 == 0 || intersections.size() == 1)) {

            // LOG("Found intersections: " << intersections.size());
            // obstacles[i].print();
            // Check all intersections with obstacle
            for (int j = 0; j < intersections.size(); j++) {

                std::vector<Eigen::Vector2d> new_intersections = get_intersections(curr_pose, next, obstacles[i].verticesCCW());

                // LOG("current distance " << distance(curr_pose, next) << " candidate distance " << distance(curr_pose, intersections[j]));
                // LOG("\tintersection " << i << " <" << intersections[j][0] << "," << intersections[j][1] << ">");
                // If the intersection is closer than the current next pose, set it as the next pose 
                if (distance(curr_pose, next) >= distance(curr_pose, intersections[j]) && curr_pose != intersections[j]) {
                    // LOG("New hit " << i);
                    next = intersections[j];
                    new_hit = i;
                
                }

                else if (curr_pose == intersections[j]) {
                    LOG("\tNew hit " << i);
                    next = left_endpoint_from_point(intersections[j], obstacles[i].verticesCCW());
                    new_hit = i;
                    i = 0;
                    break;
                }

            }

        }

        // invalid obstacle case, should we quit here? (maybe started inside obstacle or maybe goal is inside obstacle)
        else if (intersections.size() % 2 != 0) {

            obstacles[i].print();
            ERROR("Dyse-Nav found an odd number of intersections with an obstacle");
        
        }

    }

    curr_pose = next;
    return new_hit;
}

std::vector<amp::Polygon> inflate_obstacles(float scale, std::vector<amp::Polygon> obstacles) {
    std::vector<amp::Polygon> inflated_obstacles;

    for (int i = 0; i < obstacles.size(); i++) {

        std::vector<Eigen::Vector2d> inflated_points;
        std::vector<Eigen::Vector2d> vertices = obstacles[i].verticesCCW();
        Eigen::Vector2d com = center_of_mass(vertices);
        
        for (int j = 0; j < vertices.size(); j++) {
            Eigen::Vector2d new_vertex = vertices[j] - com;
            new_vertex[0] += (new_vertex[0] / std::abs(new_vertex[0])) * scale;
            new_vertex[1] += (new_vertex[1] / std::abs(new_vertex[1])) * scale;
            inflated_points.push_back(new_vertex + com);
        }

        inflated_obstacles.push_back(amp::Polygon(inflated_points));
        inflated_obstacles[i].print();

    }

    return inflated_obstacles;
}

amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
//  Let curr_pose = start; i = 1
//  Repeat
//      Repeat                                                                          move_along_m_line()
//          from curr_pose move toward goal
//      Until goal is reached or obstacle hit encountered
//      If goal is reached,
//          exit
//      Repeat                                                                          circumnavigate()
//          follow boundary recording point curr_pose with shortest distance to goal
//      Until goal is reached or obstacle hit is re-encountered
//      If goal is reached
//          exit
//      Go to obstacle leave                                                            retrace_to_closest()
//      If move toward goal moves into obstacle (part of mline)
//          exit with failure
//      Else
//          i=i+1
//          continue

    amp::Problem2D problem2 = problem;
    problem2.obstacles = inflate_obstacles(0.25, problem.obstacles);

    // return object to fill with path
    amp::Path2D path;

    // current pose of the bug
    Eigen::Vector2d curr_pose = problem.q_init;

    // start path at q_init
    path.waypoints.push_back(curr_pose);
    LOG("Initial pose: <" << curr_pose[0] << "," << curr_pose[1] << ">");

    int hit = -1;

    // repeat until goal or error
    for (int i = 0; i < 10; i++) { // (curr_pose != problem.q_goal) {
    
        hit = move_along_mline(hit, curr_pose, problem.q_goal, problem2.obstacles);
        path.waypoints.push_back(curr_pose);
        LOG("M line step: " << hit << " <" << curr_pose[0] << "," << curr_pose[1] << ">");
    
        // goal was reached
        if (hit == -1) {
            return path;
        }
        
        else {

            Eigen::Vector2d hit_point = right_endpoint_from_point(curr_pose, problem2.obstacles[hit].verticesCCW());
            Eigen::Vector2d leave_point = curr_pose;
            std::vector<int> prev_hits = {hit};
            int perimeter = problem2.obstacles[hit].verticesCCW().size();

            for (int i = 0; i < perimeter; i++) {

                hit = follow_obstacle(hit, curr_pose, problem2.obstacles);
                path.waypoints.push_back(curr_pose);

                bool new_hit = true;
                for (int j = 0; j < prev_hits.size(); j++) {
                    if (prev_hits[j] == hit) {
                        new_hit = false;
                        break;
                    }
                }

                if (new_hit) {
                    prev_hits.push_back(hit);
                    perimeter += problem2.obstacles[hit].verticesCCW().size();
                }

                LOG("Searching: " << hit << " <" << curr_pose[0] << "," << curr_pose[1] << ">, hit point <" << hit_point[0] << "," << hit_point[1] << ">");

                if (distance(leave_point, problem.q_goal) > distance(curr_pose, problem.q_goal)) {
                    leave_point = curr_pose;
                }
                
                if (hit == -1) {
                    return path;
                }

                if (curr_pose == hit_point) {
                    break;
                }
            }

            for (int i = 0; i < perimeter; i++) {

                hit = follow_obstacle(hit, curr_pose, problem2.obstacles);
                path.waypoints.push_back(curr_pose);

                LOG("Backtracking " << hit << " : <" << curr_pose[0] << "," << curr_pose[1] << ">, leave point <" << leave_point[0] << "," << leave_point[1] << ">");
                
                if (hit == -1) {
                    return path;
                }

                if (curr_pose == leave_point) {
                    break;
                }
            }
        }

        amp::Visualizer::makeFigure(problem2, path);

    }

    return path;
}
