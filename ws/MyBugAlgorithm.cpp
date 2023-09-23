#include "MyBugAlgorithm.h"
#include "PolygonCollision.h"

std::vector<PolygonCollision> line_collisions(Eigen::Vector2d pose, Eigen::Vector2d next, std::vector<amp::Polygon> obstacles) {

    std::vector<PolygonCollision> collisions;
    
    for (int i = 0; i < obstacles.size(); i++) { 

        PolygonCollision collision = polygon_intersections(i, pose, next, obstacles[i].verticesCCW());

        if (collision.hit_points.size() > 0) {

            collisions.push_back(collision);
        
        }

    }

    return collisions;
}

bool line_is_safe(int obstacle_hit, Eigen::Vector2d pose, Eigen::Vector2d next, std::vector<amp::Polygon> obstacles) {

    std::vector<int> shared_edges = {};
    std::vector<PolygonCollision> collisions = line_collisions(pose, next, obstacles);

    for (int i = 0; i < collisions.size(); i++) {

        std::vector<Eigen::Vector2d> vertices = obstacles[collisions[i].object].verticesCCW();

        if (collisions[i].hit_points.size() == 0) {
            continue;
        }
    
        if (collisions[i].hit_points.size() >= 1) {
    
            if (!point_is_inside(next, vertices) && !line_overlaps_polygon(pose, next, vertices)) {
                
                if (points_share_edge(pose, next, vertices)) {

                    shared_edges.push_back(i);
                    // LOG("shared edge");
                
                }

                continue;
            
            }
            
        }

        LOG("\t\t\tCollision <" << next[0] << "," << next[1] << "> with " << collisions[i].object << " num " << 
            collisions[i].hit_points.size() << " inside " <<
            point_is_inside(next, vertices));
        LOG("\t\t\tCollision <" << 
            collisions[i].hit_points[0][0] << "," << collisions[i].hit_points[0][1] << "><" << 
            collisions[i].hit_points[1][0] << "," << collisions[i].hit_points[1][1] << ">");

        return false;

    }

    for (int i = 0; i < shared_edges.size(); i++) {
        
        for (int j = i+1; j < shared_edges.size(); j++) {

            if (!polygons_overlap(obstacles[collisions[i].object].verticesCCW(), obstacles[collisions[j].object].verticesCCW())) {

                return false;
            
            }

        }

    }

    return true;
}

std::vector<PolygonCollision> safe_line_collisions(int obstacle_hit, Eigen::Vector2d pose, Eigen::Vector2d next, std::vector<amp::Polygon> obstacles) {

    std::vector<PolygonCollision> safe_collisions;
    std::vector<PolygonCollision> collisions = line_collisions(pose, next, obstacles);
    
    LOG("\t" << collisions.size() << " collisions");

    for (int i = 0; i < collisions.size(); i++) { 

        if (collisions[i].object == obstacle_hit) {
            continue;
        }

        std::vector<Eigen::Vector2d> safe_intersections = {};

        for (int j = 0; j < collisions[i].hit_points.size(); j++) {

            // if (is_ccw(pose, collisions[i].hit_points[j], obstacles[collisions[i].object].verticesCCW())) {

                
            //     continue;

            // }


            if (pose == collisions[i].hit_points[j]) {
                
                Eigen::Vector2d collision_next = endpoint_from_point(collisions[i].hit_points[j], obstacles[collisions[i].object].verticesCCW(), false);
                
                if (!has_point(collision_next, collisions[i].hit_points)) {

                    collisions[i].hit_points[j] = collision_next;
                
                }

                else {
                    continue;
                }
            }


            bool collis = line_is_safe(collisions[i].object, pose, collisions[i].hit_points[j], obstacles);
            LOG("\t\tcheck collision with " << collisions[i].object << ", <" 
            << collisions[i].hit_points[j][0] << "," << collisions[i].hit_points[j][1] << "> safe " 
            << collis);

            if (collis) {
            
                safe_intersections.push_back(collisions[i].hit_points[j]);
            
            }

            else {

                std::vector<PolygonCollision> more_collisions = line_collisions(pose, collisions[i].hit_points[j], obstacles);
                for (int k = 0; k < more_collisions.size(); k++) {
                    bool has_object = false;
                    for (int l = 0; l < collisions.size(); l++) {
                        if (more_collisions[k].object == collisions[l].object) {
                            has_object = true;
                        }
                    }
                    if (!has_object) {
                        collisions.push_back(more_collisions[k]);
                    }
                }
            
            }

        }

        if (safe_intersections.size() > 0) {

            // Collision object for trajectory and obstacle i
            safe_collisions.push_back(PolygonCollision {
                collisions[i].object,
                safe_intersections,
            });

        }

    }

    return safe_collisions;
}


int closest_safe_line_collision(int obstacle_hit, Eigen::Vector2d& pose, Eigen::Vector2d next, std::vector<amp::Polygon> obstacles, std::vector<Eigen::Vector2d>& waypoints) {

    int closest_hit = -1;

    if (!line_is_safe(obstacle_hit, pose, next, obstacles)) {


        std::vector<PolygonCollision> safe_collisions = safe_line_collisions(obstacle_hit, pose, next, obstacles);

        next = pose;
    
        LOG("\t" << safe_collisions.size() << " safe collisions");

        for (int i = 0; i < safe_collisions.size(); i++) { 

            for (int j = 0; j < safe_collisions[i].hit_points.size(); j++) {
    
                if (has_point(safe_collisions[i].hit_points[j], waypoints)) {
                    continue;
                }

                LOG("\t\tcheck distance with " << safe_collisions[i].object 
                    << ", <" << safe_collisions[i].hit_points[j][0] << "," << safe_collisions[i].hit_points[j][1] << "> distances " 
                    << distance(pose, next) << ", " << distance(pose, safe_collisions[i].hit_points[j]));
                
                if (distance(pose, next) == 0 || distance(pose, next) >= distance(pose, safe_collisions[i].hit_points[j])) {
    
    
                    next = safe_collisions[i].hit_points[j];
                    closest_hit = safe_collisions[i].object;
    
    
                    LOG("\t\tclosest <" << next[0] << "," << next[1] << ">");
                
                }
    
            }
    
        }

        if (!line_is_safe(obstacle_hit, pose, next, obstacles)) {
            ERROR("No safe move found " << obstacle_hit << " <" << pose[0] << "," << pose[1] << "><" << next[0] << "," << next[1] << ">");
        }
    
    }
    
    else {
        closest_hit = obstacle_hit;
    }

    // LOG("\tclosest safe collision <" << next[0] << "," << next[1] << "> hit: " << closest_hit);

    pose = next;
    return closest_hit;
}

int move_along_mline(int obstacle_hit, Eigen::Vector2d& curr_pose, Eigen::Vector2d goal, std::vector<amp::Polygon> obstacles, std::vector<Eigen::Vector2d>& waypoints) {
    LOG("");
    LOG("");
    LOG("");
    LOG("Mline candidate <" << curr_pose[0] << "," << curr_pose[1] << "><" << goal[0] << "," << goal[1] << "> hit: " << obstacle_hit);
    return closest_safe_line_collision(obstacle_hit, curr_pose, goal, obstacles, waypoints);
}

int move_along_obstacle(int obstacle_hit, Eigen::Vector2d& curr_pose, std::vector<amp::Polygon> obstacles, std::vector<Eigen::Vector2d>& waypoints) {
    Eigen::Vector2d next = endpoint_from_point(curr_pose, obstacles[obstacle_hit].verticesCCW(), false);
    LOG("");
    LOG("");
    LOG("Obstacle candidate <" << curr_pose[0] << "," << curr_pose[1] << "><" << next[0] << "," << next[1] << "> hit: " << obstacle_hit);
    int hit = closest_safe_line_collision(obstacle_hit, curr_pose, next, obstacles, waypoints);

    if (hit == -1) {
        return obstacle_hit;
    }
    else {
        return hit;
    }
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
//      Repeat                                                                          move_along_obstacle(hit_point)
//          follow boundary recording point curr_pose with shortest distance to goal
//      Until goal is reached or obstacle hit is re-encountered
//      If goal is reached
//          exit
//      Go to obstacle leave                                                            move_along_obstacle(leave_point)
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
    LOG("Initial pose <" << curr_pose[0] << "," << curr_pose[1] << ">");

    int hit = -1;

    // repeat until goal or error
    for (int i = 0; i < 10; i++) { // (curr_pose != problem.q_goal) {
    
        hit = move_along_mline(hit, curr_pose, problem.q_goal, problem2.obstacles, path.waypoints);
        path.waypoints.push_back(curr_pose);
    
        // goal was reached
        if (hit == -1) {
            return path;
        }
        
        else {

            Eigen::Vector2d leave_point;
            std::vector<int> prev_hits = {hit};
            int perimeter = problem2.obstacles[hit].verticesCCW().size();
            Eigen::Vector2d hit_point = endpoint_from_point_repeats(curr_pose, problem2.obstacles[hit].verticesCCW(), true);

            LOG("Hit point <" << hit_point[0] << "," << hit_point[1] << ">");

            for (int i = 0; i < perimeter; i++) {
                hit = move_along_obstacle(hit, curr_pose, problem2.obstacles, path.waypoints);
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

            LOG("Leave point <" << leave_point[0] << "," << leave_point[1] << ">");
            
            for (int i = 0; i < perimeter; i++) {

                std::vector<Eigen::Vector2d> tmp;
                hit = move_along_obstacle(hit, curr_pose, problem2.obstacles, tmp);
                path.waypoints.push_back(curr_pose);
                
                if (hit == -1) {
                    return path;
                }

                if (curr_pose == leave_point) {
                    break;
                }
            }
        }

        // amp::Visualizer::makeFigure(problem2, path);

    }

    return path;
}
