///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Thomas Herring,
// Date: FILL ME OUT!!
//////////////////////////////////////

#include "CollisionChecking.h"


/*
* Function to check for collision between a point and rectangular obstacles
* Point is represented as (x,y)
* Obstacles are represented as a vector of Rectangle structs
* Returns true if the point does not collide with any obstacles, false otherwise
*
* @param x: x-coordinate of the point
* @param y: y-coordinate of the point
* @param obstacles: vector of Rectangle structs representing the obstacles in the environment
* @return: true if the point is valid (no collision), false otherwise
*/
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    for (const auto& rect : obstacles) {
        if (x >= rect.x && x <= rect.x + rect.width &&
            y >= rect.y && y <= rect.y + rect.height) {
            return false; // Point is inside an obstacle
        }
    }
    return true; // Point is valid (not inside any obstacle)
}

/*  
* Function to check for collision between a square robot and rectangular obstacles

* Robot is represented as a square with center (x,y), orientation theta (in radians),
* and side length sideLength
* Obstacles are represented as a vector of Rectangle structs
* The environment is bounded by hbound and lbound in both x and y directions
* Returns true if the square robot does not collide with any obstacles and is within bounds, false
* otherwise
* 
* @param x: x-coordinate of the center of the square robot
* @param y: y-coordinate of the center of the square robot
* @param theta: orientation of the square robot in radians
* @param sideLength: length of the sides of the square robot
* @param obstacles: vector of Rectangle structs representing the obstacles in the environment
* @param hbound: Dimension of the environment in the x direction
* @param lbound: Dimension of the environment in the y direction
* @return: true if the square robot is valid (no collision and within bounds), false otherwise
*/
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles, double hbound, double lbound)
{
    // Calculate half the side length for convenience
    double halfSide = sideLength / 2.0;

    // Calculate the coordinates of the square's corners
    std::vector<std::pair<double, double>> corners(4);
    corners[0] = { x + halfSide * cos(theta) - halfSide * sin(theta), y + halfSide * sin(theta) + halfSide * cos(theta) }; // Top-right
    corners[1] = { x - halfSide * cos(theta) - halfSide * sin(theta), y - halfSide * sin(theta) + halfSide * cos(theta) }; // Top-left
    corners[2] = { x - halfSide * cos(theta) + halfSide * sin(theta), y - halfSide * sin(theta) - halfSide * cos(theta) }; // Bottom-left
    corners[3] = { x + halfSide * cos(theta) + halfSide * sin(theta), y + halfSide * sin(theta) - halfSide * cos(theta) }; // Bottom-right

    // Check if any corner is out of bounds
    for (const auto& corner : corners) {
        if (corner.first < 0 || corner.first > hbound || corner.second < 0 || corner.second > lbound) {
            return false; // Corner is out of bounds
        }
    }

    // Check for collision with each obstacle
    for (const auto& rect : obstacles) {
        // Check if any corner of the square is inside the rectangle
        for (const auto& corner : corners) {
            if (corner.first >= rect.x && corner.first <= rect.x + rect.width &&
                corner.second >= rect.y && corner.second <= rect.y + rect.height) {
                return false; // Corner is inside an obstacle
            }
        }

        // Check if any edge of the rectangle intersects with any edge of the square
        std::vector<std::pair<double, double>> rectCorners = {
            {rect.x, rect.y}, {rect.x + rect.width, rect.y},
            {rect.x + rect.width, rect.y + rect.height}, {rect.x, rect.y + rect.height}
        };

        for (size_t i = 0; i < 4; ++i) {
            size_t nextI = (i + 1) % 4;
            for (size_t j = 0; j < 4; ++j) {
                size_t nextJ = (j + 1) % 4;
                // Check if edges (corners[i], corners[nextI]) and (rectCorners[j], rectCorners[nextJ]) intersect
                double denom = (corners[nextI].first - corners[i].first) * (rectCorners[nextJ].second - rectCorners[j].second) -
                               (corners[nextI].second - corners[i].second) * (rectCorners[nextJ].first - rectCorners[j].first);
                if (denom == 0) continue; // Parallel lines

                // Calculate intersection parameters
                double ua = ((rectCorners[nextJ].first - rectCorners[j].first) * (corners[i].second - rectCorners[j].second) -
                             (rectCorners[nextJ].second - rectCorners[j].second) * (corners[i].first - rectCorners[j].first)) / denom;
                double ub = ((corners[nextI].first - corners[i].first) * (corners[i].second - rectCorners[j].second) -
                             (corners[nextI].second - corners[i].second) * (corners[i].first - rectCorners[j].first)) / denom;

                if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) {
                    return false; // Edges intersect, collision detected
                }
            }
        }
    }

    return true; // No collision detected
}