///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Dhruv Madan
//////////////////////////////////////

#include "CollisionChecking.h"

// TODO: Copy your implementation from previous projects


/** 
 * Check if a point is valid (not inside any obstacles)
 * 
 * @param x X coordinate of the point
 * @param y Y coordinate of the point
 * @param obstacles Vector of Rectangle obstacles
 * @return true if the point is valid, false otherwise
 */
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    for (const auto& rect : obstacles) {
        if (x >= rect.x && x <= rect.x + rect.width &&
            y >= rect.y && y <= rect.y + rect.height) {
            return false; // Point is inside an obstacle
        }
    }
    return true; // Point is valid (not inside any obstacle)
}


/** 
 * Check if a circle is valid (not intersecting any obstacles)
 * 
 * @param x X coordinate of the circle center
 * @param y Y coordinate of the circle center
 * @param radius Radius of the circle
 * @param obstacles Vector of Rectangle obstacles
 * @return true if the circle is valid, false otherwise
 */
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    for (const auto& rect : obstacles) {
        // Find the closest point on the rectangle to the circle's center
        double closestX = std::max(rect.x, std::min(x, rect.x + rect.width));
        double closestY = std::max(rect.y, std::min(y, rect.y + rect.height));

        // Calculate the distance from the circle's center to this closest point
        double distanceX = x - closestX;
        double distanceY = y - closestY;
        double distanceSquared = (distanceX * distanceX) + (distanceY * distanceY);

        // If the distance is less than the radius squared, there is a collision
        if (distanceSquared < (radius * radius)) {
            return false; // Circle collides with an obstacle
        }
    }
    return true; // Circle is valid (not colliding with any obstacle)
}

/** 
 * Check if a square is valid (not intersecting any obstacles)
 * 
 * @param x X coordinate of the square center
 * @param y Y coordinate of the square center
 * @param theta Orientation of the square in radians
 * @param sideLength Length of the sides of the square
 * @param obstacles Vector of Rectangle obstacles
 * @return true if the square is valid, false otherwise
 */
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // Calculate half the side length for convenience
    double halfSide = sideLength / 2.0;

    // Calculate the coordinates of the square's corners
    std::vector<std::pair<double, double>> corners(4);
    corners[0] = { x + halfSide * cos(theta) - halfSide * sin(theta), y + halfSide * sin(theta) + halfSide * cos(theta) }; // Top-right
    corners[1] = { x - halfSide * cos(theta) - halfSide * sin(theta), y - halfSide * sin(theta) + halfSide * cos(theta) }; // Top-left
    corners[2] = { x - halfSide * cos(theta) + halfSide * sin(theta), y - halfSide * sin(theta) - halfSide * cos(theta) }; // Bottom-left
    corners[3] = { x + halfSide * cos(theta) + halfSide * sin(theta), y + halfSide * sin(theta) - halfSide * cos(theta) }; // Bottom-right

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
