/* Author: Ali Golestaneh and Constantinos Chamzas */
#include "DiskSampler.h"

/**
 * Check if the state is valid (not in collision with obstacles)
 * 
 * Collision checker for a point and a square
 * Square is of size 2*sqrt(2) and is located in location [-3,-2,] and rotated pi/4 degrees around its center.
 * 
 * @param state: the state to be checked
 * @return true if the state is valid, false otherwise
 */

bool isStateValid(const ob::State *state) {

    // cast the abstract state type to the type we expect
    const auto *r2state = state->as<ob::RealVectorStateSpace::StateType>();
    double x = r2state->values[0]; // Extract the x coordinate in cartesian space
    double y = r2state->values[1]; // Extract the y coordinate in cartesian space
    // A square obstacle with and edge of size 2*sqrt(2) is located in location [-3,-2,] and rotated pi/4 degrees around its center.
    // Fill out this function that returns False when the state is inside/or the obstacle and True otherwise// 

    // ******* START OF YOUR CODE HERE *******//
    // Translate point to origin
    // Center of square is at (-3, -2)
    double translatedX = x + 3; 
    double translatedY = y + 2; 
    // Rotate point by -pi/4 radians
    // rotate to align the point with a straightened square
    double rotatedX = translatedX * cos(-M_PI / 4) - translatedY * sin(-M_PI / 4);
    double rotatedY = translatedX * sin(-M_PI / 4) + translatedY * cos(-M_PI / 4);

    // Check if the point is inside the axis-aligned bounding box of the square
    // The square extends from -sqrt(2) to sqrt(2) in both x and y directions
    if (std::abs(rotatedX) <= sqrt(2) && std::abs(rotatedY) <= sqrt(2))
        return false; // Point is inside the obstacle
    else
        return true; // Point is outside the obstacle

    // ******* END OF YOUR CODE HERE *******//
}

/**
 * sampleNaive
 * 
 * this function will perform naive sampling on a disk of radius 10
 * first sample a random polar coordinate (r, theta)
 * then convert it to cartesian coordinates (x, y)
 * 
 * Then evaluate if the points are uniformly distributed on the disk
 * 
 * @param state: the state to be sampled
 * @return true if the state is valid, false otherwise
 */

bool DiskSampler::sampleNaive(ob::State *state) 
{
    // ******* START OF YOUR CODE HERE *******//

    // Sample a random point in polar coordinates
    double r = rng_.uniformReal(0, 10); // radius between 0 and 10
    double theta = rng_.uniformReal(0, 2 * M_PI); // angle between 0 and 2*pi

    // Convert polar coordinates to cartesian coordinates
    double x = r * cos(theta);
    double y = r * sin(theta);

    // Set the state to the new sampled position
    state->as<ob::RealVectorStateSpace::StateType>()->values[0] = x;
    state->as<ob::RealVectorStateSpace::StateType>()->values[1] = y;

    // ******* END OF YOUR CODE HERE *******//
    
    //The valid state sampler must return false if the state is in-collision
    return isStateValid(state);
}

/**
 * sampleCorrect
 * 
 * this function will perform correct sampling on a disk of radius 10
 * 
 * generate uniformly random points on the disk
 * 
 * Then evaluate if the points are uniformly distributed on the disk
 * 
 * @param state: the state to be sampled
 * @return true if the state is valid, false otherwise
 */
bool DiskSampler::sampleCorrect(ob::State *state)
{
    // ******* START OF YOUR CODE HERE *******//

    // Generate a random point in polar coordinates
    // Utilize the square root of a uniformly distributed random number to ensure uniform distribution over the disk area
    double r = sqrt(rng_.uniformReal(0, 100)); // radius between 0 and 10
    double theta = rng_.uniformReal(0, 2 * M_PI); // angle between 0 and 2*pi

    // Convert polar coordinates to cartesian coordinates
    double x = r * cos(theta);
    double y = r * sin(theta);

    // Set the state to the new sampled position
    state->as<ob::RealVectorStateSpace::StateType>()->values[0] = x;
    state->as<ob::RealVectorStateSpace::StateType>()->values[1] = y;

    // ******* END OF YOUR CODE HERE *******//


    //The valid state sampler must return false if the state is in-collision
    return isStateValid(state);
}
