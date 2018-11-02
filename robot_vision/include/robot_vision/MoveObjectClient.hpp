#ifndef MOVEOBJECTCLIENT_HPP_
#define MOVEOBJECTCLIENT_HPP_

#include <SpecificationConfig.hpp>
#include <robot_controller/MoveObject.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

class MoveObjectClient
{
public:
    /**
     * @brief Construct a new Move Object Client object
     * 
     * @param shape the shape (uint8_t) that should be picked up and moved
     * @param shapePos the center position (x, y) of the objected to be picked up
     * @param shapeVertices vector of the vertices of the object detected (or radius if its a circle)
     * @param targetPos the position (x, y) where the picked up object should be put down
     */
    MoveObjectClient(const supportedShape shape, const geometry_msgs::Point& shapePos, const std::vector<geometry_msgs::Point>& shapeVertices, const geometry_msgs::Point& targetPos);
    /**
     * @brief Destroy the Move Object Client object
     */
    ~MoveObjectClient();
    /**
     * @brief function that calls the move object services
     */
    void moveObject();

private:
    ros::NodeHandle nh;
    ros::ServiceClient moveObjectClient;
    robot_controller::MoveObject moveObjectSrv;
};

#endif //MOVEOBJECTCLIENT_HPP_