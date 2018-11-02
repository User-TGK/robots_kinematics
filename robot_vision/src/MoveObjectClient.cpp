#include <MoveObjectClient.hpp>

MoveObjectClient::MoveObjectClient(const supportedShape shape, const geometry_msgs::Point& shapePos, const std::vector<geometry_msgs::Point>& shapeVertices, const geometry_msgs::Point& targetPos): 
    nh(),
    moveObjectClient(nh.serviceClient<robot_controller::MoveObject>("robot_controller_move_object"))
{
    moveObjectSrv.request.shape = (uint8_t)shape;
    moveObjectSrv.request.shape_vertices = shapeVertices;
    moveObjectSrv.request.shape_pos = shapePos;
    moveObjectSrv.request.target_pos = targetPos;
}

MoveObjectClient::~MoveObjectClient()
{
}

void MoveObjectClient::moveObject()
{
    moveObjectClient.call(moveObjectSrv);
}