#include <FilterExecutor.hpp>
#include <MoveObjectClient.hpp>

#include <iostream>
#include <opencv2/aruco.hpp>
#include <map>
#include <cmath>

FilterExecutor::FilterExecutor(const DisplayMode& aDisplayMode):
    displayMode(aDisplayMode), 
    realTopLeft(cv::Point3f(-0.126, 0.166, 0)),
    realTopRight(cv::Point3f(0.132, 0.163, 0)),
    realBottomLeft(cv::Point3f(-0.136, 0.265, 0)),
    realBottomRight(cv::Point3f(0.132, 0.265, 0))
{
    cv::FileStorage fs((ros::package::getPath("robot_vision") + "/calibration/out_camera_data.xml").c_str(), cv::FileStorage::READ);
    fs["camera_matrix"] >> this->cameraMatrix;
    fs["distortion_coefficients"] >> this->distCoeffs;
    fs.release();
}

FilterExecutor::~FilterExecutor()
{
}

void FilterExecutor::computeC2MC1(const cv::Mat &R1, const cv::Mat &tvec1, const cv::Mat &R2, const cv::Mat &tvec2,
                  cv::Mat &R_1to2, cv::Mat &tvec_1to2)
{
    R_1to2 = R2 * R1.t();
    tvec_1to2 = R2 * (-R1.t()*tvec1) + tvec2;
}

cv::Mat FilterExecutor::filter(const cv::Mat& original, const Specification& specification, bool& found, std::vector<FilterDetails>& result, std::clock_t& clockTicks)
{   
    std::clock_t totalClockTicks = 0;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::Mat undistorted = original;

    cv::Mat img_bird_eye_view = undistorted.clone();

    cv::Mat R_desired = (cv::Mat_<double>(3,3) <<
                    1, 0, 0,
                    0, 1, 0,
                    0, 0, 1);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat normal = (cv::Mat_<double>(3,1) << 0, 0, 1);
    cv::Mat normal1 = R*normal;
    cv::Mat origin(3, 1, CV_64F, cv::Scalar(0));
    cv::Mat origin1 = R*origin + tvec;
    double d_inv1 = 1.0 / normal1.dot(origin1);
    cv::Mat R_1to2, tvec_1to2;
    cv::Mat tvec_desired = tvec.clone();

    computeC2MC1(R, tvec, R_desired, tvec_desired, R_1to2, tvec_1to2);
    cv::Mat H = R_1to2 + d_inv1 * tvec_1to2*normal1.t();
    H = cameraMatrix * H * cameraMatrix.inv();
    H = H/H.at<double>(2,2);

    cv::warpPerspective(undistorted, img_bird_eye_view, H, undistorted.size());
    cv::Mat compare;
    
    // Extraction process
    cv::Mat extractedImage = extractor.extract(img_bird_eye_view, specification.color);
    
    // Finding all contours of the shapes in the extrated image
    cv::findContours(extractedImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Start counting clock ticks for detection process
    auto detectionTimerStart = std::clock();

    // For each detected shape: determine if the shape is the requested shape (from the given specification)
    for (unsigned int detectedShape = 0; detectedShape < contours.size(); ++detectedShape)
    {
        FilterDetails filterDetails;
        auto startDetectionShape = std::clock();

        if (isObject(specification.shape, contours, detectedShape, filterDetails))
        {
            auto endDetectionShape = std::clock();
            filterDetails.clockTicks = endDetectionShape - startDetectionShape;
            
            // Drawing borders and displaying information
            cv::Scalar color = cv::Scalar(255, 0, 135);
            cv::drawContours(extractedImage, contours, detectedShape, color, 2, 8, hierarchy, 0, cv::Point());

            result.push_back(filterDetails);
        }
    }

    auto detectionTimerEnd = std::clock();
    totalClockTicks = detectionTimerEnd - detectionTimerStart;
    clockTicks = totalClockTicks;

    if (result.size() >= 1)
    {
        found = true;
    }

    return img_bird_eye_view;
}

bool FilterExecutor::isObject(const supportedShape& shape, const std::vector<std::vector<cv::Point>>& contours, 
                              int index, FilterDetails& filterDetails) const
{
    auto possibleShape = detector.detectShape(contours[index]);

    Parser parser;

    if (!possibleShape.has_value())
    {
        return false;
    }

    if (possibleShape != shape)
    {
        if (shape == supportedShape::halfCircle && possibleShape == supportedShape::rectangle)
        {
            if (detector.checkIfRect(contours, index, filterDetails)) { return false; }
            else { return detector.checkIfHalfCircle(contours, index, filterDetails); }
        }
        else if (shape == supportedShape::rectangle && possibleShape == supportedShape::halfCircle)
        {
            if (detector.checkIfRect(contours, index, filterDetails)) { return true; }
            else { return false; }
        }
        else if (shape == supportedShape::rectangle && possibleShape == supportedShape::triangle)
        {
            if (detector.checkIfTriangle(contours, index, filterDetails)) { return false;}
            else { return detector.checkIfRect(contours, index, filterDetails); }
        }
        else if (shape == supportedShape::triangle && possibleShape == supportedShape::rectangle)
        {
            if (detector.checkIfRect(contours, index, filterDetails)) { return false; }
            else { return detector.checkIfTriangle(contours, index, filterDetails); }
        }
        else if (shape == supportedShape::rectangle && possibleShape == supportedShape::square)
        {
            return detector.checkIfRect(contours, index, filterDetails);
        }
        else if (shape == supportedShape::square && possibleShape == supportedShape::rectangle)
        {
            return detector.checkIfSquare(contours, index, filterDetails);
        }
        return false;
    }

    switch (shape)
    {
        case supportedShape::circle:
        {
            return detector.checkIfCircle(contours, index, filterDetails);
        }
        case supportedShape::halfCircle:
        {
            return detector.checkIfHalfCircle(contours, index, filterDetails);
        }
        case supportedShape::square:
        {
            return detector.checkIfSquare(contours, index, filterDetails);
        }
        case supportedShape::triangle:
        {
            return detector.checkIfTriangle(contours, index, filterDetails);
        }
        case supportedShape::rectangle:
        {
            return detector.checkIfRect(contours, index, filterDetails);
        }
    }
    return false;
}

const geometry_msgs::Point FilterExecutor::getTargetPos(const cv::Mat& frame)
{
    Specification calibrationSpecification;
    geometry_msgs::Point targetPos;
    calibrationSpecification.color = supportedColor::white;
    calibrationSpecification.shape = supportedShape::circle;

    std::vector<FilterDetails> targetResult;
    std::clock_t clockTicks;
    bool found;

    auto filteredFrame = this->filter(frame, calibrationSpecification, found, targetResult, clockTicks);

    if (targetResult.size() < 1)
    {
        throw std::runtime_error("ONE TARGET SHOULD BE PLACED: (NOT 0 OR MORE THAN 1).");
    }
    
    targetPos.x = targetResult[0].xPosCenter;
    targetPos.y = targetResult[0].yPosCenter;
    targetPos.z = 0;
    return targetPos;
}

bool FilterExecutor::setCalibrationSquares(cv::Mat& frame)
{
    cv::Mat extractedImage = extractor.extract(frame, supportedColor::white);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<FilterDetails> squareResult;

    cv::findContours(extractedImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    for (unsigned int detectedShape = 0; detectedShape < contours.size(); ++detectedShape)
    {
        FilterDetails filterDetails;

        if (isObject(supportedShape::square, contours, detectedShape, filterDetails))
        {
            squareResult.push_back(filterDetails);
        }
    }
    if (squareResult.size() != 4)
    {
        ROS_ERROR("Four calibration squares should be placed.");
        std::cout << "found: " << squareResult.size() << std::endl;
        return false;
    }

    std::sort(squareResult.begin(), squareResult.end(), [](
                FilterDetails filterDetails, FilterDetails filterDetailsRef){
                    return filterDetails.yPosCenter < filterDetailsRef.yPosCenter;} );

    ROS_INFO("CALIBRATED");

    cv::Point2f topLeft, topRight, bottomLeft, bottomRight;

    if (squareResult[0].xPosCenter > squareResult[1].xPosCenter)
	{
		topRight.x = squareResult[0].xPosCenter;
        topRight.y = squareResult[0].yPosCenter;
		topLeft.x = squareResult[1].xPosCenter;
        topLeft.y = squareResult[1].yPosCenter;
	}
	else
	{
		topRight.x = squareResult[1].xPosCenter;
        topRight.y = squareResult[1].yPosCenter;
		topLeft.x = squareResult[0].xPosCenter;
        topLeft.y = squareResult[0].yPosCenter;
	}
	if (squareResult[2].xPosCenter > squareResult[3].xPosCenter)
	{
		bottomRight.x = squareResult[2].xPosCenter;
        bottomRight.y = squareResult[2].yPosCenter;
		bottomLeft.x = squareResult[3].xPosCenter;
        bottomLeft.y = squareResult[3].yPosCenter;
	}
	else
	{
		bottomRight.x = squareResult[3].xPosCenter;
        bottomRight.y = squareResult[3].yPosCenter;
		bottomLeft.x = squareResult[2].xPosCenter;
        bottomLeft.y = squareResult[2].yPosCenter;
	}
    
    std::cout << "TOPLEFT: x: " << topLeft.x << " y: " << topLeft.y << std::endl;
    std::cout << "TOPRIGHT: x: " << topRight.x << " y: " << topRight.y << std::endl;
    std::cout << "BOTTOMLEFT: x: " << bottomLeft.x << " y: " << bottomLeft.y << std::endl;
    std::cout << "BOTTOMRIGHT: x: " << bottomRight.x << " y: " << bottomRight.y << std::endl;

    corners.push_back(topLeft);
    corners.push_back(topRight);
    corners.push_back(bottomLeft);
    corners.push_back(bottomRight);
    
    return true;
}

bool FilterExecutor::setPixelPerMeter(const cv::Mat& frame)
{
    Specification calibrationSpecification;
    calibrationSpecification.color = supportedColor::white;
    calibrationSpecification.shape = supportedShape::square;

    std::vector<FilterDetails> squareResult;
    std::clock_t clockTicks;
    bool found;
    auto filteredFrame = this->filter(frame, calibrationSpecification, found, squareResult, clockTicks);

    if (squareResult.size() != 4)
    {
        ROS_ERROR("PIXEL PER METER Four calibration squares should be placed.");
        std::cout << "found: " << squareResult.size() << std::endl;
        return false;
    }

    std::sort(squareResult.begin(), squareResult.end(), [](
                FilterDetails filterDetails, FilterDetails filterDetailsRef){
                    return filterDetails.yPosCenter < filterDetailsRef.yPosCenter;} );

    cv::Point2f topLeft, topRight, bottomLeft, bottomRight;

    if (squareResult[0].xPosCenter > squareResult[1].xPosCenter)
	{
		topRight.x = squareResult[0].xPosCenter;
        topRight.y = squareResult[0].yPosCenter;
		topLeft.x = squareResult[1].xPosCenter;
        topLeft.y = squareResult[1].yPosCenter;
	}
	else
	{
		topRight.x = squareResult[1].xPosCenter;
        topRight.y = squareResult[1].yPosCenter;
		topLeft.x = squareResult[0].xPosCenter;
        topLeft.y = squareResult[0].yPosCenter;
	}
	if (squareResult[2].xPosCenter > squareResult[3].xPosCenter)
	{
		bottomRight.x = squareResult[2].xPosCenter;
        bottomRight.y = squareResult[2].yPosCenter;
		bottomLeft.x = squareResult[3].xPosCenter;
        bottomLeft.y = squareResult[3].yPosCenter;
	}
	else
	{
		bottomRight.x = squareResult[3].xPosCenter;
        bottomRight.y = squareResult[3].yPosCenter;
		bottomLeft.x = squareResult[2].xPosCenter;
        bottomLeft.y = squareResult[2].yPosCenter;
	}
    topLeftSquareCenter = topLeft;
    topRightSquareCenter = topRight;
    bottomLeftSquareCenter = bottomLeft;
    bottomRightSquareCenter = bottomRight;

    pixelPerMeter += std::fabs(topLeft.y - bottomLeft.y) / std::fabs(realTopLeft.y - realBottomLeft.y);
    pixelPerMeter += std::fabs(topRight.y - bottomRight.y) / std::fabs(realTopRight.y - realBottomRight.y);
    pixelPerMeter += std::fabs(topLeft.x - topRight.x) / std::fabs(realTopLeft.x - realTopRight.x);
    pixelPerMeter += std::fabs(bottomLeft.x - bottomRight.x) / std::fabs(realBottomLeft.x - realBottomRight.x);

    pixelPerMeter = (pixelPerMeter / 4);
    std::cout << "pixelpermeter: " << pixelPerMeter << std::endl;

    return true;
}

bool FilterExecutor::calibrateCameraPos()
{
    objectPoints.push_back(realTopLeft);
    objectPoints.push_back(realTopRight);
    objectPoints.push_back(realBottomLeft);
    objectPoints.push_back(realBottomRight);

    cv::solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
    return true;
}

const geometry_msgs::Point FilterExecutor::getRealCoordinate(const geometry_msgs::Point& coordinate) const
{
    geometry_msgs::Point realCoordinate;

    auto offsetXFromTopLeft = (coordinate.x - topLeftSquareCenter.x) / pixelPerMeter;
    auto offsetYFromTopLeft = (coordinate.y - topLeftSquareCenter.y) / pixelPerMeter;
    
    realCoordinate.x = realTopLeft.y + offsetYFromTopLeft;
    realCoordinate.y = -0.040;
    realCoordinate.z = realTopLeft.x + offsetXFromTopLeft;
    
    return realCoordinate;
}

void FilterExecutor::showResult(cv::Mat& original, const Specification& originalSpec, const std::clock_t& clockTicks, const geometry_msgs::Point targetPos, const std::vector<FilterDetails>& result) const
{
    Parser parser;
    cv::circle(original, topLeftSquareCenter, 4, cv::Scalar(0, 0, 0), 1, 8);
    cv::circle(original, topRightSquareCenter, 4, cv::Scalar(0, 0, 0), 1, 8);
    cv::circle(original, bottomLeftSquareCenter, 4, cv::Scalar(0, 0, 0), 1, 8);
    cv::circle(original, bottomRightSquareCenter, 4, cv::Scalar(0, 0, 0), 1, 8);
    cv::circle(original, cv::Point(result[0].xPosCenter, result[0].yPosCenter), 4, cv::Scalar(255, 0, 0), 1, 8);
    cv::circle(original, cv::Point(targetPos.x, targetPos.y), 4, cv::Scalar(255, 0, 0), 1, 8);

    cv::imshow("Bird eye view", original);
    cv::waitKey(2000);

    switch (this->displayMode)
    {
        case DisplayMode::toConsole:
        {
            if (result.empty())
            {
                std::cout   << "Er zijn geen objecten gedetecteerd die voldeden aan de specificatie: [" 
                            << parser.strFromShape(originalSpec.shape) << "] [" << parser.strFromColor(originalSpec.color) << "]. "
                            << "Aantal klok tikken: " << clockTicks << "." << std::endl;
            }
            else 
            {
                for (const auto& details : result)
                {
                    std::cout   << parser.strFromShape(originalSpec.shape) << " gedetecteerd met een oppervlakte van: " << details.surface 
                                << " en het middelpunt (x, y) op (" << details.xPosCenter << "," << details.yPosCenter << "). " 
                                << "Het aantal klok tikken was: " << details.clockTicks << "." << std::endl;
                }
            }
            break;
        }
        case DisplayMode::inImage:
        {
            auto textColor = cv::Scalar(50, 39, 86);
            double textSize = 0.47;
            int textWidth = 2;

            if (result.empty())
            {
                std::string textA = "Er zijn geen objecten gedetecteerd die voldeden aan de specificatie:";
                std::string textB = "[" + parser.strFromShape(originalSpec.shape) + "] [" + parser.strFromColor(originalSpec.color)
                                    + "]. Aantal klok tikken: " + std::to_string(clockTicks) + ".";
                cv::putText(original, textA, cv::Point(25, 25),
                cv::FONT_ITALIC, textSize, textColor, textWidth);

                cv::putText(original, textB, cv::Point(25, 50),
                cv::FONT_ITALIC, textSize, textColor, textWidth);
            }
            else
            {
                for (const auto& details : result)
                {
                    std::string textA = "Oppervlakte: " + std::to_string(int(details.surface)) + ".";
                    std::string textB = "X en Y: " +
                    std::to_string(int(details.xPosCenter)) + " , " + std::to_string(int(details.yPosCenter)) + 
                    ".";
                    std::string textC = "Klok tikken: " + std::to_string(details.clockTicks) + ".";
                    
                    cv::putText(original, textA, cv::Point(details.xPosCenter - 15, details.yPosCenter),
                    cv::FONT_ITALIC, textSize, textColor, textWidth);

                    cv::putText(original, textB, cv::Point(details.xPosCenter - 15, details.yPosCenter+14),
                    cv::FONT_ITALIC, textSize, textColor, textWidth);

                    cv::putText(original, textC, cv::Point(details.xPosCenter - 15, details.yPosCenter+28),
                    cv::FONT_ITALIC, textSize, textColor, textWidth);
                }
            }
            break;
        }
        case DisplayMode::toRobot:
        {
            if (result.empty())
            {
                std::cout   << "Er zijn geen objecten gedetecteerd die voldeden aan de specificatie: [" 
                            << parser.strFromShape(originalSpec.shape) << "] [" << parser.strFromColor(originalSpec.color) << "]. "
                            << "Aantal klok tikken: " << clockTicks << "." << std::endl;
            }
            else
            {
                for (const auto& details: result)
                {
                    geometry_msgs::Point shapePos, newTargetPos;
                    std::vector<geometry_msgs::Point> shapeVertices = details.shapeVertices;
                    
                    shapePos.x = details.xPosCenter;
                    shapePos.y = details.yPosCenter;

                    std::cout << "OPENCV COORDINATES: x: " << shapePos.x << "y: " << shapePos.y << std::endl;

                    shapePos = getRealCoordinate(shapePos);
                    newTargetPos = getRealCoordinate(targetPos);

                    for (auto& point: shapeVertices)
                    {
                        point = getRealCoordinate(point);
                    }
                    std::cout << "service called" << std::endl;
                    MoveObjectClient client(originalSpec.shape, shapePos, shapeVertices, newTargetPos);
                    
                    client.moveObject();
                    std::cout << "Service called with the following values: " << '\n' << "Shape position: (" << 
                    shapePos.x << ", " << shapePos.y << ", " << shapePos.z << ")" << '\n' << "Target position: (" << newTargetPos.x <<
                    ", " << newTargetPos.y << ", " << newTargetPos.z << ")" << '\n' << "Shape: " << " (" << parser.strFromShape(originalSpec.shape) << ")" << std::endl; 
                    
                    std::cout << "Vertices: " << std::endl;
                    uint16_t i = 0;
                    for (const auto& p: shapeVertices)
                    {
                        std::cout << "ID: " << i << " x: " << p.x << " y: " << p.y << " z: " << p.z << std::endl;
                        i++;
                    }
                    if (result.size() > 1)
                    {
                        ROS_INFO("Since more than one object was detected, the one with the lowest ID will be picked up.");
                    }
                    continue; // take the first object (by id)
                }
            }
            break;
        }
    }
}