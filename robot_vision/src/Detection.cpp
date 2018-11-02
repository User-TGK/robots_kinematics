#include <Detection.hpp>

#include <algorithm>
#include <initializer_list>
#include <cmath>

void FilterDetails::setDetectionResult(bool aFound, int aSurface, int aXPosCenter, int aYPosCenter)
{
    this->found = aFound;
    this->surface = aSurface;
    this->xPosCenter = aXPosCenter;
    this->yPosCenter = aYPosCenter;
}

Detector::Detector()
{
    this->loadReferenceContours();    
}

Detector::~Detector()
{
}

bool Detector::loadReferenceContours()
{
    std::multimap<supportedShape, cv::Mat> referenceImages;
    for (const auto& referenceImagePath : shapeReferencePaths)
    {
        referenceImages.insert(std::make_pair(referenceImagePath.first, cv::imread(referenceImagePath.second, CV_LOAD_IMAGE_GRAYSCALE)));
    }
    for (const auto& referenceImage: referenceImages)
    {
        cv::Mat binary;
        std::vector<std::vector<cv::Point>> contours;
        cv::inRange(referenceImage.second, cv::Scalar(0), cv::Scalar(100), binary);
		cv::findContours(binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        if (contours.size() != 1)
        {
            return false;
        }
        referenceContours.insert(std::make_pair(referenceImage.first, contours[0]));
    }
    return true;
}

std::optional<supportedShape> Detector::detectShape(const std::vector<cv::Point>& contour) const
{
    std::multimap<supportedShape, double> shapeMap;
    for (auto referenceContour: referenceContours)
    {
        shapeMap.insert(std::make_pair(referenceContour.first, cv::matchShapes(contour, referenceContour.second, CV_CONTOURS_MATCH_I1, 0)));
    }
    supportedShape bestResult;
    double bestResultFactor = 1.0;

    for (auto result: shapeMap)
    {
        if (result.second < bestResultFactor)
        {
            bestResult = result.first;
            bestResultFactor = result.second;
        }
    }
    if (bestResultFactor <= minResultFactor)
    {
        return bestResult;
    }
    return std::nullopt;
}

bool Detector::checkIfSquare(const std::vector<std::vector<cv::Point>>& contours, int index,
                            FilterDetails& filterDetails) const
{
    std::vector<cv::Point> approx;
    cv::approxPolyDP(cv::Mat(contours[index]), approx, cv::arcLength(cv::Mat(contours[index]), true)*0.04, true);
    
    auto area = cv::contourArea(contours[index]);

    const unsigned short nrOfSides = 4;
    if (approx.size() == nrOfSides && area > this->minimumObjectSize)
    {
        /** Factor determines how close the detected shape should be to a square to be treated like a square */
        double factor = 0.90;
        double widthA = cv::norm(approx[0] - approx[1]);
        double widthB = cv::norm(approx[1] - approx[2]);
        double widthC = cv::norm(approx[2] - approx[3]);
        double widthD = cv::norm(approx[3] - approx[0]);

        double averageWidth = (widthA + widthB + widthC + widthD) / nrOfSides;
        if (widthA == 0 || widthB == 0 || widthC == 0 || widthD == 0 ) //|| averageWidth < minimumObjectSize/10
        {
            return false;
        }
        if /** Any side is > base side divided by the factor: return false */
        (
            widthA < (averageWidth / 3 + (factor * averageWidth)) &&
            widthB < (averageWidth / 3 + (factor * averageWidth)) &&
            widthC < (averageWidth / 3 + (factor * averageWidth)) &&
            widthD < (averageWidth / 3 + (factor * averageWidth)) 
        )
        {
            auto moments = cv::moments(contours[index]);

            auto xPos = int(moments.m10 / moments.m00);
            auto yPos = int(moments.m01 / moments.m00);

            filterDetails.setDetectionResult(true, (int)area, (int)xPos, (int)yPos);

            sortVertices(approx);
            for (const auto& app: approx)
            {
                geometry_msgs::Point point;
                point.x = app.x;
                point.y = app.y;
                point.z = 0;
                filterDetails.shapeVertices.push_back(point);
            }
    
            /** Else: the shape is (close enough to) a square: return true */
            return true;
        }
    }
    filterDetails.found = false;

    return false;
}

bool Detector::checkIfRect(const std::vector<std::vector<cv::Point>>& contours, int index,
                            FilterDetails& filterDetails) const
{
    std::vector<cv::Point> approx;
    cv::approxPolyDP(cv::Mat(contours[index]), approx, cv::arcLength(cv::Mat(contours[index]), true)*0.04, true);
    
    auto area = cv::contourArea(contours[index]);

    const unsigned short nrOfSides = 4;
    if (approx.size() == nrOfSides && area > this->minimumObjectSize + 200)
    {
        double deviationFactor = 2.35;
        double surface = cv::contourArea(approx);
        double AB = cv::norm(approx[0] - approx[1]);
        double BC = cv::norm(approx[1] - approx[2]);
        double CD = cv::norm(approx[2] - approx[3]);
        double AD = cv::norm(approx[3] - approx[0]);
 
        double maxSide = std::max({AB, BC, CD, AD});
        double minSide = std::min({AB, BC, CD, AD});

        if (std::abs(surface - (maxSide * minSide)) < (maxSide * deviationFactor))
        {
            auto moments = cv::moments(contours[index]);

            auto xPos = int(moments.m10 / moments.m00);
            auto yPos = int(moments.m01 / moments.m00);

            filterDetails.setDetectionResult(true, (int)area, (int)xPos, (int)yPos);
            sortVertices(approx);
            
            for (const auto& app: approx)
            {
                geometry_msgs::Point point;
                point.x = app.x;
                point.y = app.y;
                point.z = 0;
                filterDetails.shapeVertices.push_back(point);
            }
            return true;
        }
    }
    return false;
}

/** A triangle is a closed shape with 3 angles where the sum of 3 angles is (almost) equal to 180° */
bool Detector::checkIfTriangle(const std::vector<std::vector<cv::Point>>& contours, int index,
                                FilterDetails& filterDetails) const
{
    std::vector<cv::Point> approx;
    cv::approxPolyDP(cv::Mat(contours[index]), approx, cv::arcLength(cv::Mat(contours[index]), true)*0.04, true);
    
    auto area = cv::contourArea(contours[index]);

    const unsigned short nrOfSides = 3;
    if (approx.size() == nrOfSides && area > this->minimumObjectSize)
    {
        double total = 0;
        double margin = 2.0; /** The triangle has been allowed a deviation of 1.0° relative to 180° */

        double AB = cv::norm(approx[0] - approx[1]);
        double BC = cv::norm(approx[1] - approx[2]);
        double AC = cv::norm(approx[2] - approx[0]);

        total += cosineLaw(AB, BC, AC);
        total += cosineLaw(BC, AB, AC);
        total += cosineLaw(AC, BC, AB);

        if (std::abs(180 - total) <= margin)
        {
            auto moments = cv::moments(contours[index]);

            auto xPos = int(moments.m10 / moments.m00);
            auto yPos = int(moments.m01 / moments.m00);

            filterDetails.setDetectionResult(true, (int)area, (int)xPos, (int)yPos);

            for (const auto& app: approx)
            {
                geometry_msgs::Point point;
                point.x = app.x;
                point.y = app.y;
                point.z = 0;
                filterDetails.shapeVertices.push_back(point);
            }

            /** If the total of the three corners dont add up to +/- 180° the shape is not a triangle */
            return true;
        }
    }
    return false;
}

/** A circle is a shape where the surface of the shape is equal to π * r2 &&
 ** Each line from the center to the border has the same length */ 
bool Detector::checkIfCircle(const std::vector<std::vector<cv::Point>>& contours, int index,
                            FilterDetails& filterDetails) const
{
    auto area = cv::contourArea(contours[index]);
    cv::Rect rect = cv::boundingRect(contours[index]);

    int radius = rect.width / 2;
    filterDetails.radius = radius;

    double margin = 0.2;

    if (std::abs(1 - ((double)rect.width / rect.height))     <= margin &&
		std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= margin &&
        area > this->minimumObjectSize                                 &&
        contours[index].size() >= 4)
        {
            auto moments = cv::moments(contours[index]);

            auto xPos = int(moments.m10 / moments.m00);
            auto yPos = int(moments.m01 / moments.m00);

            filterDetails.setDetectionResult(true, (int)area, (int)xPos, (int)yPos);
            geometry_msgs::Point point;
            point.x = radius;
            point.y = 0;
            point.z = 0;
            filterDetails.shapeVertices.push_back(point);
            return true;
        }
    return false;
}

bool Detector::checkIfHalfCircle(const std::vector<std::vector<cv::Point>>& contours, int index,
                                    FilterDetails& filterDetails) const
{
    auto area = cv::contourArea(contours[index]);
    cv::RotatedRect mappedRect = cv::minAreaRect(contours[index]);
    cv::Rect rect(cv::Point(0,0), mappedRect.size);

    int radius = std::min(rect.height, rect.width);
    filterDetails.radius = radius;

    double width = std::max(mappedRect.size.height, mappedRect.size.width);
    double surface = CV_PI * std::pow(radius, 2)/2;
    double result = 1.0;

    std::pair<geometry_msgs::Point, geometry_msgs::Point> bestMatch;

    for (const auto& point: contours[index])
    {
        for (const auto& referencePoint: contours[index])
        {
            if (point == referencePoint)
            {
                continue;
            }
            
            double resultM = std::abs(cv::norm(point - referencePoint) - width);

            if (resultM < result)
            {
                result = resultM;
                geometry_msgs::Point verticeA;
                geometry_msgs::Point verticeB;

                verticeA.x = point.x;
                verticeA.y = point.y;

                verticeB.x = referencePoint.x;
                verticeB.y = referencePoint.y;

                bestMatch.first = verticeA;
                bestMatch.second = verticeB;
            }
        }
    }

    if (
        contours[index].size() >= 4     &&
        area > this->minimumObjectSize  &&
        surface <= (area * 1.10)        &&
        surface >= (area * 0.90)        &&
        std::abs((width/2) - radius) < (0.75 * radius)
        )
    {
        auto moments = cv::moments(contours[index]);

        auto xPos = int(moments.m10 / moments.m00);
        auto yPos = int(moments.m01 / moments.m00);

        filterDetails.setDetectionResult(true, (int)area, (int)xPos, (int)yPos);
        filterDetails.shapeVertices.push_back(bestMatch.first);
        filterDetails.shapeVertices.push_back(bestMatch.second);

        return true;
    }
    return false;
}

/** Implementation based on: https://en.wikipedia.org/wiki/Law_of_cosines 
 ** C++ implementation of: 'a² = b² + c² -2bc * cos(A)' */
double Detector::cosineLaw(const double a, const double b, const double c) const
{
    double angle;

    angle = (b*b + c*c - a*a) / (2.0 * b * c);
    angle = std::acos(angle) * 180.0/M_PI;

    return angle;
}

void Detector::sortVertices(std::vector<cv::Point>& vertices) const
{
    std::sort(vertices.begin(), vertices.end(), [](
                cv::Point p, cv::Point pRef){
                    return p.y < pRef.y;} );

    cv::Point topLeft, topRight, bottomLeft, bottomRight;

    if (vertices[0].x > vertices[1].x)
	{
		topRight.x = vertices[0].x;
        topRight.y = vertices[0].y;
		topLeft.x = vertices[1].x;
        topLeft.y = vertices[1].y;
	}
	else
	{
		topRight.x = vertices[1].x;
        topRight.y = vertices[1].y;
		topLeft.x = vertices[0].x;
        topLeft.y = vertices[0].y;
	}
	if (vertices[2].x > vertices[3].x)
	{
		bottomRight.x = vertices[2].x;
        bottomRight.y = vertices[2].y;
		bottomLeft.x = vertices[3].x;
        bottomLeft.y = vertices[3].y;
	}
	else
	{
		bottomRight.x = vertices[3].x;
        bottomRight.y = vertices[3].y;
		bottomLeft.x = vertices[2].x;
        bottomLeft.y = vertices[2].y;
	}

    vertices.clear();
    vertices.push_back(topLeft);
    vertices.push_back(topRight);
    vertices.push_back(bottomRight);
    vertices.push_back(bottomLeft);
}