#include <Specification.hpp>

Specification::Specification()
{
    color = supportedColor::red;
    shape = supportedShape::circle;
    exit = false;
}

Specification::Specification(const Specification& rhs)
{
    shape = rhs.shape;
    color = rhs.color;
    exit = rhs.exit;
}

Specification::~Specification()
{
}

Specification& Specification::operator=(const Specification& rhs)
{
    shape = rhs.shape;
    color = rhs.color;
    exit = rhs.exit;
    return *this;
}