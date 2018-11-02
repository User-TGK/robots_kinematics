#ifndef SPECIFICATION_HPP_
#define SPECIFICATION_HPP_

#include <SpecificationConfig.hpp>

/**
 * @brief filtering process is based on a specification. This struct describes the attributes a specification has.
 * 
 */
struct Specification
{
    /**
     * @brief default constructor of a specification. The default constructor will initialize 
     * a red circle.
     */
    Specification();

    /**
     * @brief Construct a new Specification object (copyconstructor)
     * 
     * @param rhs the object to be copied
     */
    Specification(const Specification& rhs);

    /**
     * @brief destructor of a specification
     */
    ~Specification();

    /**
    * @brief Assignment operator for a specification
    * 
    * @param rhs contains the values to be assigned
    * @return Specification& the specification instance with the set values
    */
    Specification& operator=(const Specification& rhs);

    /** The shape to be detected according to this specification instance */
    supportedShape shape;

    /** The color that the shape is supposed to have according to this specification instance */
    supportedColor color;

    /** Contains a bool value, true if the specification tells the interactive mode to shutdown */
    bool exit = false;
};

#endif //SCHEDULERCONFIG_HPP_