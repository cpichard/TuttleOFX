#ifndef _TUTTLE_HOST_PARAM_HPP_
#define _TUTTLE_HOST_PARAM_HPP_

#include "Attribute.hpp"
#include <tuttle/host/ofx/attribute/OfxhParamAccessor.hpp>

namespace tuttle {
namespace host {

class ImageEffectNode;

namespace attribute {

class Param : public Attribute
	, virtual public ofx::attribute::OfxhParamAccessor
{
public:
	Param( ImageEffectNode& effect );
	virtual ~Param() = 0;

	//bool isOutput() const { return false; } // FIXME: This function seems to be unused, so remove it ?

	//
	// FIXME getName has to call its parent class OfxhParamAccessor::getName 
	// because Attribute has also a getName function which is virtual pure.
	// Investigate if it is possible to get rid of one of the getName or reorganize this class hierarchy
	const std::string& getName() const { return ofx::attribute::OfxhParamAccessor::getName(); }
};

}
}
}

#endif

