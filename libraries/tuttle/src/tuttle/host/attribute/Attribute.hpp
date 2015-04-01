#ifndef _TUTTLE_HOST_ATTRIBUTE_HPP_
#define _TUTTLE_HOST_ATTRIBUTE_HPP_

#include <string>

namespace tuttle {
namespace host {

class INode;

namespace attribute {



// NOTE : this class shares a lot of similarities with 
// IVertex
class Attribute
{
protected:
	/*const*/ INode& _effect;

public:
	Attribute( INode& effect );
	Attribute( const Attribute& other );
	virtual ~Attribute() = 0;

	Attribute& operator=( const Attribute& other );
	
	virtual const std::string& getName() const = 0;
	const INode&     getNode() const;
};

}
}
}

#endif

