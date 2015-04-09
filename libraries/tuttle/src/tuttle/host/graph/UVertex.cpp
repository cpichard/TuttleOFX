#include "UVertex.hpp"

namespace tuttle {
namespace host {
namespace graph {


UVertex::UVertex( const std::string& name, ImageEffectNode& processNode )
: IVertex( name, processNode )
{
}

std::ostream& operator<<( std::ostream& os, const UVertex& v )
{
	return operator<<( os, static_cast<const IVertex&>(v) );
}

}
}
}
