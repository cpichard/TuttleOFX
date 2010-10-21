#ifndef _TUTTLE_PROCESSEDGE_HPP_
#define _TUTTLE_PROCESSEDGE_HPP_

#include "IEdge.hpp"
#include "ProcessVertex.hpp"

#include <map>
#include <set>

namespace tuttle {
namespace host {
namespace graph {

class ProcessEdge : public IEdge
{
public:
	ProcessEdge();
	ProcessEdge( const ProcessVertex::Key& out, const ProcessVertex::Key& in, const std::string& inAttrName );
	ProcessEdge( const ProcessEdge& e );
	~ProcessEdge();

	inline ProcessEdge& operator=( const ProcessEdge& e )
	{
		IEdge::operator=( e );
		_timesNeeded = e._timesNeeded;
		return *this;
	}

	std::ostream& exportDotDebug( std::ostream& os ) const;

public:
	typedef std::set<OfxTime> TimesSet;
	typedef std::map<OfxTime, TimesSet> TimeMap;
	TimeMap _timesNeeded;
};

}
}
}

#endif

