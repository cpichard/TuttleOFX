#ifndef _TUTTLE_HOST_IVERTEX_HPP_
#define	_TUTTLE_HOST_IVERTEX_HPP_

#include <tuttle/host/INode.hpp>
#include <tuttle/host/exceptions.hpp>
#include <tuttle/common/utils/global.hpp>

#include <iostream>
#include <set>

namespace tuttle {
namespace host {
namespace graph {

class IVertex
{
public:
	IVertex( const std::string& name = "Undefined" );

	IVertex( const std::string& name, INode& processNode );

	IVertex( const IVertex& v );

	virtual ~IVertex() = 0;

	IVertex& operator=( const IVertex& v )
	{
		_name           = v._name;
		_processNode    = v._processNode; 
		_used           = v._used; 
		return *this;
	}

    // FIXME: "fake" doesn't mean anything. 
    // to double check but it more likeky means hasNoProcessNode()
    bool                  hasProcessNode() const                             { return _processNode!=NULL;}
	void                  setUsed( const bool used = true )                  { _used = used; }
	bool                  isUsed() const                                     { return _used; }
	const std::string&    getName() const                                    { return _name; }
	INode&                getProcessNode()
	{
		if( !_processNode )
		{
			BOOST_THROW_EXCEPTION( exception::Bug()
				<< exception::dev() + "Process node not set on IVertex \"" + getName() + "\"." );
		}
		return *_processNode;
	}
	const INode&          getProcessNode() const
	{
		if( !_processNode )
		{
			BOOST_THROW_EXCEPTION( exception::Bug()
				<< exception::dev() + "Process node not set on IVertex \"" + getName() + "\"." );
		}
		return *_processNode;
	}

	void                  setProcessNode( INode* p )
	{
		_processNode = p;
	}

	virtual std::ostream& exportDotDebug( std::ostream& os ) const;
	friend std::ostream& operator<<( std::ostream& os, const IVertex& v );

public:
	std::string _name;
	
private:
	INode* _processNode; // FIXME : why is it stored in a IVertex ?
	bool _used; // FIXME : remove as it is only used to compute connected component in 1 function and
                //         can be replaced 
	static int _count;

public:
	int _id;

};

}
}
}

#endif

