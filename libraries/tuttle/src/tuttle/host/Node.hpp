#ifndef _TUTTLE_HOST_NODE_HPP_
#define _TUTTLE_HOST_NODE_HPP_

#include "ImageEffectNode.hpp"
#include "ComputeOptions.hpp"
#include "memory/MemoryCache.hpp"

#include <boost/assign/list_of.hpp>

#include <memory>

namespace tuttle {
namespace host {

class NodeInit;

using boost::assign::list_of;

ImageEffectNode* createNode( const std::string& pluginName );

bool compute( const std::vector<NodeInit>& nodes,
		const ComputeOptions& options = ComputeOptions() );

bool compute( memory::IMemoryCache& memoryCache,
		const std::vector<NodeInit>& nodes,
		const ComputeOptions& options = ComputeOptions() );

bool compute( memory::IMemoryCache& memoryCache,
		const std::vector<NodeInit>& nodes,
		const ComputeOptions& options,
		memory::IMemoryCache& internMemoryCache );


/**
 * @brief Node initializer class.
 */
class NodeInit
{
public:
	NodeInit(){}
	NodeInit( const std::string& pluginName );
	NodeInit( ImageEffectNode& node );
	/**
	 * @brief Non-standard copy contructor that steals the data.
	 */
	NodeInit( const NodeInit& other )
	{
		setNode( other.release() );
	}

	NodeInit& operator=( const NodeInit& other )
	{
		setNode( other.release() );
		return *this;
	}
	
	ImageEffectNode& operator->() { return *_node.get(); }
	const ImageEffectNode& operator->() const { return *_node.get(); }
	
	/**
	 * @brief Set parameter values. If it's a multi-dimensional parameter,
	 * you should put all dimensions values.
	 * @exemple setParam("redColor", 1.0, 0.0, 0.0, 1.0)
	 */
	NodeInit& setParam( const std::string& paramName, ... );
	
	/**
	 * @brief Set parameter value from a string expression.
	 */
	NodeInit& setParamExp( const std::string& paramName, const std::string& paramValue );
	
	const ImageEffectNode& get() const { return *_node; }
	ImageEffectNode& get() { return *_node; }
	
	void setNode( ImageEffectNode& node ) { _node.reset(&node); }
	ImageEffectNode& release() const { return *_node.release(); }

    void setBeforeRenderCallback(Callback *cb);

private:
	mutable std::auto_ptr<ImageEffectNode> _node;
};

}
}

#endif
