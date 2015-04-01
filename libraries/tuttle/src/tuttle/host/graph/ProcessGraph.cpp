#include "ProcessGraph.hpp"
#include "ProcessVisitors.hpp"
#include <tuttle/common/utils/color.hpp>
#include <tuttle/host/graph/GraphExporter.hpp>

#include <boost/foreach.hpp>


#ifndef TUTTLE_PRODUCTION
#define TUTTLE_EXPORT_PROCESSGRAPH_DOT
#endif

//#define TUTTLE_EXPORT_WITH_TIMER



#ifdef TUTTLE_EXPORT_WITH_TIMER
#include <boost/timer/timer.hpp>
#endif

namespace tuttle {
namespace host {
namespace graph {


template<class TGraph>
inline void connectClips( TGraph& graph )
{
	BOOST_FOREACH( typename TGraph::edge_descriptor ed, graph.getEdges() )
	{
		typename TGraph::Edge& edge           = graph.instance( ed );
		typename TGraph::Vertex& vertexTarget = graph.targetInstance( ed );
		typename TGraph::Vertex& vertexSource = graph.sourceInstance( ed );

		TUTTLE_TLOG( TUTTLE_TRACE, "[Connect Clips] " << edge );
		TUTTLE_TLOG( TUTTLE_TRACE, "[Connect Clips] " << vertexTarget << " -> " << vertexSource );
		//TUTTLE_TLOG_VAR( TUTTLE_TRACE, edge.getInAttrName() );
		
		if( vertexTarget.hasProcessNode() && vertexSource.hasProcessNode() )
		{
			INode& targetNode = vertexTarget.getProcessNode();
			INode& sourceNode = vertexSource.getProcessNode();
            // calls the connectClips function of the inherited class, ImageProcessNode or ...
			sourceNode.connectClips( targetNode, sourceNode.getAttribute( edge.getInAttrName() ) );
		}
	}
}

// FIXME : what happens if a node is named TUTTLE_FAKE_OUTPUT ??
// should store the Vertex instead of the name
const std::string ProcessGraph::_outputId( "TUTTLE_FAKE_OUTPUT" );

ProcessGraph::ProcessGraph( const ComputeOptions& options, Graph& userGraph, const std::list<std::string>& outputNodes, memory::IMemoryCache& internMemoryCache )
	: _options(options)
	, _internMemoryCache(internMemoryCache)
	, _defaultProcessData(&_internMemoryCache)
{
	_defaultProcessData._interactive = _options.getIsInteractive();
	// imageEffect specific...
	_defaultProcessData._renderScale = _options.getRenderScale();

    // Build render graph    
	initRenderGraph( userGraph, outputNodes );
}

ProcessGraph::~ProcessGraph()
{}


ProcessGraph::VertexAtTime::Key ProcessGraph::getOutputKeyAtTime( const OfxTime time )
{
	return VertexAtTime(Vertex(_defaultProcessData, _outputId), time).getKey();
}

ProcessGraph::InternalGraphAtTimeImpl::vertex_descriptor ProcessGraph::getOutputVertexAtTime( const OfxTime time )
{
	return _renderGraphAtTime.getVertexDescriptor( getOutputKeyAtTime( time ) );
}

void ProcessGraph::bakeGraphInformationToNodes( InternalGraphAtTimeImpl& _renderGraphAtTime )
{
	BOOST_FOREACH( const InternalGraphAtTimeImpl::vertex_descriptor vd, _renderGraphAtTime.getVertices() )
	{
		VertexAtTime& v = _renderGraphAtTime.instance( vd );
		ProcessVertexAtTimeData& vData = v.getProcessDataAtTime();
		
		TUTTLE_TLOG( TUTTLE_INFO, "[bake graph information to nodes] node: " << v.getName() );

		vData._outDegree = _renderGraphAtTime.getInDegree( vd );
		vData._inDegree = _renderGraphAtTime.getOutDegree( vd );

		vData._outEdges.clear();
		vData._outEdges.reserve( vData._outDegree );
		BOOST_FOREACH( const InternalGraphAtTimeImpl::edge_descriptor ed, _renderGraphAtTime.getInEdges( vd ) )
		{
			const ProcessEdgeAtTime* e = &_renderGraphAtTime.instance(ed);

			TUTTLE_TLOG( TUTTLE_INFO, "[bake graph information to nodes] in edge " << e->getInAttrName() << ", at time " << e->getInTime() );
			vData._outEdges.push_back( e );
		}
		vData._inEdges.clear();
		BOOST_FOREACH( const InternalGraphAtTimeImpl::edge_descriptor ed, _renderGraphAtTime.getOutEdges( vd ) )
		{
			const ProcessEdgeAtTime* e = &_renderGraphAtTime.instance(ed);
			TUTTLE_TLOG( TUTTLE_INFO, "[bake graph information to nodes] out edge " << e->getInAttrName() << ", at time " << e->getInTime() );
			std::pair<std::string, OfxTime> key( e->getInAttrName(), e->getInTime() );
			vData._inEdges[key] = e;
		}
	}
	TUTTLE_TLOG( TUTTLE_INFO, "[bake graph information to nodes] connect clips" );
	connectClips<InternalGraphAtTimeImpl>( _renderGraphAtTime );

}

void ProcessGraph::beginSequence( const TimeRange& timeRange )
{
	_options.beginSequenceHandle();
	_defaultProcessData._renderTimeRange.min = timeRange._begin;
	_defaultProcessData._renderTimeRange.max = timeRange._end;
	_defaultProcessData._step                = timeRange._step;

	TUTTLE_TLOG( TUTTLE_INFO, "[begin sequence] start" );
	//	BOOST_FOREACH( NodeMap::value_type& p, _nodes )
	for( NodeMap::iterator it = _nodes.begin(), itEnd = _nodes.end();
		it != itEnd;
		++it )
	{
		NodeMap::value_type& p = *it;
		p.second->beginSequence( _defaultProcessData );
	}
}

void ProcessGraph::endSequence()
{
	_options.endSequenceHandle();
	TUTTLE_TLOG( TUTTLE_INFO, "[Process render] process end sequence" );
	//--- END sequence render
	BOOST_FOREACH( NodeMap::value_type& p, _nodes )
	{
		p.second->endSequence( _defaultProcessData ); // node option... or no option here ?
	}
}

void ProcessGraph::initRenderGraph( Graph& userGraph, const std::list<std::string>& outputNodes )
{
    // Copy the relevant information from the user graph to the process graph
    // using the Copier functor. It copies the user graph structure and populate this 
	// with ProcessNodes
    _renderGraph.copyTransposed( userGraph.getGraph() );

    // Create and connect an output vertex to all the nodes we want to render 
    // FIXME: this output vertex could be created in the constructor and be a member of
    // the class
	Vertex outputVertex( _defaultProcessData, _outputId );
	if( outputNodes.size() )
	{
		_renderGraph.addVertex( outputVertex );
		BOOST_FOREACH( const std::string & s, outputNodes )
		{
			_renderGraph.connect( _outputId, s, "Output" );
			TUTTLE_LOG_DEBUG( TUTTLE_INFO, "MY OUTPUT: " << s );
		}
	}
	else
	{
		// Detect root nodes and add them to the list of nodes to process
		std::vector<InternalGraphImpl::vertex_descriptor> rootVertices = _renderGraph.rootVertices();
		_renderGraph.addVertex( outputVertex );
		BOOST_FOREACH( const InternalGraphImpl::vertex_descriptor vd, rootVertices )
		{
			InternalGraphImpl::VertexKey vk = _renderGraph.instance( vd ).getKey();
			_renderGraph.connect( _outputId, vk, "Output" );
		}
	}
	
    // Remove unconnected components
	_renderGraph.removeUnconnectedVertices( _renderGraph.getVertexDescriptor( _outputId ) );


    // FIXME : what is the purpose of this following "relink" code ??
    // the processNode is already instanciated in copyTransposed
    // why not directly clone it there ? 
    // even if the remove unconnected component pass is not done
	BOOST_FOREACH( InternalGraphImpl::vertex_descriptor vd, _renderGraph.getVertices() )
	{
		Vertex& v = _renderGraph.instance( vd );

        // FIXME : if a "fake" node is just the output node, why not use the output node instead of
        // storing a "fake" variable ?
		if( v.hasProcessNode() )
		{
            // FIXME : why using 2 methods ? USE_LINK and not USE_LINK ?
            //          decide !!!!
#ifdef PROCESSGRAPH_USE_LINK
			tuttle::host::INode& origNode = v.getProcessNode(); // pointer of the copied graph, we don't own it !
#else
			const tuttle::host::INode& origNode = v.getProcessNode(); // pointer of the copied graph, we don't own it !
#endif
			std::string key( origNode.getName() ); // Shouldn't it be origNode.getKey() ??
			NodeMap::iterator it = _nodes.find( key );
			tuttle::host::INode* newNode;
			if( it != _nodes.end() )
			{
				newNode = it->second;
			}
			else
			{
#ifdef PROCESSGRAPH_USE_LINK
				newNode = &origNode;
				_nodes[key] = dynamic_cast<Node*>( newNode ); // link to the original node
#else
				newNode = origNode.clone();
				/// @todo tuttle: no dynamic_cast here, _nodes must use tuttle::host::Node
				_nodes.insert( key, dynamic_cast<Node*>( newNode ) ); // owns the new pointer
#endif
			}
			// our vertices have a link to our Nodes
			v.setProcessNode( newNode );
		}
	}
}


// FIXME: find a name that says something meaningful
void ProcessGraph::setup()
{
	using namespace boost;
	using namespace boost::graph;
	TUTTLE_TLOG( TUTTLE_INFO, "[Process render] setup" );
	
	// Initialize the data of all process nodes with the default value
    // NOTE: couldn't it be done when the copy transpose takes place ?
	BOOST_FOREACH( InternalGraphImpl::vertex_descriptor vd, _renderGraph.getVertices() )
	{
		Vertex& v = _renderGraph.instance(vd);
		if( v.hasProcessNode() )
		{
			v.copyProcessData( _defaultProcessData ); 
            // Link the pointer of the newly created v._data to the process node
            // NOTE that it changes the user graph if the INode is the original shared
			v.getProcessNode().setProcessData( &v._data );
		}
	}

    //    
	connectClips<InternalGraphImpl>( _renderGraph );
	
	{
		TUTTLE_TLOG( TUTTLE_INFO, "[Process render] Time domain propagation" );
		graph::visitor::TimeDomain<InternalGraphImpl> timeDomainPropagationVisitor( _renderGraph );
		_renderGraph.depthFirstVisit( timeDomainPropagationVisitor, _renderGraph.getVertexDescriptor( _outputId ) );
	}

	{	
		TUTTLE_TLOG( TUTTLE_INFO, "[Process render] setup visitors" );
		graph::visitor::Setup1<InternalGraphImpl> setup1Visitor( _renderGraph );
		_renderGraph.depthFirstVisit( setup1Visitor, _renderGraph.getVertexDescriptor( _outputId ) );
		graph::visitor::MaximizeBitDepth<InternalGraphImpl> maximizeBitDepth( _renderGraph );
		_renderGraph.depthFirstVisit( maximizeBitDepth, _renderGraph.getVertexDescriptor( _outputId ) );
		graph::visitor::Setup3<InternalGraphImpl> setup3Visitor( _renderGraph );
		_renderGraph.depthFirstVisit( setup3Visitor, _renderGraph.getVertexDescriptor( _outputId ) );
	}
}

std::list<TimeRange> ProcessGraph::computeTimeRange()
{
	std::list<TimeRange> timeRanges = _options.getTimeRanges();

	TUTTLE_TLOG_INFOS;
	if( timeRanges.empty() )
	{
		BOOST_FOREACH( InternalGraphImpl::edge_descriptor ed, boost::out_edges( _renderGraph.getVertexDescriptor(_outputId), _renderGraph.getGraph() ) )
		{
			//TUTTLE_TLOG_INFOS;
			ProcessVertex& v = _renderGraph.targetInstance( ed );
			// compute the time domain for each output node
			//TUTTLE_TLOG_INFOS;
			OfxRangeD timeDomain = v.getProcessData()._timeDomain;
			TUTTLE_TLOG_VAR2( TUTTLE_TRACE, timeDomain.min, timeDomain.max );
			
			if( _options.getBegin() != std::numeric_limits<int>::min() && timeDomain.min < _options.getBegin() )
				timeDomain.min = _options.getBegin();
			if( _options.getEnd() != std::numeric_limits<int>::max() && timeDomain.max > _options.getEnd() )
				timeDomain.max = _options.getEnd();
			
			TUTTLE_TLOG_VAR2( TUTTLE_TRACE, timeDomain.min, timeDomain.max );
			// special case for infinite time domain (eg. a still image)
			if( timeDomain.min <= kOfxFlagInfiniteMin )
				timeDomain.min = 0;
			if( timeDomain.max >= kOfxFlagInfiniteMax )
				timeDomain.max = 0;

			//TUTTLE_TLOG_INFOS;
			timeRanges.push_back( TimeRange( timeDomain ) );
			TUTTLE_TLOG_INFOS;
			TUTTLE_TLOG( TUTTLE_INFO, "Compute " << quotes(v.getName()) << " full time domain: from " << timeDomain.min << " to " << timeDomain.max << "." );
		}
	}
	return timeRanges;
}

void ProcessGraph::setupAtTime( const OfxTime time )
{
	_options.setupAtTimeHandle();
#ifdef TUTTLE_EXPORT_WITH_TIMER
	boost::timer::cpu_timer timer;
#endif
	
	TUTTLE_LOG_TRACE( "[Setup at time " << time << "] start" );
	graph::visitor::DeployTime<InternalGraphImpl> deployTimeVisitor( _renderGraph, time );
	_renderGraph.depthFirstVisit( deployTimeVisitor, _renderGraph.getVertexDescriptor( _outputId ) );
#ifdef TUTTLE_EXPORT_PROCESSGRAPH_DOT
	graph::exportDebugAsDOT( "graphProcess_c.dot", _renderGraph );
#endif

	TUTTLE_LOG_TRACE( "[Setup at time " << time << "] build render graph" );
	// create a new graph with time information
	_renderGraphAtTime.clear();
	
	{
		BOOST_FOREACH( InternalGraphAtTimeImpl::vertex_descriptor vd, _renderGraph.getVertices() )
		{
			Vertex& v = _renderGraph.instance( vd );
			BOOST_FOREACH( const OfxTime t, v._data._times )
			{
				TUTTLE_TLOG( TUTTLE_INFO, "[Setup at time " << time << "] add connection from node: " << v << " for time: " << t );
				_renderGraphAtTime.addVertex( ProcessVertexAtTime(v, t) );
			}
		}
		BOOST_FOREACH( const InternalGraphAtTimeImpl::edge_descriptor ed, _renderGraph.getEdges() )
		{
			const Edge& e = _renderGraph.instance( ed );
			const Vertex& in = _renderGraph.sourceInstance( ed );
			const Vertex& out = _renderGraph.targetInstance( ed );
			TUTTLE_TLOG( TUTTLE_INFO, "[Setup at time " << time << "] set connection " << e );
			BOOST_FOREACH( const Edge::TimeMap::value_type& tm, e._timesNeeded )
			{
				const VertexAtTime procIn( in, tm.first );
				BOOST_FOREACH( const OfxTime t2, tm.second )
				{
					//TUTTLE_TLOG_VAR( TUTTLE_TRACE, tm.first );
					//TUTTLE_TLOG_VAR( TUTTLE_TRACE, t2 );
					const VertexAtTime procOut( out, t2 );

					const VertexAtTime::Key inKey( procIn.getKey() );
					const VertexAtTime::Key outKey( procOut.getKey() );

					//TUTTLE_TLOG_VAR( TUTTLE_TRACE, inKey );
					//TUTTLE_TLOG_VAR( TUTTLE_TRACE, outKey );
					//TUTTLE_TLOG_VAR( TUTTLE_TRACE, e.getInAttrName() );

					const EdgeAtTime eAtTime( outKey, inKey, e.getInAttrName() );

					_renderGraphAtTime.addEdge(
						_renderGraphAtTime.getVertexDescriptor( inKey ),
						_renderGraphAtTime.getVertexDescriptor( outKey ),
						eAtTime );
				}
			}
		}
	}

	InternalGraphAtTimeImpl::vertex_descriptor outputAtTime = getOutputVertexAtTime( time );
	
	// declare final nodes
	BOOST_FOREACH( const InternalGraphAtTimeImpl::edge_descriptor ed, boost::out_edges( outputAtTime, _renderGraphAtTime.getGraph() ) )
	{
		VertexAtTime& v = _renderGraphAtTime.targetInstance( ed );
		v.getProcessDataAtTime()._isFinalNode = true; /// @todo: this is maybe better to move this into the ProcessData? Doesn't depend on time?
	}

	TUTTLE_TLOG( TUTTLE_INFO, "[Setup at time " << time << "] set data at time" );
	// give a link to the node on its attached process data
	BOOST_FOREACH( const InternalGraphAtTimeImpl::vertex_descriptor vd, _renderGraphAtTime.getVertices() )
	{
		VertexAtTime& v = _renderGraphAtTime.instance(vd);
		if( v.hasProcessNode() )
		{
			//TUTTLE_TLOG( TUTTLE_INFO, "setProcessDataAtTime: " << v._name << " id: " << v._id << " at time: " << v._data._time );
			v.getProcessNode().setProcessDataAtTime( &v._data );
		}
	}

	bakeGraphInformationToNodes( _renderGraphAtTime );


#ifdef TUTTLE_EXPORT_PROCESSGRAPH_DOT
	graph::exportDebugAsDOT( "graphProcessAtTime_a.dot", _renderGraphAtTime );
#endif

	if( ! _options.getForceIdentityNodesProcess() )
	{
		TUTTLE_LOG_TRACE( "[Setup at time " << time << "] remove identity nodes" );
		// The "Remove identity nodes" step need to be done after preprocess steps, because the RoI need to be computed.
		std::vector<graph::visitor::IdentityNodeConnection<InternalGraphAtTimeImpl> > toRemove;

		graph::visitor::RemoveIdentityNodes<InternalGraphAtTimeImpl> vis( _renderGraphAtTime, toRemove );
		_renderGraphAtTime.depthFirstVisit( vis, outputAtTime );
		TUTTLE_LOG_TRACE( "[Setup at time " << time << "] removing " << toRemove.size() << " nodes" );
		if( toRemove.size() )
		{
			graph::visitor::removeIdentityNodes( _renderGraphAtTime, toRemove );

			// Bake graph information again as the connections have changed.
			bakeGraphInformationToNodes( _renderGraphAtTime );
		}
	}

#ifdef TUTTLE_EXPORT_PROCESSGRAPH_DOT
	graph::exportDebugAsDOT( "graphProcessAtTime_b.dot", _renderGraphAtTime );
#endif

	{
		TUTTLE_LOG_TRACE( "[Setup at time " << time << "] preprocess 1" );
		graph::visitor::PreProcess1<InternalGraphAtTimeImpl> preProcess1Visitor( _renderGraphAtTime );
		_renderGraphAtTime.depthFirstVisit( preProcess1Visitor, outputAtTime );
	}

	{
		TUTTLE_LOG_TRACE( "[Setup at time " << time << "] preprocess 2" );
		graph::visitor::PreProcess2<InternalGraphAtTimeImpl> preProcess2Visitor( _renderGraphAtTime );
		_renderGraphAtTime.depthFirstVisit( preProcess2Visitor, outputAtTime );
	}

#ifdef TUTTLE_EXPORT_PROCESSGRAPH_DOT
	graph::exportDebugAsDOT( "graphProcessAtTime_c.dot", _renderGraphAtTime );
#endif

	/*
	TUTTLE_TLOG( TUTTLE_INFO, "---------------------------------------- optimize graph" );
	graph::visitor::OptimizeGraph<InternalGraphAtTimeImpl> optimizeGraphVisitor( _renderGraphAtTime );
	_renderGraphAtTime.depthFirstVisit( optimizeGraphVisitor, outputAtTime );
	*/
#ifdef TUTTLE_EXPORT_PROCESSGRAPH_DOT
	graph::exportDebugAsDOT( "graphProcessAtTime_d.dot", _renderGraphAtTime );
#endif
	/*
	InternalGraphImpl tmpGraph;
	output = _renderGraph.getVertexDescriptor( _outputId );
	/// @todo tuttle: out_edges sort don't work...
	TUTTLE_TLOG( TUTTLE_INFO, "---------------------------------------- sorting graph" );
	BOOST_FOREACH( InternalGraphImpl::vertex_descriptor vd, _renderGraph.getVertices() )
	{
		std::vector<InternalGraphImpl::Edge> edges( boost::out_degree(vd, _renderGraph.getGraph()) );

		BOOST_FOREACH( InternalGraphImpl::edge_descriptor ed, boost::out_edges( vd, _renderGraph.getGraph() ) )
		{
			edges.push_back( _renderGraph.instance(ed) );
		}

		Vertex& v = _renderGraph.instance(vd);

		std::size_t i = 0;
		TUTTLE_TLOG( TUTTLE_INFO, "before sort edges of " << v.getName() );
		BOOST_FOREACH( InternalGraphImpl::edge_descriptor ed, boost::out_edges( vd, _renderGraph.getGraph() ) )
		{
			Edge& e = _renderGraph.instance(ed);
			e._localId = i++;
			e._name += " -- ";
			e._name += boost::lexical_cast<std::string>(e._localId); // tmp
			TUTTLE_TLOG( TUTTLE_INFO, e.getName() << " - " <<  _renderGraph.targetInstance(ed).getProcessDataAtTime()._globalInfos._memory  );
		}
		std::sort( edges.begin(), edges.end(), SortEdgeByMemorySize<InternalGraphImpl>(_renderGraph) );
		TUTTLE_TLOG( TUTTLE_INFO, "after sort edges of " << v.getName() );
		BOOST_FOREACH( InternalGraphImpl::edge_descriptor ed, boost::out_edges( vd, _renderGraph.getGraph() ) )
		{
			Edge& e = _renderGraph.instance(ed);
			TUTTLE_TLOG( TUTTLE_INFO, e.getName() << " - " <<  _renderGraph.targetInstance(ed).getProcessDataAtTime()._globalInfos._memory );
		}
		InternalGraphImpl::out_edge_iterator oe_it, oe_itEnd;
		boost::tie( oe_it, oe_itEnd ) = boost::out_edges( vd, _renderGraph.getGraph() );
		for( ; oe_it != oe_itEnd; ++oe_it )
		{
			Edge& e = _renderGraph.instance(*oe_it);
			TUTTLE_TLOG( TUTTLE_INFO, e.getName() << " - " <<  _renderGraph.targetInstance(*oe_it).getProcessDataAtTime()._globalInfos._memory );
		}
	}
#ifdef TUTTLE_EXPORT_PROCESSGRAPH_DOT
	graph::exportDebugAsDOT( "graphprocess_e.dot", tmpGraph );
#endif
	*/

}

void ProcessGraph::computeHashAtTime( NodeHashContainer& outNodesHash, const OfxTime time )
{
#ifdef TUTTLE_EXPORT_WITH_TIMER
	boost::timer::cpu_timer timer;
#endif
	setupAtTime( time );
	TUTTLE_TLOG( TUTTLE_INFO, "[Compute hash at time] begin" );
	graph::visitor::ComputeHashAtTime<InternalGraphAtTimeImpl> computeHashAtTimeVisitor( _renderGraphAtTime, outNodesHash, time );
	InternalGraphAtTimeImpl::vertex_descriptor outputAtTime = getOutputVertexAtTime( time );
	_renderGraphAtTime.depthFirstVisit( computeHashAtTimeVisitor, outputAtTime );
	TUTTLE_TLOG( TUTTLE_INFO, "[Compute hash at time] end" );
}

void ProcessGraph::processAtTime( memory::IMemoryCache& outCache, const OfxTime time )
{
	_options.processAtTimeHandle();
#ifdef TUTTLE_EXPORT_WITH_TIMER
	boost::timer::cpu_timer timer;
#endif
	
	TUTTLE_LOG_TRACE( "[Process at time " << time << "] Output node : " << _renderGraph.getVertex( _outputId ).getName() );
	InternalGraphAtTimeImpl::vertex_descriptor outputAtTime = getOutputVertexAtTime( time );

    // Launch a pass of callbacks on the nodes
    graph::visitor::BeforeRenderCallbackVisitor<InternalGraphAtTimeImpl> 
        callbackRun( _renderGraphAtTime );
    _renderGraphAtTime.depthFirstVisit( callbackRun, outputAtTime );

	// do the process
	graph::visitor::Process<InternalGraphAtTimeImpl> processVisitor( _renderGraphAtTime, _internMemoryCache );
	if( _options.getReturnBuffers() )
	{
		// accumulate output nodes buffers into the @p outCache MemoryCache
		processVisitor.setOutputMemoryCache( outCache );
	}

	_renderGraphAtTime.depthFirstVisit( processVisitor, outputAtTime );

	TUTTLE_LOG_TRACE( "[Process at time " << time << "] Post process" );
	graph::visitor::PostProcess<InternalGraphAtTimeImpl> postProcessVisitor( _renderGraphAtTime );
	_renderGraphAtTime.depthFirstVisit( postProcessVisitor, outputAtTime );

	///@todo clean datas...
	TUTTLE_LOG_TRACE( "[Process at time " << time << "] Clear data at time" );
	// give a link to the node on its attached process data
	BOOST_FOREACH( const InternalGraphAtTimeImpl::vertex_descriptor vd, _renderGraphAtTime.getVertices() )
	{
		VertexAtTime& v = _renderGraphAtTime.instance(vd);
		if( v.hasProcessNode() )
		{
			v.getProcessNode().clearProcessDataAtTime();
		}
	}

	// clear cache at each frame
	// @todo: remove
	_internMemoryCache.clearUnused();

	TUTTLE_LOG_TRACE( "[Process at time " << time << "] Memory cache size: " << _internMemoryCache.size() );
	TUTTLE_LOG_TRACE( "[Process at time " << time << "] Out cache size: " << outCache.size() );
}


// Compute the nodes connected to the output node
bool ProcessGraph::process( memory::IMemoryCache& outCache )
{
#ifdef TUTTLE_EXPORT_WITH_TIMER
	boost::timer::cpu_timer all_process_timer;
#endif

#ifdef TUTTLE_EXPORT_PROCESSGRAPH_DOT
	graph::exportAsDOT( "graphProcess_a.dot", _renderGraph );
#endif

	setup();

	std::list<TimeRange> timeRanges = computeTimeRange();

#ifdef TUTTLE_EXPORT_PROCESSGRAPH_DOT
	graph::exportDebugAsDOT( "graphProcess_b.dot", _renderGraph );
#endif

	/// @todo Bug: need to use a map 'OutputNode': 'timeRanges'
	/// And check if all Output nodes share a common timeRange

	TUTTLE_LOG_INFO( "[Process render] start" );

	//--- RENDER
	// at each frame
	BOOST_FOREACH( const TimeRange& timeRange, timeRanges )
	{
		TUTTLE_LOG_TRACE( "[Process render] timeRange: [" << timeRange._begin << ", " << timeRange._end << ", " << timeRange._step << "]" );

		beginSequence( timeRange );

		if( _options.getAbort() )
		{
			TUTTLE_LOG_ERROR( "[Process render] PROCESS ABORTED before first frame." );
			endSequence();
			_internMemoryCache.clearUnused();
			return false;
		}

		for( int time = timeRange._begin; time <= timeRange._end; time += timeRange._step )
		{
			_options.beginFrameHandle();

			try
			{
#ifdef TUTTLE_EXPORT_WITH_TIMER
				boost::timer::cpu_timer setup_timer;
#endif
				setupAtTime( time );
#ifdef TUTTLE_EXPORT_WITH_TIMER
				TUTTLE_LOG_INFO( "[process timer] setup " << boost::timer::format(setup_timer.elapsed()) );
#endif

#ifdef TUTTLE_EXPORT_WITH_TIMER
				boost::timer::cpu_timer processAtTime_timer;
#endif
				processAtTime( outCache, time );
#ifdef TUTTLE_EXPORT_WITH_TIMER
				TUTTLE_LOG_INFO( "[process timer] took " << boost::timer::format(processAtTime_timer.elapsed()) );
#endif
			}
			catch( tuttle::exception::FileInSequenceNotExist& e ) // @todo tuttle: change that.
			{
				e << tuttle::exception::time(time);
				if( _options.getContinueOnError() || _options.getContinueOnMissingFile() )
				{
					TUTTLE_LOG_WARNING( "[Process render] Missing input file at frame " << time << "." << std::endl
							<< tuttle::exception::format_exception_message(e) << std::endl
							<< tuttle::exception::format_exception_info(e)
						);
				}
				else
				{
					TUTTLE_LOG_ERROR( "[Process render] Missing input file at frame " << time << "." << std::endl );
					_options.endFrameHandle();
					endSequence();
					_renderGraphAtTime.clear();
					_internMemoryCache.clearUnused();
					throw;
				}
			}
			catch( ::boost::exception& e )
			{
				e << tuttle::exception::time(time);
				if( _options.getContinueOnError() )
				{
					TUTTLE_LOG_ERROR( "[Process render] Skip frame " << time << "." << std::endl
							<< tuttle::exception::format_exception_message(e) << std::endl
							<< tuttle::exception::format_exception_info(e)
						);
				}
				else
				{
					TUTTLE_LOG_ERROR( "[Process render] Stopped at frame " << time << "." << std::endl );
					_options.endFrameHandle();
					endSequence();
					_renderGraphAtTime.clear();
					_internMemoryCache.clearUnused();
					throw;
				}
			}
			catch(...)
			{
				if( _options.getContinueOnError() )
				{
					TUTTLE_LOG_ERROR( "[Process render] Skip frame " << time << "." << std::endl
							<< tuttle::exception::format_current_exception()
						);
				}
				else
				{
					TUTTLE_LOG_ERROR( "[Process render] Error at frame " << time << "." << std::endl );
					_options.endFrameHandle();
					endSequence();
					_renderGraphAtTime.clear();
					_internMemoryCache.clearUnused();
					throw;
				}
			}

			if( _options.getAbort() )
			{
				TUTTLE_LOG_ERROR( "[Process render] PROCESS ABORTED at time " << time << "." );
				_options.endFrameHandle();
				endSequence();
				_renderGraphAtTime.clear();
				_internMemoryCache.clearUnused();
				return false;
			}
			_options.endFrameHandle();
		}

		endSequence();
	}

#ifdef TUTTLE_EXPORT_WITH_TIMER
	TUTTLE_LOG_INFO( "[all process timer] " << boost::timer::format(all_process_timer.elapsed()) );
#endif
	return true;
}

}
}
}

