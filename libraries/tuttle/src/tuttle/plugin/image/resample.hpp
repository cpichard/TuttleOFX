#ifndef _TUTTLE_PLUGIN_GIL_RESAMPLE_HPP_
#define	_TUTTLE_PLUGIN_GIL_RESAMPLE_HPP_

#include <terry/numeric/sampler.hpp>
#include <terry/numeric/resample.hpp>

#include <terry/sampler/details.hpp>
#include <terry/sampler/bc.hpp>
#include <terry/sampler/bilinear.hpp>
#include <terry/sampler/gaussian.hpp>
#include <terry/sampler/lanczos.hpp>
#include <terry/sampler/nearestNeighbor.hpp>
#include <terry/sampler/sampler.hpp>
#include <tuttle/plugin/context/SamplerDefinition.hpp>


namespace tuttle {
namespace plugin {

/**
 * @brief Set each pixel in the destination view as the result of a sampling function over the transformed coordinates of the source view
 * @ingroup ImageAlgorithms
 *
 * The provided implementation works for 2D image views only
 */
template <typename Sampler, // Models SamplerConcept
          typename SrcView, // Models RandomAccess2DImageViewConcept
          typename DstView, // Models MutableRandomAccess2DImageViewConcept
          typename MapFn>
// Models MappingFunctionConcept
void resample_pixels_progress( const SrcView& src_view, const DstView& dst_view, const MapFn& dst_to_src, const OfxRectI& procWindow, const ::terry::sampler::EParamFilterOutOfImage& outOfImageProcess, tuttle::plugin::IProgress* p, Sampler sampler = Sampler() )
{
	typedef typename DstView::point_t Point2;
	typedef typename DstView::value_type Pixel;
	Point2 dst_p;
	Pixel black;
	color_convert( boost::gil::rgba32f_pixel_t( 0.0, 0.0, 0.0, 0.0 ), black );
	for( dst_p.y = procWindow.y1; dst_p.y < procWindow.y2; ++dst_p.y )
	{
		typename DstView::x_iterator xit = dst_view.row_begin( dst_p.y );
		for( dst_p.x = procWindow.x1; dst_p.x < procWindow.x2; ++dst_p.x )
		{

			if( ! ::terry::sampler::sample( sampler, src_view, ::terry::transform( dst_to_src, dst_p ), xit[dst_p.x], outOfImageProcess ) )
			{
				xit[dst_p.x] = black; // if it is outside of the source image
			}
		}
		if( p->progressForward() )
			return;
	}
}

}
}

#endif
