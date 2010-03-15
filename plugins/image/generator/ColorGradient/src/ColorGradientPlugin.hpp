#ifndef _TUTTLE_PLUGIN_COLORGRADIENT_PLUGIN_HPP_
#define _TUTTLE_PLUGIN_COLORGRADIENT_PLUGIN_HPP_

#include "ColorGradientDefinitions.hpp"

#include <tuttle/common/utils/global.hpp>
#include <ofxsImageEffect.h>
#include <boost/gil/gil_all.hpp>

namespace tuttle {
namespace plugin {
namespace colorGradient {

template<class View>
struct ColorGradientProcessParams
{
	typedef typename View::value_type Pixel;
	typedef boost::gil::point2<double> Point2;
	Point2 _cornerA;
	Point2 _cornerB;
	std::vector<Point2> _points;
	std::vector<boost::gil::rgba32f_pixel_t> _colors;
};


/**
 * @brief ColorGradient plugin
 */
class ColorGradientPlugin : public OFX::ImageEffect
{
public:
	typedef boost::gil::point2<double> Point2;
public:
    ColorGradientPlugin( OfxImageEffectHandle handle );

private:
	template<template<typename> class Functor>
	void renderFunctor( const OFX::RenderArguments &args );

public:
    void render( const OFX::RenderArguments &args );
    void changedParam( const OFX::InstanceChangedArgs &args, const std::string &paramName );

	template<class View>
	ColorGradientProcessParams<View> getProcessParams() const;
	
public:
	typedef std::vector<OFX::Double2DParam*> Double2DParamVector;
	typedef std::vector<OFX::RGBAParam*> RGBAParamVector;
	
    // do not need to delete these, the ImageEffect is managing them for us
    OFX::Clip* _srcClip;       ///< Source image clip
    OFX::Clip* _dstClip;       ///< Destination image clip
	Double2DParamVector _points;
	RGBAParamVector _colors;
	OFX::ChoiceParam* _gradientType;
	OFX::IntParam* _nbPoints;
};

}
}
}

#endif