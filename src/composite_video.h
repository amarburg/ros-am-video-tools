
#ifndef __COMPOSITE_VIDEO_H__
#define __COMPOSITE_VIDEO_H__

#include <string>

#include <opencv2/core/core.hpp>

#include "composite_canvas.h"

namespace AplCam {

  using cv::Mat;
  using cv::Rect;
  using cv::Size;

  struct CompositeVideo
  { 
    CompositeVideo( const std::string &filepath )
      : _filepath( filepath ), _video( filepath ) {;}

    bool isOpened( void ) const { return _video.isOpened(); }

    bool read(  CompositeCanvas &canvas )
    { 
      Mat frame;
      if( _video.read( frame ) ) {
        canvas = CompositeCanvas( frame );
        return true;
      }
      return false;
    }

    double get( int flag )
    {
      return _video.get( flag );
    }

    bool seek( double frame ) { return _video.set( CV_CAP_PROP_POS_FRAMES, frame ); }
    bool rewind( void )       { _video.release(); return _video.open( _filepath ); }

    double fps( void )        { return _video.get( CV_CAP_PROP_FPS ); }
    Size   fullSize( void )   { return cv::Size( _video.get( CV_CAP_PROP_FRAME_WIDTH ),
                                                 _video.get( CV_CAP_PROP_FRAME_HEIGHT ) ); }
    Size   frameSize( void )   { return cv::Size( _video.get( CV_CAP_PROP_FRAME_WIDTH ) / 2.0,
                                                 _video.get( CV_CAP_PROP_FRAME_HEIGHT ) ); }


    std::string _filepath;
    cv::VideoCapture _video;
  };




}

#endif

