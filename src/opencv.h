// Smooth over the change in include file arrangement in OpenCV 3.x
//


#if CV_MAJOR_VERSION > 2                  // Only for OpenCV 3.x
  #include <opencv2/core.hpp>
  #include <opencv2/highgui.hpp>
  #include <opencv2/imgproc.hpp>
  #include <opencv2/videoio.hpp>
#else
  #include <opencv2/core/core.hpp>
  #include <opencv2/highgui/highgui.hpp>
  #include <opencv2/imgproc/imgproc.hpp>
#endif
