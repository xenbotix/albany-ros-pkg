/*
 * Filter Experiments
 */

//#include <iostream>
#include <cv.h>
#include <highgui.h>
//#include <stdio.h>

using namespace cv;
using namespace std;

int main(int argc, char ** args){    

  const char* imagename = argc > 1 ? args[1] : "kodim23.png";
  Mat img_in = imread( imagename );
  if(!img_in.data)
    return -1;
  Mat img_out; 
 
  /* Blurring */
  //blur(img_in, img_out, Size(15,15));
  medianBlur(img_in, img_out, 31);

  /* Edge Filters */
  //Laplacian(img_in, img_out, img_in.depth());

  /* Geometric Image Transformations */
  //resize(img_in,img_out, Size(320,240));
  //resize(img_in,img_out, Size(), 0.5, 0.5);

  
  imshow("Image", img_out);   
  waitKey();

}
