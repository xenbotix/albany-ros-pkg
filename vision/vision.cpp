/*
 * Tekkotsu Social Vision Server
 * Michael E Ferguson
 * 
 * Change Log:
 *   03-03-10 - Added Tekkotsu TextMsg Interface - MEF
 *   02-20-10 - Created test server - MEF
 */

/* Build Configuration */
#define DO_FIDR         0
#define DO_FACR         1
#define USE_GUI         1
#define USE_CONNECTION  0

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "libv4l2cam/libcam.h"
#include "FiducialFinder.h"

using namespace std;

#define TEKKOTSU_PORT 10001

int main(int argc, char **args) {  
  // video parameters
  int ww=640;
  int hh=480;
  int fps=5;
  const char *dev="/dev/video0";

  // tcpip parameters
  int sockfd, n;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  const char *host = "localhost";
  const char *PRE = "!msg FIDR:";

  printf("Usage is:\n %s -h host -w width -h height -d device -f fps\n\n", args[0]);

  //Processing arguments
  for(int i=1; i<argc-1; i++){
    string a=args[i];
    if(a=="-w") {
      ww=atoi(args[i+1]);
    } else if(a=="-h") {
      hh=atoi(args[i+1]);
    } else if(a=="-d") {
      dev=args[i+1];
    } else if(a=="-f") {
      fps=atoi(args[i+1]);
    } else if(a=="-h") {
      host = args[i+1];
    }
  }

#if USE_CONNECTION
  // Setup Tekkotsu Interface
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0){
    fprintf(stderr,"ERROR: Could not open socket.\n");
    exit(0);
  }
  server = gethostbyname(host);
  if (server == NULL) {
    fprintf(stderr,"ERROR: No such host.\n");
    exit(0);
  }
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, 
        (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
  serv_addr.sin_port = htons(TEKKOTSU_PORT);
  if (connect(sockfd,(struct sockaddr*)&serv_addr,sizeof(serv_addr)) < 0){
    fprintf(stderr,"ERROR: Could not connect to Tekkotsu.\n");
    exit(0);
  }
#endif 
  char buffer[256];

  // Instance a Camera object, buffers, etc. 
  Camera c(dev, ww, hh, fps);
  cvNamedWindow("l", CV_WINDOW_AUTOSIZE);

  IplImage *l=cvCreateImage(cvSize(ww, hh), 8, 3);
  unsigned char *l_=(unsigned char *)l->imageData;

#if DO_FIDR
  IplImage *t=cvCreateImage(cvSize(ww,hh), IPL_DEPTH_8U, 1);
  unsigned char *pixels = new unsigned char[ww*hh];
  FiducialFinder *f = new FiducialFinder(); 
#endif

  while(1){
    // Grab next frame
    c.Update();
    // Convert to OpenCV format  (default is YUYV, stored into c.data[] )
    c.toIplImage(l);

#if DO_FIDR
    // Convert to grayscale, and find fiducials (TODO: clean this up, avoid OpenCV?)
    cvCvtColor(l,t,CV_BGR2GRAY); 
    f->findFiducials(*t); 
    // send information about fiducials
    for(int i=0; i<f->_fiducials.size(); i++){
      Fiducial x = f->_fiducials[i];
      // send message to Tekkotsu for this fiducial
      sprintf(buffer, "%s%d,%f,%f,%f\n", PRE, x.getId(),x.getX(),x.getY(),x.getRootSize());
      n = 0;
#if USE_CONNECTION
      while(n < strlen(buffer))
        n = n + write(sockfd,buffer,strlen(buffer));
#endif
      std::cout << buffer;
    }    
#endif

#if DO_FACR
    //CvScalar pixel;
    for(int i=0; i<l->height; i++){
      for(int j=0; j<l->width; j++){  // bgr
        int b = l->imageData[i*l->widthStep+j*3];
        int g = l->imageData[i*l->widthStep+j*3+1];
        int r = l->imageData[i*l->widthStep+j*3+2];
        /* a pixel is not skin-toned if:
            r < 1.1g, r < 0.9b, r > 2.0 * max(b,g),
            r < 20 or r > 250 */
        if( (r<20) || (r > 250) || (r< (1.1*g)) || (r < (0.9*b)) || (r > 2*(max(g,b))) ) {
            // not a face pixel
        }else{
            l->imageData[i*l->widthStep+j*3] = 255; // face pixel candidate
            l->imageData[i*l->widthStep+j*3+1] = 255;
            l->imageData[i*l->widthStep+j*3+2] = 0;
        }
      }
    }  
#endif
    
#if USE_GUI
    cvShowImage("Social Vision Server", l);
    if( (cvWaitKey(10) & 255) == 27 ) break;
#endif
  }

#if USE_CONNECTION
  close(sockfd);
#endif
#if USE_GUI
  cvDestroyWindow("Social Vision Server");
#endif
  cvReleaseImage(&l);

  return 0;
}

