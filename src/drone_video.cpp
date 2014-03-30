/* 
  This is a video decoder wrapper for video stream from AR Drone 2.0 (! 1.0 uses UDP)
  depends on: ffmpeg and x264 (http://ffmpeg.org/trac/ffmpeg/wiki/UbuntuCompilationGuide)
  publishes: AR Drone video on topic 'ardrone/image_raw' and camera info (front looking camera) on topics 'ardrone/camera_info' and 'camera/camera_info'
*/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>



#ifdef __cplusplus
    extern "C" {
#endif


#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>



#include <stdio.h>

#ifdef __cplusplus
    }
#endif


#ifdef __cplusplus
    extern "C" {
#endif


int main(int argc, char *argv[]) {

  ros::init(argc, argv, "video");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<sensor_msgs::Image>("ardrone/image_raw", 15);
  ros::Publisher pub_cam_info = n.advertise<sensor_msgs::CameraInfo>("ardrone/camera_info", 15);
  

  sensor_msgs::Image img_msg;
  sensor_msgs::CameraInfo cam_info_msg;
  

  AVFormatContext *pFormatCtx = avformat_alloc_context();
  int             i, j, videoStream;
  AVCodecContext  *pCodecCtx;
  AVCodec         *pCodec;
  AVFrame         *pFrame; 
  AVFrame         *pFrameRGB;
  AVPacket        packet;
  int             frameFinished;
  int             numBytes;
  uint8_t         *buffer;
  struct SwsContext *pConvertCtx_BGR24;
  char              drone_addr[] = "http://192.168.1.1:5555";
  char              formatName[] = "h264";
  AVInputFormat   *pFormat; 
  
  // Register all formats and codecs
  av_register_all();
  avcodec_register_all();
  avformat_network_init();
  
 


if (!(pFormat = av_find_input_format(formatName))) { 
     printf("can't find input format %s\n", formatName);
     return -1; 
} 

 while(avformat_open_input(&pFormatCtx, drone_addr, pFormat, NULL) != 0)
    printf("Could not open the video file\nRetrying...\n");

  
  // Retrieve stream information
  if(avformat_find_stream_info(pFormatCtx, NULL)<0)
    return -1; // Couldn't find stream information
  

  // Dump information about file onto standard error
  av_dump_format(pFormatCtx, 0, drone_addr, 0);
  
  ROS_INFO("Number of streams: %d", pFormatCtx->nb_streams);
  videoStream=-1;
  for(i=0; i<=pFormatCtx->nb_streams; i++)
    if(pFormatCtx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO) {
      videoStream=i;
      break;
    }
  if(videoStream==-1) {
    printf("Didn't find any video stream in the file\n");
    return -1;
  }
     // Didn't find a video stream
  
  // Get a pointer to the codec context for the video stream
  pCodecCtx=pFormatCtx->streams[videoStream]->codec;
  
  // Find the decoder for the video stream
  pCodec=avcodec_find_decoder(pCodecCtx->codec_id);
  if(pCodec==NULL) {
    fprintf(stderr, "Unsupported codec!\n");
    return -1; // Codec not found
  }
  // Open codec
  if(avcodec_open2(pCodecCtx, pCodec, NULL)<0)
    return -1; // Could not open codec
  
  // Allocate video frame
  pFrame=avcodec_alloc_frame();
  
  // Allocate an AVFrame structure
  pFrameRGB=avcodec_alloc_frame();
  if(pFrameRGB==NULL)
    return -1;
  
  // Determine required buffer size and allocate buffer
  numBytes=avpicture_get_size(PIX_FMT_RGB24, pCodecCtx->width,
            pCodecCtx->height);
  buffer=(uint8_t *)av_malloc(numBytes);
  
  // Assign appropriate parts of buffer to image planes in pFrameRGB
  // Note that pFrameRGB is an AVFrame, but AVFrame is a superset
  // of AVPicture

  

  avpicture_fill((AVPicture *)pFrameRGB, buffer, PIX_FMT_RGB24,
     pCodecCtx->width, pCodecCtx->height);
  
  pConvertCtx_BGR24 = sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, 
                                     pCodecCtx->width, pCodecCtx->height, PIX_FMT_RGB24, 
                                     SWS_SPLINE, NULL, NULL, NULL);
  
 


  
  
  while(av_read_frame(pFormatCtx, &packet)>=0) {
    // Is this a packet from the video stream?
    if(packet.stream_index==videoStream) {
      // Decode video frame
      avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, 
         &packet);
      
      
      // Did we get a video frame?
      if(frameFinished) {
         sws_scale(pConvertCtx_BGR24,   (const uint8_t * const*)pFrame->data, pFrame->linesize, 0,
                  pCodecCtx->height,   pFrameRGB->data,   pFrameRGB->linesize);

         std_msgs::Header header;
         header.frame_id = "ardrone_base_frontcam";
        
         img_msg.height = 360;
         img_msg.width = 640;
         img_msg.encoding = "rgb8";
         img_msg.is_bigendian = 0;
         img_msg.step = 1920;

         int imageSize = img_msg.height * img_msg.width * 3;
         
         img_msg.data = std::vector<uint8_t>(pFrameRGB->data[0], pFrameRGB->data[0] + imageSize);
        
         
   
         /* the following camera info is for AR Drone's front looking camera */
         cam_info_msg.height = 360;
         cam_info_msg.width = 640;
         cam_info_msg.distortion_model = "plumb_bob";
         double d[] = {-0.524504, 0.280327, -0.003387, -0.000744, 0.0};
         cam_info_msg.D.assign(d, d+5);
         cam_info_msg.K = { {569.099223, 0.0, 305.976477, 0.0, 566.507254, 166.537639, 0.0, 0.0, 1.0} };
         cam_info_msg.R = { {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0} };
         cam_info_msg.P = { {461.463745, 0.0, 298.69605, 0.0, 0.0, 532.096985, 163.679894, 0.0, 0.0, 0.0, 1.0, 0.0} };

         cam_info_msg.binning_x = 0;
         cam_info_msg.binning_y = 0;
         
         header.stamp = ros::Time::now();
               img_msg.header = header;
         cam_info_msg.header = header;

   
         pub.publish(img_msg); /* publish image as ros message sensor_msgs::Image  */
   /* just the same camera info message published on two topics */
         pub_cam_info.publish(cam_info_msg);
         
      }
    }
    
    // Free the packet that was allocated by av_read_frame
    av_free_packet(&packet);
  }
  
  // Free the RGB image
  av_free(buffer);
  av_free(pFrameRGB);
  
  // Free the YUV frame
  av_free(pFrame);
  
  // Close the codec
  avcodec_close(pCodecCtx);
  
  // Close the video file
  avformat_close_input(&pFormatCtx);
  return 0;
}


#ifdef __cplusplus
    }
#endif
