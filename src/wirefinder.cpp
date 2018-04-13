// stereo.cpp: îïðåäåëÿåò òî÷êó âõîäà äëÿ êîíñîëüíîãî ïðèëîæåíèÿ.
//

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>
#include "wirefinder.h"

float calcLine(const cv::Mat &src, int min, int max, float *pa, float *pb)
{
    float sumX = 0, sumX2 = 0, sumY = 0, sumXY = 0;
	int n = 0;
	const uint8_t *p = src.data;
	for (int x = 0; x < src.rows; x++) 
		for (int y = 0; y < src.cols; y++, p++) {
			uint8_t v = *p;
			if (v >= min && v <= max) {
                sumX += x;
                sumX2 += x * x;
                sumY += y;
                sumXY += x * y;
				n++;
			}
        }
    float d = sumX2 * n - sumX * sumX;
	if (fabs(d) < 1E-10) return NAN;
    float b = (sumX2 * sumY - sumX * sumXY) / d;
    float a = (n * sumXY - sumX * sumY) / d;
	float sse = 0;
	p = src.data;
	for (int x = 0; x < src.rows; x++)
		for (int y = 0; y < src.cols; y++, p++) {
			uint8_t v = *p;
			if (v >= min && v <= max) {
				float e = a * x + b - y;
				sse += e * e;
			}
		}
	*pa = a;
	*pb = b;
	return sqrt(sse / n);
}

cv::Mat WireFinder::drawHistogram(const cv::Mat &hist, int min, int max, const size_t histogramWidth, const size_t histogramHeight)
{
    int histSize = hist.cols, bin_w = cvRound((double)histogramWidth / histSize);
    cv::Mat histImage(histogramHeight, histogramWidth + 10, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 1; i < histSize; i++)
    {
        cv::line(histImage,
            cv::Point(bin_w * (i - 1), histogramHeight - cvRound(hist.at<float>(i - 1))),
            cv::Point(bin_w * (i), histogramHeight - cvRound(hist.at<float>(i))),
            cv::Scalar(255, 0, 0), 2, 8, 0);
    }
    cv::line(histImage, cv::Point(bin_w * max, 0), cv::Point(bin_w * max, histogramHeight), cv::Scalar(0, 0, 255));
    cv::line(histImage, cv::Point(bin_w * min, 0), cv::Point(bin_w * min, histogramHeight), cv::Scalar(0, 255, 0));
    return histImage;
}

void WireFinder::scanHistogram(const cv::Mat &hist, int *min, int *max)
{
    const float *ph = (const float *) hist.data;
    int k = 0;
    float sum = 0;
    int onTrigger = 0;
    bool active = false;
    for (const float *p = ph + 255; p >= ph; p--)
    {
        const float value = *p;
        if (value > 100 && onTrigger < 5) {
            onTrigger += int(value / 100);
            if (onTrigger >= 5 && !active) {
                active = true;
                *max = p - ph + 5;
            }
        }
        if (onTrigger > 0 && value < 30) {
            onTrigger--;
            if (onTrigger == 0 && active) {
                active = false;
                *min = p - ph;
                break;
            }
        }
    }
}

void WireFinder::selectRange(const cv::Mat &disparity, int *min, int *max)
{
    // Calculate histogram
    cv::Mat histogram;
	int histSize = 256;
	float range[] = { 0, 256 };
	const float* histRange = { range };
    cv::calcHist(
                &disparity, // Input disparity map
                1,          // One input image
                0,          // Only first channel
                cv::Mat(),  // No mask
                histogram,  // Output histogram
                1,          // One histogram dimension
                &histSize,  // Size of histogram 256
                &histRange  // Accunt only values in the range [0..255]
        );
    assert(histogram.type() == CV_32F);

    scanHistogram(histogram, min, max);

    float a, b;
    float sse = calcLine(disparity, *min, *max, &a, &b);
    if (!std::isnan(sse) && sse < 50) {
        int h = disparity.rows, w = disparity.cols;
        a_ = a;
        b_ = (b + 0.5 * h * a) / w - 0.5;
        wireWidth_ = sse / w;
        //cv::line(fdisp, cv::Point(int(b - sse), 0), cv::Point(int(b + h * a - sse), h), cv::Scalar(0, 0, 255));
        //cv::line(fdisp, cv::Point(int(b + sse), 0), cv::Point(int(b + h * a + sse), h), cv::Scalar(0, 0, 255));
    } else {
        wireWidth_ = NAN;
        a_ = NAN;
        b_ = NAN;
    }
    //cv::imshow("FDisp", fdisp);
    //cv::waitKey(1);
}

void WireFinder::drawResult(cv::Mat *image, const cv::Size &size, const cv::Mat &disparity, const int min, const int max)
{
    // Converting to image
    cv::cvtColor(disparity, *image, CV_GRAY2RGB);
    if (!size.empty())
        cv::resize(*image, *image, size);

    // Show disparity range
    for (uint8_t *p = image->data; p < image->dataend; p += 3) {
        if (p[0] < min || p[0] > max) {
            p[1] = 0;
            p[2] = 0;
        }
    }
    using namespace std;
    //std::string minmax = std::
    cv::putText(*image,
                to_string(min) + " - " + to_string(max) + " FPS: " + to_string(currentFps_),
                cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255));

    // Show line
    if (!std::isnan(wireWidth_)) {
        int h = image->rows, w = image->cols;
        float x1 = 0.5 + b_ - a_ * 0.5;
        float x2 = 0.5 + b_ + a_ * 0.5;
        cv::line(*image, cv::Point(int((x1 - wireWidth_) * w), 0), cv::Point(int((x2 - wireWidth_) * w), h - 1), cv::Scalar(0, 0, 255));
        cv::line(*image, cv::Point(int((x1 + wireWidth_) * w), 0), cv::Point(int((x2 + wireWidth_) * w), h - 1), cv::Scalar(0, 0, 255));
    }
}

void toGray(const cv::Mat &src, cv::Mat &out)
{
	cv::Size size = src.size();
	cv::Mat dst(size, CV_8U);
	const uint8_t *s = src.data;
	uint8_t *d = dst.data;
	for (int y = size.height, k = 0; --y >= 0; ) {
		if (++k >= 3) k = 0;
		for (int x = size.width; --x >= 0; s += 3, d++) *d = s[k];
	}
	out = dst;
}

void WireFinder::disparityCallback(const stereo_msgs::DisparityImage& msg)
{
    try
    {
        using namespace std::chrono;
        milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        milliseconds diff = ms - lastDisparity_;
        currentFps_ = 1000.0 / diff.count();
        lastDisparity_ = ms;

        /*auto &mi = msg.image;
        cv::Mat fi( mi.height, mi.width, CV_32FC1, (void *)&(mi.data[0])), bi;
        fi.convertTo(bi, CV_8U, 255.0 / msg.max_disparity);
        int min, max;
        selectRange(bi, &min, &max);


        if (stateImagePub_.getNumSubscribers() > 0 && ms > lastOut_)
        {
            cv::Mat stateImage;
            drawResult(&stateImage, cv::Size(640, 480), bi, min, max);
            stateImagePub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", stateImage).toImageMsg());
            lastOut_ = ms + minOutDiff_;
        }*/
    }
    catch (cv_bridge::Exception& e)
    {
        //ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

WireFinder::WireFinder(const std::string &topic) : a_(NAN), lastOut_(0)
{
    ros::NodeHandle nh, nhp("~");
    disparitySub_ = nh.subscribe(topic, 1, &WireFinder::disparityCallback, this);
    image_transport::ImageTransport it(nh);
    stateImagePub_ = it.advertise(std::string("state_image"), 1);
    int maxOutFps = 10;
    nhp.getParam("max_out_fps", maxOutFps);
    minOutDiff_ = std::chrono::milliseconds(1000 / maxOutFps);
    ROS_INFO("ms = %d", (int)minOutDiff_.count());
}


