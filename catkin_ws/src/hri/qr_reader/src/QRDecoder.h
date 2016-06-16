/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** QRDecoder.h
** Class definition for wrapping over the ZBar library running
** asynchronously along with a ros node.
** 
** Author: Mauricio Matamoros
** -------------------------------------------------------------------------*/
#pragma once
#ifndef __ROS_QR_DECODER_H__
#define __ROS_QR_DECODER_H__

#include <cstdlib>
#include <string>
#include <zbar.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/signals2/signal.hpp>
#include "Types.h"

namespace qr_reader{

	typedef void (*stringCallback)(const std::string&);

	class QRDecoder{
		public:
			QRDecoder();
			virtual ~QRDecoder();
			void addTextRecognizedHandler(const stringFunctionType& handler);
			void beginRecognize(cv_bridge::CvImageConstPtr& imgPtr);
			std::string info();
			bool recognize(cv_bridge::CvImageConstPtr& imgPtr, std::string& text);
			void run();
			void runAsync();

		private:
			zbar::ImageScanner scanner;
			stringFunction textRecognized;
			cv_bridge::CvImageConstPtr sImgPtr;
			boost::mutex mutex;
			boost::condition_variable condition;
			boost::thread *mainThread;
			void mainThreadTask();
			bool decode(cv_bridge::CvImageConstPtr& imgPtr, std::string& text);
	};

} /* namespace qr_reader */

#endif /* __ROS_QR_DECODER_H__ */