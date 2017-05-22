/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** QRDecoder.cpp
** Class implementation for wrapping over the ZBar library running
** asynchronously along with a ros node.
** 
** Author: Mauricio Matamoros
** -------------------------------------------------------------------------*/

#include "QRDecoder.h"
using namespace qr_reader;

QRDecoder::QRDecoder(): mainThread(NULL){
}

QRDecoder::~QRDecoder(){
}

void QRDecoder::addTextRecognizedHandler(const stringFunctionType& handler){
	textRecognized.connect(handler);
}

void QRDecoder::beginRecognize(cv_bridge::CvImageConstPtr& imgPtr){
	boost::mutex::scoped_lock lock(mutex, boost::try_to_lock);
	{
		sImgPtr = imgPtr;
		condition.notify_one();
	}
}

std::string QRDecoder::info(){
	std::string s("zBar version: ");
	
	return s;
}

void QRDecoder::mainThreadTask(){
	run();
}

bool QRDecoder::decode(cv_bridge::CvImageConstPtr& imgPtr, std::string& text){
	cv::Mat gray;
	int width = imgPtr->image.cols;
	int height = imgPtr->image.rows;

	try{
		// Convert image to grayscale
		cvtColor(imgPtr->image, gray, CV_BGR2GRAY);
		// Wrap image data
		zbar::Image image(width, height, "Y800", (unsigned char*)gray.data, width * height);
		// Scan the image for barcodes
		int n = scanner.scan(image);
		if(n < 1)
			return false;
		// Extract results
		zbar::Image::SymbolIterator symbol = image.symbol_begin();
		text = symbol->get_data();
		return true;
		for(; symbol != image.symbol_end(); ++symbol) {
			/*
			* symbol->get_type_name() tells teh type (barcode or QRcode)
			* symbol->get_data() fetch the data
			* symbol->get_location_size() fetch the size of the code
			* symbol->get_location_x(i) x-coordinate of the code
			* symbol->get_location_y(i) y-coordinate of the code
			*/
		}
	}
	catch ( ... ){
		return false;
	}
}

bool QRDecoder::recognize(cv_bridge::CvImageConstPtr& imgPtr, std::string& text){
	if(!decode(imgPtr, text))
		return false;
	std::cout << "QR Text: " << text << std::flush << std::endl;
	return true;
}

void QRDecoder::run(){
	while(true){
		boost::mutex::scoped_lock lock(mutex);
		{
			condition.wait(lock);
			std::string text;
			if(recognize(sImgPtr, text) && !textRecognized.empty())
				textRecognized(text);
		}
	}
}

void QRDecoder::runAsync(){
	if(mainThread != NULL)
		return;
	mainThread = new boost::thread(&QRDecoder::mainThreadTask, this);
}