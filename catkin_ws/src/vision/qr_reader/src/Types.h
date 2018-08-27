/* ---------------------------------------------------------------------------
** This software is in the public domain, furnished "as is", without technical
** support, and with no warranty, express or implied, as to its usefulness for
** any purpose.
**
** Types.h
** Defines common types for the qr_reader namespace
** 
** Author: Mauricio Matamoros
** -------------------------------------------------------------------------*/
#pragma once
#ifndef __TYPES_H__
#define __TYPES_H__

#include <string>
#include <cv_bridge/cv_bridge.h>
#include <boost/signals2/signal.hpp>

namespace qr_reader{

typedef boost::signals2::signal<void (const std::string&)> stringFunction;
typedef stringFunction::slot_type stringFunctionType;

typedef boost::signals2::signal<void (cv_bridge::CvImageConstPtr& imgPtr)> cvImgFunction;
typedef cvImgFunction::slot_type cvImgFunctionType;

} /* namespace qr_reader */

#endif /* __TYPES_H__ */