#ifndef _SingleEMAFilter_h
#define _SingleEMAFilter_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

template <typename T>
class SingleEMAFilter {
	private:
		double alpha_;
		T lowPassFilter_;
		T highPassFilter_;
	public:
		SingleEMAFilter<T>(const double alpha);
		T addValue(const T value);
		T getLowPass();
		T getHighPass();

};
#endif

template<typename T>
SingleEMAFilter<T>::SingleEMAFilter(const double alpha) {
	alpha_ = alpha;
}

template<typename T>
T SingleEMAFilter<T>::addValue(const T value) {
	lowPassFilter_ = static_cast<T>(alpha_ * value + (1 - alpha_) * lowPassFilter_);
	highPassFilter_ = value - lowPassFilter_;
	return getLowPass();
}

template<typename T>
inline T SingleEMAFilter<T>::getLowPass() {
	return lowPassFilter_;
}

template<typename T>
inline T SingleEMAFilter<T>::getHighPass() {
	return highPassFilter_;
}  