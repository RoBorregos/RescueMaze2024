/* #include "SingleEMAFilter.h"

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
} */