#ifndef _SingleEMAFilter_h
#define _SingleEMAFilter_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

template <typename T>
class SingleEMAFilter
{
public:
	SingleEMAFilter<T>(const double alpha);
	T AddValue(const T value);
	T GetLowPass();
	T GetHighPass();

private:
	double _alpha;
	T _lowPassFilter;
	T _highPassFilter;
};
#endif

template<typename T>
SingleEMAFilter<T>::SingleEMAFilter(const double alpha)
{
	_alpha = alpha;
}

template<typename T>
T SingleEMAFilter<T>::AddValue(const T value)
{
	_lowPassFilter = static_cast<T>(_alpha * value + (1 - _alpha) * _lowPassFilter);
	_highPassFilter = value - _lowPassFilter;
	return GetLowPass();
}

template<typename T>
inline T SingleEMAFilter<T>::GetLowPass()
{
	return _lowPassFilter;
}

template<typename T>
inline T SingleEMAFilter<T>::GetHighPass()
{
	return _highPassFilter;
}