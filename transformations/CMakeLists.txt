cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_sources(
	TransformCloud.cc TransformCloud.h
	RemapCloud.cc RemapCloud.h
	RemoveRemap.cc RemoveRemap.h
	CDFTransform.h CDFTransform.cc
	CDF.h CDF.hpp
	LRTTransform.h LRTTransform.cc
	LRT.h LRT.hpp
	FeaturePCA.h FeaturePCA.cc)
