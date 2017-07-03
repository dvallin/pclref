cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_sources(
	CalculateOverlap.cc CalculateOverlap.h
	Repeatability.cc Repeatability.h
	Uniqueness.cc Uniqueness.h
	UniquenessSelf.cc UniquenessSelf.h
	SampleMU.cc SampleMU.h
 	CorrespondenceInlierRate.h CorrespondenceInlierRate.cc
	PointCloudStats.cc PointCloudStats.h
	Persistence.h Persistence.cc
	KeypointStatsSelf.cc KeypointStatsSelf.h)
