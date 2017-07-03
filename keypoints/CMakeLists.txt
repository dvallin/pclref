cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_sources(
	Harris3DKeypoints.h Harris3DKeypoints.cc
	ISSKeypoints.h ISSKeypoints.cc
	NARFKeypoints.h NARFKeypoints.cc
	RandomKeypoints.h RandomKeypoints.cc
	UniformKeypoints.h UniformKeypoints.cc
	SIFTKeypoints.h SIFTKeypoints.cc
	GrowingRegions.h GrowingRegions.cc
	KeypointClustering.h KeypointClustering.cc
	KeypointNeighborhood.h KeypointNeighborhood.cc
	SSKeypoints.h SSKeypoints.cc
)
