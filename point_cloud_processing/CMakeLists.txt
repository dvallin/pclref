cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_sources(
        Visualizer.cc Visualizer.h
 	LoadCloud.cc LoadCloud.h
	LoadGroundTruth.cc LoadGroundTruth.h
	DownsampleCloud.cc DownsampleCloud.h
	CreateKeypointIndices.cc CreateKeypointIndices.h
	CreateKeypointsFromIndices.cc CreateKeypointsFromIndices.h
	ConvertToRangeImage.cc ConvertToRangeImage.h
	EstimateNormals.cc EstimateNormals.h
	KFInfo.cc KFInfo.h
	TInfo.cc TInfo.h)
