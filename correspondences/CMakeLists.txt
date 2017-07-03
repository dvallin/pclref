cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_sources(
	GTCorrespondences.h GTCorrespondences.cc
	EstimateCorrespondences.h EstimateCorrespondences.cc
	NDT.h NDT.cc
	MST.h MST.cc
	RANSAC.h RANSAC.cc
	ICP.h ICP.cc
	PredictCorrespondences.h PredictCorrespondences.cc)
