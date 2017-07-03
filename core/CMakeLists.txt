cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_sources(PclRefLIB.h PclRefLIB.cc
  PointTypeWrappers.h
  Arrays.h Arrays.cc Csv.h Csv.cc   
  DataAccessor.h DataAccessor.cc DataMerge.h DataMerge.cc
  DataTable.h DataTable.cc DataView.h DataView.cc
  FileTools.h FileTools.cc Histogram.h Histogram.cc
  MersenneTwister.h MersenneTwister.cc)

