cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_sources(
	ProcessingContext.h
	ParameterContext.h
	ParameterTemplate.h ParameterTemplate.cc
	Combiner.h Combiner.cc
	PipelineFactory.h
	ProcessingStep.h
)
