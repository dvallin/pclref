cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_sources(
	correspondence_estimation_improved.hpp
	correspondence_estimation_improved.h
	generic_ransac_correspondence_rejector.h
	sac_model_general_registration.h
	sac_model_general_registration.hpp
	flann_search_improved.h flann_search_improved.hpp
	general_pca.h general_pca.hpp
	esf_improved.h esf_improved.hpp)
