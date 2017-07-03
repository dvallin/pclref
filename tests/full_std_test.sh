#!/bin/bash
echo 'ETH_STAIRS 1/8'
../build/elysium -t 4 -P HARRIS_SHOT -r 1 -v -p ../configs/harris_shot.conf --pca-stats-file ../fpfh_pca.stats -c --has-header -s , -S eth_stairs_fullset.list -O eth_stairs
echo 'ETH_APARTMENT 2/8'
../build/elysium -t 4 -P HARRIS_SHOT -r 1 -v -p ../configs/harris_shot.conf --pca-stats-file ../fpfh_pca.stats -c --has-header -s , -S eth_apartment_fullset.list -O eth_apartment
echo 'ETH_GAZEBO_WINTER 3/8'
../build/elysium -t 4 -P HARRIS_SHOT -r 1 -v -p ../configs/harris_shot.conf --pca-stats-file ../fpfh_pca.stats -c --has-header -s , -S eth_gaz_winter_fullset.list -O eth_gaz_winter
echo 'ETH_GAZEBO_SUMMER 4/8'
../build/elysium -t 4 -P HARRIS_SHOT -r 1 -v -p ../configs/harris_shot.conf --pca-stats-file ../fpfh_pca.stats -c --has-header -s , -S eth_gaz_summer_fullset.list -O eth_gaz_summer
echo 'ETH_HAUPT 5/8'
../build/elysium -t 4 -P HARRIS_SHOT -r 1 -v -p ../configs/harris_shot.conf --pca-stats-file ../fpfh_pca.stats -c --has-header -s , -S eth_haupt_fullset.list -O eth_haupt
echo 'ETH_MOUNTAIN 6/8'
../build/elysium -t 4 -P HARRIS_SHOT -r 1 -v -p ../configs/harris_shot.conf --pca-stats-file ../fpfh_pca.stats -c --has-header -s , -S eth_mountain_fullset.list -O eth_mountain
echo 'ETH_WOOD_SUMMER 7/8'
../build/elysium -t 4 -P HARRIS_SHOT -r 1 -v -p ../configs/harris_shot.conf --pca-stats-file ../fpfh_pca.stats -c --has-header -s , -S eth_wood_summer_fullset.list -O eth_wood_summer
echo 'ETH_WOOD_AUTUMN 8/8'
../build/elysium -t 4 -P HARRIS_SHOT -r 1 -v -p ../configs/harris_shot.conf --pca-stats-file ../fpfh_pca.stats -c --has-header -s , -S eth_wood_autumn_fullset.list -O eth_wood_autumn
