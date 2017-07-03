#!/bin/bash
echo 'Spacebot 1/3'
../release/elysium -t 4 -P HARRIS_FPFH -r 1 -v -p ../configs/harris_fpfh.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S spacebot_fullset.list -O spacebot
#../release/elysium -t 4 -P HARRIS_SHOT -r 1 -v -p ../configs/harris_shot.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S spacebot_fullset.list -O spacebot
#../release/elysium -t 4 -P UNIFORM_FPFH -r 1 -v -p ../configs/uniform_fpfh.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S spacebot_fullset.list -O spacebot
#../release/elysium -t 4 -P ISS_FPFH -r 1 -v -p ../configs/iss_fpfh.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S spacebot_fullset.list -O spacebot
echo 'FZI_TIEFGARAGE 2/3'
#../release/elysium -t 4 -P HARRIS_FPFH -r 1 -v -p ../configs/harris_fpfh.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S fzi_tiefgarage_smallset.list -O fzi_tiefgarage
#../release/elysium -t 4 -P HARRIS_SHOT -r 1 -v -p ../configs/harris_shot.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S fzi_tiefgarage_smallset.list -O fzi_tiefgarage
#../release/elysium -t 4 -P UNIFORM_FPFH -r 1 -v -p ../configs/uniform_fpfh.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S fzi_tiefgarage_smallset.list -O fzi_tiefgarage
#../release/elysium -t 4 -P ISS_FPFH -r 1 -v -p ../configs/iss_fpfh.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S fzi_tiefgarage_smallset.list -O fzi_tiefgarage
echo 'FZI_GEBAEUDE 3/3'
#../release/elysium -t 4 -P HARRIS_FPFH -r 1 -v -p ../configs/harris_fpfh.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S fzi_gebaeude_smallset.list -O fzi_gebaeude
#../release/elysium -t 4 -P HARRIS_SHOT -r 1 -v -p ../configs/harris_shot.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S fzi_gebaeude_smallset.list -O fzi_gebaeude
#../release/elysium -t 4 -P UNIFORM_FPFH -r 1 -v -p ../configs/uniform_fpfh.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S fzi_gebaeude_smallset.list -O fzi_gebaeude
#../release/elysium -t 4 -P ISS_FPFH -r 1 -v -p ../configs/iss_fpfh.conf --pca-stats-file ../fpfh_pca.stats --has-header -s , -S fzi_gebaeude_smallset.list -O fzi_gebaeude
