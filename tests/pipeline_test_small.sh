#!/bin/bash
../build/elysium -P HARRIS_FPFH -p ../standard_params.conf -c --has-header -s , -S eth_apartment_smallset.list -O pipe_test
../build/elysium -P ISS_FPFH -p ../standard_params.conf -c --has-header -s , -S eth_apartment_smallset.list -O pipe_test
../build/elysium -P HARRIS_VFH -p ../standard_params.conf -c --has-header -s , -S eth_apartment_smallset.list -O pipe_test
../build/elysium -P ISS_VFH -p ../standard_params.conf -c --has-header -s , -S eth_apartment_smallset.list -O pipe_test
../build/elysium -P HARRIS_SHOT -p ../standard_params.conf -c --has-header -s , -S eth_apartment_smallset.list -O pipe_test
../build/elysium -P ISS_SHOT -p ../standard_params.conf -c --has-header -s , -S eth_apartment_smallset.list -O pipe_test
../build/elysium -P HARRIS_ESF -p ../standard_params.conf -c --has-header -s , -S eth_apartment_smallset.list -O pipe_test
../build/elysium -P ISS_ESF -p ../standard_params.conf -c --has-header -s , -S eth_apartment_smallset.list -O pipe_test
