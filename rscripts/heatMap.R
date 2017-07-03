library(ggplot2)
library(stringr)
require(reshape2)
require(Hmisc)

min_overlap <<- 0.0
            symmetric <<- 3000

#names = c("ransac_translation_error.f", "co_cloud_size.f", "co_keypoint_size.f")
#methods = c("log", "linear", "linear")
#scaling = c(FALSE, FALSE, FALSE)
            names = c("ransac_score.f", "co_cloud_size.f")
                    methods = c("linear", "linear")
                              scaling = c(FALSE, FALSE, FALSE)
#names = c("re_rep.f", "re_u_rep.f", "re_us_rep.f", "re_r_rep.f")
#methods = c("linear", "linear", "linear", "linear")
#scaling = c(TRUE, TRUE, TRUE, TRUE, TRUE)

                                        setwd("F:/Projects/elysium/rscripts/")
                                        source("impl/utils.R")
                                        source("impl/plotting.R")
                                        source("impl/evaluations.R")
                                        source("impl/heatMapImpl.R")

#setwd("../tests/archived_tests/")
                                        setwd("../tests/testing/")
                                        setwd("../tests/archived_tests/iss_fpfh_radius/eth_wood_summer");
full_dir_eval(nullInit, nullSeedInit, heatmapCalc, nullPrint)
