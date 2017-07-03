library(ggplot2)
library(stringr)
require(reshape2)
require(Hmisc)

names = c("persistence-histogram")

        setwd("F:/Projects/elysium/rscripts/")
        source("impl/utils.R")
        source("impl/plotting.R")
        source("impl/evaluations.R")
        source("impl/histogramImpl.R")

#setwd("../tests/archived_tests/")
        setwd("../tests/archived_tests/harris_fpfh_mu_pe/eth_apartment/")
        full_dir_eval(nullInit, nullSeedInit, histogramCalc, nullPrint)
