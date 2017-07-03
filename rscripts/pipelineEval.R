library(ggplot2)
library(stringr)
require(reshape2)
require(Hmisc)

min_overlap <<- 0.2;
symmetric <<- 1000;

valid_error <<- 0.1;
valid_time <<- 20000;
min_success <<- 0.0;
score_threshold <<- 0.1;

setwd("F:/Projects/elysium/rscripts/")
source("impl/utils.R");
source("impl/plotting.R");
source("impl/evaluations.R");
source("impl/pipelineEvalImpl.R");

#setwd("../tests/testing/");
#full_dir_no_add_eval(pipelineEvalInit, pipelineEvalSeedInit, pipelineCalc, pipelineEvalPrint);
setwd("../tests/archived_tests/feature_tests/harris_shot_mu_pe");
full_archive_no_add_eval(pipelineEvalInit, pipelineEvalSeedInit, pipelineCalc, pipelineEvalPrint);
