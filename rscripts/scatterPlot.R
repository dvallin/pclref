library(ggplot2)
library(GGally)
library(stringr)
require(reshape2)
require(Hmisc)
library(hexbin)

#options(device="png")

min_overlap <<- 0.2;
max_overlap <<- 1.1;
symmetric <<- 3000;

#names = c("re_r_rep.f", "ransac_translation_error.f");
methods = c("linear", "linear");
names = c("L2_keypoints.f","L2_features.f", "set.e");

setwd("F:/Projects/elysium/rscripts/")
source("impl/utils.R");
source("impl/plotting.R");
source("impl/evaluations.R");
source("impl/scatterImpl.R");

sampleDist = function(n, hist)
{
  sample(x = hist$mids, n, replace = T, prob = hist$density)
}

scatterEvalHistos <- function(d2, i, j)
{
  df <- seed_scatter[[1]]
  dc <- subset(df, df$set.e == "c")
  dm <- subset(df, df$set.e == "m")
  du <- subset(df, df$set.e == "u")
  bin_max <- max(dc[,c(5)])+1
  bin_max <- max(max(dm[,c(5)])+1, bin_max)
  bin_max <- max(max(du[,c(5)])+1, bin_max)
  bin_min <- max(0, min(dc[,c(5)]))
  bin_min <- min(max(0, min(dm[,c(5)])), bin_min)
  bin_min <- min(max(0, min(du[,c(5)])), bin_min)

  chist <- hist(dc[,c(5)], breaks=seq(bin_min,bin_max,by=(bin_max-bin_min)/200), plot=F)
  mhist <- hist(dm[,c(5)], breaks=seq(bin_min,bin_max,by=(bin_max-bin_min)/200), plot=F)
  uhist <- hist(du[,c(5)], breaks=seq(bin_min,bin_max,by=(bin_max-bin_min)/200), plot=F)

  mu0 <- l1_norm(mhist$density, uhist$density)
  cm0 <- l1_norm(chist$density, mhist$density)
  mu1 <- l2_norm(mhist$density, uhist$density)
  cm1 <- l2_norm(chist$density, mhist$density)
  mu2 <- d_intersection(mhist$density, uhist$density)
  cm2 <- d_intersection(chist$density, mhist$density)
  mu3 <- d_chi2(mhist$density, uhist$density)
  cm3 <- d_chi2(chist$density, mhist$density)
  mu4 <- d_hellinger(mhist$density, uhist$density)
  cm4 <- d_hellinger(chist$density, mhist$density)
  mu5 <- d_corr(mhist$density, uhist$density)
  cm5 <- d_corr(chist$density, mhist$density)


  samples <- sampleDist(10000, chist)
  v <- 0
  c <- 0
  for(i in 1:10000)
  {
    i_m <- which(abs(mhist$mids-samples[i])==min(abs(mhist$mids-samples[i])))
    mval <- mhist$counts[i_m]

    i_u <- which(abs(uhist$mids-samples[i])==min(abs(uhist$mids-samples[i])))
    uval <- uhist$counts[i_u]

    if(mval + uval > 0)
    {
      v <- v + mval / (mval + uval)
      c <- c + 1
    }

  }
  v <- v / c

  cat(sprintf('%.3f & %.3f & %.3f & %.3f & %.3f & %.3f & %.3f', mu0 / cm0, mu1 / cm1, mu2 / cm2, mu3 / cm3, mu4 / cm4, mu5 / cm5, v))
}

#setwd("../tests/archived_tests/feature_tests/fpfh_radius_old/eth_apartment");
#setwd("../tests/archived_tests/feature_tests/harris_fpfh_mu_pe/eth_apartment");
setwd("../tests/archived_tests/ec_neighbors_iss_fpfh/eth_apartment");
#full_dir_eval(scatterInit, scatterSeedInit, scatterCalc, scatterPrint);
#full_dir_eval(scatterInit, scatterSeedInit, scatterCalc, scatterPrintColored);
#full_dir_eval(scatterInit, scatterSeedInit, scatterCalcAdd, scatterPrint);
full_dir_eval(scatterInit, scatterSeedInit, scatterCalcAdd, scatterPrintPairs);

#setwd("F:/Projects/elysium/rscripts/")
#setwd("../tests/archived_tests/feature_tests/harris_fpfh_mu_pe/eth_apartment");
#setwd("../tests/archived_tests/feature_tests/harris_shot_mu_pe/eth_apartment");
#full_dir_eval(scatterInit, scatterSeedInit, scatterCalcAdd, scatterEvalHistos);
#setwd("../eth_gaz_summer");
#full_dir_eval(scatterInit, scatterSeedInit, scatterCalcAdd, scatterEvalHistos);
#setwd("../eth_gaz_winter");
#full_dir_eval(scatterInit, scatterSeedInit, scatterCalcAdd, scatterEvalHistos);
#setwd("../eth_haupt");
#full_dir_eval(scatterInit, scatterSeedInit, scatterCalcAdd, scatterEvalHistos);
#setwd("../eth_mountain");
#full_dir_eval(scatterInit, scatterSeedInit, scatterCalcAdd, scatterEvalHistos);
#setwd("../eth_stairs");
#full_dir_eval(scatterInit, scatterSeedInit, scatterCalcAdd, scatterEvalHistos);
#setwd("../eth_wood_autumn");
#full_dir_eval(scatterInit, scatterSeedInit, scatterCalcAdd, scatterEvalHistos);
#setwd("../eth_wood_summer");
#full_dir_eval(scatterInit, scatterSeedInit, scatterCalcAdd, scatterEvalHistos);
