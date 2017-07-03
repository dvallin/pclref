library(ggplot2)
library(stringr)
require(reshape2)
require(Hmisc)

min_overlap <<- 0.20;
symmetric <<- 3000;

valid_error <<- 0.1;
valid_time <<- 20000;
min_success <<- 0.0;
score_threshold <<- 0.08;

multi_plot <<- T

           setwd("F:/Projects/elysium/rscripts/")
           source("impl/utils.R");
source("impl/plotting.R");
source("impl/evaluations.R");

extractError <- function(kf, ce, k)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  median(ce$ransac_translation_error.f, na.rm = TRUE)
}
extractErrorPerTime <- function(kf, ce, k)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  error <- median(ce$ransac_translation_error.f, na.rm = TRUE)

  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  time = mean(kf$c_time.f) + mean(ce$c_time.f)


         error / time
}
extractSuccess <- function(kf, ce, k)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  positives <- apply(X=ce,2,FUN=function(x) length(which(x<=valid_error)))[["ransac_translation_error.f"]]
  negatives <- apply(X=ce,2,FUN=function(x) length(which(x>valid_error)))[["ransac_translation_error.f"]]

  positives / (positives+negatives)
}
extractSuccessPerTime <- function(kf, ce, k)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  positives <- apply(X=ce,2,FUN=function(x) length(which(x<=valid_error)))[["ransac_translation_error.f"]]
  negatives <- apply(X=ce,2,FUN=function(x) length(which(x>valid_error)))[["ransac_translation_error.f"]]

  success <- positives / (positives+negatives)

  time = mean(kf$c_time.f) + mean(ce$c_time.f)


         success / time
}
extractSuccessPerKeypoint <- function(kf, ce, k)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  positives <- apply(X=ce,2,FUN=function(x) length(which(x<=valid_error)))[["ransac_translation_error.f"]]
  negatives <- apply(X=ce,2,FUN=function(x) length(which(x>valid_error)))[["ransac_translation_error.f"]]

  success <- positives / (positives+negatives)


  keypoints = mean(kf$ks_size.f)

              success / keypoints
}
extractSuccessPerCorrespondence <- function(kf, ce, k)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  positives <- apply(X=ce,2,FUN=function(x) length(which(x<=valid_error)))[["ransac_translation_error.f"]]
  negatives <- apply(X=ce,2,FUN=function(x) length(which(x>valid_error)))[["ransac_translation_error.f"]]

  success <- positives / (positives+negatives)


  keypoints = mean(ce$ec_size.f)

              success / keypoints
}
extractRep <- function(kf, ce, k)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  mean(ce$re_rs_rep.f)
}
extractKeypoints <- function(kf, ce, k)
{
  mean(kf$ks_size.f)
}
extractTime <- function(kf, ce, k)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)
  dt <- median(kf$lg_time.f) + median(kf$lc_time.f) + median(kf$tc_time.f) + median(kf$ps_time.f) + median(kf$lg_time.f) + median(kf$uq_time.f)
  dt <- dt + median(ce$co_time.f) + median(ce$re_time.f) + median(ce$uq_time.f) + median(ce$mu_time.f) + median(ce$ci_time.f)

  mean(kf$c_time.f) + mean(ce$c_time.f) - dt
}
extractTimeKF <- function(kf, ce, k)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)
  dt <- median(kf$lg_time.f) + median(kf$lc_time.f) + median(kf$tc_time.f) + median(kf$ps_time.f) + median(kf$lg_time.f) + median(kf$uq_time.f)

  mean(kf$c_time.f) - dt
}
extractTimeCE <- function(kf, ce, k)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)
  dt <- median(ce$co_time.f) + median(ce$re_time.f) + median(ce$uq_time.f) + median(ce$mu_time.f) + median(ce$ci_time.f)

  mean(ce$c_time.f) - dt
}

printFunc <- function(values, parameters)
{
  print(values)
  pnames <- NULL
  for(i in 1:length(parameters))
  {
    p <- parameters[i]
    if(p != "std")
    {
      p <- substring(p, 5)
      patternVal <- "[0-9]+(.[0-9]+)?"
      matches <- do.call(c,regmatches(p, gregexpr(patternVal, p)))

      if(is.null(pnames)) pnames <- array(dim=c(length(matches), length(parameters)))
        for(j in 1:length(matches))
        {
          pnames[j,i] <- as.numeric(matches[j])
        }
    }
  }
  df <- data.frame(values)
  names <- array(dim=c(nrow(pnames)))
  for(i in 1:nrow(pnames))
  {
    names[i] <- paste("arg",toString(i),sep="")
    df[names[i]] <- pnames[i,]
  }
  print(names)

#g <- ggplot(data=df, aes(x=arg2, y=values, color=factor(arg1), group=factor(arg1))) +
#  geom_line() + geom_point()
#g <- ggplot(data=df, aes(x=arg1, y=values)) +
#  geom_line(size=1) + geom_point()
  if(multi_plot)
  {
    df <- df[with(df, order(arg2)), ]
    df$args <- paste(df[,3],df[,2])
    g <- ggplot(data=df, aes(x=arg2, y=values, color=factor(arg1), group=factor(arg1))) +
    geom_smooth() +
    geom_point()+ theme_bw()
  }
  else
  {
    df <- df[with(df, order(arg1)), ]
    df$args <- paste(df[,2])
    g <- ggplot(data=df, aes(x=arg1, y=values)) +
    geom_smooth() +
    geom_point()+ theme_bw()
  }
  print(g)
}

setwd("F:/Projects/elysium/rscripts/")
setwd("../tests/archived_tests/harris_fpfh_method_radius/eth_apartment");
#setwd("../tests/archived_tests/additional_tests/pca_gaz/eth_apartment");
#setwd("../tests/archived_tests/iss_fpfh_radius/eth_apartment");
parameter_eval(extractSuccess, printFunc);
#parameter_eval(extractKeypoints, printFunc);
#parameter_eval(extractTime, printFunc);
#parameter_eval(extractSuccessPerKeypoint, printFunc);
#parameter_eval(extractSuccessPerCorrespondence, printFunc);
#parameter_eval(extractRep, printFunc);
