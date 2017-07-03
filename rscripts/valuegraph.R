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

multi_plot <<- F
           param_plot <<- T

           setwd("F:/Projects/elysium/rscripts/")
           source("impl/utils.R");
source("impl/plotting.R");
source("impl/evaluations.R");

score_valid_2 <- function(score, score_threshold)
{
  if(is.null(score)) return(FALSE);
  if(is.na(score)) return(FALSE);
  if(is.nan(score)) return(FALSE);
  if(score > score_threshold && score < 1.0) return(TRUE);

  return(FALSE);
}

extractDescriptivity <- function(kf, ce, kfAdd, ceAdd, k, p)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  positives <<- apply(X=ce,2,FUN=function(x) length(which(x<=valid_error)))[["ransac_translation_error.f"]]
            negatives <<- apply(X=ce,2,FUN=function(x) length(which(x>valid_error)))[["ransac_translation_error.f"]]


            t <- -1

            p <- substring(p, 5)
            patternVal <- "[0-9]+(.[0-9]+)?"
            matches <- do.call(c,regmatches(p, gregexpr(patternVal, p)))

            x <- mean(ce$ec_size.f)
            if(param_plot)
  {
    p <- substring(p, 5)
    patternVal <- "[0-9]+(.[0-9]+)?"
    matches <- do.call(c,regmatches(p, gregexpr(patternVal, p)))
    if(length(matches) > 1)
    {
      x <- as.numeric(matches[2])
      if(multi_plot)
        t <- as.numeric(matches[1])
      }
    else
      x <- as.numeric(matches[1])
    }
  else if(p != "std" && multi_plot)
  {
    p <- substring(p, 5)
    patternVal <- "[0-9]+(.[0-9]+)?"
    matches <- do.call(c,regmatches(p, gregexpr(patternVal, p)))

    if(length(matches) > 1)
    {
      t <- as.numeric(matches[1])
    }
  }
  {
    desc_col <- 5
    df <- ceAdd
    dc <- subset(df, df$set.e == "c")
    dm <- subset(df, df$set.e == "m")
    du <- subset(df, df$set.e == "u")
    bin_max <- max(dc[,c(desc_col)])+1
    bin_max <- max(max(dm[,c(desc_col)])+1, bin_max)
    bin_max <- max(max(du[,c(desc_col)])+1, bin_max)
    bin_min <- max(0, min(dc[,c(desc_col)]))
    bin_min <- min(max(0, min(dm[,c(desc_col)])), bin_min)
    bin_min <- min(max(0, min(du[,c(desc_col)])), bin_min)

    chist <- hist(dc[,c(desc_col)], breaks=seq(bin_min,bin_max,by=(bin_max-bin_min)/128), plot=F)
    mhist <- hist(dm[,c(desc_col)], breaks=seq(bin_min,bin_max,by=(bin_max-bin_min)/128), plot=F)
    uhist <- hist(du[,c(desc_col)], breaks=seq(bin_min,bin_max,by=(bin_max-bin_min)/128), plot=F)


    mu <- d_hellinger(mhist$density, uhist$density)
    cm <- d_hellinger(chist$density, mhist$density)
    cu <- d_hellinger(chist$density, uhist$density)
    print(mu)
#print(cu)
#print(cm)

    y <- mu / cm
  }
#y <- positives / (positives+negatives)
  xyt <- c(list(), x, y)

  xyt <- c(xyt, t)
  xyt
}
sampleDist <- function(s, dist)
{
  sample(x = 1:length(dist$density), s, replace = T, prob = dist$density)
}

d_pmu <- function(chist, mhist, uhist, s)
{
  samples <- sampleDist(s, chist)
  v <- 0
  c <- 0
  for(i in 1:s)
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
  v
}

extractPMU <- function(kf, ce, kfAdd, ceAdd, k, p)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  positives <<- apply(X=ce,2,FUN=function(x) length(which(x<=valid_error)))[["ransac_translation_error.f"]]
            negatives <<- apply(X=ce,2,FUN=function(x) length(which(x>valid_error)))[["ransac_translation_error.f"]]

            t <- -1

            p <- substring(p, 5)
            patternVal <- "[0-9]+(.[0-9]+)?"
            matches <- do.call(c,regmatches(p, gregexpr(patternVal, p)))

            keypoints <- mean(kf$ks_size.f)
            ci <- mean(ce$ec_size.f)
            x <- keypoints
            if(param_plot)
  {
    p <- substring(p, 5)
    patternVal <- "[0-9]+(.[0-9]+)?"
    matches <- do.call(c,regmatches(p, gregexpr(patternVal, p)))
    if(length(matches) > 1)
    {
      x <- as.numeric(matches[2])
      if(multi_plot)
        t <- as.numeric(matches[1])
      }
    else
      x <- as.numeric(matches[1])
    }
  else if(p != "std" && multi_plot)
  {
    p <- substring(p, 5)
    patternVal <- "[0-9]+(.[0-9]+)?"
    matches <- do.call(c,regmatches(p, gregexpr(patternVal, p)))

    if(length(matches) > 1)
    {
      t <- as.numeric(matches[1])
    }
  }

  {
    desc_col <- 5
    df <- ceAdd
    dc <- subset(df, df$set.e == "c")
    dm <- subset(df, df$set.e == "m")
    du <- subset(df, df$set.e == "u")
    bin_max <- max(dc[,c(desc_col)])+1
    bin_max <- max(max(dm[,c(desc_col)])+1, bin_max)
    bin_max <- max(max(du[,c(desc_col)])+1, bin_max)
    bin_min <- max(0, min(dc[,c(desc_col)]))
    bin_min <- min(max(0, min(dm[,c(desc_col)])), bin_min)
    bin_min <- min(max(0, min(du[,c(desc_col)])), bin_min)
    chist <- hist(dc[,c(desc_col)], breaks=seq(bin_min,bin_max,by=(bin_max-bin_min)/100), plot=F)
    mhist <- hist(dm[,c(desc_col)], breaks=seq(bin_min,bin_max,by=(bin_max-bin_min)/100), plot=F)
    uhist <- hist(du[,c(desc_col)], breaks=seq(bin_min,bin_max,by=(bin_max-bin_min)/100), plot=F)

    mu <- d_hellinger(mhist$density, uhist$density)
    cm <- d_hellinger(chist$density, mhist$density)
    cu <- d_hellinger(chist$density, uhist$density)

    cmu <- d_pmu(chist, mhist, uhist, 1000)
    mmu <- d_pmu(mhist, mhist, uhist, 1000)

    y <-  cmu
#y <- cmu / mmu
#y <- cu / cm
  }
#y <- positives / (positives+negatives)
  xyt <- c(list(), x, y)

  xyt <- c(xyt, t)
  xyt
}
extractXY <- function(kf, ce, kfAdd, ceAdd, k, p)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  positives <<- apply(X=ce,2,FUN=function(x) length(which(x<=valid_error)))[["ransac_translation_error.f"]]
            negatives <<- apply(X=ce,2,FUN=function(x) length(which(x>valid_error)))[["ransac_translation_error.f"]]

            dt <- mean(kf$lg_time.f) + mean(kf$lc_time.f) + mean(kf$tc_time.f) + mean(kf$ps_time.f) + mean(kf$lg_time.f) + mean(kf$uq_time.f)
            dt <- dt + mean(ce$co_time.f) + mean(ce$re_time.f) + mean(ce$uq_time.f) + mean(ce$mu_time.f) + mean(ce$ci_time.f)

            error <- median(ce$ransac_rotation_error.f, na.rm = TRUE)
            keypoints <- mean(kf$ks_size.f)
            corr <- mean(ce$ec_size.f)
            time <- mean(kf$c_time.f) + mean(ce$c_time.f) #- dt
            ci <- mean(ce$ci_norm1.f)
            success <- (positives / (positives+negatives))
            rep <- mean(ce$re_rep.f)
            rep_u <- mean(ce$re_u_rep.f)
            rep_r <- mean(ce$re_r_rep.f)
            rep_s <- mean(ce$re_s_rep.f)
            rep_us <- mean(ce$re_us_rep.f)
            rep_rs <- mean(ce$re_rs_rep.f)

            x <- keypoints
            y <- ci

#if(mean(kf$ks_size.f)<200)
  {
    t <- -1

    if(param_plot)
    {
      p <- substring(p, 5)
      patternVal <- "[0-9]+(.[0-9]+)?"
      matches <- do.call(c,regmatches(p, gregexpr(patternVal, p)))
      if(length(matches) > 1)
      {
        x <- as.numeric(matches[2])
        if(multi_plot)
          t <- as.numeric(matches[1])
        }
      else
        x <- as.numeric(matches[1])

      }
    else if(p != "std" && multi_plot)
    {
      p <- substring(p, 5)
      patternVal <- "[0-9]+(.[0-9]+)?"
      matches <- do.call(c,regmatches(p, gregexpr(patternVal, p)))

      t <- as.numeric(matches[1])
    }

  }

  xyt <- list()
  xyt <- c(xyt, x, y)
  xyt <- c(xyt, t)
#y <- rep
#xyt <- c(xyt, x, y)
#xyt <- c(xyt, 0)
#y <- rep_u
#xyt <- c(xyt, x, y)
#xyt <- c(xyt, 1)
#y <- rep_r
#xyt <- c(xyt, x, y)
#xyt <- c(xyt, 2)
#y <- rep_s
#xyt <- c(xyt, x, y)
#xyt <- c(xyt, 3)
#y <- rep_us
#xyt <- c(xyt, x, y)
#xyt <- c(xyt, 4)
#y <- rep_rs
#xyt <- c(xyt, x, y)
#xyt <- c(xyt, 5)

  xyt
}

normalize <- F
printFunc <- function(values)
{
  xdata <- values[seq(1, length(values), 3)]
  ydata <- values[seq(2, length(values), 3)]
  tdata <- values[seq(3, length(values), 3)]
  for(i in 1:length(tdata))
  {
    t <- tdata[i]
    if(t == 0) t <- "Harris" #"Standard"
      if(t == 1) t <- "Nobel" #"Unique"
        if(t == 2) t <- "Lowe" #"Reciprocal"
          if(t == 3) t <- "Tomasi" #"Standard S"
            if(t == 4) t <- "Curvature" #"Unique S"
              if(t == 5) t <- "ISS" #"Reciprocal S"
                if(t == 8) t <- "Uniform"
                  tdata[i] <- t
                }
  dfIn = data.frame(x = xdata, y = ydata, Method = factor(tdata))
         df = data.frame(x = xdata, y = ydata, Method = factor(tdata))

              if(normalize)
  {
    c <- apply(X=df,2,FUN=function(x) length(which(x==-1)))[["Method"]]
    i0 <- apply(X=df,2,FUN=function(x) which.max(x == -1))[["Method"]]
    for(i in 1:length(df$y))
    {
      df$y[i] = dfIn$y[i] / dfIn$y[((i-1) %% c)+i0]
    }
  }

  print(df)
  g <- ggplot(df,aes(x=x,y=y,color=Method,group=Method))+
  stat_smooth(method=loess , fullrange=F, alpha = 0.1)+
  geom_point()+
  labs(y = "Repeatability",
       x = "Support Radius") #+ theme(legend.position="none")
  print(g)
}
setwd("F:/Projects/elysium/rscripts/")
#setwd("../tests/archived_tests/");
#setwd("../tests/archived_tests/feature_tests/shot_radius/eth_mountain");
#setwd("../tests/archived_tests/feature_tests/fpfh_radius/eth_apartment");
setwd("../tests/archived_tests/pca_gaz/eth_apartment");
#setwd("../tests/archived_tests/keypoints/eth_apartment");
#setwd("../tests/archived_tests/harris_fpfh_method_radius/eth_apartment");
#setwd("../tests/archived_tests/iss_fpfh_radius/eth_apartment");
xy_eval(extractDescriptivity, printFunc);
