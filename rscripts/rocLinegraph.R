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

multi_plot <<- TRUE

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

extractROC <- function(kf, ce, k)
{
  ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
  ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)

  positives <<- apply(X=ce,2,FUN=function(x) length(which(x<=valid_error)))[["ransac_translation_error.f"]]
            negatives <<- apply(X=ce,2,FUN=function(x) length(which(x>valid_error)))[["ransac_translation_error.f"]]

            roc <- list()
            tpr_last <- 1
            fpr_last <- 1
            roc <- c(roc, 1)
            roc <- c(roc, 1)
            roc <- c(roc, 0)
            for(st in 0:200.0)
  {
    tp <- 0
    tn <- 0
    for(row in 1:length(ce$ransac_translation_error.f))
    {
      if (!is.na(ce$ransac_translation_error.f[row]))
      {
        if (ce$ransac_translation_error.f[row] <= valid_error && score_valid_2(ce$ransac_score.f[row], st / 200.0))
        {
          tp <- tp + 1
        }
        if (ce$ransac_translation_error.f[row] > valid_error && !score_valid_2(ce$ransac_score.f[row], st / 200.0))
        {
          tn <- tn + 1
        }
      }
    }
    tpr = tp / positives
          fpr = 1.0 - (tn / negatives)
                if(fpr_last > fpr)
    {
      fpr_last = fpr
                 roc <- c(roc, fpr)
                 roc <- c(roc, tpr)
                 roc <- c(roc, st / 200.0)
    }
  }
  roc
}

printFunc <- function(values)
{
  xdata <- values[seq(1, length(values), 3)]
  ydata <- values[seq(2, length(values), 3)]
  thresh <- values[seq(3, length(values), 3)]
  df = data.frame(x = xdata, y = ydata, t = thresh, m = sqrt(xdata*xdata+(ydata-1)*(ydata-1)))
     i <- 2:
       nrow(df)
       auc <- (df$x[i-1] - df$x[i]) %*% ((df$y[i] + df$y[i - 1])/2)
       q1 <- auc/(2-auc)
       q2 <- (2*auc^2)/(1+auc)
       se <- sqrt(((auc * (1 - auc)) + ((positives -1)*(q1 - auc^2)) + ((negatives -1)*(q2 - auc^2)))/(positives*negatives))
       upper <- auc + (se * 0.96)
       lower <- auc - (se * 0.96)
       best_i <- which.min(df$m)
       annotation <- paste("AUC=",signif(auc, 2), " (95%CI ", signif(upper, 2), " - ", signif(lower, 2), ")", sep="")
       annotation2 <- paste("BEST=",signif(df$t[best_i], 2), " (FPR ", signif(df$x[best_i], 2), ", TPR ", signif(df$y[best_i], 2), ")", sep="")
       print(df)
       g <- ggplot(df,aes(x=x,y=y,color=t))+geom_line(size = 1)+
       scale_x_continuous(limits = c(0, 1))+
       scale_y_continuous(limits = c(0, 1))+
       annotate("text", x = 0.75, y = 0.05, size = 7, label = annotation) +
       annotate("text", x = 0.75, y = 0.10, size = 7, label = annotation2) +
       labs(x = "False Positive Rate (1-Specificity)",
            y = "True Positive Rate (Sensitivity)")
       print(g)
}

setwd("F:/Projects/elysium/rscripts/")
#setwd("../tests/archived_tests/");
setwd("../tests/archived_tests/ec_neighbors_iss_fpfh/eth_apartment");
roc_eval(extractROC, printFunc);
