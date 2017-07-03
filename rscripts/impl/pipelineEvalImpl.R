
score_valid <- function(score)
{
 if(is.null(score)) return(FALSE); 
 if(is.na(score)) return(FALSE); 
 if(is.nan(score)) return(FALSE);
 if(score > score_threshold && score < 1.0) return(TRUE);

 return(FALSE);
}

pipelineEvalInit <- function(r, c)
{
  t_error <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
  r_error <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
  corr <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
  repe <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
  time_ce <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
  time_kf <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
  keypoints <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
 
  final_means <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
  success_rates <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
  tp_rates <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
  fp_rates <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
  inliers <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
}

pipelineEvalSeedInit <- function(l)
{
	seed_t_error <<- vector(length = l)
	seed_r_error <<- vector(length = l)
	seed_corr <<- vector(length = l)
	seed_repe <<- vector(length = l)
	seed_t_ce <<- vector(length = l)
	seed_t_kf <<- vector(length = l)
	seed_keypoints <<- vector(length = l)
	seed_success_rates <<- vector(length = l)
	seed_tp_rates <<- vector(length = l)
	seed_fp_rates <<- vector(length = l)
	seed_inliers <<- vector(length = l)
}

pipelineCalc <- function(kf, ce, k)
{
  dt <- median(kf$lg_time.f) + median(kf$lc_time.f) + median(kf$tc_time.f) + median(kf$ps_time.f)+ median(kf$pe_time.f)+median(kf$uq_time.f)
	seed_t_kf[k] <<- mean(kf$c_time.f) - dt
	seed_keypoints[k] <<- mean(kf$ks_size.f)

	ce <- subset(ce, ce$co_cloud_size.f >= min_overlap) 
	ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)    
 
        seed_t_error[k] <<- median(ce$ransac_translation_error.f, na.rm = TRUE)
        seed_r_error[k] <<- median(ce$ransac_rotation_error.f, na.rm = TRUE)
        seed_repe[k] <<- median(ce$re_rep.f, na.rm=TRUE)
  seed_inliers[k] <<- median(ce$ci_norm1.f, na.rm=TRUE)
        
	seed_corr[k] <<- median(ce$ci_norm1.f)
  
  dt <- median(ce$co_time.f) + median(ce$re_time.f) + median(ce$uq_time.f) + median(ce$mu_time.f) + median(ce$ci_time.f)
        seed_t_ce[k] <<- mean(ce$c_time.f) - dt

        positives <<- apply(X=ce,2,FUN=function(x) length(which(x<=valid_error)))[["ransac_translation_error.f"]]
        negatives <<- apply(X=ce,2,FUN=function(x) length(which(x>valid_error)))[["ransac_translation_error.f"]]

	tp <<- 0
	tn <<- 0 
	for(row in 1:length(ce$ransac_translation_error.f))
	{
          if (!is.na(ce$ransac_translation_error.f[row]))
	  {
    	  if (ce$ransac_translation_error.f[row] <= valid_error && score_valid(ce$ransac_score.f[row]))
	  {
	    tp <<- tp + 1
	  }
    	  if (ce$ransac_translation_error.f[row] > valid_error && !score_valid(ce$ransac_score.f[row]))
	  {
	    tn <<- tn + 1
	  }
	  }
	}
  seed_success_rates[k] <<- positives / (positives+negatives)
	seed_tp_rates[k] <<- tp / positives
	seed_fp_rates[k] <<- 1.0 - (tn / negatives)
}

pipelineEvalPrint <- function(d2, i, j)
{
      t_error[i,j] <<- median(seed_t_error)
      r_error[i,j] <<- median(seed_r_error)
      corr[i,j] <<- median(seed_corr)
      repe[i,j] <<- median(seed_repe)
      time_ce[i,j] <<- median(seed_t_ce)
      time_kf[i,j] <<- median(seed_t_kf)
      success_rates[i,j] <<- median(seed_success_rates)
      tp_rates[i,j] <<- median(seed_tp_rates)
      fp_rates[i,j] <<- median(seed_fp_rates)
      keypoints[i,j] <<- median(seed_keypoints)
      inliers[i,j] <<- median(seed_inliers)

     #if(success_rates[i,j]>=min_success && time_kf[i,j]+time_ce[i,j] < valid_time)
     {
      cat(sprintf('s(%.3f, %.2f, %.2f) ', success_rates[i,j], sd(seed_success_rates), success_rates[i,j]-sd(seed_success_rates)))
      cat(sprintf('tp(%.2f, %.2f) ', tp_rates[i,j], sd(seed_tp_rates)))
      cat(sprintf('fp(%.2f, %.2f) ', fp_rates[i,j], sd(seed_fp_rates)))
      cat(sprintf('r(%.2f) ', repe[i,j]))
     # cat(sprintf('c(%.3f, %.2f) ', corr[i,j], sd(seed_corr)))
      cat(sprintf('k(%.0f) ', keypoints[i,j]))
      cat(sprintf('te(%.2f, %.2f) ', t_error[i,j], sd(seed_t_error)))
      cat(sprintf('tr(%.2f, %.2f) ', r_error[i,j], sd(seed_r_error)))
      cat(sprintf('t(%.1f, %.1f, %.1f) ', (time_kf[i,j]+time_ce[i,j])/1000, time_kf[i,j]/1000, time_ce[i,j]/1000))
     cat(sprintf('in(%.2f, %.2f) ', inliers[i,j], sd(seed_inliers)))
      print(d2)
     }
}

