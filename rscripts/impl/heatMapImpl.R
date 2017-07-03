heatmapCalc <- function(kf, ce, kfAdd, ceAdd, k)
{ 
	ce <- subset(ce, ce$co_cloud_size.f >= min_overlap) 
	ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric) 
  
  for(i in 1:length(names))
    print(plot_matrix(ce, names[i], methods[i], scaling[i]))
}


