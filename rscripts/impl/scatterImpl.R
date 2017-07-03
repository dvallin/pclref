scatterInit <- function(r, c)
{
  scatter <<- matrix(data = NA, nrow = r, ncol = c, byrow = T)
}

scatterSeedInit <- function(l)
{
	seed_scatter <<- list(length = l)
	seed_scatter_add <<- list(length = l)
}

scatterCalc <- function(kf, ce, kfAdd, ceAdd, k)
{
	ce <- subset(ce, ce$co_cloud_size.f < max_overlap)
	ce <- subset(ce, ce$co_cloud_size.f >= min_overlap)
	ce <- subset(ce, abs(ce$source_id.f - ce$target_id.f) <= symmetric)
	seed_scatter[[k]] <<- ce
}
scatterCalcAdd <- function(kf, ce, kfAdd, ceAdd, k)
{
	seed_scatter[[k]] <<- ceAdd
}

scatterPrint <- function(d2, i, j)
{
	cbPalette <- c("#66A61E", "#D95F02", "#7570B3",	"#E7298A", "#1B9E77", "#A6761D", "#666666")
	#seed_scatter[[1]] <- subset(seed_scatter[[1]], seed_scatter[[1]]$set.e == "c")
	g <- ggplot(seed_scatter[[1]], aes_string(x=names[1], y=names[2]))
  g <- g + stat_binhex()
  if(methods[1] == "log") g <- g + scale_x_log10(limits=c(0.005, 1))

  if(methods[2] == "log") g <- g + scale_y_log10(limits=c(0.005, 1))

	print(g)
}
scatterPrintPairs <- function(d2, i, j)
{
	cbPalette <- c("#66A61E", "#D95F02", "#7570B3",	"#E7298A", "#1B9E77", "#A6761D", "#666666")
	g <- ggpairs(seed_scatter[[1]], columns=4:5, color=names[[3]])
  #g <- g + stat_binhex()
  #if(methods[1] == "log") g <- g + scale_x_log10(limits=c(0.005, 1))
  #if(methods[2] == "log") g <- g + scale_y_log10(limits=c(0.005, 1))

  
  
	print(g)
}
scatterPrintColored <- function(d2, i, j)
{
	cbPalette <- c("#66A61E", "#D95F02", "#7570B3",	"#E7298A", "#1B9E77", "#A6761D", "#666666")
	g <- ggplot(seed_scatter[[1]], aes_string(x=names[1], y=names[2], color=names[3]))
  g <- g + geom_point(shape=1) + scale_colour_manual(values=cbPalette)
	#g <- ggplot(seed_scatter[[1]], aes_string(x=names[1], y=names[2], fill=names[3]))
  #g <- g + stat_binhex(aes(alpha=..count..))
  if(methods[1] == "log") g <- g + scale_x_log10(limits=c(0.005, 1))

  if(methods[2] == "log") g <- g + scale_y_log10(limits=c(0.005, 1))

	print(g)
}
