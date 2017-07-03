
histogramCalc <- function(kf, ce, kfAdd, ceAdd, k)
{
  d <- subset(kfAdd, kfAdd$log_name.e %in% names)
  print(nrow(d))
  if(nrow(d) == 0) return(0)

  d <- subset(d, select=-n.f)
  d <- subset(d, select=-mean.f)
  d <- subset(d, select=-m2.f)
  d <- subset(d, select=-mean_all.f)
  d <- subset(d, select=-m2_all.f)
  d <- subset(d, select=-bin_lo.f)
  d <- subset(d, select=-bin_hi.f)
  dm <- melt(d, id.vars=c("id.f","log_name.e"))
  g <- ggplot(dm, aes(x=as.numeric(variable), y=value, colour=log_name.e))
#  print(g + geom_bar(stat="identity") + facet_wrap("log_name.e"))
#  print(g + facet_wrap("log_name.e") + stat_summary(fun.y="mean", geom="bar"))
#  print(g + geom_point() + facet_wrap("log_name.e") + stat_summary(fun.y="mean", geom="bar"))
  print(g + facet_wrap("log_name.e") + stat_summary(fun.data = mean_cl_normal, geom = "errorbar", mult = 1))
}
