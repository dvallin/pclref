plot_whiskers <- function(mat)
{
  ggplot(melt(mat), aes('Var2',value)) + geom_boxplot()
}


plot_matrix <- function (df, value, method, scaling)
{  
  c = max(df[c('source_id.f')])
  m = matrix(nrow=c, ncol=c);
  
  for(n in 1:nrow(df)) {
    x <- df[n,]
      
    i = x[c('source_id.f')][1,1]
    j = x[c('target_id.f')][1,1]
    v = x[c(value)][1,1]
    m[i,j] = v
  }
  
  g <- ggplot(melt(m), aes(Var1,Var2, fill=value)) + geom_raster() +
    scale_x_discrete(limits=c:1) + scale_y_reverse(breaks=c:1) + xlab("source id") + ylab("target id")
  if(method == "log")
  {
    b <- c(0.02,0.05,0.1,0.2,0.4,0.8)
    if(scaling) g <- g + scale_fill_gradient(trans='log', breaks=b, low="yellow", high="blue")
    else  g <- g + scale_fill_gradient(trans='log', breaks=b, low="yellow", high="blue", limits=c(0.005, 1))
  }
  else if(method == "linear")
  {
    b <- c(0.1,0.3,0.5,0.7,0.9)
    if(scaling) g <- g + scale_fill_gradient(breaks=b, low="blue", high="yellow")
    else  g <- g + scale_fill_gradient(breaks=b, low="blue", high="yellow", limits=c(0, 1))
  }
  g
}


