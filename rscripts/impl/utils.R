
colSdColMeans <- function(x, na.rm=TRUE)
{
  if (na.rm)
  {
    n <- colSums(!is.na(x)) # thanks @flodel
  }
  else
  {
    n <- nrow(x)
  }
  colVar <- colMeans(x*x, na.rm=na.rm) - (colMeans(x, na.rm=na.rm))^2
  return(sqrt(colVar * n/(n-1)))
}


gm_mean = function(x, na.rm=TRUE){
  exp(sum(log(x[x > 0]), na.rm=na.rm) / length(x))
}

d_intersection <- function(h, k)
{
  s1 <- 0
  s2 <- 0
  for(i in 1:length(h))
  {
    s1 <- s1 + min(h[i], k[i]);
    s2 <- s2 + k[i];
  }
  1 - (s1 / s2)
}
l1_norm <- function(h, k)
{
  s1 <- 0
  for(i in 1:length(h))
  {
    s1 <- s1 + abs(h[i] - k[i]);
  }
  s1
}
l2_norm <- function(h, k)
{
  s1 <- 0
  for(i in 1:length(h))
  {
    t <- h[i] - k[i]
    s1 <- s1 + t*t;
  }
  sqrt(s1)
}
d_chi2 <- function(h, k)
{
  s1 <- 0
  s2 <- 0
  for(i in 1:length(h))
  {
    t <- (h[i] - k[i])
    s1 <- s1 + t*t;
    s2 <- s2 + (h[i] + k[i]);
  }
  0.5 * (s1 / s2)
}
d_hellinger <- function(h, k)
{
  s <- 0
  t <- mean(h)*mean(k)*(length(h)*length(k))
  for(i in 1:length(h))
  {
    s <- s + sqrt(h[i]*k[i]);
  }
  t <- 1/sqrt(t)
  sqrt(max(0, 1 - t*s))
}
d_corr <- function(h, k)
{
  s1 <- 0
  s2 <- 0
  s3 <- 0
  mh = mean(h)
  mk = mean(k)
  for(i in 1:length(h))
  {
    a <- (h[i] - mh)
    b <- (k[i] - mk)
    s1 <- s1 + a*b;
    s2 <- s2 + a*a;
    s3 <- s3 + b*b;
  }
  1 - (s1 / (sqrt(s2*s3)))
}


