
nullInit <- function(x, y) {}
nullSeedInit <- function(x) {}
nullCalc <- function(kf, ce, kfAdd, ceAdd, k) {}
nullPrint <- function(x, y, z) {}

prefilter <- function(kf, ce, kfAdd, ceAdd, k)
{
}

parameter_eval <- function (extractValueFunc, printFunc)
{
  dirs = dir() # pipeline level

  parameter_count <- 0;
  for(d in dirs)
     parameter_count <- max(parameter_count, length(dir(d)))

  cat(sprintf('%s parameters on %s pipelines\n', parameter_count, length(dirs)))

  i = 1
  for(d in dirs)
  {
    print(d)
    p <- cbind(d, "/")

    j = 1
    values <- list()
    parameters <- list()
    for(d2 in dir(d))
    {
      print(d2)
      p2 <- cbind(p, d2)
      p2 <- cbind(p2, "/")
      p2 <- str_c(p2, collapse = "")
      
      k = 1

      perseed_values <- list()
      perseed_parameters <- list()
      for(d3 in dir(p2))
      {	
        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "celog.csv")
	if(!file.exists(str_c(p3, collapse = "")))
	 next
        ce <- read.csv(str_c(p3, collapse = ""), header=TRUE)

        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "kflog.csv")
	if(!file.exists(str_c(p3, collapse = "")))
	  next
        kf <- read.csv(str_c(p3, collapse = ""), header=TRUE)

        perseed_values <- c(perseed_values, extractValueFunc(kf, ce, k))
        k <- k + 1
      }
      j = j + 1

      parameters <- c(parameters, d2)
      values <- c(values, lapply(perseed_values, mean))
    }
    i = i + 1
    printFunc(do.call(c, values), do.call(c,parameters))
  }
}
roc_eval <- function (extractValueFunc, printFunc)
{
  dirs = dir() # pipeline level
  
  parameter_count <- 0;
  for(d in dirs)
    parameter_count <- max(parameter_count, length(dir(d)))
  
  cat(sprintf('%s parameters on %s pipelines\n', parameter_count, length(dirs)))
  
  i = 1
  for(d in dirs)
  {
    print(d)
    p <- cbind(d, "/")
    
    j = 1
    for(d2 in dir(d))
    {
      print(d2)
      p2 <- cbind(p, d2)
      p2 <- cbind(p2, "/")
      p2 <- str_c(p2, collapse = "")
      
      k = 1
      
      values <- list()
      perseed_values <- list()
      for(d3 in dir(p2))
      {  
        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "celog.csv")
        if(!file.exists(str_c(p3, collapse = "")))
          next
        ce <- read.csv(str_c(p3, collapse = ""), header=TRUE)
        
        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "kflog.csv")
        if(!file.exists(str_c(p3, collapse = "")))
          next
        kf <- read.csv(str_c(p3, collapse = ""), header=TRUE)
        
        perseed_values <- c(perseed_values, extractValueFunc(kf, ce, k))
        k <- k + 1
      }
      j = j + 1
      
      values <- c(values, lapply(perseed_values, mean))
      printFunc(do.call(c, values))
    }
    i = i + 1
  }
}
xy_eval <- function (extractValueFunc, printFunc)
{
  dirs = dir() # pipeline level
  
  parameter_count <- 0;
  for(d in dirs)
    parameter_count <- max(parameter_count, length(dir(d)))
  
  cat(sprintf('%s parameters on %s pipelines\n', parameter_count, length(dirs)))
  
  i = 1
  for(d in dirs)
  {
    print(d)
    p <- cbind(d, "/")
    
    j = 1
    values <- list()
    for(d2 in dir(d))
    {
      print(d2)
      p2 <- cbind(p, d2)
      p2 <- cbind(p2, "/")
      p2 <- str_c(p2, collapse = "")
      
      k = 1
      
      perseed_values <- list()
      for(d3 in dir(p2))
      {  
        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "celog.csv")
        if(!file.exists(str_c(p3, collapse = "")))
          next
        ce <- read.csv(str_c(p3, collapse = ""), header=TRUE)
        
        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "kflog.csv")
        if(!file.exists(str_c(p3, collapse = "")))
          next
        kf <- read.csv(str_c(p3, collapse = ""), header=TRUE)
        
        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "kfadditional_log.csv")
        if(!file.exists(str_c(p3, collapse = "")))
          next
        kfAdd <- tryCatch(read.csv(str_c(p3, collapse = ""), header=TRUE), error=function(e) NULL)
        
        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "ceadditional_log.csv")
        if(!file.exists(str_c(p3, collapse = "")))
          ceAdd <- list()
        else
          ceAdd <- tryCatch(read.csv(str_c(p3, collapse = ""), header=TRUE), error=function(e) NULL)
        
        perseed_values <- c(perseed_values, extractValueFunc(kf, ce, kfAdd, ceAdd, k, d2))
        k <- k + 1
      }
      j = j + 1
      
      values <- c(values, lapply(perseed_values, mean))
    }
    printFunc(do.call(c, values))
    i = i + 1
  }
}
full_dir_eval <- function (initFunc, perSeedInitFunc, calcFunc, printFunc)
{
  dirs = dir() # pipeline level

  parameter_count <- 0;
  for(d in dirs)
     parameter_count <- max(parameter_count, length(dir(d)))

  cat(sprintf('%s parameters on %s pipelines\n', parameter_count, length(dirs)))
 
  initFunc(length(dirs), parameter_count)

  i = 1
  for(d in dirs)
  {
    print(d)
    p <- cbind(d, "/")
    j = 1
    for(d2 in dir(d))
    {
      print(d2)
      p2 <- cbind(p, d2)
      p2 <- cbind(p2, "/")
      p2 <- str_c(p2, collapse = "")
      k = 1
      
      perSeedInitFunc(length(dir(p2)))

      for(d3 in dir(p2))
      {	
        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "celog.csv")
	if(!file.exists(str_c(p3, collapse = "")))
	 next
        ce <- read.csv(str_c(p3, collapse = ""), header=TRUE)

        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "kflog.csv")
	if(!file.exists(str_c(p3, collapse = "")))
	  next
        kf <- read.csv(str_c(p3, collapse = ""), header=TRUE)

        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "kfadditional_log.csv")
	if(!file.exists(str_c(p3, collapse = "")))
	  next
        kfAdd <- tryCatch(read.csv(str_c(p3, collapse = ""), header=TRUE), error=function(e) NULL)

        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "ceadditional_log.csv")
	if(!file.exists(str_c(p3, collapse = "")))
	  next
        ceAdd <- tryCatch(read.csv(str_c(p3, collapse = ""), header=TRUE), error=function(e) NULL)

        calcFunc(kf, ce, kfAdd, ceAdd, k)

        k <- k + 1
      }
      printFunc(d2, i, j)
    
      j = j + 1
    }
    i = i + 1
  
  }
}
full_dir_no_add_eval <- function (initFunc, perSeedInitFunc, calcFunc, printFunc)
{
  dirs = dir() # pipeline level

  parameter_count <- 0;
  for(d in dirs)
     parameter_count <- max(parameter_count, length(dir(d)))

  cat(sprintf('%s parameters on %s pipelines\n', parameter_count, length(dirs)))
 
  initFunc(length(dirs), parameter_count)

  i = 1
  for(d in dirs)
  {
    print(d)
    p <- cbind(d, "/")
    j = 1
    for(d2 in dir(d))
    {
      p2 <- cbind(p, d2)
      p2 <- cbind(p2, "/")
      p2 <- str_c(p2, collapse = "")
      k = 1
      
      perSeedInitFunc(length(dir(p2)))

      for(d3 in dir(p2))
      {	
        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "celog.csv")
	if(!file.exists(str_c(p3, collapse = "")))
	 next
        ce <- read.csv(str_c(p3, collapse = ""), header=TRUE)

        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- cbind(p3, "kflog.csv")
	if(!file.exists(str_c(p3, collapse = "")))
	  next
        kf <- read.csv(str_c(p3, collapse = ""), header=TRUE)

        calcFunc(kf, ce, k)

        k <- k + 1
      }
      printFunc(d2, i, j)
    
      j = j + 1
    }
    i = i + 1
  
  }
}

full_archive_no_add_eval <- function (initFunc, perSeedInitFunc, calcFunc, printFunc)
{
  dirs = dir() # pipeline level

  for(d in dirs)
  {
    print(d)
    p <- cbind(d, "/")

    i = 1
    parameter_count <- 0;
    pipes_count <- length(dir(d));
    for(d2 in dir(d))
    {
      p2 <- cbind(p, d2)
      p2 <- cbind(p2, "/")
      p2 <- str_c(p2, collapse = "")
      parameter_count <- max(parameter_count, length(dir(p2)))
    }
    cat(sprintf('%s parameters on %s pipelines\n', parameter_count, pipes_count))
   
    initFunc(pipes_count, parameter_count)

    for(d2 in dir(d))
    {
      j = 1
      p2 <- cbind(p, d2)
      p2 <- cbind(p2, "/")
      p2 <- str_c(p2, collapse = "")
      for(d3 in dir(p2))
      {
        p3 <- cbind(p2, d3)
        p3 <- cbind(p3, "/")
        p3 <- str_c(p3, collapse = "")
        k = 1
        
        perSeedInitFunc(length(dir(p3)))

        for(d4 in dir(p3))
        {	
          p4 <- cbind(p3, d4)
          p4 <- cbind(p4, "/")
          p4 <- cbind(p4, "celog.csv")
	  if(!file.exists(str_c(p4, collapse = "")))
	   next
          ce <- read.csv(str_c(p4, collapse = ""), header=TRUE)

          p4 <- cbind(p3, d4)
          p4 <- cbind(p4, "/")
          p4 <- cbind(p4, "kflog.csv")
	  if(!file.exists(str_c(p4, collapse = "")))
	    next
          kf <- read.csv(str_c(p4, collapse = ""), header=TRUE)

          calcFunc(kf, ce, k)

          k <- k + 1
        }
        printFunc(d3, i, j)
      
        j = j + 1
      }
      i = i + 1
    }
  
  }
}


